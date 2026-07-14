#!/usr/bin/env python3
"""UART open-loop teleop for the STM32 smoke-test mode.

The firmware side should be running with `SMOKE_TEST == 2`.

`terminal` mode is the default for interactive shells because it avoids Tk
focus issues and keeps local key handling visible in the same window.
`gui` mode preserves the original hold-to-drive behavior with key
press/release events.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import time

try:
    import termios
    import tty
except ImportError:  # pragma: no cover - non-POSIX fallback
    termios = None
    tty = None

try:
    import tkinter as tk
    from tkinter import scrolledtext
except ImportError:  # pragma: no cover - tkinter optional in terminal mode
    tk = None
    scrolledtext = None

try:
    import serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover - import failure path
    raise SystemExit(
        "pyserial is required. Install it with `python3 -m pip install pyserial`."
    ) from exc


MOTION_KEYS = {"w", "a", "s", "d", "q", "e", "z", "c"}
STOP_KEYS = {"x", "space"}
ONE_SHOT_KEYS = {"[", "]", "<", ">", ",", ".", "h", "?"}
TRANSMIT_INTERVAL_MS = 60
SERIAL_POLL_INTERVAL_MS = 50
GUI_RELEASE_DEBOUNCE_MS = 50
TERMINAL_TIMEOUT_MS = 600
MAX_LOG_CHARS = 12000
FRAME_HEADER_0 = 0xA5
FRAME_HEADER_1 = 0x5A
PACKET_STOP = 0
PACKET_FORWARD = 1
PACKET_BACKWARD = 2
PACKET_TURN_LEFT = 3
PACKET_TURN_RIGHT = 4
PACKET_CURVE_LEFT = 5
PACKET_CURVE_RIGHT = 6
PACKET_BACKWARD_CURVE_LEFT = 7
PACKET_BACKWARD_CURVE_RIGHT = 8
DEFAULT_DUTY = 35
MIN_DUTY = 5
MAX_DUTY = 100
DUTY_STEP = 5
DEFAULT_CURVE_RATIO = 50
HELP_TEXT = (
    "W/A/S/D/Q/E/Z/C set motion.\n"
    "X or Space stops immediately.\n"
    "[ and ] adjust duty, < and > adjust ratio, H or ? prints help.\n"
    "This tool sends full-state frames, not repeated ASCII characters.\n"
)


def motion_name_from_key(key: str | None) -> str:
    if key == "w":
        return "forward"
    if key == "s":
        return "backward"
    if key == "a":
        return "turn_left"
    if key == "d":
        return "turn_right"
    if key == "q":
        return "curve_left"
    if key == "e":
        return "curve_right"
    if key == "z":
        return "backward_curve_left"
    if key == "c":
        return "backward_curve_right"
    return "stop"


def motion_code_from_key(key: str | None) -> int:
    if key == "w":
        return PACKET_FORWARD
    if key == "s":
        return PACKET_BACKWARD
    if key == "a":
        return PACKET_TURN_LEFT
    if key == "d":
        return PACKET_TURN_RIGHT
    if key == "q":
        return PACKET_CURVE_LEFT
    if key == "e":
        return PACKET_CURVE_RIGHT
    if key == "z":
        return PACKET_BACKWARD_CURVE_LEFT
    if key == "c":
        return PACKET_BACKWARD_CURVE_RIGHT
    return PACKET_STOP


class UartTeleopSession:
    def __init__(self, port: str, baud: int) -> None:
        try:
            self.serial_port = serial.Serial(port=port, baudrate=baud, timeout=0)
        except SerialException as exc:
            raise SystemExit(f"Failed to open serial port {port}: {exc}") from exc

        self.port = port
        self.baud = baud
        self.duty = DEFAULT_DUTY
        self.curve_ratio = DEFAULT_CURVE_RATIO
        self.active_motion_key: str | None = None
        self.last_tx_command = "stop"
        self.last_status = f"Connected to {port} @ {baud}"

    def status_text(self) -> str:
        return f"{self.last_status} | motion: {self.last_tx_command} | duty: {self.duty} | ratio: {self.curve_ratio}"

    def send_state_frame(self) -> None:
        motion_code = motion_code_from_key(self.active_motion_key)
        payload = bytes(
            [
                FRAME_HEADER_0,
                FRAME_HEADER_1,
                motion_code,
                self.duty,
                self.curve_ratio,
                FRAME_HEADER_0 ^ FRAME_HEADER_1 ^ motion_code ^ self.duty ^ self.curve_ratio,
            ]
        )

        self.serial_port.write(payload)
        self.serial_port.flush()
        self.last_tx_command = motion_name_from_key(self.active_motion_key)

    def set_motion_key(self, key: str | None) -> None:
        self.active_motion_key = key
        self.send_state_frame()

    def adjust_duty(self, delta: int) -> None:
        self.duty = max(MIN_DUTY, min(MAX_DUTY, self.duty + delta))
        self.send_state_frame()

    def adjust_curve_ratio(self, delta: int) -> None:
        self.curve_ratio = max(0, min(100, self.curve_ratio + delta))
        self.send_state_frame()

    def poll_serial_rx(self) -> str:
        available = self.serial_port.in_waiting
        if not available:
            return ""
        return self.serial_port.read(available).decode("utf-8", errors="replace")

    def close(self) -> None:
        try:
            self.active_motion_key = None
            self.send_state_frame()
        except SerialException:
            pass
        finally:
            self.serial_port.close()


class UartTeleopGuiApp:
    def __init__(self, root: tk.Tk, port: str, baud: int) -> None:
        self.root = root
        self.root.title("UART Open-Loop Teleop")
        self.root.geometry("760x480")
        self.session = UartTeleopSession(port=port, baud=baud)

        self.pending_release_jobs: dict[str, str] = {}
        self._build_ui()
        self._bind_keys()
        self._log(
            "Focus this window and hold W/A/S/D.\n"
            "Q/E are forward curves. Z/C are backward curves.\n"
            "Space or X stops immediately. [ and ] adjust duty.\n"
            "This tool sends full-state frames, not repeated ASCII characters.\n"
        )
        self.session.send_state_frame()
        self._refresh_status()
        self._schedule_tasks()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        info_lines = [
            f"Port: {self.session.port}    Baud: {self.session.baud}",
            "Hold W: forward, S: backward, A: left turn, D: right turn",
            "Hold Q/E: forward curve, Z/C: backward curve",
            "Press Space/X: stop, [: duty down, ]: duty up",
        ]
        header = tk.Label(
            self.root,
            text="\n".join(info_lines),
            justify="left",
            anchor="w",
            font=("TkDefaultFont", 11),
        )
        header.pack(fill="x", padx=12, pady=(12, 8))

        self.status_var = tk.StringVar(value=self.session.status_text())
        status_label = tk.Label(
            self.root,
            textvariable=self.status_var,
            justify="left",
            anchor="w",
            font=("TkDefaultFont", 11, "bold"),
        )
        status_label.pack(fill="x", padx=12, pady=(0, 8))

        self.log_widget = scrolledtext.ScrolledText(
            self.root,
            wrap="word",
            height=20,
            state="disabled",
            font=("Courier New", 10),
        )
        self.log_widget.pack(fill="both", expand=True, padx=12, pady=(0, 12))

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.focus_force()

    def _schedule_tasks(self) -> None:
        self.root.after(TRANSMIT_INTERVAL_MS, self._periodic_transmit)
        self.root.after(SERIAL_POLL_INTERVAL_MS, self._poll_serial_rx)

    def _refresh_status(self) -> None:
        self.status_var.set(self.session.status_text())

    def _log(self, text: str) -> None:
        self.log_widget.configure(state="normal")
        self.log_widget.insert("end", text)
        if int(self.log_widget.index("end-1c").split(".")[0]) > 400:
            self.log_widget.delete("1.0", "80.0")
        current_text = self.log_widget.get("1.0", "end-1c")
        if len(current_text) > MAX_LOG_CHARS:
            trim_start = len(current_text) - MAX_LOG_CHARS
            self.log_widget.delete("1.0", f"1.0+{trim_start}c")
        self.log_widget.see("end")
        self.log_widget.configure(state="disabled")

    def _log_local_state(self, prefix: str) -> None:
        self._log(
            f"[local] {prefix}: motion={motion_name_from_key(self.session.active_motion_key)} "
            f"duty={self.session.duty}\n"
        )

    def _set_motion_key(self, key: str | None, *, reason: str) -> None:
        self.session.set_motion_key(key)
        self._refresh_status()
        self._log_local_state(reason)

    def _cancel_pending_release(self, key: str) -> None:
        job = self.pending_release_jobs.pop(key, None)
        if job is not None:
            self.root.after_cancel(job)

    def _finalize_key_release(self, key: str) -> None:
        self.pending_release_jobs.pop(key, None)
        if self.session.active_motion_key == key:
            self._set_motion_key(None, reason="release")

    def _normalize_keysym(self, event: tk.Event) -> str | None:
        keysym = (event.keysym or "").lower()
        if keysym == "space":
            return "space"

        if keysym in MOTION_KEYS:
            return keysym

        char = (event.char or "").lower()
        if char in MOTION_KEYS or char in {"x", "[", "]", "h", "?", "<", ">", ",", "."}:
            return char

        return None

    def _on_key_press(self, event: tk.Event) -> None:
        key = self._normalize_keysym(event)
        if key is None:
            return

        if key in MOTION_KEYS:
            for pending_key in list(self.pending_release_jobs):
                self._cancel_pending_release(pending_key)
            self._set_motion_key(key, reason="press")
            return

        if key in STOP_KEYS:
            for pending_key in list(self.pending_release_jobs):
                self._cancel_pending_release(pending_key)
            self._set_motion_key(None, reason="stop")
            return

        if key == "[":
            self.session.adjust_duty(-DUTY_STEP)
            self._refresh_status()
            self._log_local_state("duty")
        elif key == "]":
            self.session.adjust_duty(DUTY_STEP)
            self._refresh_status()
            self._log_local_state("duty")
        elif key in {"<", ","}:
            self.session.adjust_curve_ratio(-DUTY_STEP)
            self._refresh_status()
            self._log_local_state("ratio")
        elif key in {">", "."}:
            self.session.adjust_curve_ratio(DUTY_STEP)
            self._refresh_status()
            self._log_local_state("ratio")
        elif key in {"h", "?"}:
            self._log(HELP_TEXT)

    def _on_key_release(self, event: tk.Event) -> None:
        key = self._normalize_keysym(event)
        if key is None:
            return

        if key in MOTION_KEYS:
            self._cancel_pending_release(key)
            if self.session.active_motion_key == key:
                self.pending_release_jobs[key] = self.root.after(
                    GUI_RELEASE_DEBOUNCE_MS,
                    lambda released_key=key: self._finalize_key_release(released_key),
                )

    def _periodic_transmit(self) -> None:
        try:
            self.session.send_state_frame()
            self._refresh_status()
        except SerialException as exc:
            self.status_var.set(f"Serial write failed: {exc}")
            return

        self.root.after(TRANSMIT_INTERVAL_MS, self._periodic_transmit)

    def _poll_serial_rx(self) -> None:
        try:
            payload = self.session.poll_serial_rx()
            if payload:
                self._log(payload)
        except SerialException as exc:
            self.status_var.set(f"Serial read failed: {exc}")
            return

        self.root.after(SERIAL_POLL_INTERVAL_MS, self._poll_serial_rx)

    def _on_close(self) -> None:
        try:
            self.session.close()
        finally:
            self.root.destroy()


class RawTerminal:
    def __init__(self, fileobj) -> None:
        self.fileobj = fileobj
        self.fd = fileobj.fileno()
        self._old_attrs: list[int] | None = None

    def __enter__(self) -> "RawTerminal":
        if termios is None or tty is None:
            raise SystemExit("Terminal mode requires termios/tty support.")
        self._old_attrs = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, exc_tb) -> None:
        if self._old_attrs is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old_attrs)


class UartTeleopTerminalApp:
    def __init__(self, port: str, baud: int) -> None:
        self.session = UartTeleopSession(port=port, baud=baud)
        self.stdin_fd = sys.stdin.fileno()
        self.input_buffer = b""

    def _print(self, text: str) -> None:
        sys.stdout.write(text)
        sys.stdout.flush()

    def _print_local_state(self, reason: str) -> None:
        self._print(
            f"[local] {reason}: motion={motion_name_from_key(self.session.active_motion_key)} "
            f"duty={self.session.duty}\n"
        )

    def _print_help(self) -> None:
        self._print(
            f"Connected to {self.session.port} @ {self.session.baud}\n"
            f"{HELP_TEXT}"
            "Arrow keys are also mapped to forward/left/backward/right.\n"
            "Q/E are forward curves. Z/C are backward curves.\n"
        )

    def _read_ready_keys(self) -> list[str]:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return []

        chunk = os.read(self.stdin_fd, 64)
        if not chunk:
            return []

        self.input_buffer += chunk
        keys: list[str] = []

        while self.input_buffer:
            if self.input_buffer.startswith(b"\x1b[A"):
                keys.append("w")
                self.input_buffer = self.input_buffer[3:]
                continue
            if self.input_buffer.startswith(b"\x1b[B"):
                keys.append("s")
                self.input_buffer = self.input_buffer[3:]
                continue
            if self.input_buffer.startswith(b"\x1b[C"):
                keys.append("d")
                self.input_buffer = self.input_buffer[3:]
                continue
            if self.input_buffer.startswith(b"\x1b[D"):
                keys.append("a")
                self.input_buffer = self.input_buffer[3:]
                continue

            if self.input_buffer[0] == 0x1B:
                if len(self.input_buffer) < 3:
                    break
                self.input_buffer = self.input_buffer[1:]
                continue

            value = self.input_buffer[0]
            self.input_buffer = self.input_buffer[1:]

            if value == 0x03:
                raise KeyboardInterrupt
            if value == 0x20:
                keys.append("space")
                continue

            char = chr(value).lower()
            if char in MOTION_KEYS or char in {"x", "[", "]", "h", "?", "<", ">", ",", "."}:
                keys.append(char)

        return keys

    def _handle_key(self, key: str) -> bool:
        if key in MOTION_KEYS:
            self.session.set_motion_key(key)
            self._print_local_state("press")
            return True

        if key in STOP_KEYS:
            self.session.set_motion_key(None)
            self._print_local_state("stop")
            return True

        if key == "[":
            self.session.adjust_duty(-DUTY_STEP)
            self._print_local_state("duty")
            return True

        if key == "]":
            self.session.adjust_duty(DUTY_STEP)
            self._print_local_state("duty")
            return True

        if key in {"<", ","}:
            self.session.adjust_curve_ratio(-DUTY_STEP)
            self._print_local_state("ratio")
            return True

        if key in {">", "."}:
            self.session.adjust_curve_ratio(DUTY_STEP)
            self._print_local_state("ratio")
            return True

        if key in {"h", "?"}:
            self._print_help()

        return True

    def run(self) -> int:
        if not sys.stdin.isatty() or not sys.stdout.isatty():
            raise SystemExit(
                "Terminal mode requires an interactive TTY. Use `--mode gui` instead."
            )

        self._print_help()
        self.session.send_state_frame()
        self._print_local_state("startup")
        next_tx_time = time.monotonic()
        next_poll_time = time.monotonic()
        last_key_time = time.monotonic()

        with RawTerminal(sys.stdin):
            try:
                while True:
                    now = time.monotonic()

                    keys = self._read_ready_keys()
                    for key in keys:
                        if not self._handle_key(key):
                            return 0
                        if key in MOTION_KEYS:
                            last_key_time = now

                    if self.session.active_motion_key is not None:
                        if now - last_key_time > TERMINAL_TIMEOUT_MS / 1000.0:
                            self.session.set_motion_key(None)
                            self._print_local_state("release")

                    if now >= next_tx_time:
                        self.session.send_state_frame()
                        next_tx_time = now + (TRANSMIT_INTERVAL_MS / 1000.0)

                    if now >= next_poll_time:
                        payload = self.session.poll_serial_rx()
                        if payload:
                            self._print(payload)
                        next_poll_time = now + (SERIAL_POLL_INTERVAL_MS / 1000.0)

                    time.sleep(0.01)
            finally:
                self.session.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="UART open-loop teleop for the STM32 smoke-test mode."
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port path")
    parser.add_argument("--baud", type=int, default=921600, help="UART baudrate")
    parser.add_argument(
        "--mode",
        choices=("auto", "terminal", "gui"),
        default="auto",
        help="Input mode. `auto` prefers terminal for interactive shells.",
    )
    return parser.parse_args()


def resolve_mode(requested_mode: str) -> str:
    if requested_mode != "auto":
        return requested_mode

    if tk is not None and scrolledtext is not None and os.environ.get("DISPLAY"):
        return "gui"

    if sys.stdin.isatty() and sys.stdout.isatty() and termios is not None and tty is not None:
        return "terminal"

    return "gui"


def main() -> int:
    args = parse_args()
    mode = resolve_mode(args.mode)

    if mode == "terminal":
        return UartTeleopTerminalApp(port=args.port, baud=args.baud).run()

    if tk is None or scrolledtext is None:
        raise SystemExit("Tkinter is not available. Use `--mode terminal` instead.")

    root = tk.Tk()
    UartTeleopGuiApp(root, port=args.port, baud=args.baud)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
