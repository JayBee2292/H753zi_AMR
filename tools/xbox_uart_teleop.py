#!/usr/bin/env python3
"""UART open-loop teleop for the STM32 using an Xbox controller.

The firmware side should be running with `APP_MODE == 2`.

Prerequisites:
  pip install pyserial pygame
"""

import argparse
import signal
import sys
import time
import os

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

try:
    import serial
    from serial import SerialException
except ImportError as exc:
    raise SystemExit(
        "pyserial is required. Install it with `python3 -m pip install pyserial`."
    ) from exc

try:
    import pygame
except ImportError as exc:
    raise SystemExit(
        "pygame is required for Xbox controller support. Install it with `python3 -m pip install pygame`."
    ) from exc

from xbox_drive_config import (
    DEFAULT_MAX_ANGULAR_RADPS,
    DEFAULT_MAX_LINEAR_MPS,
    DEFAULT_MAX_TRACK_SPEED_MPS,
    DEFAULT_TRACK_BELT_WIDTH_M,
    DEFAULT_TRACK_CONTACT_LENGTH_M,
    DEFAULT_TRACK_GAUGE_M,
    MOTION_NAMES,
    SERIAL_POLL_INTERVAL_MS,
    TRANSMIT_INTERVAL_MS,
)
from xbox_drive_core import (
    DriveControlLock,
    DriveSessionLog,
    apply_deadzone,
    default_drive_log_path,
    encode_legacy_frame,
    encode_stop_frame,
    encode_twist_frame,
    is_probably_canable_port,
    joystick_to_twist,
    limit_twist_to_track_speed,
    resolve_stm_uart_port,
    serial_port_description,
    track_speeds_to_uart_packet,
)


def install_shutdown_handlers() -> None:
    def shutdown_handler(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGHUP, shutdown_handler)


class XboxUartTeleop:
    def __init__(
        self,
        port: str | None,
        baud: int,
        debug: bool = False,
        dry_run: bool = False,
        protocol: str = "twist",
        joy_idx: int = -1,
        max_linear_mps: float = DEFAULT_MAX_LINEAR_MPS,
        max_angular_radps: float = DEFAULT_MAX_ANGULAR_RADPS,
        track_gauge_m: float = DEFAULT_TRACK_GAUGE_M,
        max_track_speed_mps: float = DEFAULT_MAX_TRACK_SPEED_MPS,
        track_contact_length_m: float = DEFAULT_TRACK_CONTACT_LENGTH_M,
        track_belt_width_m: float = DEFAULT_TRACK_BELT_WIDTH_M,
        log_file: str = default_drive_log_path(),
    ):
        self.debug = debug
        self.dry_run = dry_run
        self.protocol = protocol
        self.max_linear_mps = max_linear_mps
        self.max_angular_radps = max_angular_radps
        self.track_gauge_m = track_gauge_m
        self.max_track_speed_mps = max_track_speed_mps
        self.track_contact_length_m = track_contact_length_m
        self.track_belt_width_m = track_belt_width_m
        self.control_lock: DriveControlLock | None = None

        if not self.dry_run:
            try:
                self.control_lock = DriveControlLock()
            except RuntimeError as exc:
                raise SystemExit(str(exc)) from exc

        # Disable SDL HIDAPI driver. This is a known workaround for Xbox Bluetooth
        # controllers under Linux, which otherwise often fail to expose themselves
        # via the traditional /dev/input/jsX paths to pygame.
        os.environ["SDL_JOYSTICK_HIDAPI"] = "0"

        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count == 0:
            raise SystemExit("No joystick/gamepad found. Please connect your Xbox controller.")

        print("Available joysticks:")
        for i in range(count):
            joy = pygame.joystick.Joystick(i)
            joy.init()
            print(f"  [{i}] {joy.get_name()}")

        if joy_idx >= 0:
            if joy_idx >= count:
                raise SystemExit(f"Requested joystick index {joy_idx} out of range (0 to {count - 1}).")
            target_idx = joy_idx
        else:
            target_idx = 0
            for i in range(count):
                joy = pygame.joystick.Joystick(i)
                joy.init()
                name = joy.get_name()
                if "xbox" in name.lower() or "x-box" in name.lower() or "microsoft" in name.lower():
                    target_idx = i
                    break

        self.joystick = pygame.joystick.Joystick(target_idx)
        if not self.joystick.get_init():
            self.joystick.init()
        print(f"Found controller: {self.joystick.get_name()}")

        self.port = resolve_stm_uart_port(port)
        self.baud = baud
        self.last_tx_frame = b""

        if is_probably_canable_port(self.port):
            raise SystemExit(
                f"Refusing to use CANable as STM UART: {serial_port_description(self.port)}\n"
                "Use the STLINK VCP port, usually /dev/ttyACM1 or the STMicroelectronics /dev/serial/by-id path."
            )

        if not self.dry_run:
            try:
                self.serial_port = serial.Serial(port=self.port, baudrate=baud, timeout=0)
            except SerialException as exc:
                raise SystemExit(f"Failed to open serial port {self.port}: {exc}") from exc
            print(f"Connected to {serial_port_description(self.port)} @ {self.baud}")
        else:
            self.serial_port = None
            print("[DRY RUN] Serial transmission disabled. Only reading joystick.")
        self.session_log = DriveSessionLog(log_file)
        print(f"Session log: {self.session_log.path}")
        print(f"UART protocol: {self.protocol}")
        self.session_log.write(
            "INFO",
            f"uart={serial_port_description(self.port)} baud={self.baud} protocol={self.protocol} "
            f"dry_run={self.dry_run} limits linear={self.max_linear_mps:.2f} "
            f"angular={self.max_angular_radps:.2f} track={self.max_track_speed_mps:.2f}",
        )

        self.last_tx_time = time.monotonic()
        self.last_poll_time = time.monotonic()

    def send_uart_frame(self, frame: bytes) -> None:
        if self.serial_port is None: return
        self.serial_port.write(frame)
        self.serial_port.flush()

    def poll_serial_rx(self) -> None:
        if self.serial_port is None: return
        available = self.serial_port.in_waiting
        if available:
            data = self.serial_port.read(available).decode("utf-8", errors="replace")
            self.session_log.feed_serial(data)
            sys.stdout.write(data)
            sys.stdout.flush()

    def run(self):
        print("Controls:")
        print("  Left Stick Y-axis : Forward / Backward")
        print("  Right Stick X-axis: Rotate in place / steer while moving")
        print("  Left Stick X-axis : Ignored")
        print(
            f"Track model: gauge={self.track_gauge_m:.3f}m, "
            f"contact={self.track_contact_length_m:.3f}m, "
            f"belt_width={self.track_belt_width_m:.3f}m"
        )
        print(
            f"Speed limits: linear={self.max_linear_mps:.2f}m/s, "
            f"angular={self.max_angular_radps:.2f}rad/s, "
            f"track={self.max_track_speed_mps:.2f}m/s"
        )
        print("Press Ctrl+C to quit.")

        try:
            while True:
                now = time.monotonic()
                pygame.event.pump()  # Update internal state

                # Axes usually go from -1.0 to 1.0
                # Left stick Y (Forward is negative on most controllers)
                lx = self.joystick.get_axis(0)
                ly = self.joystick.get_axis(1)
                # Right stick X (Right is positive)
                rx = self.joystick.get_axis(3) if self.joystick.get_numaxes() >= 4 else self.joystick.get_axis(2)

                # Deadzones
                lx = apply_deadzone(lx)
                ly = apply_deadzone(ly)
                rx = apply_deadzone(rx)

                throttle = -ly
                steer = rx
                linear_mps, angular_radps = joystick_to_twist(
                    throttle,
                    steer,
                    self.max_linear_mps,
                    self.max_angular_radps,
                )
                linear_mps, angular_radps, left_mps, right_mps = limit_twist_to_track_speed(
                    linear_mps,
                    angular_radps,
                    self.track_gauge_m,
                    self.max_track_speed_mps,
                )
                motion_code, duty, curve_ratio, left_mix, right_mix = track_speeds_to_uart_packet(
                    left_mps,
                    right_mps,
                    self.max_track_speed_mps,
                )

                if self.debug:
                    motion_name = MOTION_NAMES.get(motion_code, f"unknown_{motion_code}")
                    sys.stdout.write(
                        f"\r[DEBUG] LX: {lx:+.2f} | LY: {ly:+.2f} | RX: {rx:+.2f} "
                        f"-> Throttle: {throttle:+.2f} | Steer: {steer:+.2f} "
                        f"-> v/w: {linear_mps:+.2f}m/s/{angular_radps:+.2f}rad/s "
                        f"-> track L/R: {left_mps:+.2f}/{right_mps:+.2f}m/s "
                        f"-> norm L/R: {left_mix:+.2f}/{right_mix:+.2f} "
                        f"-> Motion: {motion_name}({motion_code}) | Duty: {duty:03} | Curve: {curve_ratio:03} "
                        f"| TX: {self.last_tx_frame.hex(' ')}    "
                    )
                    sys.stdout.flush()

                if now - self.last_tx_time >= TRANSMIT_INTERVAL_MS / 1000.0:
                    if self.protocol == "legacy":
                        self.last_tx_frame = encode_legacy_frame(motion_code, duty, curve_ratio)
                    else:
                        self.last_tx_frame = encode_twist_frame(linear_mps, angular_radps)
                    self.send_uart_frame(self.last_tx_frame)
                    self.last_tx_time = now
                    motion_name = MOTION_NAMES.get(motion_code, f"unknown_{motion_code}")
                    self.session_log.write(
                        "TX",
                        f"lx={lx:+.2f} ly={ly:+.2f} rx={rx:+.2f} "
                        f"v/w={linear_mps:+.3f}/{angular_radps:+.3f} "
                        f"track L/R={left_mps:+.3f}/{right_mps:+.3f} "
                        f"cmd={motion_name} duty={duty:03d} ratio={curve_ratio:03d} "
                        f"frame={self.last_tx_frame.hex(' ')}",
                    )

                if now - self.last_poll_time >= SERIAL_POLL_INTERVAL_MS / 1000.0:
                    self.poll_serial_rx()
                    self.last_poll_time = now

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nExiting...")
        finally:
            if self.protocol == "legacy":
                self.send_uart_frame(encode_legacy_frame(0, 0, 50))
            else:
                self.send_uart_frame(encode_stop_frame())
            if self.serial_port is not None:
                self.serial_port.close()
            self.session_log.close()
            if self.control_lock is not None:
                self.control_lock.close()
            # SDL joystick shutdown can block after an Xbox device disconnects.
            # Process exit releases pygame resources after the stop frame is sent.


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Xbox controller teleop for the STM32 via UART."
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port path. If omitted, an STLINK /dev/serial/by-id port is auto-selected.",
    )
    parser.add_argument("--baud", type=int, default=921600, help="UART baudrate")
    parser.add_argument("--debug", action="store_true", help="Print joystick mapping and transmitted frame")
    parser.add_argument("--dry-run", action="store_true", help="Do not open UART; only read joystick input")
    parser.add_argument(
        "--protocol",
        choices=("twist", "legacy"),
        default="twist",
        help="UART packet protocol. Use legacy only to test old STM firmware.",
    )
    parser.add_argument("--joy-idx", type=int, default=-1, help="Force specific joystick index instead of auto-detecting")
    parser.add_argument("--max-linear", type=float, default=DEFAULT_MAX_LINEAR_MPS, help="Maximum linear speed command in m/s")
    parser.add_argument("--max-angular", type=float, default=DEFAULT_MAX_ANGULAR_RADPS, help="Maximum angular speed command in rad/s")
    parser.add_argument("--track-gauge", type=float, default=DEFAULT_TRACK_GAUGE_M, help="Effective distance between left/right track centers in meters")
    parser.add_argument("--max-track-speed", type=float, default=DEFAULT_MAX_TRACK_SPEED_MPS, help="Track speed that maps to 100 percent duty")
    parser.add_argument("--track-contact-length", type=float, default=DEFAULT_TRACK_CONTACT_LENGTH_M, help="Track ground contact length in meters")
    parser.add_argument("--track-belt-width", type=float, default=DEFAULT_TRACK_BELT_WIDTH_M, help="Track belt width in meters")
    parser.add_argument(
        "--log-file",
        default=default_drive_log_path(),
        help="Session log output path. The file is overwritten for each run.",
    )
    return parser.parse_args()


def main() -> int:
    install_shutdown_handlers()
    args = parse_args()
    app = XboxUartTeleop(
        port=args.port,
        baud=args.baud,
        debug=args.debug,
        dry_run=args.dry_run,
        protocol=args.protocol,
        joy_idx=args.joy_idx,
        max_linear_mps=args.max_linear,
        max_angular_radps=args.max_angular,
        track_gauge_m=args.track_gauge,
        max_track_speed_mps=args.max_track_speed,
        track_contact_length_m=args.track_contact_length,
        track_belt_width_m=args.track_belt_width,
        log_file=args.log_file,
    )
    install_shutdown_handlers()
    app.run()
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    # SDL/Pygame finalizers can block after Xbox input teardown.
    # UART stop/close and log flush have already completed inside run().
    os._exit(exit_code)
