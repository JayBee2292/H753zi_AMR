#!/usr/bin/env python3
"""CANable2 slcan CAN FD smoke monitor.

This talks directly to the CANable2 slcan firmware. It is useful when CAN FD
support is needed and the device appears as /dev/ttyACM* instead of can0.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import dataclass

import serial


FD_DLC_TO_LENGTH = {
    **{str(i): i for i in range(9)},
    "9": 12,
    "A": 16,
    "B": 20,
    "C": 24,
    "D": 32,
    "E": 48,
    "F": 64,
}

CAN_TELEMETRY_COMMAND_ID = 0x1FF


@dataclass
class SlcanFrame:
    can_id: int
    is_extended: bool
    is_fd: bool
    brs: bool
    length: int
    data: bytes
    raw: str


def parse_slcan_frame(line: str) -> SlcanFrame | None:
    if not line:
        return None

    frame_type = line[0]
    if frame_type in {"t", "r"} and len(line) >= 5:
        can_id = int(line[1:4], 16)
        dlc = int(line[4], 16)
        data_start = 5
        is_extended = False
        is_fd = False
        brs = False
    elif frame_type in {"T", "R"} and len(line) >= 10:
        can_id = int(line[1:9], 16)
        dlc = int(line[9], 16)
        data_start = 10
        is_extended = True
        is_fd = False
        brs = False
    elif frame_type in {"d", "b"} and len(line) >= 5:
        can_id = int(line[1:4], 16)
        dlc_char = line[4].upper()
        dlc = FD_DLC_TO_LENGTH[dlc_char]
        data_start = 5
        is_extended = False
        is_fd = True
        brs = frame_type == "b"
    elif frame_type in {"D", "B"} and len(line) >= 10:
        can_id = int(line[1:9], 16)
        dlc_char = line[9].upper()
        dlc = FD_DLC_TO_LENGTH[dlc_char]
        data_start = 10
        is_extended = True
        is_fd = True
        brs = frame_type == "B"
    else:
        return None

    hex_data = line[data_start : data_start + dlc * 2]
    data = bytes.fromhex(hex_data) if hex_data else b""
    return SlcanFrame(
        can_id=can_id,
        is_extended=is_extended,
        is_fd=is_fd,
        brs=brs,
        length=dlc,
        data=data,
        raw=line,
    )


def send_command(port: serial.Serial, command: str) -> None:
    port.write(command.encode("ascii") + b"\r")
    port.flush()


def send_telemetry_enable(port: serial.Serial, enabled: bool) -> None:
    payload = bytes([1, 1 if enabled else 0]).ljust(8, b"\x00")
    send_command(
        port,
        f"d{CAN_TELEMETRY_COMMAND_ID:03X}8{payload.hex().upper()}",
    )


def drain(port: serial.Serial, duration_s: float = 0.1) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        if port.in_waiting:
            port.read(port.in_waiting)
        time.sleep(0.01)


def configure_channel(args: argparse.Namespace, port: serial.Serial) -> None:
    send_command(port, "C")
    time.sleep(0.05)
    drain(port)

    send_command(port, args.nominal)
    send_command(port, args.data)
    send_command(port, "M1" if args.silent else "M0")
    send_command(port, "A0" if args.no_retransmit else "A1")
    send_command(port, "O")
    time.sleep(0.1)
    drain(port)


def print_frame(frame: SlcanFrame) -> None:
    frame_kind = "FD" if frame.is_fd else "CAN"
    flags = []
    if frame.is_extended:
        flags.append("EXT")
    if frame.brs:
        flags.append("BRS")
    flag_text = ",".join(flags) if flags else "-"
    data_text = frame.data.hex(" ").upper()
    print(
        f"{frame_kind:<3} id=0x{frame.can_id:X} len={frame.length:<2} "
        f"flags={flag_text:<7} data={data_text}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor CANable2 slcan CAN FD frames.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="CANable2 serial port")
    parser.add_argument("--baud", type=int, default=921600, help="CANable2 UART baudrate")
    parser.add_argument("--nominal", default="S8", help="slcan nominal bitrate command, S8=1M")
    parser.add_argument("--data", default="Y5", help="slcan data bitrate command, Y5=5M")
    parser.add_argument("--silent", action="store_true", help="listen-only mode; does not ACK frames")
    parser.add_argument("--no-retransmit", action="store_true", default=True)
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to run; 0 means forever")
    parser.add_argument("--raw", action="store_true", help="print raw non-frame lines")
    parser.add_argument(
        "--enable-telemetry",
        action="store_true",
        help="enable STM32 telemetry on 0x1FF and refresh it once per second",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stop = False

    def handle_signal(signum: int, frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        configure_channel(args, port)
        print(
            f"monitoring {args.port}: nominal={args.nominal} data={args.data} "
            f"mode={'silent' if args.silent else 'normal'}"
        )

        next_enable_refresh = time.monotonic()
        if args.enable_telemetry:
            send_telemetry_enable(port, True)

        deadline = time.monotonic() + args.duration if args.duration > 0 else None
        buffer = bytearray()

        while not stop:
            now = time.monotonic()
            if args.enable_telemetry and now >= next_enable_refresh:
                send_telemetry_enable(port, True)
                next_enable_refresh = now + 1.0

            if deadline is not None and time.monotonic() >= deadline:
                break

            chunk = port.read(256)
            if not chunk:
                continue

            buffer.extend(chunk)
            while b"\r" in buffer:
                raw_line, _, remainder = buffer.partition(b"\r")
                buffer = bytearray(remainder)
                line = raw_line.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                try:
                    frame = parse_slcan_frame(line)
                except (KeyError, ValueError):
                    frame = None

                if frame is not None:
                    print_frame(frame)
                elif args.raw:
                    print(line)

        if args.enable_telemetry:
            send_telemetry_enable(port, False)
            time.sleep(0.02)

        send_command(port, "E")
        time.sleep(0.1)
        error_text = port.read(port.in_waiting or 128).decode("ascii", errors="replace")
        if error_text.strip():
            print(error_text.replace("\r", "\n").strip())
        send_command(port, "C")

    return 0


if __name__ == "__main__":
    sys.exit(main())
