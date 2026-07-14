#!/usr/bin/env python3
"""Minimal SocketCAN teleop and monitor for the STM32 CAN odom bridge.

Protocol:
  0x100  STM32 -> host  [float linear_v, float angular_w]
  0x200  host  -> STM32 [float linear_v, float angular_w]

Example:
  sudo ip link set can0 up type can bitrate 500000
  python3 tools/can_teleop.py --if can0
"""

from __future__ import annotations

import argparse
import os
import select
import socket
import struct
import sys
import termios
import time
import tty


ODOM_TX_ID = 0x100
CMD_RX_ID = 0x200
FRAME_STRUCT = struct.Struct("=IB3x8s")
PAYLOAD_STRUCT = struct.Struct("<ff")
DEFAULT_SEND_PERIOD_S = 0.05


def encode_can_frame(can_id: int, payload: bytes) -> bytes:
    data = payload.ljust(8, b"\x00")
    return FRAME_STRUCT.pack(can_id, len(payload), data)


def decode_can_frame(frame: bytes) -> tuple[int, bytes]:
    can_id, dlc, data = FRAME_STRUCT.unpack(frame)
    return can_id & socket.CAN_EFF_MASK, data[:dlc]


def open_can_socket(ifname: str) -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((ifname,))
    sock.setblocking(False)
    return sock


def put_terminal_raw(fd: int) -> list[int]:
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old_attrs


def restore_terminal(fd: int, old_attrs: list[int]) -> None:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)


def print_help() -> None:
    print("Keys: w/s linear +/- | a/d angular +/- | x stop | q quit")


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive CAN cmd_vel sender + odometry monitor."
    )
    parser.add_argument("--if", dest="ifname", default="can0", help="SocketCAN interface")
    parser.add_argument("--linear-step", type=float, default=0.05, help="m/s per key press")
    parser.add_argument("--angular-step", type=float, default=0.20, help="rad/s per key press")
    parser.add_argument("--linear-limit", type=float, default=1.0, help="max abs linear m/s")
    parser.add_argument("--angular-limit", type=float, default=3.0, help="max abs angular rad/s")
    parser.add_argument("--period", type=float, default=DEFAULT_SEND_PERIOD_S, help="send period in seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not hasattr(socket, "AF_CAN"):
        raise SystemExit("This platform does not support SocketCAN.")

    sock = open_can_socket(args.ifname)
    stdin_fd = sys.stdin.fileno()
    old_term = put_terminal_raw(stdin_fd)

    linear_v = 0.0
    angular_w = 0.0
    next_tx_time = time.monotonic()
    last_status = ""

    print(f"Connected to {args.ifname}")
    print_help()

    try:
        while True:
            timeout = max(0.0, next_tx_time - time.monotonic())
            readable, _, _ = select.select([sock, stdin_fd], [], [], timeout)

            if stdin_fd in readable:
                key = os.read(stdin_fd, 1).decode("utf-8", errors="ignore")
                if key == "q":
                    break
                if key == "w":
                    linear_v = clamp(linear_v + args.linear_step, args.linear_limit)
                elif key == "s":
                    linear_v = clamp(linear_v - args.linear_step, args.linear_limit)
                elif key == "a":
                    angular_w = clamp(angular_w + args.angular_step, args.angular_limit)
                elif key == "d":
                    angular_w = clamp(angular_w - args.angular_step, args.angular_limit)
                elif key == "x":
                    linear_v = 0.0
                    angular_w = 0.0
                elif key in {"h", "?"}:
                    print()
                    print_help()

                status = f"cmd_vel  linear={linear_v:+.2f} m/s  angular={angular_w:+.2f} rad/s"
                if status != last_status:
                    print(f"\r{status:<80}", end="", flush=True)
                    last_status = status

            if sock in readable:
                frame = sock.recv(FRAME_STRUCT.size)
                can_id, payload = decode_can_frame(frame)
                if can_id == ODOM_TX_ID and len(payload) == 8:
                    odom_linear, odom_angular = PAYLOAD_STRUCT.unpack(payload)
                    print(
                        f"\rODOM  linear={odom_linear:+.3f} m/s  angular={odom_angular:+.3f} rad/s"
                        f"  |  cmd linear={linear_v:+.2f} angular={angular_w:+.2f}      "
                    )
                    last_status = ""

            now = time.monotonic()
            if now >= next_tx_time:
                payload = PAYLOAD_STRUCT.pack(linear_v, angular_w)
                sock.send(encode_can_frame(CMD_RX_ID, payload))
                next_tx_time = now + args.period

    finally:
        try:
            sock.send(encode_can_frame(CMD_RX_ID, PAYLOAD_STRUCT.pack(0.0, 0.0)))
        except OSError:
            pass
        restore_terminal(stdin_fd, old_term)
        sock.close()
        print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
