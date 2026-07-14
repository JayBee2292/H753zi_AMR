#!/usr/bin/env python3
"""Xbox UART drive control with CAN FD telemetry monitoring.

Firmware runtime:
  - UART: Xbox drive packets to STM32
  - CAN FD+BRS 0x100: STM32 telemetry to host
  - CAN FD+BRS 0x1FF: host telemetry control command
"""

from __future__ import annotations

import argparse
import os
import re
import signal
import socket
import struct
import sys
import time
from dataclasses import dataclass

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
        "pygame is required. Install it with `python3 -m pip install pygame`."
    ) from exc

from xbox_drive_config import (  # noqa: E402
    DEFAULT_MOVING_INNER_RATIO,
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
from xbox_drive_core import (  # noqa: E402
    DriveControlLock,
    DriveSessionLog,
    apply_deadzone,
    default_drive_log_path,
    encode_legacy_frame,
    encode_stop_frame,
    encode_twist_frame,
    is_probably_canable_port,
    joystick_to_drive_targets,
    resolve_stm_uart_port,
    serial_port_description,
    track_speeds_to_uart_packet,
)


CAN_TELEMETRY_TX_ID = 0x100
CAN_TELEMETRY_ENCODER_TX_ID = 0x101
CAN_TELEMETRY_RX_ID = 0x1FF
CANFD_BRS = 0x01
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_FD_FRAMES = getattr(socket, "CAN_RAW_FD_FRAMES", 5)
CAN_EFF_MASK = getattr(socket, "CAN_EFF_MASK", 0x1FFFFFFF)
CANFD_FRAME = struct.Struct("=IBB2x64s")
STATE_PAYLOAD = struct.Struct("<IIffffffhhhhBB")
ENCODER_PAYLOAD = struct.Struct("<IIqqqqIIIIBB")
LEGACY_STATE_PAYLOAD = struct.Struct("<IIffffffiiiihhhhBB")
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
FD_LENGTH_TO_DLC = {value: key for key, value in FD_DLC_TO_LENGTH.items()}
ENCODER_CONSOLE_PATTERN = re.compile(
    r"enc\(FL,FR,RL,RR\)=(?P<ticks>[^\s]+)\s+delta=(?P<delta>[^\s]+)"
)


def install_shutdown_handlers() -> None:
    def shutdown_handler(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGHUP, shutdown_handler)


def consume_encoder_console_lines(pending: str, data: str) -> tuple[str, list[str]]:
    pending += data.replace("\r", "\n")
    lines = pending.split("\n")
    pending = lines.pop()
    summaries: list[str] = []

    for line in lines:
        match = ENCODER_CONSOLE_PATTERN.search(line)
        if match is None:
            continue
        summaries.append(
            "enc(FL,FR,RL,RR)="
            f"{match.group('ticks')} delta={match.group('delta')}"
        )

    return pending, summaries


@dataclass
class Telemetry:
    seq: int
    timestamp_ms: int
    robot_linear_v: float
    robot_angular_w: float
    wheel_velocity_mps: tuple[float, float, float, float]
    left_duty_percent: int
    right_duty_percent: int
    duty_percent: int
    curve_ratio_percent: int
    motion: int
    flags: int


@dataclass
class EncoderTelemetry:
    seq: int
    timestamp_ms: int
    encoder_tick: tuple[int, int, int, int]
    encoder_delta_abs: tuple[int, int, int, int]
    valid_mask: int = 0x0F
    fault_mask: int = 0x00


class SocketCanTelemetry:
    def __init__(self, ifname: str) -> None:
        self.sock = open_canfd_socket(ifname)
        self.legacy_encoder_tick: tuple[int, int, int, int] | None = None

    def enable(self, enabled: bool) -> None:
        send_socketcan_telemetry_enable(self.sock, enabled)

    def recv_latest(self) -> tuple[Telemetry | None, EncoderTelemetry | None]:
        state, encoder, legacy_tick = recv_socketcan_telemetry(self.sock, self.legacy_encoder_tick)
        if legacy_tick is not None:
            self.legacy_encoder_tick = legacy_tick
        return state, encoder

    def close(self) -> None:
        self.sock.close()


class CanableSerialTelemetry:
    def __init__(self, port_path: str, baud: int, nominal: str, data: str) -> None:
        try:
            self.port = serial.Serial(port_path, baud, timeout=0)
        except SerialException as exc:
            raise SystemExit(f"Failed to open CANable port {port_path}: {exc}") from exc

        self.buffer = bytearray()
        self.legacy_encoder_tick: tuple[int, int, int, int] | None = None
        self.configure(nominal=nominal, data=data)

    def configure(self, nominal: str, data: str) -> None:
        self.send_command("C")
        self.drain(0.05)
        self.send_command(nominal)
        self.send_command(data)
        self.send_command("M0")
        self.send_command("A0")
        self.send_command("O")
        self.drain(0.1)

    def send_command(self, command: str) -> None:
        self.port.write(command.encode("ascii") + b"\r")
        self.port.flush()

    def drain(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            waiting = self.port.in_waiting
            if waiting:
                self.port.read(waiting)
            time.sleep(0.01)

    def send_frame(self, can_id: int, payload: bytes, brs: bool = True) -> None:
        if len(payload) not in FD_LENGTH_TO_DLC:
            raise ValueError(f"Unsupported CAN FD payload length: {len(payload)}")

        frame_type = "b" if brs else "d"
        dlc = FD_LENGTH_TO_DLC[len(payload)]
        line = f"{frame_type}{can_id:03X}{dlc}{payload.hex().upper()}"
        self.send_command(line)

    def enable(self, enabled: bool) -> None:
        payload = bytes([1, 1 if enabled else 0]).ljust(8, b"\x00")
        self.send_frame(CAN_TELEMETRY_RX_ID, payload)

    def recv_latest(self) -> tuple[Telemetry | None, EncoderTelemetry | None]:
        latest_state: Telemetry | None = None
        latest_encoder: EncoderTelemetry | None = None
        waiting = self.port.in_waiting

        if waiting:
            self.buffer.extend(self.port.read(waiting))

        while b"\r" in self.buffer:
            raw_line, _, remainder = self.buffer.partition(b"\r")
            self.buffer = bytearray(remainder)
            line = raw_line.decode("ascii", errors="replace").strip()
            decoded_state, decoded_encoder, legacy_tick = decode_canable_telemetry_line(
                line,
                self.legacy_encoder_tick,
            )
            if decoded_state is not None:
                latest_state = decoded_state
            if decoded_encoder is not None:
                latest_encoder = decoded_encoder
            if legacy_tick is not None:
                self.legacy_encoder_tick = legacy_tick

        return latest_state, latest_encoder

    def close(self) -> None:
        try:
            self.send_command("C")
        finally:
            self.port.close()


def open_canfd_socket(ifname: str) -> socket.socket:
    if not hasattr(socket, "AF_CAN"):
        raise SystemExit("This platform does not support SocketCAN.")

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
    sock.bind((ifname,))
    sock.setblocking(False)
    return sock


def send_canfd(sock: socket.socket, can_id: int, payload: bytes, flags: int = CANFD_BRS) -> None:
    if len(payload) > 64:
        raise ValueError("CAN FD payload must be 64 bytes or less")

    data = payload.ljust(64, b"\x00")
    sock.send(CANFD_FRAME.pack(can_id, len(payload), flags, data))


def send_socketcan_telemetry_enable(sock: socket.socket, enabled: bool) -> None:
    payload = bytes([1, 1 if enabled else 0]).ljust(8, b"\x00")
    send_canfd(sock, CAN_TELEMETRY_RX_ID, payload)


def telemetry_state_is_plausible(telemetry: Telemetry) -> bool:
    return (
        -100 <= telemetry.left_duty_percent <= 100
        and -100 <= telemetry.right_duty_percent <= 100
        and 0 <= telemetry.duty_percent <= 100
        and 0 <= telemetry.curve_ratio_percent <= 100
        and 0 <= telemetry.motion <= 8
        and (telemetry.flags & ~0x07) == 0
    )


def decode_state_telemetry(payload: bytes) -> Telemetry | None:
    if len(payload) < STATE_PAYLOAD.size:
        return None

    values = STATE_PAYLOAD.unpack_from(payload)
    return Telemetry(
        seq=values[0],
        timestamp_ms=values[1],
        robot_linear_v=values[2],
        robot_angular_w=values[3],
        wheel_velocity_mps=(values[4], values[5], values[6], values[7]),
        left_duty_percent=values[8],
        right_duty_percent=values[9],
        duty_percent=values[10],
        curve_ratio_percent=values[11],
        motion=values[12],
        flags=values[13],
    )


def decode_legacy_state_telemetry(
    payload: bytes,
    previous_tick: tuple[int, int, int, int] | None,
) -> tuple[Telemetry | None, EncoderTelemetry | None, tuple[int, int, int, int] | None]:
    if len(payload) < LEGACY_STATE_PAYLOAD.size:
        return None, None, previous_tick

    values = LEGACY_STATE_PAYLOAD.unpack_from(payload)
    encoder_tick = (values[8], values[9], values[10], values[11])

    if previous_tick is None:
        encoder_delta_abs = (0, 0, 0, 0)
    else:
        encoder_delta_abs = tuple(
            abs(current - previous)
            for current, previous in zip(encoder_tick, previous_tick)
        )

    telemetry = Telemetry(
        seq=values[0],
        timestamp_ms=values[1],
        robot_linear_v=values[2],
        robot_angular_w=values[3],
        wheel_velocity_mps=(values[4], values[5], values[6], values[7]),
        left_duty_percent=values[12],
        right_duty_percent=values[13],
        duty_percent=values[14],
        curve_ratio_percent=values[15],
        motion=values[16],
        flags=values[17],
    )

    encoder = EncoderTelemetry(
        seq=values[0],
        timestamp_ms=values[1],
        encoder_tick=encoder_tick,
        encoder_delta_abs=encoder_delta_abs,
    )

    return telemetry, encoder, encoder_tick


def decode_state_or_legacy_telemetry(
    payload: bytes,
    previous_tick: tuple[int, int, int, int] | None,
) -> tuple[Telemetry | None, EncoderTelemetry | None, tuple[int, int, int, int] | None]:
    state = decode_state_telemetry(payload)
    if state is not None and telemetry_state_is_plausible(state):
        return state, None, previous_tick

    return decode_legacy_state_telemetry(payload, previous_tick)


def decode_encoder_telemetry(payload: bytes) -> EncoderTelemetry | None:
    if len(payload) < ENCODER_PAYLOAD.size:
        return None

    values = ENCODER_PAYLOAD.unpack_from(payload)
    return EncoderTelemetry(
        seq=values[0],
        timestamp_ms=values[1],
        encoder_tick=(values[2], values[3], values[4], values[5]),
        encoder_delta_abs=(values[6], values[7], values[8], values[9]),
        valid_mask=values[10],
        fault_mask=values[11],
    )


def recv_socketcan_telemetry(
    sock: socket.socket,
    legacy_tick: tuple[int, int, int, int] | None,
) -> tuple[Telemetry | None, EncoderTelemetry | None, tuple[int, int, int, int] | None]:
    latest_state: Telemetry | None = None
    latest_encoder: EncoderTelemetry | None = None

    while True:
        try:
            frame = sock.recv(CANFD_FRAME.size)
        except BlockingIOError:
            return latest_state, latest_encoder, legacy_tick

        if len(frame) != CANFD_FRAME.size:
            continue

        can_id, length, _flags, data = CANFD_FRAME.unpack(frame)
        can_id &= CAN_EFF_MASK
        if can_id == CAN_TELEMETRY_TX_ID:
            decoded_state, decoded_encoder, legacy_tick = decode_state_or_legacy_telemetry(
                data[:length],
                legacy_tick,
            )
            if decoded_state is not None:
                latest_state = decoded_state
            if decoded_encoder is not None:
                latest_encoder = decoded_encoder
        elif can_id == CAN_TELEMETRY_ENCODER_TX_ID:
            decoded_encoder = decode_encoder_telemetry(data[:length])
            if decoded_encoder is not None:
                latest_encoder = decoded_encoder


def decode_canable_telemetry_line(
    line: str,
    legacy_tick: tuple[int, int, int, int] | None,
) -> tuple[Telemetry | None, EncoderTelemetry | None, tuple[int, int, int, int] | None]:
    if not line:
        return None, None, legacy_tick

    frame_type = line[0]
    if frame_type in {"d", "b"} and len(line) >= 5:
        can_id = int(line[1:4], 16)
        dlc_char = line[4].upper()
        data_start = 5
    elif frame_type in {"D", "B"} and len(line) >= 10:
        can_id = int(line[1:9], 16)
        dlc_char = line[9].upper()
        data_start = 10
    else:
        return None, None, legacy_tick

    length = FD_DLC_TO_LENGTH.get(dlc_char)
    if length is None:
        return None, None, legacy_tick

    data_hex = line[data_start : data_start + length * 2]
    try:
        payload = bytes.fromhex(data_hex)
    except ValueError:
        return None, None, legacy_tick

    if can_id == CAN_TELEMETRY_TX_ID:
        return decode_state_or_legacy_telemetry(payload, legacy_tick)

    if can_id == CAN_TELEMETRY_ENCODER_TX_ID:
        return None, decode_encoder_telemetry(payload), legacy_tick

    return None, None, legacy_tick


def select_joystick(joy_idx: int) -> pygame.joystick.Joystick:
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
            name = joy.get_name().lower()
            if "xbox" in name or "x-box" in name or "microsoft" in name:
                target_idx = i
                break

    joystick = pygame.joystick.Joystick(target_idx)
    if not joystick.get_init():
        joystick.init()

    print(f"Using controller: {joystick.get_name()}")
    return joystick


def format_telemetry(telemetry: Telemetry, encoder: EncoderTelemetry | None) -> str:
    motion_name = MOTION_NAMES.get(telemetry.motion, f"unknown_{telemetry.motion}")
    fl, fr, rl, rr = telemetry.wheel_velocity_mps
    ready = "ready" if telemetry.flags & 0x01 else "not-ready"
    active = "active" if telemetry.flags & 0x02 else "stop"
    encoder_state = "enc-fault" if telemetry.flags & 0x04 else "enc-ok"
    encoder_text = "enc=(pending) delta=(pending)"

    if encoder is not None:
        efl, efr, erl, err = encoder.encoder_tick
        dfl, dfr, drl, drr = encoder.encoder_delta_abs
        encoder_text = (
            f"enc=({efl},{efr},{erl},{err}) "
            f"delta=({dfl},{dfr},{drl},{drr}) "
            f"valid=0x{encoder.valid_mask:X} fault=0x{encoder.fault_mask:X}"
        )

    return (
        f"CAN seq={telemetry.seq:06d} t={telemetry.timestamp_ms:010d}ms "
        f"V={telemetry.robot_linear_v:+.3f}m/s W={telemetry.robot_angular_w:+.3f}rad/s "
        f"wheel=({fl:+.3f},{fr:+.3f},{rl:+.3f},{rr:+.3f}) "
        f"{encoder_text} "
        f"duty L/R={telemetry.left_duty_percent:+d}/{telemetry.right_duty_percent:+d} "
        f"cmd={motion_name} duty={telemetry.duty_percent} ratio={telemetry.curve_ratio_percent} "
        f"{ready}/{active}/{encoder_state}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Xbox UART remote drive plus CAN FD telemetry monitor."
    )
    parser.add_argument(
        "--port",
        default=None,
        help="STM32 UART serial port. If omitted, an STLINK /dev/serial/by-id port is auto-selected.",
    )
    parser.add_argument("--baud", type=int, default=921600, help="UART baudrate")
    parser.add_argument(
        "--protocol",
        choices=("twist", "legacy"),
        default="twist",
        help="UART packet protocol. Use legacy only to test old STM firmware.",
    )
    parser.add_argument("--can-if", default="can0", help="SocketCAN FD interface")
    parser.add_argument(
        "--can-port",
        default=None,
        help="CANable serial port, e.g. /dev/ttyACM1. If set, SocketCAN is not used.",
    )
    parser.add_argument("--can-baud", type=int, default=921600, help="CANable serial baudrate")
    parser.add_argument("--can-nominal", default="S8", help="CANable nominal bitrate command, S8=1M")
    parser.add_argument("--can-data", default="Y5", help="CANable data bitrate command, Y5=5M")
    parser.add_argument("--no-can", action="store_true", help="Run UART control without CAN telemetry")
    parser.add_argument("--joy-idx", type=int, default=-1, help="Force joystick index")
    parser.add_argument("--debug", action="store_true", help="Print local controller mapping")
    parser.add_argument(
        "--encoder-only",
        action="store_true",
        help="Print only STM encoder cumulative ticks and deltas; keep the full session log.",
    )
    parser.add_argument(
        "--log-file",
        default=default_drive_log_path(),
        help="Session log output path. The file is overwritten for each run.",
    )
    parser.add_argument("--print-period", type=float, default=0.10, help="Telemetry print period in seconds")
    parser.add_argument("--max-linear", type=float, default=DEFAULT_MAX_LINEAR_MPS, help="Maximum linear speed command in m/s")
    parser.add_argument("--max-angular", type=float, default=DEFAULT_MAX_ANGULAR_RADPS, help="Maximum rotate-in-place angular speed command in rad/s")
    parser.add_argument("--track-gauge", type=float, default=DEFAULT_TRACK_GAUGE_M, help="Effective distance between left/right track centers in meters")
    parser.add_argument("--max-track-speed", type=float, default=DEFAULT_MAX_TRACK_SPEED_MPS, help="Track speed that maps to 100 percent duty")
    parser.add_argument("--moving-inner-ratio", type=float, default=DEFAULT_MOVING_INNER_RATIO, help="Minimum inside/outside track speed ratio while translating and steering")
    parser.add_argument("--track-contact-length", type=float, default=DEFAULT_TRACK_CONTACT_LENGTH_M, help="Track ground contact length in meters")
    parser.add_argument("--track-belt-width", type=float, default=DEFAULT_TRACK_BELT_WIDTH_M, help="Track belt width in meters")
    return parser.parse_args()


def main() -> int:
    install_shutdown_handlers()
    args = parse_args()
    try:
        control_lock = DriveControlLock()
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc

    joystick = select_joystick(args.joy_idx)
    install_shutdown_handlers()
    control_port = resolve_stm_uart_port(args.port)

    if is_probably_canable_port(control_port):
        raise SystemExit(
            f"Refusing to use CANable as STM UART: {serial_port_description(control_port)}\n"
            "Use the STLINK VCP port, usually /dev/ttyACM1 or the STMicroelectronics /dev/serial/by-id path."
        )

    try:
        serial_port = serial.Serial(port=control_port, baudrate=args.baud, timeout=0)
    except SerialException as exc:
        raise SystemExit(f"Failed to open serial port {control_port}: {exc}") from exc

    can_reader: SocketCanTelemetry | CanableSerialTelemetry | None = None
    if not args.no_can:
        if args.can_port is not None:
            can_reader = CanableSerialTelemetry(
                port_path=args.can_port,
                baud=args.can_baud,
                nominal=args.can_nominal,
                data=args.can_data,
            )
            can_reader.enable(True)
            print(
                f"Connected CANable telemetry on {args.can_port} @ {args.can_baud} "
                f"({args.can_nominal}, {args.can_data})"
            )
        else:
            can_reader = SocketCanTelemetry(args.can_if)
            can_reader.enable(True)
            print(f"Connected SocketCAN FD telemetry on {args.can_if}")

    session_log = DriveSessionLog(args.log_file)
    print(f"Connected UART control on {serial_port_description(control_port)} @ {args.baud}")
    print(f"Session log: {session_log.path}")
    print(f"UART protocol: {args.protocol}")
    print("Controls: left stick Y = forward/back, right stick X = rotate/steer")
    print("          left stick X is ignored to keep forward/back driving stable")
    print(
        f"Track model: gauge={args.track_gauge:.3f}m, "
        f"contact={args.track_contact_length:.3f}m, "
        f"belt_width={args.track_belt_width:.3f}m"
    )
    print(
        f"Speed limits: linear={args.max_linear:.2f}m/s, "
        f"spin_angular={args.max_angular:.2f}rad/s, "
        f"track={args.max_track_speed:.2f}m/s, "
        f"moving_inner_ratio={args.moving_inner_ratio:.2f}"
    )
    session_log.write(
        "INFO",
        f"uart={serial_port_description(control_port)} baud={args.baud} protocol={args.protocol} "
        f"limits linear={args.max_linear:.2f} spin_angular={args.max_angular:.2f} "
        f"track={args.max_track_speed:.2f} moving_inner_ratio={args.moving_inner_ratio:.2f}",
    )
    print("Press Ctrl+C to stop.")

    last_tx_time = time.monotonic()
    last_serial_poll_time = time.monotonic()
    last_print_time = 0.0
    last_tx_frame = b""
    latest_telemetry: Telemetry | None = None
    latest_encoder: EncoderTelemetry | None = None
    encoder_console_pending = ""

    try:
        while True:
            now = time.monotonic()
            pygame.event.pump()

            lx = apply_deadzone(joystick.get_axis(0))
            ly = apply_deadzone(joystick.get_axis(1))
            rx = apply_deadzone(
                joystick.get_axis(3) if joystick.get_numaxes() >= 4 else joystick.get_axis(2)
            )

            throttle = -ly
            steer = rx
            linear_mps, angular_radps, left_mps, right_mps = joystick_to_drive_targets(
                throttle,
                steer,
                args.max_linear,
                args.max_angular,
                args.track_gauge,
                args.max_track_speed,
                args.moving_inner_ratio,
            )
            motion_code, duty, curve_ratio, left_mix, right_mix = track_speeds_to_uart_packet(
                left_mps,
                right_mps,
                args.max_track_speed,
            )

            if now - last_tx_time >= TRANSMIT_INTERVAL_MS / 1000.0:
                if args.protocol == "legacy":
                    last_tx_frame = encode_legacy_frame(motion_code, duty, curve_ratio)
                else:
                    last_tx_frame = encode_twist_frame(linear_mps, angular_radps)
                serial_port.write(last_tx_frame)
                serial_port.flush()
                last_tx_time = now
                motion_name = MOTION_NAMES.get(motion_code, f"unknown_{motion_code}")
                session_log.write(
                    "TX",
                    f"lx={lx:+.2f} ly={ly:+.2f} rx={rx:+.2f} "
                    f"v/w={linear_mps:+.3f}/{angular_radps:+.3f} "
                    f"track L/R={left_mps:+.3f}/{right_mps:+.3f} "
                    f"cmd={motion_name} duty={duty:03d} ratio={curve_ratio:03d} "
                    f"frame={last_tx_frame.hex(' ')}",
                )

            if args.debug:
                motion_name = MOTION_NAMES.get(motion_code, f"unknown_{motion_code}")
                sys.stdout.write(
                    f"\rLOCAL lx={lx:+.2f} ly={ly:+.2f} rx={rx:+.2f} "
                    f"throttle={throttle:+.2f} steer={steer:+.2f} "
                    f"v/w={linear_mps:+.2f}/{angular_radps:+.2f} "
                    f"track L/R={left_mps:+.2f}/{right_mps:+.2f} "
                    f"norm L/R={left_mix:+.2f}/{right_mix:+.2f} "
                    f"cmd={motion_name} duty={duty:03d} ratio={curve_ratio:03d} "
                    f"tx={last_tx_frame.hex(' ')}    "
                )
                sys.stdout.flush()

            if now - last_serial_poll_time >= SERIAL_POLL_INTERVAL_MS / 1000.0:
                available = serial_port.in_waiting
                if available:
                    data = serial_port.read(available).decode("utf-8", errors="replace")
                    session_log.feed_serial(data)
                    if args.encoder_only:
                        encoder_console_pending, summaries = consume_encoder_console_lines(
                            encoder_console_pending,
                            data,
                        )
                        for summary in summaries:
                            print(summary)
                    else:
                        sys.stdout.write(data)
                        sys.stdout.flush()
                last_serial_poll_time = now

            if can_reader is not None:
                telemetry, encoder = can_reader.recv_latest()
                if telemetry is not None:
                    latest_telemetry = telemetry
                if encoder is not None:
                    latest_encoder = encoder

                if latest_telemetry is not None and now - last_print_time >= args.print_period:
                    telemetry_text = format_telemetry(latest_telemetry, latest_encoder)
                    session_log.write("CAN", telemetry_text)
                    print("\r" + telemetry_text + " " * 8)
                    last_print_time = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            if args.protocol == "legacy":
                serial_port.write(encode_legacy_frame(0, 0, 50))
            else:
                serial_port.write(encode_stop_frame())
            serial_port.flush()
        except SerialException:
            pass

        if can_reader is not None:
            can_reader.close()

        serial_port.close()
        session_log.close()
        control_lock.close()
        # SDL joystick shutdown can block after an Xbox device disconnects.
        # Process exit releases pygame resources after the stop frame is sent.

    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    # SDL/Pygame finalizers can block after Xbox input teardown.
    # UART stop/close and log flush have already completed inside main().
    os._exit(exit_code)
