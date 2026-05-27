from __future__ import annotations

import glob
import os
import fcntl
from datetime import datetime
from pathlib import Path

from xbox_drive_config import (
    CANABLE_PORT_HINTS,
    DEFAULT_MAX_ANGULAR_RADPS,
    DEFAULT_MAX_LINEAR_MPS,
    DEFAULT_MAX_TRACK_SPEED_MPS,
    DEFAULT_TRACK_GAUGE_M,
    FALLBACK_STM_UART_PORTS,
    FRAME_HEADER_0,
    FRAME_HEADER_1,
    PACKET_BACKWARD,
    PACKET_BACKWARD_CURVE_LEFT,
    PACKET_BACKWARD_CURVE_RIGHT,
    PACKET_CURVE_LEFT,
    PACKET_CURVE_RIGHT,
    PACKET_FORWARD,
    PACKET_STOP,
    PACKET_TURN_LEFT,
    PACKET_TURN_RIGHT,
    PACKET_TWIST,
    STM_UART_PORT_HINTS,
    STICK_DEADZONE,
)


def default_drive_log_path() -> str:
    return str(Path(__file__).resolve().parent / "logs" / "xbox_drive_latest.log")


def default_drive_lock_path() -> str:
    return str(Path(__file__).resolve().parent / "logs" / "xbox_uart_control.lock")


class DriveControlLock:
    def __init__(self, lock_path: str | None = None) -> None:
        self.path = Path(lock_path or default_drive_lock_path()).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("a+", encoding="ascii")
        try:
            fcntl.flock(self._stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self._stream.seek(0)
            active_pid = self._stream.read().strip() or "unknown"
            self._stream.close()
            raise RuntimeError(
                f"Another Xbox UART drive process is already active (pid={active_pid})."
            ) from exc

        self._stream.seek(0)
        self._stream.truncate()
        self._stream.write(f"{os.getpid()}\n")
        self._stream.flush()

    def close(self) -> None:
        if self._stream.closed:
            return
        fcntl.flock(self._stream.fileno(), fcntl.LOCK_UN)
        self._stream.close()


class DriveSessionLog:
    def __init__(self, log_path: str) -> None:
        self.path = Path(log_path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("w", encoding="utf-8", buffering=1)
        self._serial_pending = ""
        self.write("INFO", "session started")

    def write(self, source: str, message: str) -> None:
        message = message.strip("\r\n")
        if not message:
            return
        timestamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
        self._stream.write(f"{timestamp} {source} {message}\n")
        self._stream.flush()

    def feed_serial(self, data: str) -> None:
        self._serial_pending += data.replace("\r", "\n")
        lines = self._serial_pending.split("\n")
        self._serial_pending = lines.pop()
        for line in lines:
            self.write("STM", line)

    def close(self) -> None:
        self.write("STM", self._serial_pending)
        self._serial_pending = ""
        self.write("INFO", "session stopped")
        self._stream.close()


def apply_deadzone(value: float, deadzone: float = STICK_DEADZONE) -> float:
    if abs(value) < deadzone:
        return 0.0
    return value


def clamp_percent(value: int, minimum: int = 0) -> int:
    return max(minimum, min(100, value))


def clamp_float(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def clamp_i16(value: int) -> int:
    return max(-32768, min(32767, value))


def _serial_by_id_links() -> list[tuple[str, str]]:
    links: list[tuple[str, str]] = []
    for path in sorted(glob.glob("/dev/serial/by-id/*")):
        links.append((path, os.path.realpath(path)))
    return links


def serial_by_id_aliases(port_path: str) -> list[str]:
    target = os.path.realpath(port_path)
    return [path for path, real_path in _serial_by_id_links() if real_path == target]


def serial_port_description(port_path: str) -> str:
    aliases = serial_by_id_aliases(port_path)
    if aliases:
        return f"{port_path} -> {os.path.realpath(port_path)} ({', '.join(aliases)})"
    return f"{port_path} -> {os.path.realpath(port_path)}"


def serial_port_matches_hints(port_path: str, hints: tuple[str, ...]) -> bool:
    names = [os.path.basename(port_path)]
    names.extend(os.path.basename(alias) for alias in serial_by_id_aliases(port_path))
    return any(
        hint.lower() in name.lower()
        for hint in hints
        for name in names
    )


def is_probably_canable_port(port_path: str) -> bool:
    return serial_port_matches_hints(port_path, CANABLE_PORT_HINTS)


def resolve_stm_uart_port(preferred: str | None = None) -> str:
    if preferred:
        return preferred

    for path, _real_path in _serial_by_id_links():
        if (
            serial_port_matches_hints(path, STM_UART_PORT_HINTS)
            and not serial_port_matches_hints(path, CANABLE_PORT_HINTS)
        ):
            return path

    for path in FALLBACK_STM_UART_PORTS:
        if os.path.exists(path) and not is_probably_canable_port(path):
            return path

    return FALLBACK_STM_UART_PORTS[0]


def joystick_to_twist(
    throttle: float,
    steer: float,
    max_linear_mps: float = DEFAULT_MAX_LINEAR_MPS,
    max_angular_radps: float = DEFAULT_MAX_ANGULAR_RADPS,
) -> tuple[float, float]:
    linear_mps = clamp_float(throttle, -1.0, 1.0) * max_linear_mps
    angular_radps = -clamp_float(steer, -1.0, 1.0) * max_angular_radps
    return linear_mps, angular_radps


def twist_to_track_speeds(
    linear_mps: float,
    angular_radps: float,
    track_gauge_m: float = DEFAULT_TRACK_GAUGE_M,
) -> tuple[float, float]:
    half_gauge = max(0.0, track_gauge_m) / 2.0
    left_mps = linear_mps - angular_radps * half_gauge
    right_mps = linear_mps + angular_radps * half_gauge
    return left_mps, right_mps


def limit_twist_to_track_speed(
    linear_mps: float,
    angular_radps: float,
    track_gauge_m: float = DEFAULT_TRACK_GAUGE_M,
    max_track_speed_mps: float = DEFAULT_MAX_TRACK_SPEED_MPS,
) -> tuple[float, float, float, float]:
    left_mps, right_mps = twist_to_track_speeds(
        linear_mps,
        angular_radps,
        track_gauge_m,
    )
    max_track_speed_mps = max(0.01, max_track_speed_mps)
    peak_track_speed_mps = max(abs(left_mps), abs(right_mps))

    if peak_track_speed_mps > max_track_speed_mps:
        scale = max_track_speed_mps / peak_track_speed_mps
        linear_mps *= scale
        angular_radps *= scale
        left_mps *= scale
        right_mps *= scale

    return linear_mps, angular_radps, left_mps, right_mps


def track_speeds_to_uart_packet(
    left_mps: float,
    right_mps: float,
    max_track_speed_mps: float = DEFAULT_MAX_TRACK_SPEED_MPS,
) -> tuple[int, int, int, float, float]:
    max_track_speed_mps = max(0.01, max_track_speed_mps)
    left = clamp_float(left_mps / max_track_speed_mps, -1.0, 1.0)
    right = clamp_float(right_mps / max_track_speed_mps, -1.0, 1.0)
    left_abs = abs(left)
    right_abs = abs(right)
    max_abs = max(left_abs, right_abs)

    if max_abs <= 0.0:
        return PACKET_STOP, 0, 50, 0.0, 0.0

    duty = clamp_percent(int(round(max_abs * 100.0)), minimum=5)
    same_direction = (left >= 0.0 and right >= 0.0) or (left <= 0.0 and right <= 0.0)

    if not same_direction:
        linear_estimate = (left + right) / 2.0
        if abs(linear_estimate) < 0.01:
            if left > right:
                return PACKET_TURN_RIGHT, duty, 100, left, right
            return PACKET_TURN_LEFT, duty, 100, left, right

        ratio = 0
        if linear_estimate > 0.0:
            if left > right:
                return PACKET_CURVE_RIGHT, duty, ratio, left, 0.0
            return PACKET_CURVE_LEFT, duty, ratio, 0.0, right
        if left_abs > right_abs:
            return PACKET_BACKWARD_CURVE_LEFT, duty, ratio, left, 0.0
        return PACKET_BACKWARD_CURVE_RIGHT, duty, ratio, 0.0, right

    if abs(left - right) < 0.01:
        if left >= 0.0:
            return PACKET_FORWARD, duty, 100, left, right
        return PACKET_BACKWARD, duty, 100, left, right

    ratio = clamp_percent(int(round((min(left_abs, right_abs) / max_abs) * 100.0)))
    if left >= 0.0 and right >= 0.0:
        if left > right:
            return PACKET_CURVE_RIGHT, duty, ratio, left, right
        return PACKET_CURVE_LEFT, duty, ratio, left, right

    if left_abs > right_abs:
        return PACKET_BACKWARD_CURVE_LEFT, duty, ratio, left, right
    return PACKET_BACKWARD_CURVE_RIGHT, duty, ratio, left, right


def encode_legacy_frame(motion_code: int, duty: int, curve_ratio: int) -> bytes:
    return bytes(
        [
            FRAME_HEADER_0,
            FRAME_HEADER_1,
            motion_code & 0xFF,
            duty & 0xFF,
            curve_ratio & 0xFF,
            FRAME_HEADER_0 ^ FRAME_HEADER_1 ^ (motion_code & 0xFF) ^ (duty & 0xFF) ^ (curve_ratio & 0xFF),
        ]
    )


def encode_twist_frame(linear_mps: float, angular_radps: float) -> bytes:
    linear_mmps = clamp_i16(int(round(linear_mps * 1000.0)))
    angular_mradps = clamp_i16(int(round(angular_radps * 1000.0)))
    payload = bytearray(
        [
            FRAME_HEADER_0,
            FRAME_HEADER_1,
            PACKET_TWIST,
            linear_mmps & 0xFF,
            (linear_mmps >> 8) & 0xFF,
            angular_mradps & 0xFF,
            (angular_mradps >> 8) & 0xFF,
        ]
    )
    checksum = 0
    for value in payload:
        checksum ^= value
    payload.append(checksum)
    return bytes(payload)


def encode_stop_frame() -> bytes:
    return encode_twist_frame(0.0, 0.0)


def encode_twist_drive(
    linear_mps: float,
    angular_radps: float,
    track_gauge_m: float = DEFAULT_TRACK_GAUGE_M,
    max_track_speed_mps: float = DEFAULT_MAX_TRACK_SPEED_MPS,
) -> tuple[int, int, int, float, float]:
    _, _, left_mps, right_mps = limit_twist_to_track_speed(
        linear_mps,
        angular_radps,
        track_gauge_m,
        max_track_speed_mps,
    )
    return track_speeds_to_uart_packet(left_mps, right_mps, max_track_speed_mps)


def encode_mixed_drive(
    throttle: float,
    steer: float,
    max_linear_mps: float = DEFAULT_MAX_LINEAR_MPS,
    max_angular_radps: float = DEFAULT_MAX_ANGULAR_RADPS,
    track_gauge_m: float = DEFAULT_TRACK_GAUGE_M,
    max_track_speed_mps: float = DEFAULT_MAX_TRACK_SPEED_MPS,
) -> tuple[int, int, int, float, float]:
    linear_mps, angular_radps = joystick_to_twist(
        throttle,
        steer,
        max_linear_mps,
        max_angular_radps,
    )
    return encode_twist_drive(linear_mps, angular_radps, track_gauge_m, max_track_speed_mps)
