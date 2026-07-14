from __future__ import annotations

from dataclasses import dataclass


TRANSMIT_INTERVAL_MS = 60
SERIAL_POLL_INTERVAL_MS = 50
STICK_DEADZONE = 0.15

DEFAULT_MAX_LINEAR_MPS = 0.60
DEFAULT_MAX_ANGULAR_RADPS = 2.67
DEFAULT_TRACK_GAUGE_M = 0.45
DEFAULT_MAX_TRACK_SPEED_MPS = 0.60
DEFAULT_MOVING_INNER_RATIO = 0.40
DEFAULT_TRACK_CONTACT_LENGTH_M = 0.26
DEFAULT_TRACK_BELT_WIDTH_M = 0.075

STM_UART_PORT_HINTS = ("STMicroelectronics", "STLINK")
CANABLE_PORT_HINTS = ("CANable", "CANable2", "Openlight", "normaldotcom")
FALLBACK_STM_UART_PORTS = ("/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyACM0")

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
PACKET_TWIST = 0x10

MOTION_NAMES = {
    PACKET_STOP: "stop",
    PACKET_FORWARD: "forward",
    PACKET_BACKWARD: "backward",
    PACKET_TURN_LEFT: "turn_left",
    PACKET_TURN_RIGHT: "turn_right",
    PACKET_CURVE_LEFT: "curve_left",
    PACKET_CURVE_RIGHT: "curve_right",
    PACKET_BACKWARD_CURVE_LEFT: "backward_curve_left",
    PACKET_BACKWARD_CURVE_RIGHT: "backward_curve_right",
    PACKET_TWIST: "twist",
}


@dataclass(frozen=True)
class DriveConfig:
    max_linear_mps: float = DEFAULT_MAX_LINEAR_MPS
    max_angular_radps: float = DEFAULT_MAX_ANGULAR_RADPS
    track_gauge_m: float = DEFAULT_TRACK_GAUGE_M
    max_track_speed_mps: float = DEFAULT_MAX_TRACK_SPEED_MPS
    moving_inner_ratio: float = DEFAULT_MOVING_INNER_RATIO
    track_contact_length_m: float = DEFAULT_TRACK_CONTACT_LENGTH_M
    track_belt_width_m: float = DEFAULT_TRACK_BELT_WIDTH_M
