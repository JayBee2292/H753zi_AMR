#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PORT="${UART_PORT:-}"
EXTRA_ARGS=()
PROTOCOL="${PROTOCOL:-twist}"
MAX_LINEAR="${MAX_LINEAR:-0.60}"
MAX_ANGULAR="${MAX_ANGULAR:-2.67}"
TRACK_GAUGE="${TRACK_GAUGE:-0.45}"
MAX_TRACK_SPEED="${MAX_TRACK_SPEED:-0.60}"
MOVING_INNER_RATIO="${MOVING_INNER_RATIO:-0.40}"
TRACK_CONTACT_LENGTH="${TRACK_CONTACT_LENGTH:-0.26}"
TRACK_BELT_WIDTH="${TRACK_BELT_WIDTH:-0.075}"

if [ "$#" -gt 0 ] && [[ "$1" != -* ]]; then
  PORT="$1"
  shift
fi

while [ "$#" -gt 0 ]; do
  case "$1" in
    --port)
      if [ "$#" -lt 2 ]; then
        echo "error: --port requires a serial device path" >&2
        exit 2
      fi
      PORT="$2"
      shift 2
      ;;
    --port=*)
      PORT="${1#--port=}"
      shift
      ;;
    --no-can)
      # This wrapper already runs the UART drive path without CAN so ROS can own CANable.
      shift
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if [ ! -d ".venv" ]; then
  python3 -m venv .venv
fi

source .venv/bin/activate
python3 -m pip install -r drive_test_requirements.txt

cmd=(
  python3 xbox_uart_can_telemetry.py
  --no-can \
  --encoder-only \
  --protocol "$PROTOCOL" \
  --max-linear "$MAX_LINEAR" \
  --max-angular "$MAX_ANGULAR" \
  --track-gauge "$TRACK_GAUGE" \
  --max-track-speed "$MAX_TRACK_SPEED" \
  --moving-inner-ratio "$MOVING_INNER_RATIO" \
  --track-contact-length "$TRACK_CONTACT_LENGTH" \
  --track-belt-width "$TRACK_BELT_WIDTH"
)

if [ -n "$PORT" ]; then
  cmd+=(--port "$PORT")
fi

cmd+=("${EXTRA_ARGS[@]}")

exec "${cmd[@]}"
