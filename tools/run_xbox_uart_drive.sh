#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PORT="${1:-}"
PROTOCOL="${PROTOCOL:-twist}"
MAX_LINEAR="${MAX_LINEAR:-0.60}"
MAX_ANGULAR="${MAX_ANGULAR:-2.67}"
TRACK_GAUGE="${TRACK_GAUGE:-0.45}"
MAX_TRACK_SPEED="${MAX_TRACK_SPEED:-0.60}"
TRACK_CONTACT_LENGTH="${TRACK_CONTACT_LENGTH:-0.26}"
TRACK_BELT_WIDTH="${TRACK_BELT_WIDTH:-0.075}"

if [ ! -d ".venv" ]; then
  python3 -m venv .venv
fi

source .venv/bin/activate
python3 -m pip install -r drive_test_requirements.txt

cmd=(
  python3 xbox_uart_can_telemetry.py
  --no-can \
  --debug \
  --protocol "$PROTOCOL" \
  --max-linear "$MAX_LINEAR" \
  --max-angular "$MAX_ANGULAR" \
  --track-gauge "$TRACK_GAUGE" \
  --max-track-speed "$MAX_TRACK_SPEED" \
  --track-contact-length "$TRACK_CONTACT_LENGTH" \
  --track-belt-width "$TRACK_BELT_WIDTH"
)

if [ -n "$PORT" ]; then
  cmd+=(--port "$PORT")
fi

exec "${cmd[@]}"
