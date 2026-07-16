#!/usr/bin/env bash
set -euo pipefail

PORT=${1:-/dev/ttyUSB0}
BAUD=${2:-115200}
DURATION=${3:-10}

source /opt/ros/humble/setup.bash || true
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

echo "Available serial devices:"
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/mti630 /dev/serial/by-id/* 2>/dev/null || true

echo
echo "Sniffing Xbus MTData2 from $PORT @ $BAUD for ${DURATION}s"
ros2 run prius_mti630_driver xbus_sniffer --port "$PORT" --baudrate "$BAUD" --duration "$DURATION"
