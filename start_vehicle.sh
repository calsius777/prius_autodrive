#!/bin/bash

set -e

echo "========================================"
echo "   Prius 車載 ROS2 環境啟動中"
echo "========================================"

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_NAME="${IMAGE_NAME:-lab_prius_base:v4}"
CONTAINER_NAME="${CONTAINER_NAME:-prius_vehicle_dev}"
CAN_IF="${CAN_IF:-can0}"

mkdir -p \
  "$PROJECT_DIR/rosbags" \
  "$PROJECT_DIR/tools" \
  "$PROJECT_DIR/config"

echo "========================================"
echo "   Host hardware check"
echo "========================================"

echo "[CAN]"
if ip link show "$CAN_IF" >/dev/null 2>&1; then
  ip -details link show "$CAN_IF"
else
  echo "WARN: $CAN_IF not found. Try:"
  echo "  sudo systemctl start prius-can0.service"
fi

echo ""
echo "[Serial devices]"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "WARN: /dev/serial/by-id not found"

echo ""
echo "[Video devices]"
ls /dev/video* 2>/dev/null || echo "WARN: no /dev/video* found"

echo ""
echo "[Network]"
ip -brief addr || true

echo "========================================"
echo "   Starting Docker container"
echo "========================================"

xhost +local:root

docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

docker run -it --rm \
  --name "$CONTAINER_NAME" \
  --net=host \
  --privileged \
  --ipc=host \
  --ulimit rtprio=99 \
  --ulimit memlock=-1 \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e CAN_IF="$CAN_IF" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v /lib/modules:/lib/modules:ro \
  -v /run/udev:/run/udev:ro \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v "$PROJECT_DIR/rosbags":/colcon_ws/rosbags \
  -v "$PROJECT_DIR/tools":/colcon_ws/tools \
  -v "$PROJECT_DIR/config":/colcon_ws/config \
  -w /colcon_ws \
  "$IMAGE_NAME" \
  bash