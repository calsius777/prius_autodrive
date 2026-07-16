#!/usr/bin/env bash

set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROGRAM="$BASE_DIR/build/camera_calibration_capture"

if [ ! -x "$PROGRAM" ]; then
    echo "[ERROR] Program not found: $PROGRAM"
    echo
    echo "Build first:"
    echo "  cd $BASE_DIR"
    echo "  mkdir -p build && cd build"
    echo "  cmake .."
    echo "  make -j\$(nproc)"
    exit 1
fi

cd "$BASE_DIR"

exec "$PROGRAM" \
    --output "$BASE_DIR/calibration_dataset"
