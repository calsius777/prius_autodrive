#!/bin/bash

TEST_PROGRAM="./build/v4l2_camera_test"

OUTPUT_DIR="./camera_test_results"

CAMERAS=(
    "/dev/video0"
    "/dev/video2"
    "/dev/video4"
    "/dev/video6"
    "/dev/video8"
    "/dev/video10"
)


mkdir -p "$OUTPUT_DIR"


PASS_COUNT=0
FAIL_COUNT=0


echo
echo "============================================"
echo " Prius 6-Camera V4L2 Test"
echo "============================================"
echo


for i in "${!CAMERAS[@]}"; do

    DEVICE="${CAMERAS[$i]}"

    OUTPUT_FILE="$OUTPUT_DIR/camera_${i}.jpg"

    LOG_FILE="$OUTPUT_DIR/camera_${i}.log"


    echo
    echo "--------------------------------------------"
    echo "Camera $i"
    echo "Device: $DEVICE"
    echo "--------------------------------------------"


    if [ ! -e "$DEVICE" ]; then

        echo "[FAIL] Device does not exist: $DEVICE"

        echo "DEVICE NOT FOUND" > "$LOG_FILE"

        ((FAIL_COUNT++))

        continue

    fi


    # 防止某一顆相機 blocking DQBUF 永久卡住
    timeout 20s \
        "$TEST_PROGRAM" \
        "$DEVICE" \
        "$OUTPUT_FILE" \
        2>&1 | tee "$LOG_FILE"


    RESULT=${PIPESTATUS[0]}


    if [ "$RESULT" -eq 0 ]; then

        echo
        echo "[PASS] Camera $i"

        ((PASS_COUNT++))

    elif [ "$RESULT" -eq 124 ]; then

        echo
        echo "[TIMEOUT] Camera $i"

        ((FAIL_COUNT++))

    else

        echo
        echo "[FAIL] Camera $i"

        ((FAIL_COUNT++))

    fi


    # 每顆相機測完休息 2 秒
    sleep 2

done


echo
echo
echo "============================================"
echo " Test Summary"
echo "============================================"

echo "PASS : $PASS_COUNT"
echo "FAIL : $FAIL_COUNT"
echo "TOTAL: ${#CAMERAS[@]}"

echo

echo "Images:"
ls -lh "$OUTPUT_DIR"/*.jpg 2>/dev/null || true

echo
echo "============================================"


if [ "$FAIL_COUNT" -eq 0 ]; then
    exit 0
else
    exit 1
fi
