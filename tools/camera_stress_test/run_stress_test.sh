#!/usr/bin/env bash
set -u

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROGRAM="$BASE_DIR/build/multi_camera_stress_test"

DURATION="${DURATION:-30}"
WARMUP="${WARMUP:-2}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1280}"
FPS="${FPS:-30}"
BUFFERS="${BUFFERS:-8}"

RESULT_DIR="$BASE_DIR/stress_results_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULT_DIR"

echo
echo "============================================================"
echo " Prius 1-to-6 Camera Stress Test"
echo "============================================================"
echo "Mode    : ${WIDTH}x${HEIGHT} UYVY @ ${FPS} FPS"
echo "Duration: ${DURATION}s per stage"
echo "Warm-up : ${WARMUP}s"
echo "Buffers : ${BUFFERS}"
echo "Results : $RESULT_DIR"
echo "============================================================"
echo

if [ ! -x "$PROGRAM" ]; then
    echo "[ERROR] Test program not found:"
    echo "        $PROGRAM"
    echo
    echo "Build it first:"
    echo "  mkdir -p build"
    echo "  cd build"
    echo "  cmake .."
    echo "  make -j\$(nproc)"
    exit 1
fi

SUMMARY="$RESULT_DIR/stage_summary.csv"
echo "stage,camera_count,exit_code,result" > "$SUMMARY"

for COUNT in 1 2 3 4 5 6; do
    echo
    echo "############################################################"
    echo "# Stage $COUNT: $COUNT simultaneous camera(s)"
    echo "############################################################"

    LOG="$RESULT_DIR/stage_${COUNT}.log"
    CSV="$RESULT_DIR/stage_${COUNT}.csv"
    KLOG="$RESULT_DIR/stage_${COUNT}_kernel.log"

    BEFORE=0
    if dmesg >/dev/null 2>&1; then
        BEFORE="$(dmesg | wc -l)"
    fi

    WATCHDOG=$((WARMUP + DURATION + 20))

    timeout --signal=INT --kill-after=5s "${WATCHDOG}s" \
        "$PROGRAM" \
        --count "$COUNT" \
        --duration "$DURATION" \
        --warmup "$WARMUP" \
        --width "$WIDTH" \
        --height "$HEIGHT" \
        --fps "$FPS" \
        --buffers "$BUFFERS" \
        --csv "$CSV" \
        2>&1 | tee "$LOG"

    RESULT=${PIPESTATUS[0]}

    if [ "$RESULT" -eq 0 ]; then
        STATUS="PASS"
    elif [ "$RESULT" -eq 124 ] || [ "$RESULT" -eq 137 ]; then
        STATUS="TIMEOUT"
    else
        STATUS="WARN_OR_FAIL"
    fi

    echo "${COUNT},${COUNT},${RESULT},${STATUS}" >> "$SUMMARY"

    if dmesg >/dev/null 2>&1; then
        START_LINE=$((BEFORE + 1))
        dmesg | tail -n +"$START_LINE" > "$KLOG" || true
    else
        echo "dmesg unavailable" > "$KLOG"
    fi

    echo "[STAGE $COUNT] $STATUS"
    sleep 3
done

echo
echo "============================================================"
echo " All stages finished"
echo "============================================================"
echo "Results directory:"
echo "  $RESULT_DIR"
echo
column -s, -t "$SUMMARY" 2>/dev/null || cat "$SUMMARY"
