# Prius Multi-Camera V4L2 Stress Test

Purpose: test 1 to 6 oToCAM222 camera chains simultaneously without ROS 2.

Default mode:

- 1920x1280
- UYVY 4:2:2
- 30 FPS
- V4L2 mmap
- blocking VIDIOC_DQBUF
- 8 mmap buffers

Default video nodes:

- /dev/video0
- /dev/video2
- /dev/video4
- /dev/video6
- /dev/video8
- /dev/video10

## Build

```bash
cd /colcon_ws/tools/camera_stress_test

mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## Test one stage manually

Three cameras for 30 seconds:

```bash
./build/multi_camera_stress_test \
  --count 3 \
  --duration 30 \
  --warmup 2 \
  --csv stage_3.csv
```

## Run automatic 1 -> 6 test

```bash
chmod +x run_stress_test.sh
./run_stress_test.sh
```

Longer test:

```bash
DURATION=60 WARMUP=3 ./run_stress_test.sh
```

PASS rule:

- effective FPS >= 90% of requested FPS
- no DQBUF error
- no QBUF error

WARN means frames are received but performance is degraded.
FAIL means no valid frame or a fatal V4L2 error.
