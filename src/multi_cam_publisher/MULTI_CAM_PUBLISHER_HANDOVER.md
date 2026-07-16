# multi_cam_publisher 同儕交接與維護說明

> 專案：Prius 多相機影像系統  
> ROS 版本：ROS 2 Humble  
> 平台：MIC-770 工業電腦  
> 相機：6 × oToCAM222_C103T  
> 影像鏈路：Camera → FPD-Link III → DES2MIPI_954 → USB 3.0 UVC → V4L2 → ROS 2  
> Package：`multi_cam_publisher`

---

## 1. 文件目的

本文件用於 `multi_cam_publisher` ROS 2 package 的交接、日常維護與故障排查。

目前此節點的主要任務是：

1. 同時開啟 6 顆 UVC 相機。
2. 使用 Linux V4L2 mmap 串流方式擷取影像。
3. 使用 blocking `VIDIOC_DQBUF`，不依賴 `select()` / `poll()`。
4. 以 `1920×1280 UYVY @ 30 FPS` 擷取。
5. 每顆相機保留最新有效 frame。
6. 以 ROS 2 timer 預設 10 Hz 發布最新影像。
7. 支援 `bgr8` 與 `yuv422` 兩種 ROS Image 輸出格式。
8. 統計 FPS、錯誤 frame、空 frame、sequence drop 等狀態。
9. 影像長時間停止或裝置錯誤時，對單一相機執行自動重新連線。

---

## 2. 已完成的實機驗證

目前已完成以下驗證。

### 2.1 六顆相機逐顆測試

使用：

```text
1920×1280
UYVY 4:2:2
30 FPS
V4L2 mmap
blocking VIDIOC_DQBUF
```

結果：

```text
PASS : 6
FAIL : 0
TOTAL: 6
```

對應影像 capture node：

```text
/dev/video0
/dev/video2
/dev/video4
/dev/video6
/dev/video8
/dev/video10
```

### 2.2 多相機同時壓力測試

已完成 1 → 6 顆逐級併發測試：

```text
Stage 1: 1 camera  PASS
Stage 2: 2 cameras PASS
Stage 3: 3 cameras PASS
Stage 4: 4 cameras PASS
Stage 5: 5 cameras PASS
Stage 6: 6 cameras PASS
```

因此目前已證明：

```text
6 × Camera
    ↓
DES2MIPI_954
    ↓
USB 3.0
    ↓
Intel xHCI
    ↓
uvcvideo
    ↓
V4L2 mmap + blocking DQBUF
```

可以在目前 MIC-770 平台上同時工作。

---

## 3. 非常重要的已知條件

### 3.1 正式使用 1920×1280，不要預設改回 1920×1080

六顆相機在以下模式已通過測試：

```text
1920×1280
UYVY
30 FPS
```

先前以 `1920×1080` 測試時，曾出現只有部分相機能取得有效 frame、其他相機 blocking `DQBUF` 等待的情況。

因此目前正式設定應保持：

```yaml
capture_width: 1920
capture_height: 1280
capture_fps: 30
```

除非重新完成完整的單顆與六顆併發驗證，否則不要任意修改解析度。

### 3.2 不建議用 `usb_cam` 取代目前的 blocking DQBUF 架構

實測 `ros-drivers/usb_cam` 在已知正常的 `/dev/video0` 上仍出現：

```text
Select timeout, exiting...
```

而自製 blocking `VIDIOC_DQBUF` 程式可以成功取得完整 frame。

因此這批相機／轉換板目前應保留：

```text
blocking VIDIOC_DQBUF
```

不要改回：

```text
select()
poll()
```

作為主要等待機制。

---

# 4. Package 目錄結構

```text
multi_cam_publisher/
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── config/
│   └── cameras.yaml
│
├── launch/
│   └── multi_cam.launch.py
│
├── include/
│   └── multi_cam_publisher/
│       └── camera_v4l2.hpp
│
└── src/
    ├── camera_v4l2.cpp
    └── multi_cam_node.cpp
```

各檔案功能：

| 檔案 | 功能 |
|---|---|
| `camera_v4l2.hpp` | V4L2 camera class、設定結構、統計結構、frame snapshot 定義 |
| `camera_v4l2.cpp` | 相機 open、格式設定、mmap、DQBUF、QBUF、重連與統計 |
| `multi_cam_node.cpp` | ROS 2 node、參數、Publisher、timer、色彩轉換、watchdog |
| `cameras.yaml` | 六顆相機裝置、Topic、Frame ID 與運行參數 |
| `multi_cam.launch.py` | 使用 YAML 啟動節點 |
| `CMakeLists.txt` | C++17、ROS 2、OpenCV、Threads 編譯設定 |
| `package.xml` | ROS 2 package metadata 與相依套件 |

---

# 5. 系統架構

## 5.1 整體資料流

```text
Camera 0 ─┐
Camera 1 ─┤
Camera 2 ─┤
Camera 3 ─┼─→ V4L2 Capture Threads
Camera 4 ─┤
Camera 5 ─┘
              │
              │ 1920×1280 UYVY @ 30 FPS
              ▼
        每顆保留最新有效 Frame
              │
              │ ROS Timer @ 10 Hz
              ▼
        讀取最新 Frame Snapshot
              │
        ┌─────┴─────┐
        │           │
        ▼           ▼
      bgr8        yuv422
   OpenCV轉換       直接發布
        │           │
        └─────┬─────┘
              ▼
       sensor_msgs/Image
              ▼
         ROS 2 Topics
```

---

## 5.2 執行緒模型

每顆 Camera 各有一條獨立 worker thread：

```text
CameraV4L2 #0 → Thread 0
CameraV4L2 #1 → Thread 1
CameraV4L2 #2 → Thread 2
CameraV4L2 #3 → Thread 3
CameraV4L2 #4 → Thread 4
CameraV4L2 #5 → Thread 5
```

ROS node 本身還有：

```text
publish timer
stats/watchdog timer
```

概念上：

```text
┌────────────────────────────────────────────┐
│ MultiCamNode                               │
│                                            │
│  Publish Timer @ 10 Hz                     │
│  Stats Timer @ 5 sec                       │
│                                            │
│  CameraV4L2[0] ─ Worker Thread             │
│  CameraV4L2[1] ─ Worker Thread             │
│  CameraV4L2[2] ─ Worker Thread             │
│  CameraV4L2[3] ─ Worker Thread             │
│  CameraV4L2[4] ─ Worker Thread             │
│  CameraV4L2[5] ─ Worker Thread             │
└────────────────────────────────────────────┘
```

單一相機發生錯誤時，其他相機不需要一起停止。

---

# 6. CameraV4L2 工作流程

`CameraV4L2` 是底層 V4L2 封裝，不負責 ROS topic。

完整流程：

```text
start()
  ↓
workerLoop()
  ↓
openAndStart()
  ├─ open()
  ├─ VIDIOC_QUERYCAP
  ├─ VIDIOC_S_FMT
  ├─ VIDIOC_G_PARM / S_PARM
  ├─ VIDIOC_REQBUFS
  ├─ VIDIOC_QUERYBUF
  ├─ mmap()
  ├─ VIDIOC_QBUF
  └─ VIDIOC_STREAMON
  ↓
captureUntilFailure()
  ↓
blocking VIDIOC_DQBUF
  ↓
檢查 ERROR / bytesused
  ↓
有效 frame → 更新最新 frame
  ↓
前一張最新 buffer QBUF 回 driver
  ↓
繼續下一次 DQBUF
```

---

# 7. Latest Frame 機制

這個版本不是每收到一張 frame 就立刻送 ROS。

Capture thread 會：

1. `DQBUF` 取得新 frame。
2. 檢查是否為有效 frame。
3. 將前一張 latest buffer 重新 `QBUF`。
4. 把新 buffer 保留為 current latest buffer。
5. ROS timer 需要發布時才複製該 buffer。

概念：

```text
Driver Buffer Queue
    │
    ▼
DQBUF New Frame
    │
    ├─ Bad Frame → QBUF
    │
    └─ Valid Frame
          │
          ▼
    release previous latest buffer
          │
          ▼
    keep newest buffer as latest
          │
          ▼
    ROS timer copies snapshot
```

此設計目的：

- Capture 維持 30 FPS。
- ROS 發布只需 10 Hz。
- 避免每張 30 FPS frame 都做 OpenCV 色彩轉換。
- ROS subscriber 取得較新的影像，而不是等待舊 frame FIFO。

---

# 8. V4L2 Buffer 設計

預設：

```yaml
buffer_count: 8
```

使用：

```text
VIDIOC_REQBUFS
V4L2_MEMORY_MMAP
```

流程：

```text
REQBUFS(8)
  ↓
QUERYBUF × N
  ↓
mmap × N
  ↓
QBUF × N
  ↓
STREAMON
```

注意：Driver 實際回傳的 buffer 數量可能不完全等於要求值，但至少必須為 2。

---

# 9. Error Frame 處理

以下 frame 不會發布：

## 9.1 V4L2 error buffer

判斷：

```cpp
buffer.flags & V4L2_BUF_FLAG_ERROR
```

處理：

```text
error_frames++
QBUF
continue
```

## 9.2 Empty frame

判斷：

```text
bytesused == 0
```

處理：

```text
empty_frames++
QBUF
continue
```

此設計是因為實機測試中，部分相機啟動串流後可能先回傳若干：

```text
bytesused = 0
V4L2_BUF_FLAG_ERROR
```

之後才開始輸出正常 30 FPS frame。

因此：

> 啟動初期出現少量 error / empty frame 不一定代表 Camera 故障。

真正需要注意的是錯誤持續增加，且 `valid_frames` 不再增加。

---

# 10. Sequence Drop 統計

Camera 每次取得有效 frame 時會比較：

```text
current_sequence
previous_sequence
```

若：

```text
current_sequence - previous_sequence > 1
```

則累加：

```text
sequence_drops
```

例如：

```text
previous = 100
current  = 104
```

代表：

```text
drop = 3 frames
```

注意：

`sequence_drops` 是 V4L2 frame sequence 的間隔統計，不能直接等同於 ROS topic 掉幀。

因為：

```text
Capture = 30 FPS
Publish = 10 FPS
```

ROS 本來就不會發布每一張 capture frame。

---

# 11. ROS 發布流程

ROS timer 預設：

```yaml
publish_hz: 10.0
```

每次 timer：

```text
for each camera:
    getLatestFrame()
        ↓
    frame validity check
        ↓
    duplicate sequence check
        ↓
    timestamp calculation
        ↓
    encoding conversion
        ↓
    publish
```

若最新 frame sequence 和上次已發布 sequence 相同：

```text
skip
```

因此不會重複發布同一張 frame。

---

# 12. Timestamp 設計

底層 Camera thread 記錄的是：

```text
std::chrono::steady_clock
```

ROS publish 時：

```text
frame_age = steady_now - capture_time

ROS timestamp =
ROS now - frame_age
```

目的：

避免把「Publish Timer 執行時間」直接當成影像 capture time。

這不是硬體 timestamp 同步方案，但比所有 frame 都直接使用 publish 時刻更接近實際擷取時間。

若未來需要：

- 多相機硬體同步
- PTP
- GNSS 時間同步
- Sensor Timestamp
- Autoware 嚴格 sensor timing

應重新設計 timestamp pipeline。

---

# 13. ROS Image 輸出格式

支援：

```yaml
output_encoding: bgr8
```

或：

```yaml
output_encoding: yuv422
```

---

## 13.1 bgr8 模式

流程：

```text
UYVY
  ↓
cv::cvtColor(
  COLOR_YUV2BGR_UYVY
)
  ↓
BGR8
  ↓
ROS Image
```

優點：

- OpenCV 使用方便。
- 大部分影像處理 node 相容性較好。
- 與常見 ROS camera pipeline 接軌較容易。

缺點：

- CPU 需要色彩轉換。
- 每張影像由 2 bytes/pixel 增加到 3 bytes/pixel。
- ROS/DDS memory bandwidth 較大。

---

## 13.2 yuv422 模式

流程：

```text
UYVY
  ↓
直接 copy
  ↓
sensor_msgs/Image
encoding = yuv422
```

優點：

- 不需要 `cvtColor()`。
- ROS message payload 較小。
- CPU 負擔較低。

缺點：

- 下游 node 必須支援 `yuv422`。
- 某些 AI / OpenCV pipeline 仍需再轉 BGR 或 RGB。

---

# 14. 理論資料量

單顆 UYVY frame：

```text
1920 × 1280 × 2
= 4,915,200 bytes
≈ 4.92 MB
```

6 顆 × 30 FPS capture：

```text
4.9152 MB × 30 × 6
≈ 884.7 MB/s
```

此值是未壓縮影像 payload 的資料量級估算，不等同 USB wire-level throughput。

ROS 10 Hz 輸出：

## yuv422

```text
4.9152 MB × 10 × 6
≈ 294.9 MB/s
```

## bgr8

```text
1920 × 1280 × 3 × 10 × 6
≈ 442.4 MB/s
```

因此在 MIC-770 上：

- 若下游支援，優先評估 `yuv422`。
- 若下游直接使用 OpenCV BGR，使用 `bgr8` 比較方便。
- 不應在 Capture Thread 中對所有 30 FPS frame 做 BGR conversion。

---

# 15. ROS 參數

預設設定檔：

```text
config/cameras.yaml
```

主要參數：

| 參數 | 預設值 | 說明 |
|---|---:|---|
| `devices` | 6 個 `/dev/videoX` | V4L2 capture node 列表 |
| `topics` | 6 個 image topic | 每顆 Camera 的 ROS topic |
| `frame_ids` | 6 個 optical frame | ROS header frame ID |
| `capture_width` | `1920` | 擷取寬度 |
| `capture_height` | `1280` | 擷取高度 |
| `capture_fps` | `30` | V4L2 requested FPS |
| `publish_hz` | `10.0` | ROS 發布頻率 |
| `output_encoding` | `bgr8` | `bgr8` 或 `yuv422` |
| `buffer_count` | `8` | mmap buffer request count |
| `reconnect_delay_ms` | `1000` | 重連前等待時間 |
| `frame_timeout_sec` | `3.0` | Watchdog 無有效 frame 門檻 |
| `stats_period_sec` | `5.0` | 統計輸出週期 |

---

# 16. 預設 Device / Topic Mapping

| Camera | V4L2 Node | ROS Topic | Frame ID |
|---|---|---|---|
| Camera 0 | `/dev/video0` | `/camera/device_0/image_raw` | `camera_0_optical_frame` |
| Camera 1 | `/dev/video2` | `/camera/device_1/image_raw` | `camera_1_optical_frame` |
| Camera 2 | `/dev/video4` | `/camera/device_2/image_raw` | `camera_2_optical_frame` |
| Camera 3 | `/dev/video6` | `/camera/device_3/image_raw` | `camera_3_optical_frame` |
| Camera 4 | `/dev/video8` | `/camera/device_4/image_raw` | `camera_4_optical_frame` |
| Camera 5 | `/dev/video10` | `/camera/device_5/image_raw` | `camera_5_optical_frame` |

注意：

奇數 video node：

```text
/dev/video1
/dev/video3
/dev/video5
/dev/video7
/dev/video9
/dev/video11
```

在目前 UVC device topology 中不是主要影像 capture node。

不要誤把它們加入 `devices`。

---

# 17. 參數一致性限制

以下三個 list 長度必須一致：

```yaml
devices:
topics:
frame_ids:
```

例如 6 顆 Camera：

```text
devices.size()   = 6
topics.size()    = 6
frame_ids.size() = 6
```

若不一致，Node 會直接丟出 runtime error。

---

# 18. 編譯方式

進入 workspace：

```bash
cd /colcon_ws
```

載入 ROS：

```bash
source /opt/ros/humble/setup.bash
```

編譯：

```bash
colcon build \
  --packages-select multi_cam_publisher \
  --cmake-clean-cache \
  --event-handlers console_direct+
```

完成後：

```bash
source /colcon_ws/install/setup.bash
```

確認 executable：

```bash
ros2 pkg executables multi_cam_publisher
```

預期：

```text
multi_cam_publisher multi_cam_publisher_node
```

---

# 19. 啟動方式

## 建議方式：Launch

```bash
ros2 launch multi_cam_publisher multi_cam.launch.py
```

Launch file 會自動載入：

```text
config/cameras.yaml
```

---

## 直接執行 Node

```bash
ros2 run multi_cam_publisher multi_cam_publisher_node
```

此方式使用程式內預設參數，除非另外傳入 parameter。

正式車輛環境建議使用 launch + YAML。

---

# 20. 正常啟動 Log

正常時會看到類似：

```text
[CAM] /dev/video0 streaming 1920x1280 UYVY @ requested 30 FPS, mmap buffers=8
[CAM] /dev/video2 streaming 1920x1280 UYVY @ requested 30 FPS, mmap buffers=8
...
```

Node：

```text
Started 6 cameras:
capture=1920x1280 UYVY @ 30 FPS
publish=10.00 Hz
output=bgr8
```

---

# 21. Runtime Statistics

預設每 5 秒印出：

```text
[/dev/video0]
connected=1
capture=29.99 FPS
valid=...
error=...
empty=...
seq_drop=...
dqerr=...
qerr=...
reconnect=...
age=...
```

欄位意義：

| 欄位 | 意義 |
|---|---|
| `connected` | 是否處於 streaming 狀態 |
| `capture` | 最近統計週期有效 frame FPS |
| `valid` | 累積有效 frame |
| `error` | `V4L2_BUF_FLAG_ERROR` 數量 |
| `empty` | `bytesused == 0` 數量 |
| `seq_drop` | V4L2 sequence 間隔推算的 drop 數 |
| `dqerr` | `VIDIOC_DQBUF` ioctl error 數 |
| `qerr` | `VIDIOC_QBUF` ioctl error 數 |
| `reconnect` | 成功進入 streaming 的累積次數 |
| `age` | 距離最後有效 frame 的秒數 |

注意：

目前程式中的 `reconnect_count` 在每次成功 `openAndStart()` 後都會加 1，因此：

```text
reconnect=1
```

通常代表首次成功連線，而不是已經斷線重連 1 次。

若想顯示純粹「實際重連次數」，未來可將首次成功連線排除。

---

# 22. Watchdog 與自動重連

預設：

```yaml
frame_timeout_sec: 3.0
```

若 Camera 顯示：

```text
connected = true
```

但：

```text
last_frame_age_sec > 3.0
```

Node 會：

```text
requestReconnect()
```

底層流程：

```text
reconnect_requested = true
        ↓
STREAMOFF
        ↓
中斷 blocking DQBUF
        ↓
capture loop 離開
        ↓
munmap
        ↓
close
        ↓
等待 reconnect_delay_ms
        ↓
重新 open
```

預設等待：

```yaml
reconnect_delay_ms: 1000
```

---

# 23. 裝置拔插的重要限制

目前使用：

```text
/dev/video0
/dev/video2
...
```

作為固定裝置名稱。

但 USB 裝置拔插、重新 enumerate 或開機順序改變後，Linux 不保證相同實體 Camera 永遠取得相同 `/dev/videoX`。

因此正式車載版本建議建立 udev symlink，例如：

```text
/dev/prius_cam/front
/dev/prius_cam/front_left
/dev/prius_cam/front_right
/dev/prius_cam/rear
/dev/prius_cam/rear_left
/dev/prius_cam/rear_right
```

然後 `cameras.yaml` 改成固定 symbolic link。

如果沒有建立 udev mapping，自動 reconnect 只能在「原 device path 再次出現」的情況下可靠工作。

---

# 24. 常用檢查指令

## 24.1 列出 Camera

```bash
v4l2-ctl --list-devices
```

## 24.2 確認某顆格式

```bash
v4l2-ctl \
  -d /dev/video0 \
  --list-formats-ext
```

## 24.3 單顆 blocking mmap 測試

```bash
timeout 30s v4l2-ctl \
  -d /dev/video0 \
  --set-fmt-video=width=1920,height=1280,pixelformat=UYVY \
  --stream-mmap=8 \
  --stream-count=30 \
  --stream-to=/dev/null \
  --verbose
```

## 24.4 確認 Topic

```bash
ros2 topic list | grep image_raw
```

## 24.5 檢查發布頻率

```bash
ros2 topic hz /camera/device_0/image_raw
```

預期約：

```text
10 Hz
```

注意：

```text
capture FPS ≈ 30
publish FPS ≈ 10
```

兩者是不同概念。

## 24.6 查看 Topic 型態

```bash
ros2 topic info -v /camera/device_0/image_raw
```

## 24.7 查看 device 是否被其他程式占用

```bash
fuser -v \
  /dev/video0 \
  /dev/video2 \
  /dev/video4 \
  /dev/video6 \
  /dev/video8 \
  /dev/video10
```

---

# 25. 常見故障排查

## 問題 A：Node 找不到 Camera

症狀：

```text
open failed
No such file or directory
```

檢查：

```bash
ls -l /dev/video*
```

再執行：

```bash
v4l2-ctl --list-devices
```

可能原因：

- USB Camera 尚未 enumerate。
- `/dev/videoX` 編號變動。
- Docker 沒有看到 Host `/dev`。
- Camera / DES 板未供電。

---

## 問題 B：STREAMON 成功，但沒有有效 frame

先單獨測：

```bash
timeout 30s v4l2-ctl \
  -d /dev/videoX \
  --set-fmt-video=width=1920,height=1280,pixelformat=UYVY \
  --stream-mmap=8 \
  --stream-count=30 \
  --stream-to=/dev/null \
  --verbose
```

若 blocking `v4l2-ctl` 可成功：

- 檢查 ROS node 是否使用相同格式。
- 檢查 Camera 是否同時被其他 process 使用。
- 檢查是否改成錯誤解析度。

---

## 問題 C：大量 error / empty frame

先觀察是否只有啟動初期出現。

若：

```text
起始 8 個 error buffer
之後穩定 30 FPS
```

目前可視為已知啟動行為。

若錯誤持續：

```text
valid 不增加
error 持續增加
age 持續上升
```

應檢查：

- Camera 電源。
- FPD-Link 線。
- DES2MIPI_954。
- USB cable。
- Kernel log。
- 是否發生 USB disconnect。

---

## 問題 D：ROS Topic 只有幾 Hz

先區分：

```text
Capture FPS
```

與：

```text
Publish FPS
```

查看 Node statistics：

```text
capture=29.99 FPS
```

但：

```bash
ros2 topic hz ...
```

只有 8～10 Hz，可能與：

- `publish_hz`
- CPU load
- BGR conversion
- DDS transport
- Subscriber QoS
- Topic monitor overhead

有關。

若 Capture FPS 本身下降，再查 V4L2 / USB。

---

## 問題 E：CPU 使用率過高

第一個調整方向：

```yaml
output_encoding: yuv422
```

這會避免：

```text
UYVY → BGR
```

的 OpenCV conversion。

第二個方向：

降低：

```yaml
publish_hz
```

例如：

```yaml
publish_hz: 5.0
```

注意：

降低 ROS publish Hz 不會降低 Camera USB capture 的 30 FPS。

---

## 問題 F：重新插入 Camera 後沒有恢復

檢查：

```bash
v4l2-ctl --list-devices
```

確認該 Camera 是否仍為原本的：

```text
/dev/videoX
```

若 device node 改名，自動 reconnect 無法追蹤新的 path。

正式系統應使用 udev symlink。

---

# 26. 修改程式時要改哪裡

## 要修改 V4L2 擷取邏輯

修改：

```text
src/camera_v4l2.cpp
include/multi_cam_publisher/camera_v4l2.hpp
```

例如：

- Format。
- Buffer 策略。
- Reconnect。
- Capture timestamp。
- Sequence statistics。
- Additional V4L2 controls。

---

## 要修改 ROS Topic / Publisher

修改：

```text
src/multi_cam_node.cpp
```

例如：

- QoS。
- Topic type。
- Compression。
- Image Transport。
- CameraInfo。
- Diagnostic topic。
- Lifecycle Node。

---

## 要修改預設設備或 Topic

修改：

```text
config/cameras.yaml
```

通常不需要重新修改 C++。

---

## 要修改啟動方式

修改：

```text
launch/multi_cam.launch.py
```

---

# 27. 建議交接測試流程

任何人重新接手後，第一次操作建議依照以下順序。

## Step 1：確認六顆裝置

```bash
v4l2-ctl --list-devices
```

確認：

```text
video0
video2
video4
video6
video8
video10
```

存在。

## Step 2：單顆測試

使用 `tools/camera_test`：

```bash
./test_all_cameras.sh
```

預期：

```text
PASS : 6
FAIL : 0
TOTAL: 6
```

## Step 3：多顆壓力測試

使用 `tools/camera_stress_test`：

```bash
./run_stress_test.sh
```

預期 Stage 1～6 全部 PASS。

## Step 4：啟動正式 ROS node

```bash
ros2 launch multi_cam_publisher multi_cam.launch.py
```

## Step 5：確認 6 個 topic

```bash
ros2 topic list | grep image_raw
```

## Step 6：確認頻率

```bash
ros2 topic hz /camera/device_0/image_raw
```

依序測到 device 5。

---

# 28. 目前程式的限制

## 28.1 尚未發布 CameraInfo

目前只有：

```text
sensor_msgs/msg/Image
```

尚未：

```text
sensor_msgs/msg/CameraInfo
```

若後續進入：

- Autoware
- Camera calibration
- Image rectification
- Multi-camera fusion
- Projection
- Calibration-aware perception

需要補 CameraInfo。

---

## 28.2 尚未做硬體同步管理

目前 6 顆相機獨立 capture。

程式本身沒有實作：

- Hardware trigger。
- Frame sync control。
- PTP。
- Sensor timestamp synchronization。

若多相機 perception 對同步要求嚴格，需要硬體與軟體共同設計。

---

## 28.3 尚未使用 ROS 2 zero-copy

目前存在 memory copy：

```text
mmap buffer
    ↓ copy
FrameSnapshot vector
    ↓ copy / conversion
ROS Image data
```

在 6 × 1920×1280 情況下，記憶體頻寬消耗不可忽略。

未來可評估：

- Loaned Message。
- Intra-process communication。
- Shared-memory DDS。
- DMA-BUF。
- GStreamer。
- Hardware accelerator pipeline。

但在功能尚未穩定整合前，不建議過早複雜化。

---

## 28.4 目前使用 Wall Timer 發布

Publish 使用：

```text
create_wall_timer()
```

因此發布節奏由 timer 驅動，而不是每收到 frame 就發布。

這是刻意設計：

```text
Capture 30 FPS
Publish 10 FPS
```

未來若需求改成每 frame 發布，需要重新評估 CPU 與 DDS bandwidth。

---

# 29. 建議未來改進項目

優先級建議：

## P1：固定 Camera Device Mapping

建立 udev rules：

```text
Physical Camera
→ Stable symlink
```

這是正式車載部署前最值得做的項目。

## P2：加入 CameraInfo

支援：

```text
Image
CameraInfo
```

並維護 calibration YAML。

## P3：加入 diagnostic_msgs

把目前 console log：

```text
capture FPS
error frames
empty frames
sequence drop
reconnect count
age
```

改成標準：

```text
/diagnostics
```

## P4：加入 Camera Name 與車身方向語意

將：

```text
device_0
device_1
```

改成：

```text
front
front_left
front_right
rear
rear_left
rear_right
```

避免後續 perception package 依賴不具物理意義的 index。

## P5：評估 yuv422 pipeline

若 AI pipeline 可以接受 YUV 或在後端統一轉換，優先測試：

```yaml
output_encoding: yuv422
```

降低 CPU 與 ROS message payload。

---

# 30. 交接重點摘要

接手此 package 時，請先記住以下幾點：

1. 六顆相機已驗證可用。
2. 正式驗證模式是 `1920×1280 UYVY @ 30 FPS`。
3. 不要直接改回 `1920×1080` 而不重新測試。
4. 此設備應使用 blocking `VIDIOC_DQBUF`。
5. `usb_cam` 在這套設備上曾出現 `Select timeout`。
6. Capture 是 30 FPS，ROS Publish 預設是 10 Hz。
7. `bgr8` 比較方便，但 CPU 與 payload 較高。
8. `yuv422` 較省資源，但下游需要支援。
9. 每顆 Camera 有獨立 worker thread。
10. 單顆失效不應直接停止其他 Camera。
11. Watchdog 會在長時間沒有有效 frame 時要求重連。
12. `/dev/videoX` 不保證拔插後保持不變，正式部署應使用 udev symlink。
13. 修改參數優先改 `config/cameras.yaml`，不要先硬改 C++。
14. Debug 時先用 V4L2 工具確認 Capture，再查 ROS Topic。
15. 在修改底層架構前，先跑單顆與 1→6 顆壓力測試作為 regression test。

---

# 31. 快速操作指令

## 編譯

```bash
cd /colcon_ws

source /opt/ros/humble/setup.bash

colcon build \
  --packages-select multi_cam_publisher \
  --cmake-clean-cache \
  --event-handlers console_direct+

source /colcon_ws/install/setup.bash
```

## 啟動

```bash
ros2 launch multi_cam_publisher multi_cam.launch.py
```

## 查看 Topic

```bash
ros2 topic list | grep image_raw
```

## 查看頻率

```bash
ros2 topic hz /camera/device_0/image_raw
```

## 單顆底層測試

```bash
timeout 30s v4l2-ctl \
  -d /dev/video0 \
  --set-fmt-video=width=1920,height=1280,pixelformat=UYVY \
  --stream-mmap=8 \
  --stream-count=30 \
  --stream-to=/dev/null \
  --verbose
```

## 查看 USB / V4L2 問題

```bash
dmesg | grep -Ei "uvc|usb|video"
```

---

# 32. 最後維護原則

建議維持以下分層：

```text
CameraV4L2
    │
    │ 只負責：
    │ V4L2
    │ mmap
    │ buffer
    │ reconnect
    │ capture stats
    ▼
FrameSnapshot
    │
    ▼
MultiCamNode
    │
    │ 只負責：
    │ ROS parameter
    │ ROS timer
    │ encoding
    │ timestamp
    │ publish
    │ watchdog
    ▼
ROS Topics
```

不要把：

```text
ROS Publisher
```

塞入 `CameraV4L2`，也不要把：

```text
VIDIOC_DQBUF
```

直接放進 ROS timer callback。

維持這個分層，後續加入 Autoware、CameraInfo、diagnostics 或不同 transport 時會比較容易維護。

---

**文件狀態：依目前 `multi_cam_publisher` 1.0.0 架構與 MIC-770 六相機實機測試結果整理。**
