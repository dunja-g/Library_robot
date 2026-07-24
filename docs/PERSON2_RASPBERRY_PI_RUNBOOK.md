# Person 2 Raspberry Pi Runbook — Legacy Vision Diagnostics

> The current borrowing runtime is fixed-grid and marker-free. ArUco commands
> below are retained only for offline diagnostics.

这份清单用于将已经通过离线测试的视觉与导航代码部署到树莓派，并与 Arduino Mega 真机联调。

## 1. 环境检查

Picamera2 应优先使用 Raspberry Pi OS 自带或 APT 安装的版本：

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install flask pyserial numpy pytest
```

确认 OpenCV 包含 ArUco：

```bash
python -c "import cv2; print(cv2.__version__, hasattr(cv2, 'aruco'))"
```

如果第二项是 `False`，需要在当前环境安装兼容的 `opencv-contrib-python`。

## 2. 先验证打印标签

```bash
python -m pi.vision_diagnostics \
  --image aruco_codes/marker_0.png \
  --output diagnostics/marker_0_annotated.png
```

输出 JSON 应包含 `"id": 0`。

## 3. 验证树莓派摄像头

将打印标签放在镜头前，执行：

```bash
python -m pi.vision_diagnostics \
  --live-seconds 10 \
  --output diagnostics/camera_snapshot.jpg
```

重点记录：

- `measured_fps` 是否接近目标 20 FPS；
- `marker_detection_rate`；
- `seen_ids` 是否正确；
- 保存图片的红蓝颜色是否正常；
- 0.5 m、1 m、1.5 m、2 m 距离下是否都能检测。

## 4. 接入已有猫识别 baseline 相机

如果 baseline 已经管理 Picamera2，不要创建第二个 Picamera2 实例。将原取帧对象适配成：

```python
class ExistingBaselineBackend:
    def __init__(self, baseline_camera):
        self.camera = baseline_camera

    def start(self):
        pass

    def capture_array(self):
        return self.camera.get_frame()  # BGR uint8, shape=(H, W, 3)

    def stop(self):
        self.camera.stop()
```

然后使用 `Camera(backend=ExistingBaselineBackend(existing_camera))`。如果 baseline 输出 RGB，适配器内先转换为 BGR。

## 5. 配置真机参数

复制 `.env.example` 中的变量到启动环境。PowerShell 使用 `$env:NAME="value"`；树莓派 Bash 使用：

```bash
export LIBRARY_ROBOT_USE_MOCK=false
export LIBRARY_ROBOT_SERIAL_PORT=/dev/ttyACM0
export LIBRARY_ROBOT_CAMERA_WIDTH=640
export LIBRARY_ROBOT_CAMERA_HEIGHT=480
export LIBRARY_ROBOT_CAMERA_FPS=20
```

其余标定参数：

| 参数 | 初始值 | 调整依据 |
|---|---:|---|
| `ALIGN_TOLERANCE_PX` | 30 | 左右摆动则增大，偏差明显则减小 |
| `STOP_DISTANCE_CM` | 25 | 实测停车距离 |
| `OBSTACLE_DISTANCE_CM` | 20 | 侧方安全距离 |
| `TARGET_CONFIRMATION_FRAMES` | 2 | 光照差或误检时增大 |
| `ALIGNMENT_CONFIRMATION_FRAMES` | 2 | 对准抖动时增大 |
| `TARGET_LOSS_TOLERANCE_FRAMES` | 3 | 短暂遮挡时增大 |
| `MIN_MARKER_AREA_PX` | 0 | 远处噪声多时逐步提高 |

## 6. 启动完整系统

Person 1 的 `pi/serial_bridge.py` 合并后：

```bash
python -m pi.serial_diagnostics --port /dev/ttyACM0

# 只有架空轮子后才能运行：
python -m pi.serial_diagnostics --port /dev/ttyACM0 --motor-test

python pi/app.py
```

浏览器打开 `http://<树莓派IP>:5000`。真机视频流读取 `RobotController.get_latest_frame()`，不会和控制线程重复抢占摄像头。

## 7. 验收顺序

1. 架空轮子测试左转、右转、前进和停止方向。
2. 地面低速测试扫描与对准，不进入接近阶段。
3. 将中间超声波目标放在 25 cm，确认进入 `ARRIVED`。
4. 将障碍物放到左右传感器 20 cm 内，确认进入 `STOPPED`。
5. 遮挡标签少于 3 帧，机器人应停住等待；持续遮挡则重新扫描。
6. 完成 20 次端到端任务并记录成功率、停车距离和失败原因。

任何相机、检测或超声波异常都必须导致停车，不能在异常状态继续发送 `FORWARD`。
