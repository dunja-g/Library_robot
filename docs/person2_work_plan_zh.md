# Person 2：视觉与导航工作计划

## 当前实现进度（2026-07-22）

- [x] ArUco ID 0～4 标签生成器及 300×300 PNG。
- [x] 支持 Picamera2 和自定义 backend 的线程安全 `Camera`。
- [x] ArUco 检测、目标筛选、面积/中心计算与画面标注。
- [x] 扫描、对准、接近、到达与安全停车状态机。
- [x] 控制器保存最新标注帧，供 Person 3 的 MJPEG 流读取。
- [x] 22 个不依赖硬件的离线测试。
- [ ] 在树莓派上验证 RGB/BGR 色序和连续取帧稳定性。
- [ ] 与 Mega 串口桥接和三个 HC-SR04 真机联调。
- [ ] 用打印标签完成距离、角度和光照标定。

运行离线验证：

```bash
python -m pip install opencv-contrib-python numpy pytest
python -m pytest -q
python -m aruco_codes.generate_markers
```

现有猫识别 baseline 可以通过一个很薄的适配器复用取帧管线：

```python
class ExistingBaselineBackend:
    def __init__(self, baseline_camera):
        self.camera = baseline_camera

    def start(self):
        pass  # 如果 baseline 已经启动相机

    def capture_array(self):
        return self.camera.get_frame()  # 必须返回 BGR numpy array

    def stop(self):
        self.camera.stop()

camera = Camera(backend=ExistingBaselineBackend(existing_camera))
```

如果 baseline 返回 RGB，在 `capture_array()` 中使用 `cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)` 转换。

## 1. 你的职责边界

你负责机器人从“获得目标 ArUco ID”到“生成运动决策”的完整链路：

```text
图书 ArUco ID
    -> Pi Camera 图像
    -> ArUco 检测结果
    -> 扫描/对准/接近状态机
    -> SerialBridge 运动命令
```

你负责的文件：

- `aruco_codes/generate_markers.py`
- `pi/camera.py`
- `pi/aruco_detector.py`
- `pi/robot_controller.py`
- `tests/test_aruco_detector.py`
- `tests/test_robot_controller.py`

你不负责电机引脚和网页界面，但需要与 Person 1、Person 3 共同冻结接口。

## 2. 开工前必须确认的接口

### Person 1 提供给你

```python
serial.send_forward()
serial.send_rotate_left()
serial.send_rotate_right()
serial.send_stop()
serial.get_ultrasonic()
```

`get_ultrasonic()` 应返回：

```python
{"left": 43.2, "center": 18.7, "right": 55.1}
```

### 你提供给 Person 3

```python
controller.request_book(aruco_id)
controller.get_state()
controller.reset()
controller.step()
controller.get_latest_frame()
```

建议增加 `get_latest_frame()`，由控制线程保存最新标注画面，网页视频流只读取该画面。这样可以避免摄像头被控制循环和网页流重复读取。

## 3. 分阶段执行计划

### 阶段 A：标签生成与离线样本（第 1 天）

任务：

- 创建 `DICT_5X5_50` 的 ID 0～4 标签。
- 输出 300×300 PNG，并留出白色边框。
- 打印为约 10×10 cm。
- 用不同距离、角度和光照拍摄测试图。

验收：

- 五个标签 ID 正确且互不重复。
- 0.5～2 m 范围内有一组可重复检测的测试样本。

### 阶段 B：摄像头抽象（第 2 天）

任务：

- 实现 `Camera.get_frame()`、`generate_mjpeg()` 和 `stop()`。
- 固定默认画面为 640×480、20 FPS。
- 加锁，保证控制线程和视频流不会同时无序访问相机。
- 摄像头不可用时抛出清楚的错误，而不是返回空帧继续运行。

验收：

- 连续采集至少 5 分钟无崩溃。
- 图像方向正确，BGR/RGB 颜色没有颠倒。
- 退出程序后摄像头资源可以再次打开。

### 阶段 C：ArUco 检测器（第 3～4 天）

任务：

- 实现 `detect(frame)` 和 `draw(frame, detections)`。
- 输出 ID、中心点、面积和四角坐标。
- 对 `ids is None`、空帧、多标签同时出现做防御处理。
- 用保存的图片编写离线测试，不依赖树莓派硬件。

验收：

- 能从测试图中识别正确 ID。
- 多个标签同时出现时，能筛选指定目标 ID。
- `draw()` 不修改原图，或明确记录它会原地修改。

### 阶段 D：状态机与 Mock 测试（第 5～7 天）

实现状态：

```text
IDLE -> SCANNING -> ALIGNING -> APPROACHING -> ARRIVED
                    |               |
                    +-> SCANNING    +-> STOPPED
```

需要补充的工程规则：

- 扫描超过限定时间仍找不到目标，进入 `NOT_FOUND` 或安全回到 `IDLE`。
- 连续多帧检测到目标后再切换状态，避免单帧误检。
- 标签短暂丢失允许容错若干帧，持续丢失才重新扫描。
- 每次状态切换先发送 `STOP`，避免上一条运动指令继续执行。
- `reset()` 和异常处理必须无条件停车。
- 超声波返回 `None` 或异常值时按故障处理，不盲目前进。

建议判定顺序：

1. 系统故障或侧面障碍物过近：`STOPPED`。
2. 中间距离不大于到达阈值：`ARRIVED`。
3. 目标偏离画面中心：回到 `ALIGNING`。
4. 其余情况：继续 `FORWARD`。

验收测试至少覆盖：

- 请求一本书后进入 `SCANNING`。
- 未找到目标时持续扫描。
- 找到错误 ID 时不响应。
- 目标位于左侧、右侧和中心时命令正确。
- 接近过程中目标丢失时停止前进。
- 到达距离时停车并进入 `ARRIVED`。
- 任一危险条件触发 `STOPPED`。
- 从任何状态调用 `reset()` 都停车并返回 `IDLE`。

### 阶段 E：真机集成和参数标定（第 8～10 天）

任务：

- 与 Person 1 联调串口和电机方向。
- 标定旋转速度、对准容差、停车距离与摄像头安装角度。
- 与 Person 3 联调控制线程、状态接口和标注视频流。
- 记录至少 20 次完整任务的结果。

建议记录指标：

| 指标 | 初版目标 |
|---|---:|
| 正确标签识别率 | >= 95% |
| 20 次任务成功率 | >= 90% |
| 最终停车距离 | 20～30 cm |
| 障碍物触发停车 | 100% |
| 单次搜索最长时间 | <= 60 s |

## 4. 每日协作节奏

- 每天开始前从远端拉取队友更新，避免长期偏离接口。
- 每个模块单独提交，不把相机、检测器和状态机混在一个巨大提交中。
- 无硬件阶段全部使用 MockCamera、MockDetector、MockSerial 开发。
- 每次向 Person 1 或 Person 3 修改接口时，同步更新文档和测试。

推荐提交顺序：

```text
feat(vision): add ArUco marker generator
feat(camera): add Pi camera capture interface
feat(vision): implement ArUco detector
test(navigation): add controller mocks and state tests
feat(navigation): implement robot state machine
fix(navigation): tune alignment and safety thresholds
```

## 5. 当前版本与深度学习的关系

V1 使用 ArUco 和规则状态机，属于传统计算机视觉，不是深度学习。建议先用它打通硬件、通信和安全闭环，再在 V2 增加：

- YOLO 图书或书架检测；
- OCR 书脊文字识别；
- 图书封面特征检索；
- 单目深度估计或目标位姿估计。

升级时保持检测器输出格式稳定，便可以将深度学习模型替换进感知层，而不用重写 Web 和 Arduino 层。

## 6. 你现在最先做的三件事

1. Arduino 已确认为 Mega；与 Person 1 最终核对超声波使用 22～27 引脚，并记录电机屏蔽板占用的引脚。
2. 和 Person 3 确认标注视频流由 `RobotController` 提供最新帧，避免两个模块争抢摄像头。
3. 先创建 Mock 对象和状态机测试，再接真实相机与电机，确保安全逻辑可离线验证。

深度学习和强化学习部分可参考团队实现仓库 `12412825-collab/cc-hackers-s-RL-robotics-project`，具体复用范围见 `docs/PROJECT_CONTEXT.md`。
