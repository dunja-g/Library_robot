# Project Context

本文件记录团队已经确认的项目事实和外部参考，供后续开发、评审和 Codex 任务持续使用。

## 已确认硬件

- 微控制器：**Arduino Mega**。
- 三个 HC-SR04 超声波传感器暂定引脚：
  - Left：TRIG 25，ECHO 24
  - Centre：TRIG 23，ECHO 22
  - Right：TRIG 27，ECHO 26
- 树莓派通过 USB Serial 与 Mega 通信，协议波特率为 115200。
- 电机由 MH Electronics/Adafruit Motor Shield V1 兼容板控制。

在编写固件前，Person 1 仍需核对电机屏蔽板与超声波引脚是否存在实际占用冲突。

## 仓库定位

### 当前 V1 仓库

- 地址：https://github.com/dunja-g/Library_robot
- 定位：ArUco 图书定位、摄像头、规则导航状态机、Flask Web UI 和 Arduino Mega 执行层。
- 当前目标：先完成安全、可重复的端到端机器人闭环。

### 团队实现参考仓库

- 地址：https://github.com/12412825-collab/cc-hackers-s-RL-robotics-project
- 2026-07-22 检查的 `main` 提交：`e41ecd6950c70f881927987fdd38aadc3e67de4e`
- 定位：基于 DonkeyCar/Webots 的多模态残差强化学习机器人实现。

后续讨论中提到“实现部分项目”或“RL 参考仓库”时，默认指这个仓库。

## 对 Person 2 有价值的参考模块

| 参考位置 | 可参考内容 | 在 V1 中的用途 |
|---|---|---|
| `parts/differential_drive.py` | 线速度/角速度和归一化控制转换 | 后续将离散转向命令升级为连续控制 |
| `parts/arduino_serial.py` | Arduino 串口适配思路 | 对照 `SerialBridge` 的异常处理和命令边界 |
| `parts/sensors.py` | 传感器融合、缺失值处理、平滑与归一化 | 改进三个超声波读数的鲁棒性 |
| `parts/residual_rl.py` | MobileNet 特征、SAC、ReplayBuffer、ResidualPilot | V2 深度学习/强化学习导航参考 |
| `simulation/webots_adapter.py` | Webots 与控制管线适配 | 真机前的导航和安全逻辑仿真 |
| `tests/` | 差速驱动、传感器融合、Webots 适配测试 | 设计本项目 Mock 测试的参考 |
| `dashboard/` | 控制台和训练状态页面 | Person 3 后续扩展训练监控页面的参考 |

参考仓库采用的核心思路包括：

- MobileNetV3-Small 视觉特征；
- 摄像头与传感器观测的多模态融合；
- SAC（Soft Actor-Critic）连续控制；
- 基础控制器输出加 RL residual 的残差控制；
- Webots 仿真与真实机器人适配层；
- 使用物理量 `v`、`omega` 表示差速驱动命令。

## 复用边界

参考仓库不能直接替换当前 V1，原因如下：

- V1 当前协议是 `FORWARD`、`ROTATE_LEFT`、`ROTATE_RIGHT`、`STOP` 等离散命令；参考仓库更偏向连续的 `v/omega` 控制。
- V1 以 ArUco ID 作为明确目标；参考仓库主要解决通用驾驶、传感器融合和残差策略。
- 参考仓库依赖 DonkeyCar、PyTorch/Webots 等较重运行环境；V1 应先保持 OpenCV + Flask + pyserial 的最小闭环。
- 安全停车必须保留在规则层或 Arduino 层，不能只依赖学习策略。

推荐演进方式：

```text
V1：ArUco + 规则状态机 + 超声波安全停车
  -> V1.5：Webots 仿真 + 连续 v/omega 控制接口
  -> V2：MobileNet/传感器融合 + residual SAC 修正导航
```

Person 2 在 V1 中应保持感知结果与控制器接口清晰，使以后能替换检测/策略模块，而不重写 Arduino 和 Web 层。
