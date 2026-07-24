# Library Robot — Residual SAC Controller Plug-in Integration

## 1. Overview

This document describes how the **Minimal Residual SAC Controller** (`library_residual`) plugs into `Library_robot` as an optional heading-correction layer.

The existing fixed-grid controller (`GridController`) remains the primary autonomous navigation and safety authority:
- Fixed-grid route planning (`1A`–`3B`).
- MPU6500 IMU + encoder fused heading feedback.
- 90-degree/180-degree rule-based turn execution.
- Single front ultrasonic obstacle safety stop (`front_ultrasonic_cm < 20cm`).
- QR check-in & borrowing mission transaction management.

When enabled, the Residual SAC plugin computes a small, bounded heading correction (`±10 PWM` max) during linear driving segments (`FORWARD` and `BACKWARD`) to compensate for unmodelled motor bias, surface friction, or chassis drift.

---

## 2. Configuration (`.env`)

Configure the RL plug-in using environment variables:

```ini
# Plugin operating mode: disabled (default), shadow, active
LIBRARY_ROBOT_RL_MODE=disabled

# Path to trained actor bundle directory containing actor.ts and manifest.json
LIBRARY_ROBOT_RL_MODEL_DIR=/path/to/models/library_sac

# Maximum residual correction applied to motor PWM (range: 5 to 15, default: 10)
LIBRARY_ROBOT_RL_MAX_RESIDUAL_PWM=10

# Maximum permitted inference latency in milliseconds (default: 50)
LIBRARY_ROBOT_RL_DEADLINE_MS=50

# Maximum permitted observation age in milliseconds (default: 250)
LIBRARY_ROBOT_RL_TELEMETRY_AGE_MS=250
```

---

## 3. Plug-in Operating Modes

### 3.1 `disabled` (Default)
- Baseline fixed-grid navigation runs without loading or invoking the PyTorch/TorchScript actor.
- Telemetry outputs `rl_status: { mode: "disabled" }`.
- Arduino Mega receives standard closed-loop PID heading corrections.

### 3.2 `shadow`
- Loads `actor.ts` from `LIBRARY_ROBOT_RL_MODEL_DIR`.
- On every control loop iteration during linear motion (`FORWARD` / `BACKWARD`), constructs the 5-dim observation and runs CPU inference.
- Validates model output, checks latency (< 50 ms), and logs recommended PWM corrections to telemetry.
- **Never sends motor corrections to Arduino** (`apply_to_motor = False`). Safe for live validation on real hardware.

### 3.3 `active`
- Runs inference and safety checks identically to `shadow` mode.
- When all safety checks pass (valid sensor data, telemetry age < 250 ms, latency < 50 ms, linear motion phase), sends `SET_RL_CORRECTION:<pwm>` to Arduino Mega over serial.

---

## 4. Hardware & Firmware Integration

- **Single Ultrasonic Sensor**: Connected to Mega pins `TRIG_FRONT = 23`, `ECHO_FRONT = 22`. Reported over serial as `US:<front_cm>`.
- **Arduino Watchdog**: If no `SET_RL_CORRECTION` command is received for 300 ms, the Mega automatically resets `rlCorrection = 0`.
- **Turn Isolation**: `rlCorrection` is forced to `0` during IMU turns, rotation, stop, or non-linear state resets.
- **Odometry Telemetry**: Mega reports 9 fields in `ODOM:<left_ticks>,<right_ticks>,<left_cm>,<right_cm>,<heading_enc>,<heading_imu>,<heading_fused>,<base_corr>,<rl_corr>`.
