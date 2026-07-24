# Library Robot — Borrowing-Only Fixed-Grid MVP

A Raspberry Pi 5 and Arduino Mega robot that helps an identified student
borrow a numbered book from a fixed six-box `1A`–`3B` layout. Navigation is
marker-free: wheel encoders measure distance, the MPU6500 measures continuous
heading, complementary fusion corrects left/right wheel speed, and three
front/side ultrasonic sensors provide fail-safe stopping.

This runtime does not use reinforcement learning, face recognition, SLAM, or
ArUco route markers. Legacy vision files remain only for historical tests.

## End-to-end borrowing flow

1. While the robot is `IDLE` or `DOCKED`, scan a QR student card.
2. Search for a book and create a `pending` borrowing mission.
3. The robot travels from Dock to the selected `1A`–`3B` box (3 rows × 2 columns, box size 35×25 cm, 5-10 cm gap).
4. At `ARRIVED`, the UI shows the exact layer and position.
5. The student takes the book and confirms pickup.
6. Only then is the book recorded as borrowed.
7. The robot reverses out, aligns with the aisle, and reverses to Dock.
8. After controller state `DOCKED` and phase `COMPLETE`, the session clears.

A pending mission is cancelled without a database write after an obstacle,
encoder stall, IMU failure, serial failure, timeout, reset, or other safety
stop. Duplicate missions are rejected.

If a confirmed mission encounters a failure during reverse return, the book loan is retained, the system flags `RECOVERY_REQUIRED`, and an operator resets the robot via **"Robot Has Been Returned to Dock"** (`/api/operator_reposition`).

## Reverse-return safety & sensor protections

The physical scene does not provide enough space for a destination U-turn, so
the configured return route uses `BACKWARD`. All ultrasonic readings must
remain valid and the left/right sensors remain active, but there is no
rear-facing sensor. The reverse corridor must therefore be cleared before
dispatch and supervised during every physical run.

Safety protections include:
- Sensor disagreement check (`LIBRARY_ROBOT_SENSOR_DISAGREEMENT_DEG`, default 25.0°)
- Reverse segment timeout (`LIBRARY_ROBOT_REVERSE_SEGMENT_TIMEOUT_SECONDS`, default 15.0s)

## Project structure

```text
arduino/library_robot.ino       Mega motor, encoder, IMU and HC-SR04 firmware
pi/app.py                       Flask API and borrowing transaction orchestration
pi/borrowing_mission.py         pending/confirmed/cancelled mission state
pi/student_db.py                atomic student loan database
pi/book_db.py                   book and physical shelf metadata
pi/grid_layout.py               parameterised 1A-3B route generation
pi/encoder_navigation.py        non-blocking fixed-grid controller
pi/qr_scanner.py                student-card QR scanner
pi/camera.py                    shared Picamera2 capture and MJPEG stream
pi/serial_bridge.py             Raspberry Pi to Arduino serial protocol
tools/calibrate_fused_navigation.py  interactive navigation auto-calibration CLI
tests/                          hardware-free unit and integration tests
docs/FIXED_GRID_ENCODER.md      calibration and reverse-return safety
docs/LIVE_DEMO_RUNBOOK.md       physical demonstration procedure
```


## Run without hardware

```powershell
pip install -r requirements.txt
pip install -r requirements-dev.txt
$env:LIBRARY_ROBOT_USE_MOCK="true"
python -m pi.app
```

Open `http://localhost:5000`, check in with `LIBSTU-S001`, select a book,
dispatch the mission, and confirm pickup after the UI reaches `ARRIVED`.

## Run on Raspberry Pi

1. Flash `arduino/library_robot.ino` to the Mega.
2. Verify the wiring in `docs/HARDWARE_PINOUT.md`.
3. Measure and configure all grid and encoder values.
4. Set `LIBRARY_ROBOT_USE_MOCK=false`.
5. Run `python -m pi.app`.
6. Open `http://<pi-ip>:5000`.

Before powering the motors, read `docs/LIVE_DEMO_RUNBOOK.md`.

Run all software checks with:

```bash
python -m pytest -q
```
