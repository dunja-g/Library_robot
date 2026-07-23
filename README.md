# Library Robot — Multi-Waypoint MVP

A Raspberry Pi 5 and Arduino Mega robot that guides a user to a numbered
fixed-grid book location using wheel encoders, an MPU6500, and ultrasonic
fail-safe stopping. The current application does not load marker scanning.
Legacy vision modules remain only for historical tests.

The default `grid` mode supports a marker-free fixed
1A-4B layout. It generates routes from measured dimensions and executes them
with left/right wheel encoders. See
[docs/FIXED_GRID_ENCODER.md](docs/FIXED_GRID_ENCODER.md).

## Current demonstration

The catalogue contains **Deep Learning** at `1A-L3-P21`: box 1A, layer 3,
position 21. The base generates a distance-and-turn route to box 1A, announces
the precise book location, then returns automatically. No marker is required.

## Software flow

`IDLE → MOVING → TURNING → MOVING → ARRIVED → RETURNING → DOCKED`

Any invalid ultrasonic reading, obstacle, timeout, or controller exception sends a motor stop and enters `STOPPED`. `/reset` stops the motors and clears the mission; it does not physically drive the robot back to Dock.

## Project structure

```text
arduino/library_robot.ino       Arduino motor and HC-SR04 firmware
pi/app.py                      Flask API, UI server, and Mock mode
pi/book_db.py                  Book and physical shelf metadata
pi/route_db.py                 Marker catalogue and route/turn configuration
pi/mission.py                  Outbound/return mission progress
pi/grid_layout.py              Parameterised 1A-4B geometry and route generator
pi/encoder_navigation.py       Encoder motion state machine
pi/robot_controller.py         Non-blocking navigation state machine
pi/aruco_detector.py           Legacy vision module (not loaded by current app)
pi/camera.py                   Picamera2 capture and MJPEG stream
pi/serial_bridge.py            Raspberry Pi ↔ Arduino serial protocol
pi/navigation_config.py        Environment-based tuning
tests/                         Hardware-free unit and integration tests
docs/MULTI_WAYPOINT_MVP.md     Placement, tuning, safety, and runbook
docs/HARDWARE_PINOUT.md        Final Mega, encoder, IMU, and sensor wiring
docs/BOOK_NUMBERING.md         1A-L3-P21 book-location numbering convention
```

## Run without hardware

```bash
pip install -r requirements.txt
pip install -r requirements-dev.txt
set LIBRARY_ROBOT_USE_MOCK=true
python -m pi.app
```

On PowerShell, use `$env:LIBRARY_ROBOT_USE_MOCK="true"`. Open `http://localhost:5000`, select **Deep Learning**, and watch the complete outbound and return simulation.

## Run on Raspberry Pi

1. Flash `arduino/library_robot.ino` to the Mega.
2. Wire and test all three HC-SR04 sensors and the differential drive.
3. Install dependencies and enable the Pi camera.
4. Copy `.env.example` values into the service environment and set `LIBRARY_ROBOT_USE_MOCK=false`.
5. From the repository root, run `python -m pi.app` and open `http://<pi-ip>:5000`.

Run offline verification with:

```bash
python -m pytest -q
```

See [docs/MULTI_WAYPOINT_MVP.md](docs/MULTI_WAYPOINT_MVP.md) before the first powered route trial.
