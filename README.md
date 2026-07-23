# Library Robot — Multi-Waypoint MVP

A Raspberry Pi and Arduino Mega robot that guides a user to a selected book by following a configured sequence of OpenCV ArUco markers. Navigation is a deterministic Python state machine with ultrasonic fail-safe stopping.

The project now also includes an optional marker-free `grid` mode for a fixed
1A-4B layout. It generates routes from measured dimensions and executes them
with left/right wheel encoders. See
[docs/FIXED_GRID_ENCODER.md](docs/FIXED_GRID_ENCODER.md).

## Current demonstration

The catalogue contains **Deep Learning** at zone B, shelf B3, level 3, slot 12.

- Outbound: `101 → 105 → 203`
- Destination: marker `203` at shelf B3
- Return: `105 → 101 → 0`
- Dock: marker `0`

At each waypoint the robot scans only for the expected marker, aligns, approaches it, validates the marker and ultrasonic distance together, then executes the configured timed turn. After reaching the shelf it waits briefly and returns automatically.

## Software flow

`IDLE → SCANNING → ALIGNING → APPROACHING → TURNING → … → ARRIVED → RETURNING → DOCKED`

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
pi/aruco_detector.py           OpenCV ArUco detection
pi/camera.py                   Picamera2 capture and MJPEG stream
pi/serial_bridge.py            Raspberry Pi ↔ Arduino serial protocol
pi/navigation_config.py        Environment-based tuning
tests/                         Hardware-free unit and integration tests
docs/MULTI_WAYPOINT_MVP.md     Placement, tuning, safety, and runbook
docs/HARDWARE_PINOUT.md        Final Mega, encoder, IMU, and sensor wiring
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
