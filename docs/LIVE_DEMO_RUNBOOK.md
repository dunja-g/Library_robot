# Borrowing-Only Fixed-Grid Live Demo

The current runtime has one navigation mode: marker-free fixed-grid
navigation. Do not place or configure ArUco route markers.

## 1. Prepare the scene

- Place the six boxes as `1A`–`3A` on the left and `1B`–`3B` on the right.
- Put the robot in its repeatable Dock guide, facing row 1.
- Clear the centre aisle and the complete reverse return corridor.
- Keep one operator beside the emergency power control.
- Because there is no rear ultrasonic sensor, nobody may enter behind the
  robot during return.

## 2. Configure the Raspberry Pi

Create a local `.env`:

```env
LIBRARY_ROBOT_USE_MOCK=false
LIBRARY_ROBOT_SERIAL_PORT=/dev/ttyACM0
LIBRARY_ROBOT_CAMERA_WIDTH=640
LIBRARY_ROBOT_CAMERA_HEIGHT=480
LIBRARY_ROBOT_CAMERA_FPS=20
LIBRARY_ROBOT_CONTROL_HZ=10
LIBRARY_ROBOT_OBSTACLE_DISTANCE_CM=20
LIBRARY_ROBOT_MISSION_TIMEOUT_SECONDS=300

LIBRARY_ROBOT_GRID_FIRST_ROW_CM=
LIBRARY_ROBOT_GRID_ROW_SPACING_CM=
LIBRARY_ROBOT_GRID_APPROACH_CM=
LIBRARY_ROBOT_ENCODER_TICKS_PER_CM=
LIBRARY_ROBOT_ENCODER_TICKS_PER_REV=4
LIBRARY_ROBOT_WHEEL_DIAMETER_CM=6.5
LIBRARY_ROBOT_ENCODER_STALL_SECONDS=2
LIBRARY_ROBOT_GRID_TURN_SOURCE=imu
LIBRARY_ROBOT_GRID_LINEAR_SOURCE=encoder
LIBRARY_ROBOT_AUTO_RETURN=true
LIBRARY_ROBOT_LEFT_TICKS_PER_CM=0.195883
LIBRARY_ROBOT_RIGHT_TICKS_PER_CM=0.195883
LIBRARY_ROBOT_WHEEL_TRACK_CM=18.0
LIBRARY_ROBOT_FUSION_ALPHA=0.95
LIBRARY_ROBOT_HEADING_KP=1.5
LIBRARY_ROBOT_MAX_HEADING_CORRECTION=30
```

Do not start a powered route while a required measurement is blank.

## 3. Preflight

```bash
python -m pytest -q
python -m pi.serial_diagnostics --port /dev/ttyACM0
```

Confirm:

- both encoders increase and reset;
- fused heading remains near zero during straight travel;
- MPU6500 turns finish with `DONE`;
- all three ultrasonic readings are finite;
- disconnecting a sensor produces a stop;
- the camera can decode a student QR code.

## 4. Borrowing demonstration

1. Start `python -m pi.app`.
2. Open `http://<pi-ip>:5000`.
3. Scan a student card only while the robot is at Dock.
4. Select a book and dispatch.
5. At `ARRIVED`, read the displayed box, layer and position.
6. Take the book and press **I Took the Book**.
7. Keep the reverse corridor clear while the robot returns.
8. Confirm the UI reaches `DOCKED` and returns to student check-in.

If the robot enters `STOPPED`, do not assume its physical pose. Inspect the
reason, move it manually back to Dock, then reset.
