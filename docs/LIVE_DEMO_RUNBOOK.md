# Live Demo Setup & Runbook

This guide contains the exact, final steps for the robotics team to set up and run the live physical demo on the Raspberry Pi.

## 1. Choose Your Navigation Mode

The robot now supports **two completely different navigation systems**. You need to decide which one to demo right now:

### Option A: Grid Mode (Encoder & IMU) - No ArUco markers needed
This mode uses wheel encoders and the IMU gyro to navigate a strict 2-column by 4-row box grid.
* **ArUco Setup:** None! You do not need to place any ArUco markers anywhere for this mode.
* **Physical Setup:** Place the robot exactly at the "Dock" position, facing perfectly straight down the center aisle towards Row 1.

### Option B: ArUco Mode (Multi-Waypoint) - Camera-based
This mode uses the camera to follow ArUco markers like GPS waypoints. 
* **ArUco Setup:** You **MUST** tape the printed markers to the floor/walls at the exact waypoint locations before starting:
  * `Marker 101` at the Main Corridor.
  * `Marker 105` at the Zone B Junction.
  * `Marker 203` at Shelf B3.
  * `Marker 0` at the Dock.
* **Important:** Each marker must face the direction the robot will be coming from when it looks for it.

---

## 2. Raspberry Pi Setup (For the Robotics Teammate)

On the Raspberry Pi terminal, make sure you have the latest code and dependencies:
```bash
cd Library_robot
git pull origin main
pip install flask opencv-contrib-python picamera2 pyserial numpy
```

---

## 3. Configure the `.env` File on the Pi

The `.env` file is ignored by Git by default to prevent overriding each other's local settings. 
**You must create a `.env` file directly on the Raspberry Pi** in the `Library_robot` folder.

If you are demoing **Grid Mode**, create `.env` with these exact measured values:

```env
LIBRARY_ROBOT_USE_MOCK=false
LIBRARY_ROBOT_SERIAL_PORT=/dev/ttyACM0
LIBRARY_ROBOT_NAVIGATION_MODE=grid

LIBRARY_ROBOT_GRID_FIRST_ROW_CM=25
LIBRARY_ROBOT_GRID_ROW_SPACING_CM=22.5
LIBRARY_ROBOT_GRID_APPROACH_CM=13
LIBRARY_ROBOT_WHEEL_DIAMETER_CM=6.5
LIBRARY_ROBOT_ENCODER_TICKS_PER_REV=4
LIBRARY_ROBOT_GRID_TURN_SOURCE=imu
```

If you are demoing **ArUco Mode**, create `.env` with:

```env
LIBRARY_ROBOT_USE_MOCK=false
LIBRARY_ROBOT_SERIAL_PORT=/dev/ttyACM0
LIBRARY_ROBOT_NAVIGATION_MODE=aruco
```

---

## 4. Run the Demo

With the Arduino connected via USB, start the server on the Pi:
```bash
python pi/app.py
```

Then, anyone on the same WiFi network can open the web UI on their phone:
`http://<RASPBERRY_PI_IP_ADDRESS>:5000`

Select a destination and hit **Send Robot**!
