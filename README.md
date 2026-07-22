# 📚 Library Robot — Version 1

A Raspberry Pi + Arduino robot that navigates to a book when a user selects it from a web interface. Books are identified using **ArUco markers** detected by the Pi Camera.

---

## How It Works

1. Robot sits at a **fixed docking position**
2. User opens the **web UI** on any device on the same WiFi
3. User selects a book from a **dropdown menu**
4. Robot **rotates** to scan for the correct ArUco marker
5. Once found, it **aligns** and drives **straight toward it**, stopping ~25cm away
6. The web page shows a **live camera feed** and status updates

---

## Hardware

| Component | Details |
|---|---|
| On-board Computer | Raspberry Pi (any model with camera port) |
| Microcontroller | Arduino Mega |
| Motor Shield | MH Electronics (AFMotor library) |
| Camera | Pi Camera Module |
| Distance Sensors | 3× HC-SR04 Ultrasonic |
| Drive | 4× DC Motors (differential drive) |

---

## Project Structure

```
Library_robot/
├── arduino/
│   └── library_robot.ino         ← Arduino firmware
│
├── pi/
│   ├── app.py                    ← Flask web server
│   ├── book_db.py                ← Book → ArUco ID mapping
│   ├── camera.py                 ← Pi Camera & MJPEG stream
│   ├── aruco_detector.py         ← OpenCV ArUco detection
│   ├── robot_controller.py       ← Navigation state machine
│   ├── serial_bridge.py          ← Serial comms to Arduino
│   ├── templates/
│   │   └── index.html            ← Web UI
│   └── static/
│       ├── style.css
│       └── app.js
│
├── aruco_codes/
│   └── generate_markers.py       ← Generates printable ArUco PNGs
│
├── docs/
│   ├── PLAN.md                   ← Full architecture & design plan
│   ├── person1_hardware.md       ← Guide for Person 1 (Hardware)
│   ├── person2_vision.md         ← Guide for Person 2 (Vision)
│   └── person3_webapp.md         ← Guide for Person 3 (Web App)
│
├── requirements.txt
└── README.md
```

---

## Team

| Person | Responsibility |
|---|---|
| **Person 1** | Arduino firmware + Python serial bridge |
| **Person 2** | ArUco detection + Camera + Navigation state machine |
| **Person 3** | Flask web server + Web UI |

See the `docs/` folder for detailed step-by-step guides for each role.

Related implementation reference: [cc-hackers-s-RL-robotics-project](https://github.com/12412825-collab/cc-hackers-s-RL-robotics-project). See `docs/PROJECT_CONTEXT.md` for the agreed reuse boundary.

---

## Setup (Run on Raspberry Pi)

```bash
# 1. Clone the repo
git clone https://github.com/dunja-g/Library_robot.git
cd Library_robot

# 2. Install Python dependencies
pip install -r requirements.txt

# 3. Flash arduino/library_robot.ino to the Arduino via Arduino IDE

# 4. Run the robot
cd pi
python app.py
```

Then open `http://<your-pi-ip-address>:5000` in any browser on the same WiFi network.

### Person 2 Offline Validation

The vision and navigation modules can be tested without Raspberry Pi hardware:

```bash
pip install opencv-contrib-python numpy pytest
python -m pytest -q
python -m aruco_codes.generate_markers
```

`Camera` accepts an injected backend, so an existing Raspberry Pi camera or baseline-model capture pipeline can be reused as long as it returns BGR NumPy frames.

---

## Dependencies

```
flask
opencv-contrib-python
picamera2
pyserial
numpy
```

> **Note:** `opencv-contrib-python` is required (NOT `opencv-python`) — the ArUco module only ships in the contrib version.
