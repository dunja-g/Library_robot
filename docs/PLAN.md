# Library Robot V1 — Full Architecture & Design Plan

## Overview

A Raspberry Pi-powered 4-wheel robot that sits at a fixed docking position. A user on the same WiFi network opens a web page, selects one of 5 books from a dropdown, and the robot:

1. **Scans** — rotates in place to find the selected book's ArUco marker
2. **Aligns** — centres the marker in the camera frame
3. **Approaches** — drives straight toward the marker, stopping ~25cm away
4. **Safety** — 3 ultrasonic sensors emergency-stop the robot if an obstacle is detected

---

## System Architecture

```
[User Browser]
     │  HTTP / MJPEG stream
     ▼
[Flask Server — app.py]
     │
     ├──► [book_db.py]          — Maps book title → ArUco ID
     ├──► [robot_controller.py] — Navigation state machine
     │         │
     │         ├──► [camera.py]         — Pi Camera capture
     │         ├──► [aruco_detector.py] — OpenCV ArUco detection
     │         └──► [serial_bridge.py]  — UART to Arduino
     │
     └──► [Arduino — library_robot.ino]
               ├── MH Electronics Motor Shield (4× DC motors)
               └── 3× HC-SR04 Ultrasonic Sensors
```

---

## Navigation State Machine

```
IDLE
  │  (user selects book)
  ▼
SCANNING ──────────────────────────────────► SCANNING
  │  (target ArUco ID detected)       (no marker → keep rotating)
  ▼
ALIGNING ─────────────────────────────────► SCANNING
  │  (marker centred ±30px)           (marker lost → re-scan)
  ▼
APPROACHING
  │  (ultrasonic centre ≤ 25cm)
  │                    └── (obstacle on any sensor) ──► STOPPED
  ▼
ARRIVED
  │  (user resets)
  ▼
IDLE
```

---

## ArUco Marker Plan

| Book Title | ArUco ID | Dictionary |
|---|---|---|
| Book 1 | 0 | DICT_5X5_50 |
| Book 2 | 1 | DICT_5X5_50 |
| Book 3 | 2 | DICT_5X5_50 |
| Book 4 | 3 | DICT_5X5_50 |
| Book 5 | 4 | DICT_5X5_50 |

Print each marker at **10×10 cm** for reliable detection at 0.5–2m range.
To generate the marker images, run: `python aruco_codes/generate_markers.py`

---

## Serial Command Protocol (Pi ↔ Arduino)

This is the agreed interface between Person 1 (hardware) and Person 2 (vision/navigation).

### Pi → Arduino Commands

| Command | Meaning |
|---|---|
| `ROTATE_LEFT\n` | Spin left in place (slow, for scanning) |
| `ROTATE_RIGHT\n` | Spin right in place (slow, for scanning) |
| `FORWARD\n` | Drive forward at moderate speed |
| `STOP\n` | Stop all motors immediately |
| `CHECK\n` | Request ultrasonic sensor readings |

### Arduino → Pi Response (to CHECK)

```
US:left_cm,center_cm,right_cm\n
```

Example: `US:43.2,18.7,55.1`

### Safety Rule
The Arduino will auto-stop all motors if no command is received for **2 seconds** (safety timeout).

---

## Team Responsibilities

| Person | Files | Summary |
|---|---|---|
| **Person 1** | `arduino/library_robot.ino`, `pi/serial_bridge.py` | Hardware firmware + serial bridge |
| **Person 2** | `pi/camera.py`, `pi/aruco_detector.py`, `pi/robot_controller.py`, `aruco_codes/generate_markers.py` | Vision + navigation |
| **Person 3** | `pi/app.py`, `pi/book_db.py`, `pi/templates/index.html`, `pi/static/*` | Web server + UI |

See individual guides in `docs/person1_hardware.md`, `docs/person2_vision.md`, `docs/person3_webapp.md`.

---

## Integration Checklist (Final Test)

- [ ] Print all 5 ArUco markers and attach to standing books
- [ ] Flash Arduino firmware and verify motors spin correctly
- [ ] Start Flask server on Pi: `python pi/app.py`
- [ ] Open `http://<pi-ip>:5000` on a phone — camera feed visible
- [ ] Select a book, click "Send Robot" — robot rotates, finds marker, approaches, stops at ~25cm
- [ ] Web UI shows correct status at each step
- [ ] Place hand in front of robot mid-approach — robot stops (ultrasonic safety)
