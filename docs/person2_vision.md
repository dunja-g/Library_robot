# Person 2 — Computer Vision & Navigation Guide

> Historical document: QR is now used only for student identification.
> Navigation uses encoders and MPU6500 without ArUco route markers.

## Your Role
You are responsible for everything that involves the camera: generating the ArUco markers to print, detecting them in real time from the Pi Camera, and the core navigation logic that decides how the robot moves.

**Your files:**
- `aruco_codes/generate_markers.py`
- `pi/camera.py`
- `pi/aruco_detector.py`
- `pi/robot_controller.py`

---

## What You Need

### Hardware
- Raspberry Pi with Pi Camera Module connected
- Robot physically assembled with Person 1's Arduino running

### Software (install on Raspberry Pi)
```bash
pip install opencv-contrib-python picamera2 numpy
```

> ⚠️ You **must** install `opencv-contrib-python`, NOT `opencv-python`. The ArUco module only exists in the contrib package.

---

## Step 1 — Generate the ArUco Markers

Write `aruco_codes/generate_markers.py`. This script generates 5 printable PNG images, one per book.

**What it must do:**
- Use dictionary `cv2.aruco.DICT_5X5_50`
- Generate markers for IDs 0, 1, 2, 3, 4
- Save each as `aruco_codes/marker_<ID>.png` at 300×300 pixels with a white border
- Print a confirmation message for each saved file

**Key OpenCV functions:**
```python
import cv2
import numpy as np

# Load the dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

# Generate one marker image (marker ID=0, image size 300x300)
marker_image = cv2.aruco.generateImageMarker(aruco_dict, 0, 300)

# Save it
cv2.imwrite("aruco_codes/marker_0.png", marker_image)
```

**After generating:** Print each marker image at **10×10 cm** on paper. Attach one to each of the 5 standing books, facing the robot.

✅ **Pass criteria:** 5 PNG files appear in `aruco_codes/`. When you hold one up in front of any camera, you can clearly see the black-and-white square pattern.

---

## Step 2 — Write `pi/camera.py`

This module manages the Pi Camera and exposes two things: a way to grab a single frame, and a way to stream MJPEG video (used by the web UI).

**Required interface:**

```python
class Camera:
    def __init__(self, width=640, height=480, fps=20):
        # Initialise picamera2
        # Configure: width, height, fps
        # Start the camera
        pass

    def get_frame(self):
        # Capture a single BGR numpy array (OpenCV format)
        # Returns: numpy array of shape (height, width, 3)
        pass

    def generate_mjpeg(self):
        # Generator function for MJPEG streaming
        # Yields: bytes in multipart/x-mixed-replace format
        # Used by Flask's video_feed route
        # Each iteration: get_frame() → encode to JPEG → yield
        pass

    def stop(self):
        # Stop and close the camera
        pass
```

**MJPEG yield format** (this is what Flask needs):
```python
ret, jpeg = cv2.imencode('.jpg', frame)
yield (b'--frame\r\n'
       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
```

**Test it standalone** on the Pi:
```python
# In pi/camera.py, add at the bottom:
if __name__ == '__main__':
    cam = Camera()
    frame = cam.get_frame()
    cv2.imwrite('test_frame.jpg', frame)
    print(f"Frame captured: {frame.shape}")
    cam.stop()
```
Run `python pi/camera.py` → a `test_frame.jpg` should appear.

✅ **Pass criteria:** `test_frame.jpg` contains a real image from the Pi Camera.

---

## Step 3 — Write `pi/aruco_detector.py`

This module takes a camera frame and detects any ArUco markers in it, returning their IDs and positions.

**Required interface:**

```python
class ArucoDetector:
    def __init__(self):
        # Load DICT_5X5_50
        # Create detector parameters (cv2.aruco.DetectorParameters)
        # Create the ArucoDetector object
        pass

    def detect(self, frame):
        # Takes a BGR numpy array (frame from camera)
        # Returns a list of detections, each as a dict:
        # [
        #   {
        #     "id": 2,
        #     "center_x": 320,   # pixel x of marker centre
        #     "center_y": 240,   # pixel y of marker centre
        #     "area": 8500,      # area of bounding box in pixels² (bigger = closer)
        #     "corners": ...     # raw corner array
        #   },
        #   ...
        # ]
        # Returns empty list [] if no markers detected
        pass

    def draw(self, frame, detections):
        # Draw bounding boxes and IDs onto the frame
        # Returns the annotated frame
        # (Used to overlay info on the live camera stream)
        pass
```

**Key OpenCV functions:**
```python
import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

corners, ids, rejected = detector.detectMarkers(frame)

# ids is None if nothing found, otherwise shape (N, 1)
# corners is a list of (1, 4, 2) arrays — one per detected marker
# Centre of marker 0:
cx = int(corners[0][0][:, 0].mean())
cy = int(corners[0][0][:, 1].mean())
```

**Test it standalone:**
```python
# In pi/aruco_detector.py, add at the bottom:
if __name__ == '__main__':
    from camera import Camera
    cam = Camera()
    det = ArucoDetector()

    # Hold a printed marker in front of the camera, then run
    frame = cam.get_frame()
    results = det.detect(frame)
    print(f"Detected: {results}")
    cam.stop()
```

✅ **Pass criteria:** When a printed marker is held in front of the camera, the script prints the correct ID and a centre position.

---

## Step 4 — Write `pi/robot_controller.py`

This is the brain of the robot. It implements the state machine that controls scanning, aligning, and approaching.

**Dependencies:** `camera.py`, `aruco_detector.py`, and Person 1's `serial_bridge.py`.

**State machine:**
```
IDLE → SCANNING → ALIGNING → APPROACHING → ARRIVED
                                    └──────► STOPPED (obstacle)
```

**Required interface:**

```python
from enum import Enum

class State(Enum):
    IDLE = "IDLE"
    SCANNING = "SCANNING"
    ALIGNING = "ALIGNING"
    APPROACHING = "APPROACHING"
    ARRIVED = "ARRIVED"
    STOPPED = "STOPPED"

class RobotController:
    FRAME_WIDTH = 640           # must match camera width
    ALIGN_TOLERANCE_PX = 30     # how centred the marker must be
    STOP_DISTANCE_CM = 25       # stop when ultrasonic reads ≤ this
    OBSTACLE_DISTANCE_CM = 20   # emergency stop threshold

    def __init__(self, serial_bridge, camera, aruco_detector):
        self.serial = serial_bridge
        self.camera = camera
        self.detector = aruco_detector
        self.state = State.IDLE
        self.target_id = None   # ArUco ID we are looking for

    def request_book(self, aruco_id):
        # Called by app.py when user selects a book
        # Sets target_id and transitions to SCANNING
        pass

    def get_state(self):
        # Returns current state as a string
        return self.state.value

    def reset(self):
        # Stop robot, return to IDLE
        pass

    def step(self):
        # Called repeatedly in a background thread (~10 Hz)
        # Implements the state machine logic
        # Returns: annotated frame (numpy array) for the live stream
        pass

    def _do_scanning(self, frame):
        # Detect all markers in frame
        # If target found → transition to ALIGNING
        # If not found → send ROTATE_LEFT (or alternate direction)
        pass

    def _do_aligning(self, frame):
        # Detect target marker
        # If not found → back to SCANNING
        # Calculate horizontal error: error = center_x - (FRAME_WIDTH / 2)
        # If abs(error) > ALIGN_TOLERANCE_PX:
        #   error > 0 → marker is right → ROTATE_RIGHT
        #   error < 0 → marker is left → ROTATE_LEFT
        # If abs(error) ≤ ALIGN_TOLERANCE_PX → marker centred → APPROACHING
        pass

    def _do_approaching(self, frame):
        # Check ultrasonics first (safety)
        # If any sensor < OBSTACLE_DISTANCE_CM → STOP → state = STOPPED
        # Check centre ultrasonic ≤ STOP_DISTANCE_CM → state = ARRIVED
        # Otherwise: send FORWARD
        # Also re-check marker is still visible; if not → back to ALIGNING
        pass
```

**Threading note:** The `step()` method should be called from a background thread in `app.py`. Person 3 will handle that. You just need to make sure `step()` is thread-safe (use a `threading.Lock()` around state changes).

**Alignment logic diagram:**
```
Camera frame (640px wide):
[    |      MARKER      |    ]
 0                          639
         centre = 320

error = marker_centre_x - 320

error < -30 → too far LEFT  → ROTATE_LEFT to correct
error > +30 → too far RIGHT → ROTATE_RIGHT to correct
-30 ≤ error ≤ +30 → ALIGNED → start approach
```

✅ **Pass criteria (dry run — no hardware needed):**
- Create a `RobotController` with mock serial bridge and a static test image
- Call `request_book(1)` → state becomes SCANNING
- Feed a frame with ArUco ID 1 centred → state becomes ALIGNING then APPROACHING

---

## Step 5 — Integration Notes

When you hand off to Person 3:
- Confirm that `RobotController` is importable: `from robot_controller import RobotController, State`
- Confirm the `step()` method returns an **annotated BGR numpy array** (this is what Person 3 will encode and stream)
- Tell Person 3 how to start the controller: `controller.request_book(aruco_id)` and how to get status: `controller.get_state()`

---

## Common Issues

| Problem | Fix |
|---|---|
| `No module named 'cv2.aruco'` | You installed `opencv-python` instead of `opencv-contrib-python`. Uninstall and reinstall: `pip uninstall opencv-python; pip install opencv-contrib-python` |
| Camera not found | Check the flat ribbon cable is fully seated. Enable camera in `raspi-config` |
| Markers detected but IDs are wrong | Make sure you are using the same dictionary (`DICT_5X5_50`) everywhere |
| Robot overshoots when aligning | Reduce rotation speed in the Arduino firmware (ask Person 1) |
