"""
app.py — Flask web server for the Library Robot.

Run with:   python app.py
Access at:  http://<pi-ip>:5000  (or http://localhost:5000 for local testing)

MockController is used when hardware (camera / Arduino) is unavailable.
Switch to RealController once Person 1 & 2 have finished their modules.
"""

import threading
import time
import logging
from flask import Flask, render_template, Response, jsonify, request

from book_db import get_aruco_id, get_all_books

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)

# ─────────────────────────────────────────────────────────────────────────────
# MockController — lets Person 3 develop & test the UI without any hardware.
# Replace this block with the real imports once teammates are done.
# ─────────────────────────────────────────────────────────────────────────────
USE_MOCK = True   # ← Set to False on the real Raspberry Pi

if USE_MOCK:
    import os
    import cv2
    import numpy as np

    class MockController:
        """Simulates the robot state machine for UI development."""

        STATES = ["IDLE", "SCANNING", "ALIGNING", "APPROACHING", "ARRIVED"]

        def __init__(self):
            self._state = "IDLE"
            self._target_id = None
            self._sim_step = 0

        def request_book(self, aruco_id: int):
            logger.info(f"[MOCK] Book requested — ArUco ID {aruco_id}")
            self._target_id = aruco_id
            self._state = "SCANNING"
            self._sim_step = 0

        def get_state(self) -> str:
            return self._state

        def reset(self):
            logger.info("[MOCK] Reset")
            self._state = "IDLE"
            self._target_id = None
            self._sim_step = 0

        def step(self):
            """Automatically advances through states to simulate the robot."""
            if self._state == "IDLE":
                return
            self._sim_step += 1
            if self._state == "SCANNING"   and self._sim_step > 25:
                self._state = "ALIGNING";   self._sim_step = 0
            elif self._state == "ALIGNING" and self._sim_step > 15:
                self._state = "APPROACHING"; self._sim_step = 0
            elif self._state == "APPROACHING" and self._sim_step > 30:
                self._state = "ARRIVED";    self._sim_step = 0

    class MockCamera:
        """Returns a generated placeholder frame."""

        COLOURS = {
            "IDLE":        (60, 60, 60),
            "SCANNING":    (30, 130, 220),
            "ALIGNING":    (220, 160, 30),
            "APPROACHING": (40, 180, 80),
            "ARRIVED":     (20, 140, 20),
            "STOPPED":     (50, 50, 200),
        }

        def generate_mjpeg(self, get_state_fn):
            while True:
                state = get_state_fn()
                colour = self.COLOURS.get(state, (60, 60, 60))
                frame = np.full((480, 640, 3), colour, dtype=np.uint8)

                # Draw mock ArUco square in centre
                cv2.rectangle(frame, (250, 175), (390, 305), (255, 255, 255), 3)
                cv2.rectangle(frame, (260, 185), (380, 295), (0, 0, 0), -1)
                cv2.rectangle(frame, (280, 205), (360, 275), (255, 255, 255), -1)

                # Overlay state text
                cv2.putText(frame, f"State: {state}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)
                cv2.putText(frame, "[MOCK CAMERA]", (20, 460),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

                _, jpeg = cv2.imencode('.jpg', frame)
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n'
                       + jpeg.tobytes() + b'\r\n')
                time.sleep(0.05)  # ~20 fps

    controller = MockController()
    camera     = MockCamera()

else:
    # ── Real hardware imports (uncomment when Person 1 & 2 are done) ──────────
    from serial_bridge    import SerialBridge
    from camera           import Camera
    from aruco_detector   import ArucoDetector
    from robot_controller import RobotController

    serial_bridge = SerialBridge(port='/dev/ttyACM0')
    camera        = Camera()
    detector      = ArucoDetector()
    controller    = RobotController(serial_bridge, camera, detector)

# ─────────────────────────────────────────────────────────────────────────────
# Background control loop — calls controller.step() at ~10 Hz
# ─────────────────────────────────────────────────────────────────────────────
def _control_loop():
    while True:
        controller.step()
        time.sleep(0.1)

threading.Thread(target=_control_loop, daemon=True).start()

# ─────────────────────────────────────────────────────────────────────────────
# Routes
# ─────────────────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    if USE_MOCK:
        stream = camera.generate_mjpeg(controller.get_state)
    else:
        stream = camera.generate_mjpeg()
    return Response(stream, mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/books')
def books():
    return jsonify(get_all_books())


@app.route('/request_book', methods=['POST'])
def request_book():
    data  = request.get_json(silent=True) or {}
    title = data.get('title', '').strip()

    if not title:
        return jsonify({'status': 'error', 'message': 'No book title provided'}), 400

    aruco_id = get_aruco_id(title)
    if aruco_id is None:
        return jsonify({'status': 'error', 'message': f'Book not found: {title}'}), 404

    controller.request_book(aruco_id)
    logger.info(f"Book requested: '{title}' → ArUco ID {aruco_id}")
    return jsonify({'status': 'ok', 'title': title, 'aruco_id': aruco_id})


@app.route('/status')
def status():
    return jsonify({'state': controller.get_state()})


@app.route('/reset', methods=['POST'])
def reset():
    controller.reset()
    logger.info("Robot reset to IDLE")
    return jsonify({'status': 'ok'})


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    # host='0.0.0.0' makes the server reachable from other devices on the WiFi
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
