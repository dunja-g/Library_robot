"""
app.py — Flask web server for the Library Robot.

Run with:   python app.py
Access at:  http://<pi-ip>:5000  (or http://localhost:5000 for local testing)

MockController is used when hardware (camera / Arduino) is unavailable.
Switch to RealController once Person 1 & 2 have finished their modules.
"""

import atexit
import os
import threading
import time
import logging
from flask import Flask, render_template, Response, jsonify, request

try:
    from .book_db import get_aruco_id, get_all_books
except ImportError:  # Supports running directly with ``python pi/app.py``.
    from book_db import get_aruco_id, get_all_books

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)

# ─────────────────────────────────────────────────────────────────────────────
# MockController — lets Person 3 develop & test the UI without any hardware.
# Replace this block with the real imports once teammates are done.
# ─────────────────────────────────────────────────────────────────────────────
USE_MOCK = os.getenv("LIBRARY_ROBOT_USE_MOCK", "true").lower() in {
    "1", "true", "yes", "on"
}

if USE_MOCK:
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
    try:
        from .serial_bridge import SerialBridge
        from .camera import Camera
        from .aruco_detector import ArucoDetector
        from .robot_controller import RobotController
        from .navigation_config import NavigationConfig
    except ImportError:
        from serial_bridge import SerialBridge
        from camera import Camera
        from aruco_detector import ArucoDetector
        from robot_controller import RobotController
        from navigation_config import NavigationConfig

    config = NavigationConfig.from_env()
    serial_bridge = SerialBridge(
        port=os.getenv("LIBRARY_ROBOT_SERIAL_PORT", "/dev/ttyACM0")
    )
    camera = Camera(
        width=config.camera_width,
        height=config.camera_height,
        fps=config.camera_fps,
    )
    detector = ArucoDetector(min_area_px=config.min_marker_area_px)
    controller = RobotController(
        serial_bridge,
        camera,
        detector,
        frame_width=config.camera_width,
        align_tolerance_px=config.align_tolerance_px,
        stop_distance_cm=config.stop_distance_cm,
        obstacle_distance_cm=config.obstacle_distance_cm,
        scan_timeout_seconds=config.scan_timeout_seconds,
        target_confirmation_frames=config.target_confirmation_frames,
        alignment_confirmation_frames=config.alignment_confirmation_frames,
        target_loss_tolerance_frames=config.target_loss_tolerance_frames,
    )

    def _shutdown_hardware():
        try:
            controller.reset()
        finally:
            camera.stop()
            serial_bridge.close()

    atexit.register(_shutdown_hardware)

# ─────────────────────────────────────────────────────────────────────────────
# Background control loop — calls controller.step() at ~10 Hz
# ─────────────────────────────────────────────────────────────────────────────
def _control_loop():
    while True:
        try:
            controller.step()
        except Exception:
            logger.exception("Robot control loop failed; controller stopped safely")
        interval = 0.1 if USE_MOCK else 1.0 / config.control_hz
        time.sleep(interval)

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
        stream = camera.generate_mjpeg(controller.get_latest_frame)
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
    if hasattr(controller, "get_status"):
        return jsonify(controller.get_status())
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
