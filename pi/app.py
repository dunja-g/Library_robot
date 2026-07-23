"""Flask control panel for deterministic multi-waypoint library navigation."""

import atexit
import logging
import os
import threading
import time

from flask import Flask, Response, jsonify, render_template, request

try:
    from .book_db import get_all_books, get_book
    from .encoder_navigation import GridController
    from .grid_layout import (
        BOX_IDS,
        EncoderCalibration,
        GridGeometry,
        build_grid_route,
    )
    from .mission import Mission, MissionPhase
    from .route_db import get_route
except ImportError:  # Supports ``python pi/app.py``.
    from book_db import get_all_books, get_book
    from encoder_navigation import GridController
    from grid_layout import BOX_IDS, EncoderCalibration, GridGeometry, build_grid_route
    from mission import Mission, MissionPhase
    from route_db import get_route

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
app = Flask(__name__)

USE_MOCK = os.getenv("LIBRARY_ROBOT_USE_MOCK", "true").lower() in {
    "1", "true", "yes", "on"
}
NAVIGATION_MODE = os.getenv("LIBRARY_ROBOT_NAVIGATION_MODE", "aruco").strip().lower()
if NAVIGATION_MODE not in {"aruco", "grid"}:
    raise ValueError("LIBRARY_ROBOT_NAVIGATION_MODE must be 'aruco' or 'grid'")
grid_geometry = GridGeometry.from_env()
encoder_calibration = EncoderCalibration.from_env()


if USE_MOCK:
    import cv2
    import numpy as np

    class MockController:
        """Hardware-free simulation of a complete outbound and return mission."""

        def __init__(self):
            self._state = "IDLE"
            self._target_id = None
            self._reason = None
            self._mission = None
            self._ticks = 0
            self._lock = threading.RLock()

        def request_book(self, aruco_id: int):
            """Compatibility entry point for older direct-marker callers."""
            with self._lock:
                self._mission = None
                self._target_id = aruco_id
                self._state = "SCANNING"
                self._reason = None
                self._ticks = 0

        def request_mission(self, book: dict):
            with self._lock:
                self._mission = Mission.from_book(book)
                self._target_id = self._mission.current_marker_id
                self._state = "SCANNING"
                self._reason = None
                self._ticks = 0

        def get_state(self) -> str:
            with self._lock:
                return self._state

        def get_status(self) -> dict:
            with self._lock:
                status = {
                    "state": self._state,
                    "target_id": self._target_id,
                    "reason": self._reason,
                    "turn": None,
                    "mock": True,
                }
                if self._mission is not None:
                    status.update(self._mission.status())
                return status

        def reset(self):
            with self._lock:
                self._state = "IDLE"
                self._target_id = None
                self._reason = None
                self._mission = None
                self._ticks = 0

        def step(self):
            with self._lock:
                if self._state in {"IDLE", "DOCKED", "STOPPED"}:
                    return
                self._ticks += 1
                if self._ticks < 3:
                    return
                self._ticks = 0
                if self._state == "SCANNING":
                    self._state = "ALIGNING"
                elif self._state == "ALIGNING":
                    self._state = "APPROACHING"
                elif self._state == "TURNING":
                    self._state = "SCANNING"
                elif self._state == "ARRIVED":
                    self._mission.start_return()
                    self._target_id = self._mission.current_marker_id
                    self._state = "TURNING"
                    self._reason = None
                elif self._state == "APPROACHING":
                    self._advance_waypoint()

        def _advance_waypoint(self):
            if self._mission is None:
                self._state = "ARRIVED"
                self._reason = "target_distance_reached"
                return
            completed = self._mission.advance_waypoint()
            if self._mission.phase == MissionPhase.AT_DESTINATION:
                self._state = "ARRIVED"
                self._target_id = None
                self._reason = "destination_reached"
            elif self._mission.phase == MissionPhase.COMPLETE:
                self._state = "DOCKED"
                self._target_id = None
                self._reason = "dock_reached"
            else:
                self._target_id = self._mission.current_marker_id
                self._state = "TURNING" if completed.get("turn_after") != "NONE" else "SCANNING"
                self._reason = None

    class MockCamera:
        COLOURS = {
            "IDLE": (60, 60, 60), "SCANNING": (30, 130, 220),
            "ALIGNING": (220, 160, 30), "APPROACHING": (40, 180, 80),
            "MOVING": (40, 180, 80),
            "TURNING": (180, 90, 190), "ARRIVED": (20, 140, 20),
            "DOCKED": (160, 110, 30), "STOPPED": (50, 50, 200),
        }

        def generate_mjpeg(self, get_state_fn):
            while True:
                state = get_state_fn()
                frame = np.full((480, 640, 3), self.COLOURS.get(state, (60, 60, 60)), dtype=np.uint8)
                cv2.rectangle(frame, (250, 175), (390, 305), (255, 255, 255), 3)
                cv2.rectangle(frame, (260, 185), (380, 295), (0, 0, 0), -1)
                cv2.putText(frame, f"State: {state}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            1.1, (255, 255, 255), 2)
                cv2.putText(frame, "[MOCK CAMERA]", (20, 460), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (200, 200, 200), 1)
                _, jpeg = cv2.imencode(".jpg", frame)
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                time.sleep(0.05)

    class MockEncoderSerial:
        """Advances encoder counts quickly while preserving controller behavior."""

        def __init__(self):
            self.left = 0
            self.right = 0

        def send_stop(self): return True
        def send_forward(self): return True
        def send_rotate_left(self): return True
        def send_rotate_right(self): return True
        def reset_encoders(self):
            self.left = self.right = 0
            return True
        def get_encoders(self):
            self.left += 10000
            self.right += 10000
            return {"left": self.left, "right": self.right}
        def get_ultrasonic(self):
            return {"left": 100, "center": 100, "right": 100}

    if NAVIGATION_MODE == "grid":
        controller = GridController(
            MockEncoderSerial(), destination_dwell_seconds=0.3
        )
        if grid_geometry.missing_fields:
            grid_geometry = GridGeometry(80, 75, 35)
        if encoder_calibration.missing_fields:
            encoder_calibration = EncoderCalibration(10, 420, 840)
    else:
        controller = MockController()
    camera = MockCamera()
else:
    try:
        from .aruco_detector import ArucoDetector
        from .camera import Camera
        from .navigation_config import NavigationConfig
        from .robot_controller import RobotController
        from .serial_bridge import SerialBridge
    except ImportError:
        from aruco_detector import ArucoDetector
        from camera import Camera
        from navigation_config import NavigationConfig
        from robot_controller import RobotController
        from serial_bridge import SerialBridge

    config = NavigationConfig.from_env()
    serial_bridge = SerialBridge(port=os.getenv("LIBRARY_ROBOT_SERIAL_PORT", "/dev/ttyACM0"))
    camera = Camera(width=config.camera_width, height=config.camera_height, fps=config.camera_fps)
    if NAVIGATION_MODE == "grid":
        controller = GridController(
            serial_bridge,
            obstacle_distance_cm=config.obstacle_distance_cm,
            destination_dwell_seconds=config.destination_dwell_seconds,
            encoder_stall_seconds=float(
                os.getenv("LIBRARY_ROBOT_ENCODER_STALL_SECONDS", "2")
            ),
        )
    else:
        detector = ArucoDetector(min_area_px=config.min_marker_area_px)
        controller = RobotController(
            serial_bridge, camera, detector,
            frame_width=config.camera_width,
            align_tolerance_px=config.align_tolerance_px,
            stop_distance_cm=config.stop_distance_cm,
            obstacle_distance_cm=config.obstacle_distance_cm,
            scan_timeout_seconds=config.scan_timeout_seconds,
            target_confirmation_frames=config.target_confirmation_frames,
            alignment_confirmation_frames=config.alignment_confirmation_frames,
            target_loss_tolerance_frames=config.target_loss_tolerance_frames,
            turn_90_seconds=config.turn_90_seconds,
            uturn_seconds=config.uturn_seconds,
            destination_dwell_seconds=config.destination_dwell_seconds,
            auto_return=config.auto_return,
        )

    def _shutdown_hardware():
        try:
            controller.reset()
        finally:
            camera.stop()
            serial_bridge.close()

    atexit.register(_shutdown_hardware)


def _control_loop():
    while True:
        try:
            controller.step()
        except Exception:
            logger.exception("Robot control loop failed; controller stopped safely")
        time.sleep(0.1 if USE_MOCK else 1.0 / config.control_hz)


threading.Thread(target=_control_loop, daemon=True).start()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    if USE_MOCK:
        stream = camera.generate_mjpeg(controller.get_state)
    elif NAVIGATION_MODE == "grid":
        stream = camera.generate_mjpeg()
    else:
        stream = camera.generate_mjpeg(controller.get_latest_frame)
    return Response(stream, mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/books")
def books():
    return jsonify(get_all_books())


@app.route("/navigation_mode")
def navigation_mode():
    return jsonify(
        {
            "mode": NAVIGATION_MODE,
            "grid_configured": not (
                grid_geometry.missing_fields or encoder_calibration.missing_fields
            ),
            "missing": grid_geometry.missing_fields
            + encoder_calibration.missing_fields,
        }
    )


@app.route("/boxes")
def boxes():
    return jsonify(list(BOX_IDS))


@app.route("/request_book", methods=["POST"])
def request_book():
    if NAVIGATION_MODE == "grid":
        return jsonify(
            {"status": "error", "message": "Grid mode requires /request_box"}
        ), 409
    data = request.get_json(silent=True) or {}
    title = str(data.get("title", "")).strip()
    if not title:
        return jsonify({"status": "error", "message": "No book title provided"}), 400
    book = get_book(title)
    if book is None:
        return jsonify({"status": "error", "message": f"Book not found: {title}"}), 404
    controller.request_mission(book)
    location = {
        "zone": book["zone"], "shelf": book["shelf_code"],
        "level": book["level"], "slot": book["slot"],
    }
    route = [step["marker_id"] for step in get_route(book["outbound_route"])["steps"]]
    logger.info("Book requested: %s -> marker %s", title, book["destination_marker"])
    return jsonify({
        "status": "ok", "title": title, "aruco_id": book["destination_marker"],
        "destination": location, "route": route,
    })


@app.route("/request_box", methods=["POST"])
def request_box():
    if NAVIGATION_MODE != "grid":
        return jsonify(
            {"status": "error", "message": "Box routes require grid mode"}
        ), 409
    data = request.get_json(silent=True) or {}
    box_id = str(data.get("box_id", "")).strip()
    try:
        plan = build_grid_route(box_id, grid_geometry, encoder_calibration)
    except ValueError as exc:
        status_code = 503 if "not calibrated" in str(exc) else 400
        return jsonify({"status": "error", "message": str(exc)}), status_code
    controller.request_grid_mission(plan)
    return jsonify(
        {
            "status": "ok",
            "box_id": plan["box_id"],
            "row": plan["row"],
            "column": plan["column"],
            "outbound": plan["outbound"],
            "return": plan["return"],
        }
    )


@app.route("/status")
def status():
    return jsonify(controller.get_status())


@app.route("/reset", methods=["POST"])
def reset():
    controller.reset()
    return jsonify({"status": "ok"})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
