"""Fixed-grid, marker-free Flask control panel for the library robot."""

import atexit
import logging
import os
import threading
import time

from flask import Flask, Response, jsonify, render_template, request
from dotenv import load_dotenv

load_dotenv()

try:
    from .student_db import get_all_students, get_student_by_qr, get_student_by_id, borrow_book, return_book
    from .book_db import (
        find_book,
        get_all_books,
        get_book,
        search_books,
    )
    from .encoder_navigation import GridController
    from .grid_layout import EncoderCalibration, GridGeometry, build_grid_route
    from .qr_scanner import QRScanner
except ImportError:  # Supports ``python pi/app.py``.
    from student_db import get_all_students, get_student_by_qr, get_student_by_id, borrow_book, return_book
    from book_db import find_book, get_all_books, get_book, search_books
    from encoder_navigation import GridController
    from grid_layout import EncoderCalibration, GridGeometry, build_grid_route
    from qr_scanner import QRScanner


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
app = Flask(
    __name__,
    template_folder=os.path.join(BASE_DIR, 'templates'),
    static_folder=os.path.join(BASE_DIR, 'static')
)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
app.config['TEMPLATES_AUTO_RELOAD'] = True

@app.after_request
def add_header(response):
    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, post-check=0, pre-check=0, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '-1'
    return response

USE_MOCK = os.getenv("LIBRARY_ROBOT_USE_MOCK", "false").lower() in {
    "1", "true", "yes", "on"
}
GRID_TURN_SOURCE = os.getenv(
    "LIBRARY_ROBOT_GRID_TURN_SOURCE", "imu"
).strip().lower()
grid_geometry = GridGeometry.from_env()
encoder_calibration = EncoderCalibration.from_env()

# --- Student session state (set by QR scanner, cleared when robot docks) ---
_session_lock = threading.Lock()
_current_student: dict | None = None  # The checked-in student dict

def _set_current_student(student: dict | None) -> None:
    global _current_student
    with _session_lock:
        _current_student = student

def _get_current_student() -> dict | None:
    with _session_lock:
        return _current_student


if USE_MOCK:
    import cv2
    import numpy as np

    class MockEncoderSerial:
        def __init__(self):
            self.left = 0
            self.right = 0
            self.turn_status = "IDLE"

        def send_stop(self):
            self.turn_status = "IDLE"
            return True

        def send_forward(self): return True
        def send_rotate_left(self): return True
        def send_rotate_right(self): return True

        def send_turn_left(self):
            self.turn_status = "ACTIVE"
            return True

        def send_turn_right(self):
            self.turn_status = "ACTIVE"
            return True

        def send_turn_uturn(self):
            self.turn_status = "ACTIVE"
            return True

        def get_turn_status(self):
            if self.turn_status == "ACTIVE":
                self.turn_status = "DONE"
                return "DONE"
            return self.turn_status

        def reset_encoders(self):
            self.left = self.right = 0
            return True

        def get_encoders(self):
            self.left += 10000
            self.right += 10000
            return {"left": self.left, "right": self.right}

        def get_ultrasonic(self):
            return {"left": 100, "center": 100, "right": 100}

    class MockCamera:
        COLOURS = {
            "IDLE": (60, 60, 60),
            "MOVING": (40, 180, 80),
            "TURNING": (180, 90, 190),
            "ARRIVED": (20, 140, 20),
            "DOCKED": (160, 110, 30),
            "STOPPED": (50, 50, 200),
        }

        def generate_mjpeg(self, get_state_fn):
            while True:
                state = get_state_fn()
                frame = np.full(
                    (480, 640, 3),
                    self.COLOURS.get(state, (60, 60, 60)),
                    dtype=np.uint8,
                )
                cv2.putText(
                    frame,
                    f"Fixed Grid: {state}",
                    (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (255, 255, 255),
                    2,
                )
                cv2.putText(
                    frame,
                    "[MOCK - NO MARKER SCANNING]",
                    (20, 460),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (220, 220, 220),
                    1,
                )
                _, jpeg = cv2.imencode(".jpg", frame)
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    + jpeg.tobytes()
                    + b"\r\n"
                )
                time.sleep(0.05)

    if grid_geometry.missing_fields:
        grid_geometry = GridGeometry(80, 75, 35)
    controller = GridController(
        MockEncoderSerial(),
        destination_dwell_seconds=0.3,
        turn_source=GRID_TURN_SOURCE,
    )
    camera = MockCamera()
    config = None
else:
    try:
        from .camera import Camera
        from .navigation_config import NavigationConfig
        from .serial_bridge import SerialBridge
    except ImportError:
        from camera import Camera
        from navigation_config import NavigationConfig
        from serial_bridge import SerialBridge

    config = NavigationConfig.from_env()
    serial_bridge = SerialBridge(
        port=os.getenv("LIBRARY_ROBOT_SERIAL_PORT", "/dev/ttyACM0")
    )
    
    # Configure motor drift compensation if provided
    trim = os.getenv("LIBRARY_ROBOT_LEFT_SPEED_REDUCTION")
    if trim is not None:
        try:
            serial_bridge.set_trim(int(trim))
        except ValueError:
            logger.error("LIBRARY_ROBOT_LEFT_SPEED_REDUCTION must be an integer")
            
    camera = Camera(
        width=config.camera_width,
        height=config.camera_height,
        fps=config.camera_fps,
    )
    controller = GridController(
        serial_bridge,
        obstacle_distance_cm=config.obstacle_distance_cm,
        destination_dwell_seconds=config.destination_dwell_seconds,
        encoder_stall_seconds=float(
            os.getenv("LIBRARY_ROBOT_ENCODER_STALL_SECONDS", "2")
        ),
        turn_source=GRID_TURN_SOURCE,
    )

    def _on_qr_detected(qr_code: str) -> None:
        """Called from the QR scanner thread when a student ID is scanned."""
        try:
            from .student_db import get_student_by_qr
        except ImportError:
            from student_db import get_student_by_qr
        student = get_student_by_qr(qr_code)
        if student is None:
            logger.info("Unknown QR code scanned: %s", qr_code)
            return
        logger.info("Student checked in via QR: %s (%s)", student['name'], student['id'])
        _set_current_student(student)

    qr_scanner = QRScanner(
        frame_provider=camera.get_frame,
        on_detect=_on_qr_detected,
    )
    qr_scanner.start()

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
            # Auto-clear student session once the robot has returned to dock
            if controller.get_state() in ("DOCKED", "IDLE") and _get_current_student() is not None:
                # Only clear if no active mission
                if controller.get_status().get("phase") is None:
                    _set_current_student(None)
                    logger.info("Student session cleared after robot docked")
        except Exception:
            logger.exception("Robot control loop failed; controller stopped safely")
        time.sleep(0.1 if USE_MOCK else 1.0 / config.control_hz)


threading.Thread(target=_control_loop, daemon=True).start()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    stream = (
        camera.generate_mjpeg(controller.get_state)
        if USE_MOCK
        else camera.generate_mjpeg()
    )
    return Response(
        stream,
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/navigation_mode")
def navigation_mode():
    missing = (
        grid_geometry.missing_fields
        + encoder_calibration.missing_fields_for(GRID_TURN_SOURCE)
    )
    return jsonify(
        {
            "mode": "grid",
            "label": "Fixed Grid - Encoder + IMU",
            "marker_scanning": False,
            "grid_configured": not missing,
            "missing": missing,
        }
    )


@app.route("/books")
def books():
    return jsonify(get_all_books(grid_only=True))


@app.route("/search_books")
def search_book_catalogue():
    return jsonify(search_books(request.args.get("q", "")))


def _resolve_book(query: str):
    book = find_book(query)
    if book is not None:
        return book
    matches = search_books(query)
    return get_book(matches[0]["title"]) if len(matches) == 1 else None


@app.route("/request_book", methods=["POST"])
def request_book():
    data = request.get_json(silent=True) or {}
    query = str(data.get("query") or data.get("title") or "").strip()
    if not query:
        return jsonify(
            {"status": "error", "message": "Enter a book title or number"}
        ), 400
    book = _resolve_book(query)
    if book is None:
        return jsonify(
            {"status": "error", "message": f"Book not found or ambiguous: {query}"}
        ), 404

    try:
        plan = build_grid_route(
            book["box_id"],
            grid_geometry,
            encoder_calibration,
            turn_source=GRID_TURN_SOURCE,
        )
    except ValueError as exc:
        status_code = 503 if "not calibrated" in str(exc) else 400
        return jsonify({"status": "error", "message": str(exc)}), status_code

    plan.update(
        book=book["title"],
        book_code=book["book_id"],
        location_code=book.get("location_code", ""),
        layer=book.get("layer"),
        position=book.get("position"),
    )
    controller.request_grid_mission(plan)
    logger.info("Book requested: %s -> %s", book["title"], book.get("location_code", ""))
    return jsonify(
        {
            "status": "ok",
            "title": book["title"],
            "book_code": book["book_id"],
            "location_code": book.get("location_code", ""),
            "destination": {
                "box_id": book["box_id"],
                "layer": book.get("layer"),
                "position": book.get("position"),
            },
            "outbound": plan["outbound"],
            "return": plan["return"],
        }
    )


@app.route("/status")
def status():
    data = controller.get_status()
    student = _get_current_student()
    data["current_student"] = {
        "id": student["id"],
        "name": student["name"],
        "has_book": student["borrowed_book_id"] is not None,
        "borrowed_book_id": student["borrowed_book_id"],
    } if student else None
    return jsonify(data)


@app.route("/reset", methods=["POST"])
def reset():
    controller.reset()
    return jsonify({"status": "ok"})


@app.route("/api/students", methods=["GET"])
def api_students():
    students = get_all_students()
    for s in students:
        s["has_book"] = s.get("borrowed_book_id") is not None
    return jsonify(students)


@app.route("/api/checkin", methods=["POST"])
def api_checkin():
    data = request.get_json(silent=True) or {}
    qr_code = data.get("qr_code")
    if not qr_code:
        return jsonify({"ok": False, "message": "Missing qr_code"}), 400
    student = get_student_by_qr(qr_code)
    if not student:
        return jsonify({"ok": False, "message": "Student not found"}), 404
    student_resp = dict(student)
    student_resp["has_book"] = student_resp.get("borrowed_book_id") is not None
    return jsonify({"ok": True, "student": student_resp}), 200


@app.route("/api/borrow", methods=["POST"])
def api_borrow():
    data = request.get_json(silent=True) or {}
    student_id = data.get("student_id")
    book_query = data.get("book_query")
    if not student_id or not book_query:
        return jsonify({"ok": False, "message": "Missing student_id or book_query"}), 400

    book = _resolve_book(book_query)
    if not book:
        return jsonify({"ok": False, "message": "Book not found"}), 404

    borrow_res = borrow_book(student_id, book["book_id"])
    if not borrow_res.get("ok"):
        return jsonify(borrow_res), 409

    try:
        plan = build_grid_route(
            book["box_id"],
            grid_geometry,
            encoder_calibration,
            turn_source=GRID_TURN_SOURCE,
        )
    except ValueError as exc:
        status_code = 503 if "not calibrated" in str(exc) else 400
        return jsonify({"ok": False, "message": str(exc)}), status_code

    plan.update(
        book=book["title"],
        book_code=book["book_id"],
        location_code=book.get("location_code", ""),
        layer=book.get("layer"),
        position=book.get("position"),
    )
    controller.request_grid_mission(plan)
    
    return jsonify({
        "ok": True,
        "book": {
            "title": book["title"],
            "book_id": book["book_id"],
            "location_code": book.get("location_code", "")
        }
    }), 200


@app.route("/api/return_book", methods=["POST"])
def api_return_book():
    data = request.get_json(silent=True) or {}
    student_id = data.get("student_id")
    if not student_id:
        return jsonify({"ok": False, "message": "Missing student_id"}), 400

    return_res = return_book(student_id)
    if not return_res.get("ok"):
        return jsonify({"ok": False, "message": "No book to return"}), 400

    returned_book_id = return_res["book_id"]
    return_box = os.getenv("LIBRARY_ROBOT_RETURN_SLOT", "1A")

    try:
        plan = build_grid_route(
            return_box,
            grid_geometry,
            encoder_calibration,
            turn_source=GRID_TURN_SOURCE,
        )
        plan.update(
            book="RETURN",
            book_code=returned_book_id,
            location_code=return_box,
        )
        controller.request_grid_mission(plan)
    except Exception as e:
        logger.error(f"Failed to route to return slot: {e}")

    return jsonify({"ok": True, "returned_book_id": returned_book_id}), 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
