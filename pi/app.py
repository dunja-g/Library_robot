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
    from .borrowing_mission import BorrowingMission, BorrowingState
    from .student_db import (
        borrow_book,
        return_book,
        get_all_students,
        get_student_by_id,
        get_student_by_qr,
        get_borrower_for_book,
        rollback_borrow_book,
    )
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
    from borrowing_mission import BorrowingMission, BorrowingState
    from student_db import (
        borrow_book,
        return_book,
        get_all_students,
        get_student_by_id,
        get_student_by_qr,
        get_borrower_for_book,
        rollback_borrow_book,
    )
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
# Apply safe defaults for timed-mode navigation so the robot can run
# without every env var configured. These values are typical for a
# small 4-wheel library robot; tune via env vars for your hardware.
_GEO_DEFAULTS = dict(
    first_row_distance_cm=80.0,
    row_spacing_cm=75.0,
    box_approach_distance_cm=35.0,
    forward_speed_cms=20.0,
)
if grid_geometry.missing_fields or grid_geometry.forward_speed_cms is None:
    grid_geometry = GridGeometry(
        first_row_distance_cm=(
            grid_geometry.first_row_distance_cm
            if grid_geometry.first_row_distance_cm is not None
            else _GEO_DEFAULTS["first_row_distance_cm"]
        ),
        row_spacing_cm=(
            grid_geometry.row_spacing_cm
            if grid_geometry.row_spacing_cm is not None
            else _GEO_DEFAULTS["row_spacing_cm"]
        ),
        box_approach_distance_cm=(
            grid_geometry.box_approach_distance_cm
            if grid_geometry.box_approach_distance_cm is not None
            else _GEO_DEFAULTS["box_approach_distance_cm"]
        ),
        forward_speed_cms=(
            grid_geometry.forward_speed_cms
            if grid_geometry.forward_speed_cms is not None
            else _GEO_DEFAULTS["forward_speed_cms"]
        ),
    )
encoder_calibration = EncoderCalibration.from_env()
FUSION_ALPHA = float(os.getenv("LIBRARY_ROBOT_FUSION_ALPHA", "0.95"))
HEADING_KP = float(os.getenv("LIBRARY_ROBOT_HEADING_KP", "1.5"))
MAX_HEADING_CORRECTION = int(
    os.getenv("LIBRARY_ROBOT_MAX_HEADING_CORRECTION", "30")
)
_wheel_track_raw = os.getenv("LIBRARY_ROBOT_WHEEL_TRACK_CM", "").strip()
WHEEL_TRACK_CM = float(_wheel_track_raw) if _wheel_track_raw else 15.5
_left_ticks_raw = os.getenv("LIBRARY_ROBOT_LEFT_TICKS_PER_CM", "").strip()
_right_ticks_raw = os.getenv("LIBRARY_ROBOT_RIGHT_TICKS_PER_CM", "").strip()
LEFT_TICKS_PER_CM = (
    float(_left_ticks_raw)
    if _left_ticks_raw
    else encoder_calibration.resolved_ticks_per_cm
)
RIGHT_TICKS_PER_CM = (
    float(_right_ticks_raw)
    if _right_ticks_raw
    else encoder_calibration.resolved_ticks_per_cm
)
if not 0 <= FUSION_ALPHA <= 1:
    raise ValueError("LIBRARY_ROBOT_FUSION_ALPHA must be between 0 and 1")
if (
    LEFT_TICKS_PER_CM <= 0
    or RIGHT_TICKS_PER_CM <= 0
    or HEADING_KP < 0
    or not 0 <= MAX_HEADING_CORRECTION <= 100
    or (WHEEL_TRACK_CM is not None and WHEEL_TRACK_CM <= 0)
):
    raise ValueError("Invalid fused-odometry calibration")
FUSION_MISSING_FIELDS = [] if WHEEL_TRACK_CM is not None else ["wheel_track_cm"]
MISSION_TIMEOUT_SECONDS = float(
    os.getenv("LIBRARY_ROBOT_MISSION_TIMEOUT_SECONDS", "300")
)
if MISSION_TIMEOUT_SECONDS <= 0:
    raise ValueError("LIBRARY_ROBOT_MISSION_TIMEOUT_SECONDS must be positive")

# --- Student session state (set by QR scanner, cleared when robot docks) ---
_session_lock = threading.Lock()
_mission_lock = threading.RLock()
_current_student: dict | None = None  # The checked-in student dict
_current_borrowing_mission: BorrowingMission | None = None

def _set_current_student(student: dict | None) -> None:
    global _current_student
    with _session_lock:
        _current_student = student

def _get_current_student() -> dict | None:
    with _session_lock:
        return _current_student


def _get_current_mission() -> BorrowingMission | None:
    with _mission_lock:
        return _current_borrowing_mission


def _check_in_by_qr(qr_code: str) -> tuple[dict | None, str | None]:
    """Shared QR/manual check-in logic; only valid while stationary."""
    global _current_borrowing_mission
    with _mission_lock:
        state = controller.get_state()
        mission = _current_borrowing_mission
        if state not in {"IDLE", "DOCKED"}:
            return None, "robot_not_idle"
        if mission is not None and mission.is_active:
            return None, "mission_active"
        student = get_student_by_qr(qr_code)
        if student is None:
            return None, "student_not_found"
        _current_borrowing_mission = None
        _set_current_student(student)
        return student, None


def _on_qr_detected(qr_code: str) -> None:
    student, reason = _check_in_by_qr(qr_code)
    if student is None:
        logger.info("QR check-in rejected: %s", reason)
        return
    logger.info(
        "Student checked in via QR: %s (%s)",
        student["name"],
        student["id"],
    )


qr_scanner = None

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
        def send_backward(self): return True
        def send_rotate_left(self): return True
        def send_rotate_right(self): return True

        def send_turn_left(self, _degrees=None):
            self.turn_status = "ACTIVE"
            return True

        def send_turn_right(self, _degrees=None):
            self.turn_status = "ACTIVE"
            return True

        def send_turn_uturn(self, _degrees=None):
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

        def get_odometry(self):
            readings = self.get_encoders()
            return {
                **readings,
                "left_cm": readings["left"] / LEFT_TICKS_PER_CM,
                "right_cm": readings["right"] / RIGHT_TICKS_PER_CM,
                "heading_encoder_deg": 0.0,
                "heading_imu_deg": 0.0,
                "heading_fused_deg": 0.0,
                "speed_correction": 0,
            }

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
    
    # Mock clock to advance rapidly so timed steps finish instantly in tests
    class MockClock:
        def __init__(self):
            self.time = 0.0
        def __call__(self):
            self.time += 1.0
            return self.time
    controller._clock = MockClock()

    
    class MockArucoDetector:
        def detect_target(self, frame, target_id):
            # Always return a massive area so the ArUco approach instantly completes
            return {"center_x": 320, "area": 100000}
            
    controller.aruco_detector = MockArucoDetector()
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
    if WHEEL_TRACK_CM is not None:
        if not serial_bridge.set_fusion_config(
            alpha=FUSION_ALPHA,
            left_ticks_per_cm=LEFT_TICKS_PER_CM,
            right_ticks_per_cm=RIGHT_TICKS_PER_CM,
            wheel_track_cm=WHEEL_TRACK_CM,
            heading_kp=HEADING_KP,
            max_correction=MAX_HEADING_CORRECTION,
        ):
            raise RuntimeError("Unable to configure Mega fused odometry")
            
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

    qr_scanner = QRScanner(
        frame_provider=camera.get_frame,
        on_detect=_on_qr_detected,
    )
    qr_scanner.start()

    def _shutdown_hardware():
        try:
            controller.reset()
        finally:
            qr_scanner.stop()
            camera.stop()
            serial_bridge.close()

    atexit.register(_shutdown_hardware)


def _cancel_pending_mission(reason: str) -> bool:
    mission = _current_borrowing_mission
    if mission is None or mission.state != BorrowingState.PENDING:
        return False
    mission.cancel(reason)
    logger.warning("Pending borrowing mission cancelled: %s", reason)
    return True


def _reconcile_borrowing_state() -> None:
    """Apply controller terminal states to the borrowing transaction."""
    global _current_borrowing_mission
    with _mission_lock:
        mission = _current_borrowing_mission
        if mission is None:
            return
        status = controller.get_status()
        state = status["state"]
        if mission.is_expired():
            controller.cancel("mission_timeout")
            _cancel_pending_mission("mission_timeout")
            return
            
        if state == "STOPPED":
            _cancel_pending_mission(status.get("reason") or "robot_stopped")
            return
            
        if (
            mission.state == BorrowingState.PENDING
            and state == "ARRIVED"
            and status.get("phase") == "AT_DESTINATION"
            and not status.get("pickup_confirmation_required", True)
        ):
            try:
                if mission.mission_type == "return":
                    result = return_book(mission.student_id)
                    action_name = "return"
                else:
                    result = borrow_book(mission.student_id, mission.book_id)
                    action_name = "pickup"
                
                if result.get("ok"):
                    try:
                        controller.confirm_pickup()
                    except Exception as e:
                        logger.debug(f"Controller did not require pickup confirmation: {e}")
                    mission.confirm()
                    logger.info(f"Auto-confirmed {action_name} for {mission.book_id}")
                else:
                    logger.error(f"Failed to auto-confirm {action_name}: {result.get('message') or result.get('reason')}")
            except Exception as e:
                logger.error(f"Failed to record auto-confirmation: {e}")
        if (
            mission.state == BorrowingState.CONFIRMED
            and state == "DOCKED"
            and status.get("phase") == "COMPLETE"
        ):
            _current_borrowing_mission = None
            _set_current_student(None)
            logger.info("Borrowing mission complete; student session cleared")


def _control_loop():
    while True:
        try:
            controller.step()
        except Exception:
            logger.exception("Robot control loop failed; controller stopped safely")
        _reconcile_borrowing_state()
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
        + ([] if USE_MOCK else FUSION_MISSING_FIELDS)
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


def _build_borrowing_plan(book: dict) -> dict:
    plan = build_grid_route(
        book["box_id"],
        grid_geometry,
        encoder_calibration,
        turn_source=GRID_TURN_SOURCE,
    )
    plan.update(
        book=book["title"],
        book_code=book["book_id"],
        location_code=book.get("location_code", ""),
        layer=book.get("layer"),
        position=book.get("position"),
        pickup_confirmation_required=False,
    )
    return plan


def _start_borrowing_mission(
    book_query: str, requested_student_id: str | None = None
) -> tuple[dict, int]:
    global _current_borrowing_mission
    with _mission_lock:
        student = _get_current_student()
        if student is None:
            return {"ok": False, "message": "Scan a student card first"}, 401
        if requested_student_id and requested_student_id != student["id"]:
            return {"ok": False, "message": "Student session mismatch"}, 403
        active = _current_borrowing_mission
        if active is not None and active.is_active:
            return {"ok": False, "message": "Another mission is active"}, 409
        if controller.get_state() not in {"IDLE", "DOCKED"}:
            return {"ok": False, "message": "Robot is not ready at Dock"}, 409

        fresh_student = get_student_by_id(student["id"])
        if fresh_student is None:
            return {"ok": False, "message": "Student not found"}, 404
        if fresh_student.get("borrowed_book_id"):
            return {"ok": False, "message": "Student already has a book"}, 409

        book = _resolve_book(book_query)
        if book is None:
            return {"ok": False, "message": "Book not found"}, 404
        if get_borrower_for_book(book["book_id"]) is not None:
            return {"ok": False, "message": "Book is already borrowed"}, 409
        # FUSION_MISSING_FIELDS is now always empty because WHEEL_TRACK_CM
        # defaults to 15.5 cm — no need to block dispatch here.
        try:
            plan = _build_borrowing_plan(book)
        except ValueError as exc:
            status_code = 503 if "not calibrated" in str(exc) else 400
            return {"ok": False, "message": str(exc)}, status_code

        mission = BorrowingMission.create(
            fresh_student,
            book,
            timeout_seconds=MISSION_TIMEOUT_SECONDS,
        )
        _current_borrowing_mission = mission
        try:
            controller.request_grid_mission(plan)
        except Exception as exc:
            mission.cancel(f"dispatch_failed:{type(exc).__name__}")
            return {"ok": False, "message": "Unable to dispatch robot"}, 503
        if controller.get_state() == "STOPPED":
            reason = controller.get_status().get("reason") or "dispatch_failed"
            mission.cancel(reason)
            return {"ok": False, "message": f"Robot stopped: {reason}"}, 503

        return {
            "ok": True,
            "mission": mission.as_dict(),
            "book": {
                "title": book["title"],
                "book_id": book["book_id"],
                "location_code": book.get("location_code", ""),
                "box_id": book["box_id"],
                "layer": book["layer"],
                "position": book["position"],
            },
        }, 202


def _start_return_mission(student_id: str | None = None) -> tuple[dict, int]:
    """Build and dispatch a return route to put a book back on the shelf."""
    global _current_borrowing_mission
    with _mission_lock:
        student = _get_current_student()
        if student is None:
            return {"ok": False, "message": "Scan a student card first"}, 401
        if student_id and student_id != student["id"]:
            return {"ok": False, "message": "Student session mismatch"}, 403
        active = _current_borrowing_mission
        if active is not None and active.is_active:
            return {"ok": False, "message": "Another mission is active"}, 409
        if controller.get_state() not in {"IDLE", "DOCKED"}:
            return {"ok": False, "message": "Robot is not ready at Dock"}, 409

        fresh_student = get_student_by_id(student["id"])
        if fresh_student is None:
            return {"ok": False, "message": "Student not found"}, 404
        
        book_id = fresh_student.get("borrowed_book_id")
        if not book_id:
            return {"ok": False, "message": "Student does not have a book to return"}, 409

        book = _resolve_book(book_id)
        if book is None:
            return {"ok": False, "message": f"Borrowed book {book_id} not found in database"}, 404

        try:
            plan = _build_borrowing_plan(book)
        except ValueError as exc:
            status_code = 503 if "not calibrated" in str(exc) else 400
            return {"ok": False, "message": str(exc)}, status_code

        mission = BorrowingMission.create(
            fresh_student,
            book,
            mission_type="return",
            timeout_seconds=MISSION_TIMEOUT_SECONDS,
        )
        _current_borrowing_mission = mission
        try:
            controller.request_grid_mission(plan)
        except Exception as exc:
            mission.cancel(f"dispatch_failed:{type(exc).__name__}")
            return {"ok": False, "message": "Unable to dispatch robot"}, 503
        if controller.get_state() == "STOPPED":
            reason = controller.get_status().get("reason") or "dispatch_failed"
            mission.cancel(reason)
            return {"ok": False, "message": f"Robot stopped: {reason}"}, 503

        return {
            "ok": True,
            "mission": mission.as_dict(),
            "book": {
                "book_id": book["book_id"],
                "title": book["title"],
                "location_code": book["location_code"],
            },
        }, 201


@app.route("/request_book", methods=["POST"])
def request_book():
    data = request.get_json(silent=True) or {}
    query = str(data.get("query") or data.get("title") or "").strip()
    if not query:
        return jsonify(
            {"status": "error", "message": "Enter a book title or number"}
        ), 400
    payload, status_code = _start_borrowing_mission(query)
    payload["status"] = "ok" if payload.get("ok") else "error"
    return jsonify(payload), status_code


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
    mission = _get_current_mission()
    data["borrowing_mission"] = None if mission is None else mission.as_dict()
    return jsonify(data)


@app.route("/reset", methods=["POST"])
def reset():
    with _mission_lock:
        mission = _current_borrowing_mission
        _cancel_pending_mission("reset")
        controller.reset()
        if mission is None or mission.state != BorrowingState.CONFIRMED:
            _set_current_student(None)
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
    student, reason = _check_in_by_qr(str(qr_code))
    if student is None:
        status_code = 404 if reason == "student_not_found" else 409
        return jsonify({"ok": False, "message": reason}), status_code
    student_resp = dict(student)
    student_resp["has_book"] = student_resp.get("borrowed_book_id") is not None
    return jsonify({"ok": True, "student": student_resp}), 200


@app.route("/api/borrow", methods=["POST"])
def api_borrow():
    data = request.get_json(silent=True) or {}
    student_id = data.get("student_id")
    book_query = data.get("book_query")
    if not book_query:
        return jsonify({"ok": False, "message": "Missing book_query"}), 400
    payload, status_code = _start_borrowing_mission(
        str(book_query),
        None if student_id is None else str(student_id),
    )
    return jsonify(payload), status_code


@app.route("/api/return", methods=["POST"])
def api_return():
    """Dispatch the robot to return the student's currently borrowed book."""
    data = request.get_json(silent=True) or {}
    student_id = data.get("student_id")
    payload, status_code = _start_return_mission(
        None if student_id is None else str(student_id)
    )
    return jsonify(payload), status_code


@app.route("/api/confirm_pickup", methods=["POST"])
def api_confirm_pickup():
    global _current_borrowing_mission
    data = request.get_json(silent=True) or {}
    requested_mission_id = data.get("mission_id")
    with _mission_lock:
        mission = _current_borrowing_mission
        if mission is None or mission.state != BorrowingState.PENDING:
            return jsonify({"ok": False, "message": "No pending mission"}), 409
        if requested_mission_id and requested_mission_id != mission.mission_id:
            return jsonify({"ok": False, "message": "Mission mismatch"}), 409
        status = controller.get_status()
        if (
            status["state"] != "ARRIVED"
            or status.get("phase") != "AT_DESTINATION"
        ):
            return jsonify({"ok": False, "message": "Robot has not arrived"}), 409

        if mission.mission_type == "return":
            return_result = return_book(mission.student_id)
            if not return_result.get("ok"):
                return jsonify(return_result), 409
        else:
            borrow_result = borrow_book(mission.student_id, mission.book_id)
            if not borrow_result.get("ok"):
                return jsonify(borrow_result), 409

        try:
            controller.confirm_pickup()
            mission.confirm()
        except Exception:
            if mission.mission_type != "return":
                rollback_borrow_book(mission.student_id, mission.book_id)
            return jsonify(
                {"ok": False, "message": "Could not start return route"}
            ), 503

        refreshed_student = get_student_by_id(mission.student_id)
        _set_current_student(refreshed_student)
        return jsonify({"ok": True, "mission": mission.as_dict()}), 200


@app.route("/api/cancel_mission", methods=["POST"])
def api_cancel_mission():
    with _mission_lock:
        if not _cancel_pending_mission("cancelled_by_user"):
            return jsonify({"ok": False, "message": "No pending mission"}), 409
        controller.cancel("cancelled_by_user")
        return jsonify(
            {"ok": True, "mission": _current_borrowing_mission.as_dict()}
        )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
