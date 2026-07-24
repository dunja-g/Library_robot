"""Thread-safe deterministic multi-waypoint ArUco navigation controller."""

from __future__ import annotations

import threading
import time
from enum import Enum
from typing import Any

import cv2
import numpy as np

try:
    from .mission import Mission, MissionPhase
    from .route_db import get_marker
except ImportError:  # Supports running modules directly from ``pi``.
    from mission import Mission, MissionPhase
    from route_db import get_marker


class State(Enum):
    IDLE = "IDLE"
    SCANNING = "SCANNING"
    ALIGNING = "ALIGNING"
    APPROACHING = "APPROACHING"
    TURNING = "TURNING"
    ARRIVED = "ARRIVED"
    DWELLING = "DWELLING"
    DOCKED = "DOCKED"
    STOPPED = "STOPPED"


class RobotController:
    FRAME_WIDTH = 640
    ALIGN_TOLERANCE_PX = 30
    STOP_DISTANCE_CM = 25.0
    OBSTACLE_DISTANCE_CM = 20.0
    SCAN_TIMEOUT_SECONDS = 60.0
    TARGET_CONFIRMATION_FRAMES = 2
    ALIGNMENT_CONFIRMATION_FRAMES = 2
    TARGET_LOSS_TOLERANCE_FRAMES = 3
    TURN_90_SECONDS = 0.8
    UTURN_SECONDS = 1.6
    DESTINATION_DWELL_SECONDS = 5.0

    def __init__(
        self,
        serial_bridge: Any,
        camera: Any,
        aruco_detector: Any,
        *,
        frame_width: int = FRAME_WIDTH,
        align_tolerance_px: int = ALIGN_TOLERANCE_PX,
        stop_distance_cm: float = STOP_DISTANCE_CM,
        obstacle_distance_cm: float = OBSTACLE_DISTANCE_CM,
        scan_timeout_seconds: float = SCAN_TIMEOUT_SECONDS,
        target_confirmation_frames: int = TARGET_CONFIRMATION_FRAMES,
        alignment_confirmation_frames: int = ALIGNMENT_CONFIRMATION_FRAMES,
        target_loss_tolerance_frames: int = TARGET_LOSS_TOLERANCE_FRAMES,
        turn_90_seconds: float = TURN_90_SECONDS,
        uturn_seconds: float = UTURN_SECONDS,
        destination_dwell_seconds: float = DESTINATION_DWELL_SECONDS,
        auto_return: bool = True,
        clock=time.monotonic,
    ):
        if frame_width <= 0 or align_tolerance_px < 0:
            raise ValueError("frame dimensions and alignment tolerance are invalid")
        if stop_distance_cm <= 0 or obstacle_distance_cm <= 0:
            raise ValueError("distance thresholds must be positive")
        if scan_timeout_seconds <= 0 or turn_90_seconds <= 0 or uturn_seconds <= 0:
            raise ValueError("navigation timing values must be positive")
        if destination_dwell_seconds < 0:
            raise ValueError("destination dwell must be non-negative")
        if target_confirmation_frames <= 0 or alignment_confirmation_frames <= 0:
            raise ValueError("confirmation frame counts must be positive")
        if target_loss_tolerance_frames < 0:
            raise ValueError("target loss tolerance must be non-negative")

        self.serial = serial_bridge
        self.camera = camera
        self.detector = aruco_detector
        self.frame_width = int(frame_width)
        self.align_tolerance_px = int(align_tolerance_px)
        self.stop_distance_cm = float(stop_distance_cm)
        self.obstacle_distance_cm = float(obstacle_distance_cm)
        self.scan_timeout_seconds = float(scan_timeout_seconds)
        self.target_confirmation_frames = int(target_confirmation_frames)
        self.alignment_confirmation_frames = int(alignment_confirmation_frames)
        self.target_loss_tolerance_frames = int(target_loss_tolerance_frames)
        self.turn_90_seconds = float(turn_90_seconds)
        self.uturn_seconds = float(uturn_seconds)
        self.destination_dwell_seconds = float(destination_dwell_seconds)
        self.auto_return = bool(auto_return)
        self._clock = clock

        self.state = State.IDLE
        self.target_id: int | None = None
        self.stop_reason: str | None = None
        self.mission: Mission | None = None
        self._scan_started_at: float | None = None
        self._latest_frame: np.ndarray | None = None
        self._target_seen_frames = 0
        self._aligned_frames = 0
        self._target_missing_frames = 0
        self._turn_action: str | None = None
        self._turn_deadline: float | None = None
        self._dwell_deadline: float | None = None
        self._lock = threading.RLock()

    def request_book(self, aruco_id: int) -> None:
        """Start legacy direct-marker navigation (kept for API compatibility)."""
        if not isinstance(aruco_id, int) or isinstance(aruco_id, bool):
            raise TypeError("aruco_id must be an integer")
        if aruco_id < 0:
            raise ValueError("aruco_id must be non-negative")
        with self._lock:
            self.serial.send_stop()
            self.mission = None
            self.target_id = aruco_id
            self._turn_action = None
            self._turn_deadline = None
            self._dwell_deadline = None
            self._begin_scanning()

    def request_mission(self, book: dict) -> None:
        """Start an outbound book mission followed by its configured return route."""
        mission = Mission.from_book(book)
        with self._lock:
            self.serial.send_stop()
            self.mission = mission
            self.target_id = mission.current_marker_id
            self.stop_reason = None
            self._dwell_deadline = None
            self._start_turn(mission.outbound_route.get("initial_turn", "NONE"))

    def get_state(self) -> str:
        with self._lock:
            return self.state.value

    def get_status(self) -> dict[str, Any]:
        with self._lock:
            status = {
                "state": self.state.value,
                "target_id": self.target_id,
                "reason": self.stop_reason,
                "turn": self._turn_action,
            }
            if self.mission is not None:
                status.update(self.mission.status())
            return status

    def get_latest_frame(self) -> np.ndarray | None:
        with self._lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    def reset(self) -> None:
        with self._lock:
            self.serial.send_stop()
            self.state = State.IDLE
            self.target_id = None
            self.stop_reason = None
            self.mission = None
            self._scan_started_at = None
            self._turn_action = None
            self._turn_deadline = None
            self._dwell_deadline = None
            self._reset_detection_counters()

    def step(self) -> np.ndarray:
        """Run one non-blocking control iteration and return the annotated frame."""
        with self._lock:
            try:
                frame = self.camera.get_frame()
                detections = self.detector.detect(frame)
                if self.state == State.SCANNING:
                    self._do_scanning(detections)
                elif self.state == State.ALIGNING:
                    self._do_aligning(detections)
                elif self.state == State.APPROACHING:
                    self._do_approaching(detections)
                elif self.state == State.TURNING:
                    self._do_turning()
                elif self.state in (State.ARRIVED, State.DWELLING):
                    self._do_arrived()

                annotated = self.detector.draw(frame, detections)
                annotated = self._draw_status(annotated)
                self._latest_frame = annotated.copy()
                return annotated
            except Exception as exc:
                self._safe_stop(f"controller_error:{type(exc).__name__}")
                raise

    def _target_detection(self, detections: list[dict[str, Any]]) -> dict[str, Any] | None:
        return next((item for item in detections if item.get("id") == self.target_id), None)

    def _do_scanning(self, detections: list[dict[str, Any]]) -> None:
        if self._read_safe_distances() is None:
            return
        if self._scan_started_at is None:
            self._scan_started_at = self._clock()
        if self._clock() - self._scan_started_at >= self.scan_timeout_seconds:
            self._safe_stop("scan_timeout")
            return
        if self._target_detection(detections) is not None:
            self.serial.send_stop()
            self._target_seen_frames += 1
            if self._target_seen_frames >= self.target_confirmation_frames:
                self.state = State.ALIGNING
                self._target_missing_frames = 0
                self._aligned_frames = 0
            return
        self._target_seen_frames = 0
        self.serial.send_rotate_left()

    def _do_aligning(self, detections: list[dict[str, Any]]) -> None:
        if self._read_safe_distances() is None:
            return
        target = self._target_detection(detections)
        if target is None:
            self.serial.send_stop()
            self._target_missing_frames += 1
            self._aligned_frames = 0
            if self._target_missing_frames > self.target_loss_tolerance_frames:
                self._begin_scanning()
            return
        self._target_missing_frames = 0
        error = int(target["center_x"]) - self.frame_width / 2.0
        if abs(error) <= self.align_tolerance_px:
            self.serial.send_stop()
            self._aligned_frames += 1
            if self._aligned_frames >= self.alignment_confirmation_frames:
                self.state = State.APPROACHING
        elif error < 0:
            self._aligned_frames = 0
            self.serial.send_rotate_left()
        else:
            self._aligned_frames = 0
            self.serial.send_rotate_right()

    def _do_approaching(self, detections: list[dict[str, Any]]) -> None:
        readings = self._read_safe_distances(check_front=False)
        if readings is None:
            return
        center = float(readings.get("front", readings.get("center", 999.0)))
        target = self._target_detection(detections)
        if target is None:
            if center < self.obstacle_distance_cm:
                self._safe_stop("front_obstacle")
                return
            self.serial.send_stop()
            self._target_missing_frames += 1
            if self._target_missing_frames > self.target_loss_tolerance_frames:
                self._begin_scanning()
            return

        self._target_missing_frames = 0
        error = int(target["center_x"]) - self.frame_width / 2.0
        if abs(error) > self.align_tolerance_px:
            if center < self.obstacle_distance_cm:
                self._safe_stop("front_obstacle")
            else:
                self.serial.send_stop()
                self.state = State.ALIGNING
            return
        if center < self.obstacle_distance_cm:
            self._safe_stop("front_obstacle")
            return
        if center <= self._arrival_distance_cm():
            self._complete_current_waypoint()
            return
        self.serial.send_forward()

    def _do_turning(self) -> None:
        if self._read_safe_distances() is None:
            return
        if self._turn_deadline is None or self._clock() >= self._turn_deadline:
            self.serial.send_stop()
            self._turn_action = None
            self._turn_deadline = None
            self._begin_scanning()
            return
        if self._turn_action == "RIGHT":
            self.serial.send_rotate_right()
        else:
            self.serial.send_rotate_left()

    def _do_arrived(self) -> None:
        self.serial.send_stop()
        if self.mission is None or self.mission.phase != MissionPhase.AT_DESTINATION:
            return
        if self._dwell_deadline is None or self._clock() < self._dwell_deadline:
            return
        if not self.auto_return:
            return
        self.mission.start_return()
        self.target_id = self.mission.current_marker_id
        self._dwell_deadline = None
        self._start_turn(self.mission.return_route.get("initial_turn", "NONE"))

    def _complete_current_waypoint(self) -> None:
        self.serial.send_stop()
        if self.mission is None:
            self.state = State.ARRIVED
            self.stop_reason = "target_distance_reached"
            return
        completed = self.mission.advance_waypoint()
        self.stop_reason = "waypoint_reached"
        if self.mission.phase == MissionPhase.AT_DESTINATION:
            self.state = State.ARRIVED
            self.target_id = None
            self.stop_reason = "destination_reached"
            self._dwell_deadline = self._clock() + self.destination_dwell_seconds
            return
        if self.mission.phase == MissionPhase.COMPLETE:
            self.state = State.DOCKED
            self.target_id = None
            self.stop_reason = "dock_reached"
            return
        self.target_id = self.mission.current_marker_id
        self._start_turn(completed.get("turn_after", "NONE"))

    def _start_turn(self, action: str) -> None:
        self.stop_reason = None
        self._reset_detection_counters()
        if action == "NONE":
            self._turn_action = None
            self._turn_deadline = None
            self._begin_scanning()
            return
        self.state = State.TURNING
        self._turn_action = action
        duration = self.uturn_seconds if action == "UTURN" else self.turn_90_seconds
        self._turn_deadline = self._clock() + duration

    def _begin_scanning(self) -> None:
        self.state = State.SCANNING
        self.stop_reason = None
        self._scan_started_at = self._clock()
        self._reset_detection_counters()

    def _arrival_distance_cm(self) -> float:
        if self.mission is None or self.target_id is None:
            return self.stop_distance_cm
        return float(get_marker(self.target_id)["arrival_distance_cm"])

    def _read_safe_distances(self, *, check_front: bool = True) -> dict | None:
        readings = self.serial.get_ultrasonic()
        if not self._valid_readings(readings):
            self._safe_stop("ultrasonic_unavailable")
            return None
        front_val = float(readings.get("front", readings.get("center", 999.0)))
        if check_front and front_val < self.obstacle_distance_cm:
            self._safe_stop("front_obstacle")
            return None
        return {"front": front_val, "center": front_val}

    def _reset_detection_counters(self) -> None:
        self._target_seen_frames = 0
        self._aligned_frames = 0
        self._target_missing_frames = 0

    @staticmethod
    def _valid_readings(readings: Any) -> bool:
        if not isinstance(readings, dict):
            return False
        if "front" in readings:
            val = readings["front"]
            return isinstance(val, (int, float)) and np.isfinite(val) and val >= 0
        try:
            values = [float(readings[key]) for key in ("left", "center", "right")]
        except (KeyError, TypeError, ValueError):
            return False
        return all(np.isfinite(value) and value >= 0 for value in values)

    def _safe_stop(self, reason: str) -> None:
        self.serial.send_stop()
        self.state = State.STOPPED
        self.stop_reason = reason
        self._turn_action = None
        self._turn_deadline = None

    def _draw_status(self, frame: np.ndarray) -> np.ndarray:
        output = frame.copy()
        target = "-" if self.target_id is None else str(self.target_id)
        phase = "DIRECT" if self.mission is None else self.mission.phase.value
        text = f"{self.state.value} | {phase} | marker {target}"
        cv2.rectangle(output, (0, 0), (min(output.shape[1], 520), 34), (0, 0, 0), -1)
        cv2.putText(output, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                    (255, 255, 255), 2, cv2.LINE_AA)
        return output
