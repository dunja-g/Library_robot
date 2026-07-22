"""Thread-safe navigation state machine for ArUco-guided movement."""

from __future__ import annotations

import threading
import time
from enum import Enum
from typing import Any

import cv2
import numpy as np


class State(Enum):
    IDLE = "IDLE"
    SCANNING = "SCANNING"
    ALIGNING = "ALIGNING"
    APPROACHING = "APPROACHING"
    ARRIVED = "ARRIVED"
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
        clock=time.monotonic,
    ):
        if frame_width <= 0 or align_tolerance_px < 0:
            raise ValueError("frame dimensions and alignment tolerance are invalid")
        if stop_distance_cm <= 0 or obstacle_distance_cm <= 0:
            raise ValueError("distance thresholds must be positive")
        if scan_timeout_seconds <= 0:
            raise ValueError("scan timeout must be positive")
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
        self._clock = clock

        self.state = State.IDLE
        self.target_id: int | None = None
        self.stop_reason: str | None = None
        self._scan_started_at: float | None = None
        self._latest_frame: np.ndarray | None = None
        self._target_seen_frames = 0
        self._aligned_frames = 0
        self._target_missing_frames = 0
        self._lock = threading.RLock()

    def request_book(self, aruco_id: int) -> None:
        if not isinstance(aruco_id, int) or isinstance(aruco_id, bool):
            raise TypeError("aruco_id must be an integer")
        if aruco_id < 0:
            raise ValueError("aruco_id must be non-negative")

        with self._lock:
            self.serial.send_stop()
            self.target_id = aruco_id
            self.stop_reason = None
            self._scan_started_at = self._clock()
            self._reset_detection_counters()
            self.state = State.SCANNING

    def get_state(self) -> str:
        with self._lock:
            return self.state.value

    def get_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "state": self.state.value,
                "target_id": self.target_id,
                "reason": self.stop_reason,
            }

    def get_latest_frame(self) -> np.ndarray | None:
        with self._lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    def reset(self) -> None:
        with self._lock:
            self.serial.send_stop()
            self.state = State.IDLE
            self.target_id = None
            self.stop_reason = None
            self._scan_started_at = None
            self._reset_detection_counters()

    def step(self) -> np.ndarray:
        """Run one control iteration and return the annotated BGR frame."""
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

                annotated = self.detector.draw(frame, detections)
                annotated = self._draw_status(annotated)
                self._latest_frame = annotated.copy()
                return annotated
            except Exception as exc:
                self._safe_stop(f"controller_error:{type(exc).__name__}")
                raise

    def _target_detection(
        self, detections: list[dict[str, Any]]
    ) -> dict[str, Any] | None:
        return next(
            (
                detection
                for detection in detections
                if detection.get("id") == self.target_id
            ),
            None,
        )

    def _do_scanning(self, detections: list[dict[str, Any]]) -> None:
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
        target = self._target_detection(detections)
        if target is None:
            self.serial.send_stop()
            self._target_missing_frames += 1
            self._aligned_frames = 0
            if self._target_missing_frames > self.target_loss_tolerance_frames:
                self.state = State.SCANNING
                self._scan_started_at = self._clock()
                self._target_seen_frames = 0
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
        readings = self.serial.get_ultrasonic()
        if not self._valid_readings(readings):
            self._safe_stop("ultrasonic_unavailable")
            return

        left = float(readings["left"])
        center = float(readings["center"])
        right = float(readings["right"])

        if left < self.obstacle_distance_cm or right < self.obstacle_distance_cm:
            self._safe_stop("side_obstacle")
            return
        if center <= self.stop_distance_cm:
            self.serial.send_stop()
            self.state = State.ARRIVED
            self.stop_reason = "target_distance_reached"
            return

        target = self._target_detection(detections)
        if target is None:
            self.serial.send_stop()
            self._target_missing_frames += 1
            if self._target_missing_frames > self.target_loss_tolerance_frames:
                self.state = State.SCANNING
                self._scan_started_at = self._clock()
                self._target_seen_frames = 0
            return

        self._target_missing_frames = 0
        error = int(target["center_x"]) - self.frame_width / 2.0
        if abs(error) > self.align_tolerance_px:
            self.serial.send_stop()
            self.state = State.ALIGNING
            return
        self.serial.send_forward()

    def _reset_detection_counters(self) -> None:
        self._target_seen_frames = 0
        self._aligned_frames = 0
        self._target_missing_frames = 0

    @staticmethod
    def _valid_readings(readings: Any) -> bool:
        if not isinstance(readings, dict):
            return False
        try:
            values = [float(readings[key]) for key in ("left", "center", "right")]
        except (KeyError, TypeError, ValueError):
            return False
        return all(np.isfinite(value) and value >= 0 for value in values)

    def _safe_stop(self, reason: str) -> None:
        self.serial.send_stop()
        self.state = State.STOPPED
        self.stop_reason = reason

    def _draw_status(self, frame: np.ndarray) -> np.ndarray:
        output = frame.copy()
        target_text = "-" if self.target_id is None else str(self.target_id)
        text = f"{self.state.value} | target {target_text}"
        cv2.rectangle(output, (0, 0), (min(output.shape[1], 360), 34), (0, 0, 0), -1)
        cv2.putText(
            output,
            text,
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return output
