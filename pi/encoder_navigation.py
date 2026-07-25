"""Non-blocking fused-odometry navigation for the fixed 1A-3B grid."""

from __future__ import annotations

import threading
import time
from copy import deepcopy
from enum import Enum
from typing import Any

import numpy as np


class GridState(str, Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    TURNING = "TURNING"
    ARRIVED = "ARRIVED"
    DWELLING = "DWELLING"
    RETURNING = "RETURNING"
    DOCKED = "DOCKED"
    STOPPED = "STOPPED"


class GridController:
    def __init__(
        self,
        serial_bridge: Any,
        *,
        obstacle_distance_cm: float = 20.0,
        destination_dwell_seconds: float = 5.0,
        encoder_stall_seconds: float = 2.0,
        turn_source: str = "encoder",
        frame_provider: Any | None = None,
        aruco_detector: Any | None = None,
        align_tolerance_px: int = 30,
        alignment_confirmation_frames: int = 2,
        target_loss_tolerance_frames: int = 3,
        aruco_target_area_px: float = 8000.0,
        aruco_scan_timeout_seconds: float = 60.0,
        aruco_align_pulse_seconds: float = 0.2,
        aruco_align_settle_seconds: float = 2.0,
        aruco_align_fine_pulse_seconds: float = 0.12,
        aruco_approach_extra_ticks: float = 0.0,
        aruco_approach_extra_seconds: float = 0.0,
        base_trim: int = 15,
        aruco_steering_kp: float = -0.15,
        clock=time.monotonic,
    ):
        if obstacle_distance_cm <= 0 or encoder_stall_seconds <= 0:
            raise ValueError("Safety distance and encoder stall timeout must be positive")
        if destination_dwell_seconds < 0:
            raise ValueError("Destination dwell must be non-negative")
        if turn_source not in {"encoder", "imu"}:
            raise ValueError("turn_source must be 'encoder' or 'imu'")
        if align_tolerance_px < 0:
            raise ValueError("align_tolerance_px must be non-negative")
        if alignment_confirmation_frames <= 0:
            raise ValueError("alignment_confirmation_frames must be positive")
        if target_loss_tolerance_frames < 0:
            raise ValueError("target_loss_tolerance_frames must be non-negative")
        if aruco_target_area_px <= 0:
            raise ValueError("aruco_target_area_px must be positive")
        if aruco_scan_timeout_seconds <= 0:
            raise ValueError("aruco_scan_timeout_seconds must be positive")
        if aruco_align_pulse_seconds <= 0:
            raise ValueError("aruco_align_pulse_seconds must be positive")
        if aruco_align_settle_seconds <= 0:
            raise ValueError("aruco_align_settle_seconds must be positive")
        if aruco_align_fine_pulse_seconds <= 0:
            raise ValueError("aruco_align_fine_pulse_seconds must be positive")
        if aruco_approach_extra_ticks < 0:
            raise ValueError("aruco_approach_extra_ticks must be non-negative")
        if aruco_approach_extra_seconds < 0:
            raise ValueError("aruco_approach_extra_seconds must be non-negative")
        self.serial = serial_bridge
        self.obstacle_distance_cm = float(obstacle_distance_cm)
        self.destination_dwell_seconds = float(destination_dwell_seconds)
        self.encoder_stall_seconds = float(encoder_stall_seconds)
        self.turn_source = turn_source
        self.frame_provider = frame_provider
        self.aruco_detector = aruco_detector
        self.align_tolerance_px = int(align_tolerance_px)
        self.alignment_confirmation_frames = int(alignment_confirmation_frames)
        self.target_loss_tolerance_frames = int(target_loss_tolerance_frames)
        self.aruco_target_area_px = float(aruco_target_area_px)
        self.aruco_scan_timeout_seconds = float(aruco_scan_timeout_seconds)
        self.aruco_align_pulse_seconds = float(aruco_align_pulse_seconds)
        self.aruco_align_settle_seconds = float(aruco_align_settle_seconds)
        self.aruco_align_fine_pulse_seconds = float(aruco_align_fine_pulse_seconds)
        self.aruco_approach_extra_ticks = float(aruco_approach_extra_ticks)
        self.aruco_approach_extra_seconds = float(aruco_approach_extra_seconds)
        self.base_trim = int(base_trim)
        self.aruco_steering_kp = float(aruco_steering_kp)
        self._clock = clock
        self._lock = threading.RLock()
        self.state = GridState.IDLE
        self.stop_reason: str | None = None
        self.plan: dict | None = None
        self.phase: str | None = None
        self.step_index = 0
        self._dwell_deadline: float | None = None
        self._last_progress_ticks = 0.0
        self._last_progress_at: float | None = None
        self._step_deadline: float | None = None  # timed mode: drive until this time
        self._latest_encoders: dict | None = None
        self._latest_odometry: dict | None = None
        self._latest_ultrasonic: dict | None = None
        self._latest_turn_status: str | None = None
        self._awaiting_pickup_confirmation = False
        self._aligned_frames = 0
        self._target_missing_frames = 0
        self._current_trim = self.base_trim
        self._align_pulse_deadline: float | None = None
        self._align_settle_until: float | None = None
        self._align_pulse_is_fine: bool = False

    def request_grid_mission(self, plan: dict) -> None:
        if not plan.get("outbound") or not plan.get("return"):
            raise ValueError("Grid plan requires outbound and return steps")
        with self._lock:
            if self.state not in {GridState.IDLE, GridState.DOCKED}:
                raise RuntimeError("A grid mission is already active")
            self.serial.send_stop()
            self.plan = deepcopy(plan)
            self.phase = "OUTBOUND"
            self.step_index = 0
            self.stop_reason = None
            self._dwell_deadline = None
            self._step_deadline = None
            self._awaiting_pickup_confirmation = bool(
                plan.get("pickup_confirmation_required", False)
            )
            self._start_current_step()

    def get_state(self) -> str:
        with self._lock:
            return self.state.value

    def get_status(self) -> dict:
        with self._lock:
            display_steps = self._status_steps()
            status = {
                "state": self.state.value,
                "reason": self.stop_reason,
                "navigation_mode": "grid_aruco_hybrid"
                if self._uses_aruco_steps()
                else "grid_fused_odometry",
                "active_controller": "GridController",
                "return_strategy": "direct_reverse",
                "phase": self.phase,
                "box_id": None if self.plan is None else self.plan["box_id"],
                "book": None if self.plan is None else self.plan.get("book"),
                "book_code": None
                if self.plan is None
                else self.plan.get("book_code"),
                "location_code": None
                if self.plan is None
                else self.plan.get("location_code"),
                "layer": None if self.plan is None else self.plan.get("layer"),
                "position": None
                if self.plan is None
                else self.plan.get("position"),
                "row": None if self.plan is None else self.plan["row"],
                "column": None if self.plan is None else self.plan["column"],
                "step_index": min(self.step_index + 1, len(display_steps))
                if display_steps
                else 0,
                "step_count": len(display_steps),
                "current_action": None,
                "current_step_label": None,
                "target_ticks": None,
                "pickup_confirmation_required": (
                    self.state == GridState.ARRIVED
                    and self._awaiting_pickup_confirmation
                ),
                "return_actions": []
                if self.plan is None
                else [item["action"] for item in self.plan["return"]],
            }
            step = self._current_step()
            if step:
                status.update(
                    current_action=step["action"],
                    current_step_label=step.get("label"),
                    target_ticks=step.get("target_ticks"),
                )
            target_ticks = float(step.get("target_ticks", 0.0)) if step else 0.0
            encoder_progress = 0.0
            if self._latest_encoders:
                encoder_progress = min(
                    abs(float(self._latest_encoders["left"])),
                    abs(float(self._latest_encoders["right"])),
                )
            if self.state == GridState.TURNING and self.turn_source == "imu":
                segment_progress = 50 if self._latest_turn_status == "ACTIVE" else 0
            elif target_ticks > 0:
                segment_progress = min(100, round(encoder_progress / target_ticks * 100))
            else:
                segment_progress = 0
            status["telemetry"] = {
                "segment_progress_percent": segment_progress,
                "encoders": {
                    "status": "OK" if self._latest_encoders is not None else "WAITING",
                    "left": None
                    if self._latest_encoders is None
                    else self._latest_encoders["left"],
                    "right": None
                    if self._latest_encoders is None
                    else self._latest_encoders["right"],
                    "left_cm": None
                    if self._latest_odometry is None
                    else self._latest_odometry.get("left_cm"),
                    "right_cm": None
                    if self._latest_odometry is None
                    else self._latest_odometry.get("right_cm"),
                    "distance_cm": None
                    if self._latest_odometry is None
                    else (
                        float(self._latest_odometry["left_cm"])
                        + float(self._latest_odometry["right_cm"])
                    )
                    / 2.0,
                },
                "imu": {
                    "status": self._latest_turn_status
                    or ("READY" if self.turn_source == "imu" else "DISABLED"),
                    "heading_encoder_deg": None
                    if self._latest_odometry is None
                    else self._latest_odometry.get("heading_encoder_deg"),
                    "heading_imu_deg": None
                    if self._latest_odometry is None
                    else self._latest_odometry.get("heading_imu_deg"),
                    "heading_fused_deg": None
                    if self._latest_odometry is None
                    else self._latest_odometry.get("heading_fused_deg"),
                    "speed_correction": None
                    if self._latest_odometry is None
                    else self._latest_odometry.get("speed_correction"),
                },
                "ultrasonic": {
                    "status": "OK"
                    if self._latest_ultrasonic is not None
                    else "WAITING",
                    **({} if self._latest_ultrasonic is None else self._latest_ultrasonic),
                },
            }
            return status

    def _status_steps(self) -> list[dict]:
        if self.plan is None:
            return []
        if self.phase in {"OUTBOUND", "AT_DESTINATION"}:
            return self.plan["outbound"]
        return self.plan["return"]

    def reset(self) -> None:
        with self._lock:
            self.serial.send_stop()
            self.state = GridState.IDLE
            self.stop_reason = None
            self.plan = None
            self.phase = None
            self.step_index = 0
            self._dwell_deadline = None
            self._step_deadline = None
            self._latest_encoders = None
            self._latest_odometry = None
            self._latest_ultrasonic = None
            self._latest_turn_status = None
            self._awaiting_pickup_confirmation = False
            self._aligned_frames = 0
            self._target_missing_frames = 0
            self._current_trim = self.base_trim
            self._align_pulse_deadline = None
            self._align_settle_until = None
            self._align_pulse_is_fine = False

    def confirm_pickup(self) -> None:
        """Start the return route after the user confirms taking the book."""
        with self._lock:
            if (
                self.state != GridState.ARRIVED
                or self.phase != "AT_DESTINATION"
                or not self._awaiting_pickup_confirmation
            ):
                raise RuntimeError("Pickup confirmation is not currently expected")
            self._awaiting_pickup_confirmation = False
            self.phase = "RETURNING"
            self.step_index = 0
            self.stop_reason = None
            self._dwell_deadline = None
            self._step_deadline = None
            self._start_current_step()

    def cancel(self, reason: str = "mission_cancelled") -> None:
        """Stop an active mission without pretending that the robot is docked."""
        with self._lock:
            self._safe_stop(reason)

    def step(self) -> None:
        with self._lock:
            try:
                if self.state in {
                    GridState.IDLE,
                    GridState.ARRIVED,
                    GridState.DWELLING,
                    GridState.DOCKED,
                    GridState.STOPPED,
                }:
                    if self.state in {GridState.ARRIVED, GridState.DWELLING}:
                        self._step_dwell()
                    return

                # Generic step timeout removed because ARUCO_APPROACH and timed linear steps handle their own.
                if not self._safety_clear():
                    return
                step = self._current_step()
                if step is None:
                    self._safe_stop("route_state_error")
                    return
                action = step["action"]
                if action == "ARUCO_ALIGN":
                    self._step_aruco_align()
                    return
                if action == "ARUCO_APPROACH":
                    self._step_aruco_approach()
                    return
                if action == "ARUCO_GUIDED_FORWARD":
                    self._step_aruco_guided_forward()
                    return
                if self.state == GridState.TURNING and self.turn_source == "imu":
                    self._step_imu_turn()
                    return
                # --- Timed forward/backward mode (no encoder wires required) ---
                if step.get("target_seconds", 0.0) > 0.0 and step["action"] in {"FORWARD", "BACKWARD"}:
                    self._step_timed_linear()
                    return
                # --- Encoder mode ---
                get_odometry = getattr(self.serial, "get_odometry", None)
                odometry = get_odometry() if callable(get_odometry) else None
                encoders = odometry if odometry is not None else self.serial.get_encoders()
                if not self._valid_encoders(encoders):
                    self._safe_stop("encoder_unavailable")
                    return
                self._latest_encoders = dict(encoders)
                if odometry is not None:
                    self._latest_odometry = dict(odometry)
                # Require both drivetrain sides to progress. Using an average
                # could hide one stalled wheel while the other keeps counting.
                progress = min(
                    abs(float(encoders["left"])),
                    abs(float(encoders["right"])),
                )
                if progress >= float(step["target_ticks"]):
                    self.serial.send_stop()
                    self.step_index += 1
                    if self.step_index >= len(self._current_steps()):
                        self._complete_phase()
                    else:
                        self._start_current_step()
                    return
                self._check_stall(progress)
                if self.state != GridState.STOPPED:
                    self._send_action(step["action"])
            except Exception as exc:
                self._safe_stop(f"controller_error:{type(exc).__name__}")
                raise

    def _current_steps(self) -> list[dict]:
        if self.plan is None or self.phase not in {"OUTBOUND", "RETURNING"}:
            return []
        key = "outbound" if self.phase == "OUTBOUND" else "return"
        return self.plan[key]

    def _current_step(self) -> dict | None:
        steps = self._current_steps()
        return None if self.step_index >= len(steps) else steps[self.step_index]

    def _start_current_step(self) -> None:
        step = self._current_step()
        if step is None:
            raise RuntimeError("No grid route step is available")
        step.pop("aruco_locked", None)
        step.pop("aruco_creep_active", None)
        step.pop("aruco_creep_target_ticks", None)
        step.pop("aruco_creep_deadline", None)
        self._aligned_frames = 0
        self._target_missing_frames = 0
        action = step["action"]
        if action == "ARUCO_ALIGN":
            self._align_pulse_deadline = None
            self._align_settle_until = None
            self._align_pulse_is_fine = False
            self._step_deadline = self._clock() + self.aruco_scan_timeout_seconds
            self.state = GridState.TURNING
            return
        if action == "ARUCO_APPROACH":
            self._step_deadline = self._clock() + self.aruco_scan_timeout_seconds
            self._latest_encoders = {"left": 0, "right": 0}
            self._latest_odometry = None
            self.state = GridState.MOVING
            return
        if action == "ARUCO_GUIDED_FORWARD":
            if not self.serial.reset_encoders():
                self._safe_stop("encoder_reset_failed")
                return
            self._last_progress_ticks = 0.0
            self._last_progress_at = self._clock()
            self._step_deadline = self._clock() + self.aruco_scan_timeout_seconds
            self._latest_encoders = {"left": 0, "right": 0}
            self._latest_odometry = None
            self.state = GridState.MOVING
            return
        # IMU turns
        if (
            self.turn_source == "imu"
            and step["action"] in {"TURN_LEFT", "TURN_RIGHT", "UTURN"}
        ):
            if not self.serial.reset_encoders():
                self._safe_stop("encoder_reset_failed")
                return
            self.state = GridState.TURNING
            self._step_deadline = None
            self._latest_encoders = {"left": 0, "right": 0}
            self._latest_odometry = None
            self._latest_turn_status = "ACTIVE"
            self._send_imu_turn(step["action"])
            return
        # Timed linear step
        if step.get("target_seconds", 0.0) > 0.0 and step["action"] in {"FORWARD", "BACKWARD"}:
            if not self.serial.reset_encoders():
                self._safe_stop("encoder_reset_failed")
                return
            self._last_progress_ticks = 0.0
            self._last_progress_at = self._clock()
            self._latest_encoders = {"left": 0, "right": 0}
            self._latest_odometry = None
            self._step_deadline = self._clock() + step["target_seconds"]
            self.state = GridState.MOVING
            self._send_action(step["action"])
            return
        # Encoder-based step
        if not self.serial.reset_encoders():
            self._safe_stop("encoder_reset_failed")
            return
        self._last_progress_ticks = 0.0
        self._last_progress_at = self._clock()
        self._step_deadline = None
        self._latest_encoders = {"left": 0, "right": 0}
        self.state = (
            GridState.TURNING
            if step["action"] in {"TURN_LEFT", "TURN_RIGHT", "UTURN"}
            else GridState.MOVING
        )
        self._send_action(step["action"])

    def _send_imu_turn(self, action: str) -> None:
        step = self._current_step()
        deg = step.get("target_degrees") if step else None
        if action == "TURN_LEFT":
            sent = self.serial.send_turn_left(deg)
        elif action == "TURN_RIGHT":
            sent = self.serial.send_turn_right(deg)
        else:
            sent = self.serial.send_turn_uturn(deg)
        if not sent:
            self._safe_stop("imu_turn_command_failed")

    def _step_imu_turn(self) -> None:
        get_odometry = getattr(self.serial, "get_odometry", None)
        if callable(get_odometry):
            odometry = get_odometry()
            if odometry is not None and self._valid_encoders(odometry):
                self._latest_encoders = dict(odometry)
                self._latest_odometry = dict(odometry)
        status = self.serial.get_turn_status()
        self._latest_turn_status = status
        if status == "ACTIVE":
            return
        if status == "DONE":
            self.serial.send_stop()
            self.step_index += 1
            if self.step_index >= len(self._current_steps()):
                self._complete_phase()
            else:
                self._start_current_step()
            return
        self._safe_stop(
            "imu_turn_error" if status in {"ERROR", "IDLE"} else "imu_unavailable"
        )

    def _complete_phase(self) -> None:
        if self.phase == "OUTBOUND":
            self.state = GridState.ARRIVED
            self.phase = "AT_DESTINATION"
            self.stop_reason = "destination_reached"
            self._dwell_deadline = (
                None
                if self._awaiting_pickup_confirmation
                else self._clock() + self.destination_dwell_seconds
            )
        else:
            self.state = GridState.DOCKED
            self.phase = "COMPLETE"
            self.stop_reason = "dock_reached"

    def _step_dwell(self) -> None:
        self.serial.send_stop()
        if self._awaiting_pickup_confirmation:
            return
        if self._dwell_deadline is None or self._clock() < self._dwell_deadline:
            return
        self.phase = "RETURNING"
        self.step_index = 0
        self.stop_reason = None
        self._dwell_deadline = None
        self._step_deadline = None
        self._start_current_step()

    def _step_timed_linear(self) -> None:
        """Advance a FORWARD or BACKWARD step using a time deadline instead of encoder ticks."""
        if self._step_deadline is None:
            # Shouldn't happen, but recover gracefully.
            self._safe_stop("timed_step_no_deadline")
            return
        step = self._current_step()
        self._send_action(step["action"])
        if self._clock() >= self._step_deadline:
            self.serial.send_stop()
            self._step_deadline = None
            self.step_index += 1
            if self.step_index >= len(self._current_steps()):
                self._complete_phase()
            else:
                self._start_current_step()

    def _uses_aruco_steps(self) -> bool:
        if self.plan is None:
            return False
        aruco_actions = {"ARUCO_ALIGN", "ARUCO_APPROACH", "ARUCO_GUIDED_FORWARD"}
        for key in ("outbound", "return"):
            for step in self.plan.get(key, []):
                if step.get("action") in aruco_actions:
                    return True
        return False

    def _vision_ready(self) -> bool:
        return self.frame_provider is not None and self.aruco_detector is not None

    def _read_frame_and_detection(self, target_id: int) -> tuple[Any | None, dict | None]:
        if not self._vision_ready():
            self._safe_stop("aruco_unavailable_no_camera")
            return None, None
        try:
            frame = self.frame_provider()
        except Exception:
            self._safe_stop("camera_read_failed")
            return None, None
        if frame is None:
            return None, None
        detection = self.aruco_detector.detect_target(frame, target_id)
        return frame, detection

    def _apply_aruco_steering(self, frame: Any, detection: dict) -> None:
        frame_width = frame.shape[1]
        error_x = float(detection["center_x"]) - (frame_width / 2.0)
        adjustment = int(error_x * self.aruco_steering_kp)
        new_trim = self.base_trim + adjustment
        new_trim = max(self.base_trim - 30, min(self.base_trim + 30, new_trim))
        if new_trim != self._current_trim and hasattr(self.serial, "set_trim"):
            self._current_trim = new_trim
            self.serial.set_trim(new_trim)

    def _reset_aruco_trim(self) -> None:
        if self._current_trim != self.base_trim and hasattr(self.serial, "set_trim"):
            self.serial.set_trim(self.base_trim)
        self._current_trim = self.base_trim

    def _advance_step(self) -> None:
        self._reset_aruco_trim()
        self.step_index += 1
        if self.step_index >= len(self._current_steps()):
            self._complete_phase()
        else:
            self._start_current_step()

    def _start_align_pulse(self, direction: str, *, fine: bool = False) -> None:
        duration = (
            self.aruco_align_fine_pulse_seconds
            if fine
            else self.aruco_align_pulse_seconds
        )
        self._align_pulse_is_fine = fine
        self._align_settle_until = None
        self._align_pulse_deadline = self._clock() + duration
        self.state = GridState.TURNING
        if direction == "left":
            self.serial.send_rotate_left()
        else:
            self.serial.send_rotate_right()

    def _begin_align_settle(self) -> None:
        """Hold still so the camera can capture a stable frame after a missed detect."""
        self.serial.send_stop()
        self._align_pulse_deadline = None
        self._align_pulse_is_fine = False
        self._align_settle_until = self._clock() + self.aruco_align_settle_seconds
        self.state = GridState.TURNING

    def _handle_align_detection(self, frame: Any, detection: dict | None) -> None:
        if detection is None:
            self._aligned_frames = 0
            self._begin_align_settle()
            return

        self._target_missing_frames = 0
        error = float(detection["center_x"]) - (frame.shape[1] / 2.0)
        if abs(error) <= self.align_tolerance_px:
            self.serial.send_stop()
            self._aligned_frames += 1
            if self._aligned_frames >= self.alignment_confirmation_frames:
                self._step_deadline = None
                self._advance_step()
            return

        self._aligned_frames = 0
        self._start_align_pulse("left" if error < 0 else "right", fine=True)

    def _step_aruco_align(self) -> None:
        """Pulse-rotate, stop, settle on miss, then search and fine-tune to centre."""
        step = self._current_step()
        target_id = int(step["target_aruco_id"])

        if self._step_deadline and self._clock() > self._step_deadline:
            self.serial.send_stop()
            self._align_pulse_deadline = None
            self._align_settle_until = None
            self._align_pulse_is_fine = False
            self._safe_stop("aruco_scan_timeout")
            return

        if self._align_settle_until is not None:
            self.serial.send_stop()
            if self._clock() < self._align_settle_until:
                return
            self._align_settle_until = None
            frame, detection = self._read_frame_and_detection(target_id)
            if frame is None and self.stop_reason:
                return
            if frame is None:
                return
            if detection is None:
                self._aligned_frames = 0
                self._start_align_pulse("left", fine=False)
                return
            self._handle_align_detection(frame, detection)
            return

        if self._align_pulse_deadline is not None:
            if self._clock() < self._align_pulse_deadline:
                return
            self.serial.send_stop()
            self._align_pulse_deadline = None
            self._align_pulse_is_fine = False
            frame, detection = self._read_frame_and_detection(target_id)
            if frame is None and self.stop_reason:
                return
            if frame is None:
                return
            self._handle_align_detection(frame, detection)
            return

        frame, detection = self._read_frame_and_detection(target_id)
        if frame is None and self.stop_reason:
            return
        if frame is None:
            return
        if detection is None:
            self._target_missing_frames += 1
            self._aligned_frames = 0
            self._start_align_pulse("left", fine=False)
            return
        self._handle_align_detection(frame, detection)

    def _step_aruco_guided_forward(self) -> None:
        """Drive forward for a fixed encoder distance while tracking a marker."""
        step = self._current_step()
        target_id = int(step["target_aruco_id"])
        target_ticks = float(step["target_ticks"])
        get_odometry = getattr(self.serial, "get_odometry", None)
        odometry = get_odometry() if callable(get_odometry) else None
        encoders = odometry if odometry is not None else self.serial.get_encoders()
        if not self._valid_encoders(encoders):
            self._safe_stop("encoder_unavailable")
            return
        self._latest_encoders = dict(encoders)
        if odometry is not None:
            self._latest_odometry = dict(odometry)
        progress = min(
            abs(float(encoders["left"])),
            abs(float(encoders["right"])),
        )
        if progress >= target_ticks:
            self.serial.send_stop()
            self._step_deadline = None
            self._advance_step()
            return
        frame, detection = self._read_frame_and_detection(target_id)
        if frame is None and self.stop_reason:
            return
        if detection is not None:
            step["aruco_locked"] = True
            self._apply_aruco_steering(frame, detection)
        elif step.get("aruco_locked"):
            self._reset_aruco_trim()
        elif self._step_deadline and self._clock() > self._step_deadline:
            step["aruco_locked"] = True
            self._step_deadline = None
        self._check_stall(progress)
        if self.state != GridState.STOPPED:
            if not self.serial.send_forward():
                self._safe_stop("serial_command_failed")

    def _begin_aruco_approach_creep(self, step: dict) -> None:
        """Drive a little further inward after the marker fills the target area."""
        self._reset_aruco_trim()
        self.serial.send_stop()
        if self.aruco_approach_extra_ticks > 0:
            if not self.serial.reset_encoders():
                self._safe_stop("encoder_reset_failed")
                return
            step["aruco_creep_active"] = True
            step["aruco_creep_target_ticks"] = self.aruco_approach_extra_ticks
            self._last_progress_ticks = 0.0
            self._last_progress_at = self._clock()
            self._latest_encoders = {"left": 0, "right": 0}
            if not self.serial.send_forward():
                self._safe_stop("serial_command_failed")
            return
        if self.aruco_approach_extra_seconds > 0:
            step["aruco_creep_active"] = True
            step["aruco_creep_deadline"] = (
                self._clock() + self.aruco_approach_extra_seconds
            )
            if not self.serial.send_forward():
                self._safe_stop("serial_command_failed")
            return
        self._advance_step()

    def _step_aruco_approach_creep(self, step: dict) -> None:
        deadline = step.get("aruco_creep_deadline")
        if deadline is not None:
            if self._clock() < deadline:
                if not self.serial.send_forward():
                    self._safe_stop("serial_command_failed")
                return
            self.serial.send_stop()
            self._advance_step()
            return

        target_ticks = float(step.get("aruco_creep_target_ticks", 0.0))
        get_odometry = getattr(self.serial, "get_odometry", None)
        odometry = get_odometry() if callable(get_odometry) else None
        encoders = odometry if odometry is not None else self.serial.get_encoders()
        if not self._valid_encoders(encoders):
            self._safe_stop("encoder_unavailable")
            return
        self._latest_encoders = dict(encoders)
        if odometry is not None:
            self._latest_odometry = dict(odometry)
        progress = min(
            abs(float(encoders["left"])),
            abs(float(encoders["right"])),
        )
        if progress >= target_ticks:
            self.serial.send_stop()
            self._advance_step()
            return
        self._check_stall(progress)
        if self.state != GridState.STOPPED:
            if not self.serial.send_forward():
                self._safe_stop("serial_command_failed")

    def _step_aruco_approach(self) -> None:
        """Steer toward a marker and stop when it fills enough of the camera frame."""
        step = self._current_step()
        if step.get("aruco_creep_active"):
            self._step_aruco_approach_creep(step)
            return
        target_id = int(step["target_aruco_id"])
        frame, detection = self._read_frame_and_detection(target_id)
        if frame is None and self.stop_reason:
            return
        if frame is None:
            return
        if detection is None:
            if not step.get("aruco_locked"):
                if self._step_deadline and self._clock() > self._step_deadline:
                    self._safe_stop("aruco_scan_timeout")
                return
            self._reset_aruco_trim()
            if not self.serial.send_forward():
                self._safe_stop("serial_command_failed")
            return

        if not step.get("aruco_locked"):
            step["aruco_locked"] = True
            self._step_deadline = None

        if float(detection["area"]) >= self.aruco_target_area_px:
            self._begin_aruco_approach_creep(step)
            return

        self._apply_aruco_steering(frame, detection)
        if not self.serial.send_forward():
            self._safe_stop("serial_command_failed")


    def _check_stall(self, progress: float) -> None:
        now = self._clock()
        if progress > self._last_progress_ticks:
            self._last_progress_ticks = progress
            self._last_progress_at = now
        elif self._last_progress_at is not None and (
            now - self._last_progress_at >= self.encoder_stall_seconds
        ):
            self._safe_stop("encoder_stall")

    def _send_action(self, action: str) -> None:
        if action == "FORWARD":
            sent = self.serial.send_forward()
        elif action == "BACKWARD":
            sent = self.serial.send_backward()
        elif action in {"TURN_LEFT", "UTURN"}:
            sent = self.serial.send_rotate_left()
        elif action == "TURN_RIGHT":
            sent = self.serial.send_rotate_right()
        else:
            raise ValueError(f"Unsupported grid action: {action}")
        if not sent:
            self._safe_stop("serial_command_failed")

    def _safety_clear(self) -> bool:
        readings = self.serial.get_ultrasonic()
        if not self._valid_ultrasonic(readings):
            self._safe_stop("ultrasonic_unavailable")
            return False
        self._latest_ultrasonic = dict(readings)

        # The chassis has no rear-facing ultrasonic sensor. During reverse we
        # can still validate all sensor data and enforce both side sensors, but
        # the front-centre sensor faces the shelf we are backing away from.
        # The rear corridor therefore must be cleared and supervised.
        step = self._current_step()
        if step is not None and step.get("action") == "BACKWARD":
            directions = ("left", "right")
        else:
            directions = ("left", "center", "right")

        for direction in directions:
            if float(readings[direction]) < self.obstacle_distance_cm:
                self._safe_stop(f"{direction}_obstacle")
                return False
        return True

    @staticmethod
    def _valid_encoders(readings: Any) -> bool:
        if not isinstance(readings, dict):
            return False
        try:
            values = [float(readings[key]) for key in ("left", "right")]
        except (KeyError, TypeError, ValueError):
            return False
        return all(np.isfinite(value) for value in values)

    @staticmethod
    def _valid_ultrasonic(readings: Any) -> bool:
        if not isinstance(readings, dict):
            return False
        try:
            values = [float(readings[key]) for key in ("left", "center", "right")]
        except (KeyError, TypeError, ValueError):
            return False
        return all(np.isfinite(value) and value >= 0 for value in values)

    def _safe_stop(self, reason: str) -> None:
        self.serial.send_stop()
        self.state = GridState.STOPPED
        self.stop_reason = reason
