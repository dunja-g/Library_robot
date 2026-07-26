"""Non-blocking fused-odometry navigation for the fixed 1A-3B grid."""

from __future__ import annotations

import threading
import time
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
from typing import Any

import numpy as np


MIN_WHEEL_COMPLETION_RATIO = 0.5


class GridState(str, Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    TURNING = "TURNING"
    ARRIVED = "ARRIVED"
    DWELLING = "DWELLING"
    RETURNING = "RETURNING"
    DOCKED = "DOCKED"
    STOPPED = "STOPPED"


class CandidateState(str, Enum):
    MISSING = "MISSING"
    PENDING = "PENDING"
    CONFIRMED = "CONFIRMED"


@dataclass(frozen=True)
class CandidateObservation:
    state: CandidateState
    error_x: float | None = None


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
        aruco_align_fine_settle_seconds: float = 0.4,
        aruco_align_max_search_pulses: int = 6,
        aruco_align_max_reacquire_pulses: int = 2,
        aruco_align_invert_turn: bool = False,
        aruco_track_candidates: bool = True,
        aruco_candidate_confirmation_frames: int = 2,
        aruco_candidate_max_jump_px: float = 80.0,
        return_obstacle_distance_cm: float = 10.0,
        invert_turn_direction: bool = False,
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
        if aruco_align_fine_settle_seconds <= 0:
            raise ValueError("aruco_align_fine_settle_seconds must be positive")
        if aruco_align_max_search_pulses <= 0:
            raise ValueError("aruco_align_max_search_pulses must be positive")
        if aruco_align_max_reacquire_pulses < 0:
            raise ValueError("aruco_align_max_reacquire_pulses must be non-negative")
        if aruco_candidate_confirmation_frames < 2:
            raise ValueError(
                "aruco_candidate_confirmation_frames must be at least 2"
            )
        if aruco_candidate_max_jump_px <= 0:
            raise ValueError("aruco_candidate_max_jump_px must be positive")
        if return_obstacle_distance_cm <= 0:
            raise ValueError("return_obstacle_distance_cm must be positive")
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
        self.aruco_align_fine_settle_seconds = float(aruco_align_fine_settle_seconds)
        self.aruco_align_max_search_pulses = int(aruco_align_max_search_pulses)
        self.aruco_align_max_reacquire_pulses = int(aruco_align_max_reacquire_pulses)
        self.aruco_align_invert_turn = bool(aruco_align_invert_turn)
        self.aruco_track_candidates = bool(aruco_track_candidates)
        self.aruco_candidate_confirmation_frames = int(
            aruco_candidate_confirmation_frames
        )
        self.aruco_candidate_max_jump_px = float(aruco_candidate_max_jump_px)
        self.return_obstacle_distance_cm = float(return_obstacle_distance_cm)
        self.invert_turn_direction = bool(invert_turn_direction)
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
        self._align_settle_short: bool = False
        self._align_last_error_x: float | None = None
        self._align_search_direction: str = "left"
        self._align_search_span: int = 1
        self._align_search_leg_done: int = 0
        self._align_search_total: int = 0
        self._align_reacquire_pulses: int = 0
        self._candidate_last_center: tuple[float, float] | None = None
        self._candidate_stable_frames: int = 0
        self._align_last_target_center: tuple[float, float] | None = None

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
                encoder_progress, _ = self._encoder_progress_values(
                    self._latest_encoders
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
                progress, slow_progress = self._encoder_progress_values(encoders)
                target_ticks = float(step["target_ticks"])
                if (
                    progress >= target_ticks
                    and slow_progress >= target_ticks * MIN_WHEEL_COMPLETION_RATIO
                ):
                    self.serial.send_stop()
                    self.step_index += 1
                    if self.step_index >= len(self._current_steps()):
                        self._complete_phase()
                    else:
                        self._start_current_step()
                    return
                # Completion follows chassis-centre distance, while the slower
                # wheel still drives stall detection.
                self._check_stall(slow_progress)
                if self.state != GridState.STOPPED:
                    self._maybe_apply_aruco_tracking(step)
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
        step.pop("aruco_approach_ticks_before_creep", None)
        self._aligned_frames = 0
        self._target_missing_frames = 0
        action = step["action"]
        if action == "ARUCO_ALIGN":
            self._align_pulse_deadline = None
            self._align_pulse_is_fine = False
            self._reset_align_search_state()
            self._step_deadline = self._clock() + self.aruco_scan_timeout_seconds
            self._begin_align_settle()
            return
        if action == "ARUCO_APPROACH":
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
        if action in {"TURN_LEFT", "UTURN"}:
            sent = self.serial.send_turn_right(deg) if self.invert_turn_direction else self.serial.send_turn_left(deg)
        elif action == "TURN_RIGHT":
            sent = self.serial.send_turn_left(deg) if self.invert_turn_direction else self.serial.send_turn_right(deg)
        else:
            raise ValueError(f"Unsupported IMU turn action: {action}")
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
        self._maybe_apply_aruco_tracking(step)
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

    def _physical_turn_direction(self, direction: str) -> str:
        if direction not in {"left", "right"}:
            raise ValueError("direction must be 'left' or 'right'")
        if self.invert_turn_direction:
            return "right" if direction == "left" else "left"
        return direction

    def _start_align_pulse(self, direction: str, *, fine: bool = False) -> None:
        # Vision alignment is a closed loop: the camera sees the result of the
        # last pulse, so it must not reuse the open-loop route-turn inversion.
        if direction not in {"left", "right"}:
            raise ValueError("direction must be 'left' or 'right'")
        if self.aruco_align_invert_turn:
            direction = "right" if direction == "left" else "left"
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

    def _marker_error_x(self, frame: Any, detection: dict) -> float:
        return float(detection["center_x"]) - (frame.shape[1] / 2.0)

    def _turn_toward_error(self, error_x: float, *, fine: bool) -> None:
        """Turn so an off-centre marker moves toward the middle of the frame."""
        if abs(error_x) <= self.align_tolerance_px:
            return
        direction = "left" if error_x < 0 else "right"
        self._start_align_pulse(direction, fine=fine)

    def _can_reacquire(self) -> bool:
        """True while a briefly lost marker may still be chased in its last direction."""
        return (
            self._align_last_error_x is not None
            and abs(self._align_last_error_x) > self.align_tolerance_px
            and self._align_reacquire_pulses < self.aruco_align_max_reacquire_pulses
        )

    def _reset_align_search_state(self) -> None:
        self._align_settle_short = False
        self._align_last_error_x = None
        self._align_search_direction = "left"
        self._align_search_span = 1
        self._align_search_leg_done = 0
        self._align_search_total = 0
        self._align_reacquire_pulses = 0
        self._reset_candidate_tracking(clear_target=True)

    def _reset_candidate_tracking(self, *, clear_target: bool = False) -> None:
        self._candidate_last_center = None
        self._candidate_stable_frames = 0
        if clear_target:
            self._align_last_target_center = None

    def _candidate_observation(
        self, frame: Any | None, target_id: int
    ) -> CandidateObservation:
        """Return a confirmed marker-like hint without treating it as the target.

        A hint must remain near the last decoded/observed position for multiple
        frames. Even a confirmed, centred hint only holds the robot still; only
        a decoded matching ID can complete alignment.
        """
        step = self._current_step()
        if (
            not self.aruco_track_candidates
            or frame is None
            or step is None
            or step.get("action") != "ARUCO_ALIGN"
        ):
            self._reset_candidate_tracking()
            return CandidateObservation(CandidateState.MISSING)
        detect_candidates = getattr(self.aruco_detector, "detect_candidates", None)
        if not callable(detect_candidates):
            return CandidateObservation(CandidateState.MISSING)
        try:
            candidates = detect_candidates(frame)
        except Exception:
            self._reset_candidate_tracking()
            return CandidateObservation(CandidateState.MISSING)
        if not candidates:
            self._reset_candidate_tracking()
            return CandidateObservation(CandidateState.MISSING)

        anchor = self._candidate_last_center or self._align_last_target_center
        if anchor is not None:
            candidates = [
                item
                for item in candidates
                if (
                    (float(item["center_x"]) - anchor[0]) ** 2
                    + (float(item["center_y"]) - anchor[1]) ** 2
                ) ** 0.5
                <= self.aruco_candidate_max_jump_px
            ]
        if not candidates:
            self._reset_candidate_tracking()
            return CandidateObservation(CandidateState.MISSING)

        best = max(candidates, key=lambda item: float(item["area"]))
        center = (float(best["center_x"]), float(best["center_y"]))
        if self._candidate_last_center is None:
            self._candidate_stable_frames = 1
        else:
            jump = (
                (center[0] - self._candidate_last_center[0]) ** 2
                + (center[1] - self._candidate_last_center[1]) ** 2
            ) ** 0.5
            self._candidate_stable_frames = (
                self._candidate_stable_frames + 1
                if jump <= self.aruco_candidate_max_jump_px
                else 1
            )
        self._candidate_last_center = center
        error = self._marker_error_x(frame, best)
        state = (
            CandidateState.CONFIRMED
            if self._candidate_stable_frames
            >= self.aruco_candidate_confirmation_frames
            else CandidateState.PENDING
        )
        return CandidateObservation(state, error)

    def _bias_search_direction(self) -> None:
        """Aim the first sweep leg at wherever a marker was last seen."""
        if self._align_search_total > 0:
            return

        if (
            self._align_last_error_x is not None
            and abs(self._align_last_error_x) > self.align_tolerance_px
        ):
            self._align_search_direction = (
                "left" if self._align_last_error_x < 0 else "right"
            )
            return

        # Decoded non-target IDs and unconfirmed quads must not bias the sweep.

    def _begin_align_search_pulse(self) -> None:
        """Sweep in widening left/right legs, then give up instead of spinning on."""
        if self._align_search_total >= self.aruco_align_max_search_pulses:
            self._safe_stop("aruco_marker_not_found")
            return

        self._bias_search_direction()
        if self._align_search_leg_done >= self._align_search_span:
            self._align_search_direction = (
                "right" if self._align_search_direction == "left" else "left"
            )
            self._align_search_span += 1
            self._align_search_leg_done = 0

        self._align_search_leg_done += 1
        self._align_search_total += 1
        self._start_align_pulse(self._align_search_direction, fine=False)

    def _begin_align_settle(self, *, short: bool = False) -> None:
        """Hold still so the camera can capture a stable frame after a missed detect."""
        self.serial.send_stop()
        self._align_pulse_deadline = None
        self._align_pulse_is_fine = False
        duration = (
            self.aruco_align_fine_settle_seconds
            if short
            else self.aruco_align_settle_seconds
        )
        self._align_settle_short = short
        self._align_settle_until = self._clock() + duration
        self.state = GridState.TURNING

    def _maybe_apply_aruco_tracking(self, step: dict) -> None:
        """Soft heading correction while moving; never blocks if the marker is lost."""
        track_id = step.get("track_aruco_id")
        if track_id is None or not self._vision_ready():
            return
        frame, detection = self._read_frame_and_detection(int(track_id))
        if frame is None:
            return
        if detection is None:
            self._reset_aruco_trim()
            return
        self._apply_aruco_steering(frame, detection)

    def _handle_align_detection(
        self,
        frame: Any,
        detection: dict | None,
        *,
        on_centered,
    ) -> None:
        if detection is None:
            self._aligned_frames = 0
            self._begin_align_settle(short=self._can_reacquire())
            return

        error = self._marker_error_x(frame, detection)
        self._align_last_target_center = (
            float(detection["center_x"]),
            float(detection["center_y"]),
        )
        self._reset_candidate_tracking()
        self._align_last_error_x = error
        self._target_missing_frames = 0
        self._align_reacquire_pulses = 0
        self._align_search_span = 1
        self._align_search_leg_done = 0
        self._align_search_total = 0
        if abs(error) <= self.align_tolerance_px:
            self.serial.send_stop()
            self._aligned_frames += 1
            if self._aligned_frames >= self.alignment_confirmation_frames:
                self._step_deadline = None
                self._align_pulse_deadline = None
                self._align_settle_until = None
                self._align_pulse_is_fine = False
                self._reset_align_search_state()
                on_centered()
            return

        self._aligned_frames = 0
        self._turn_toward_error(error, fine=True)

    def _step_aruco_centering(self, step: dict, target_id: int, *, on_centered) -> None:
        """Pulse-rotate and settle until the marker is centred, then run ``on_centered``."""
        if self._step_deadline and self._clock() > self._step_deadline:
            self.serial.send_stop()
            self._align_pulse_deadline = None
            self._align_settle_until = None
            self._align_pulse_is_fine = False
            self._reset_align_search_state()
            self._safe_stop("aruco_scan_timeout")
            return

        if self._align_settle_until is not None:
            self.serial.send_stop()
            if self._clock() < self._align_settle_until:
                return
            short_settle = self._align_settle_short
            self._align_settle_until = None
            self._align_settle_short = False
            frame, detection = self._read_frame_and_detection(target_id)
            if frame is None and self.stop_reason:
                return
            if frame is None:
                return
            if detection is None:
                self._aligned_frames = 0
                candidate = self._candidate_observation(frame, target_id)
                if candidate.state == CandidateState.PENDING:
                    self._begin_align_settle(short=True)
                    return
                if candidate.state == CandidateState.CONFIRMED:
                    candidate_error = float(candidate.error_x)
                    self._align_last_error_x = candidate_error
                    if abs(candidate_error) <= self.align_tolerance_px:
                        # A centred undecoded quad is only a hint: stay stopped
                        # and keep trying to decode the requested ID.
                        self._begin_align_settle(short=True)
                    else:
                        self._turn_toward_error(candidate_error, fine=True)
                    return
                if short_settle and self._can_reacquire():
                    self._align_reacquire_pulses += 1
                    self._turn_toward_error(self._align_last_error_x, fine=True)
                    return
                self._begin_align_search_pulse()
                return
            self._handle_align_detection(frame, detection, on_centered=on_centered)
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
            self._handle_align_detection(frame, detection, on_centered=on_centered)
            return

        frame, detection = self._read_frame_and_detection(target_id)
        if frame is None and self.stop_reason:
            return
        if frame is None:
            return
        if detection is None:
            self._target_missing_frames += 1
            self._aligned_frames = 0
            self._begin_align_settle(short=False)
            return
        self._handle_align_detection(frame, detection, on_centered=on_centered)

    def _step_aruco_align(self) -> None:
        """Pulse-rotate, stop, settle on miss, then search and fine-tune to centre."""
        step = self._current_step()
        target_id = int(step["target_aruco_id"])
        self._step_aruco_centering(
            step,
            target_id,
            on_centered=self._complete_aruco_align,
        )

    def _complete_aruco_align(self) -> None:
        self._advance_step()

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
        progress, slow_progress = self._encoder_progress_values(encoders)
        if (
            progress >= target_ticks
            and slow_progress >= target_ticks * MIN_WHEEL_COMPLETION_RATIO
        ):
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
        self._check_stall(slow_progress)
        if self.state != GridState.STOPPED:
            if not self.serial.send_forward():
                self._safe_stop("serial_command_failed")

    def _begin_aruco_approach_creep(self, step: dict) -> None:
        """Drive a little further inward after the marker fills the target area."""
        self._reset_aruco_trim()
        self.serial.send_stop()
        progress_values = self._read_encoder_progress()
        if progress_values is None:
            return
        progress, _ = progress_values
        step["aruco_approach_ticks_before_creep"] = progress
        base_target_ticks = float(step.get("target_ticks", 0.0))
        creep_target_ticks = max(0.0, base_target_ticks - progress)
        if creep_target_ticks > 0:
            if not self.serial.reset_encoders():
                self._safe_stop("encoder_reset_failed")
                return
            step["aruco_creep_active"] = True
            step["aruco_creep_target_ticks"] = creep_target_ticks
            self._last_progress_ticks = 0.0
            self._last_progress_at = self._clock()
            self._latest_encoders = {"left": 0, "right": 0}
            if not self.serial.send_forward():
                self._safe_stop("serial_command_failed")
            return
        self._sync_return_backout_to_approach(step, progress)
        self._advance_step()

    def _read_encoder_progress(self) -> tuple[float, float] | None:
        get_odometry = getattr(self.serial, "get_odometry", None)
        odometry = get_odometry() if callable(get_odometry) else None
        encoders = odometry if odometry is not None else self.serial.get_encoders()
        if not self._valid_encoders(encoders):
            self._safe_stop("encoder_unavailable")
            return None
        self._latest_encoders = dict(encoders)
        if odometry is not None:
            self._latest_odometry = dict(odometry)
        return self._encoder_progress_values(encoders)

    def _sync_return_backout_to_approach(
        self, step: dict, creep_progress_ticks: float
    ) -> None:
        """Back out by the same encoder distance actually driven toward the shelf."""
        if self.plan is None or not self.plan.get("return"):
            return
        return_step = self.plan["return"][0]
        if float(return_step.get("target_seconds", 0.0)) > 0:
            return
        approach_ticks = float(
            step.get("aruco_approach_ticks_before_creep", 0.0)
        )
        return_step["target_ticks"] = max(
            0,
            round(approach_ticks + float(creep_progress_ticks)),
        )
        return_step["measured_from_outbound"] = True

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
        progress_values = self._read_encoder_progress()
        if progress_values is None:
            return
        progress, slow_progress = progress_values
        if (
            progress >= target_ticks
            and slow_progress >= target_ticks * MIN_WHEEL_COMPLETION_RATIO
        ):
            self.serial.send_stop()
            self._sync_return_backout_to_approach(step, progress)
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
        base_target_ticks = float(step.get("target_ticks", 0.0))
        planned_target_ticks = base_target_ticks
        if planned_target_ticks > 0:
            progress_values = self._read_encoder_progress()
            if progress_values is None:
                return
            progress, slow_progress = progress_values
            if (
                progress >= planned_target_ticks
                and slow_progress
                >= planned_target_ticks * MIN_WHEEL_COMPLETION_RATIO
            ):
                self.serial.send_stop()
                self._sync_return_backout_to_approach(step, progress)
                self._advance_step()
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
            sent = (
                self.serial.send_rotate_right()
                if self.invert_turn_direction
                else self.serial.send_rotate_left()
            )
        elif action == "TURN_RIGHT":
            sent = (
                self.serial.send_rotate_left()
                if self.invert_turn_direction
                else self.serial.send_rotate_right()
            )
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

        step = self._current_step()
        action = step.get("action") if step is not None else None

        # Vision alignment, turns, and reverse escape only require valid readings.
        if action in {"ARUCO_ALIGN", "ARUCO_APPROACH", "ARUCO_GUIDED_FORWARD"}:
            return True
        if self.state == GridState.TURNING:
            return True
        if self.phase == "RETURNING" and action == "BACKWARD":
            return True

        # Side sensors face shelf walls in the aisle; check front path only.
        if action == "FORWARD":
            directions = ("center",)
            threshold = self.obstacle_distance_cm
        elif action == "BACKWARD":
            directions = ("left", "right")
            threshold = self.obstacle_distance_cm
        elif self.phase == "RETURNING":
            directions = ("left", "right")
            threshold = self.return_obstacle_distance_cm
        else:
            directions = ("left", "center", "right")
            threshold = self.obstacle_distance_cm

        for direction in directions:
            if float(readings[direction]) < threshold:
                self._safe_stop(f"{direction}_obstacle")
                return False
        return True

    @staticmethod
    def _encoder_progress_values(readings: dict) -> tuple[float, float]:
        left = abs(float(readings["left"]))
        right = abs(float(readings["right"]))
        return (left + right) / 2.0, min(left, right)

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
