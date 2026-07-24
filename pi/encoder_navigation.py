"""Non-blocking fused-odometry navigation for the fixed 1A-3B grid."""

from __future__ import annotations

import logging
import threading
import time
from copy import deepcopy
from enum import Enum
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


class GridState(str, Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    TURNING = "TURNING"
    ARRIVED = "ARRIVED"
    DWELLING = "DWELLING"
    RETURNING = "RETURNING"
    DOCKED = "DOCKED"
    STOPPED = "STOPPED"


def normalize_angle_180(angle_deg: float) -> float:
    """Normalize angle in degrees to the [-180, +180) range."""
    return float((angle_deg + 180.0) % 360.0 - 180.0)


class GridController:
    def __init__(
        self,
        serial_bridge: Any,
        *,
        obstacle_distance_cm: float = 20.0,
        destination_dwell_seconds: float = 5.0,
        encoder_stall_seconds: float = 2.0,
        turn_source: str = "encoder",
        sensor_disagreement_threshold_deg: float = 25.0,
        reverse_segment_timeout_seconds: float = 15.0,
        rl_adapter: Any | None = None,
        clock=time.monotonic,
    ):
        if obstacle_distance_cm <= 0 or encoder_stall_seconds <= 0:
            raise ValueError("Safety distance and encoder stall timeout must be positive")
        if destination_dwell_seconds < 0:
            raise ValueError("Destination dwell must be non-negative")
        if sensor_disagreement_threshold_deg <= 0 or reverse_segment_timeout_seconds <= 0:
            raise ValueError("Disagreement threshold and reverse timeout must be positive")
        if turn_source not in {"encoder", "imu"}:
            raise ValueError("turn_source must be 'encoder' or 'imu'")
        self.serial = serial_bridge
        self.obstacle_distance_cm = float(obstacle_distance_cm)
        self.destination_dwell_seconds = float(destination_dwell_seconds)
        self.encoder_stall_seconds = float(encoder_stall_seconds)
        self.turn_source = turn_source
        self.sensor_disagreement_threshold_deg = float(sensor_disagreement_threshold_deg)
        self.reverse_segment_timeout_seconds = float(reverse_segment_timeout_seconds)
        self.rl_adapter = rl_adapter
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
        self._step_start_at: float | None = None
        self._step_deadline: float | None = None
        self._latest_encoders: dict | None = None
        self._latest_odometry: dict | None = None
        self._latest_ultrasonic: dict | None = None
        self._latest_turn_status: str | None = None
        self._latest_disagreement_deg: float | None = None
        self._awaiting_pickup_confirmation = False

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
            self._step_start_at = None
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
                "navigation_mode": "grid_fused_odometry",
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
            }
            step = self._current_step()
            if step:
                status.update(
                    current_action=step["action"],
                    current_step_label=step.get("label"),
                    target_ticks=step["target_ticks"],
                )
            target_ticks = float(step["target_ticks"]) if step else 0.0
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

            # Single front ultrasonic status report
            us_data = {}
            if self._latest_ultrasonic is not None:
                front_val = self._latest_ultrasonic.get("front")
                if front_val is None and "center" in self._latest_ultrasonic:
                    front_val = self._latest_ultrasonic["center"]
                us_data["front"] = front_val

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
                    if (
                        self._latest_odometry is None
                        or "left_cm" not in self._latest_odometry
                        or "right_cm" not in self._latest_odometry
                    )
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
                    "disagreement_deg": self._latest_disagreement_deg,
                    "speed_correction": None
                    if self._latest_odometry is None
                    else self._latest_odometry.get("speed_correction"),
                    "rl_correction": None
                    if self._latest_odometry is None
                    else self._latest_odometry.get("rl_correction", 0),
                },
                "ultrasonic": {
                    "status": "OK"
                    if self._latest_ultrasonic is not None
                    else "WAITING",
                    **us_data,
                },
            }

            if self.rl_adapter is not None and hasattr(self.rl_adapter, "get_status"):
                status["rl_status"] = self.rl_adapter.get_status()

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
            self._step_start_at = None
            self._step_deadline = None
            self._latest_encoders = None
            self._latest_odometry = None
            self._latest_ultrasonic = None
            self._latest_turn_status = None
            self._latest_disagreement_deg = None
            self._awaiting_pickup_confirmation = False

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
            self._step_start_at = None
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
                if not self._safety_clear():
                    return
                step = self._current_step()
                if step is None:
                    self._safe_stop("route_state_error")
                    return

                action = step.get("action", "")

                # Reverse segment timeout check
                if action == "BACKWARD" and self._step_start_at is not None:
                    if self._clock() - self._step_start_at > self.reverse_segment_timeout_seconds:
                        self._safe_stop("reverse_timeout")
                        return

                if self.state == GridState.TURNING and self.turn_source == "imu":
                    self._step_imu_turn()
                    return

                # Timed forward/backward mode
                if step.get("target_seconds", 0.0) > 0.0 and action in {"FORWARD", "BACKWARD"}:
                    self._step_timed_linear()
                    self._process_rl_residual(step)
                    return

                # Encoder mode
                get_odometry = getattr(self.serial, "get_odometry", None)
                odometry = get_odometry() if callable(get_odometry) else None
                encoders = odometry if odometry is not None else self.serial.get_encoders()
                if not self._valid_encoders(encoders):
                    self._safe_stop("encoder_unavailable")
                    return
                self._latest_encoders = dict(encoders)
                if odometry is not None:
                    self._latest_odometry = dict(odometry)
                    if "heading_imu_deg" in odometry and "heading_encoder_deg" in odometry:
                        imu_deg = float(odometry["heading_imu_deg"])
                        enc_deg = float(odometry["heading_encoder_deg"])
                        disagreement = abs(normalize_angle_180(imu_deg - enc_deg))
                        self._latest_disagreement_deg = disagreement
                        if disagreement > self.sensor_disagreement_threshold_deg:
                            self._safe_stop("sensor_disagreement")
                            return

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
                    self._send_action(action)
                    self._process_rl_residual(step)

            except Exception as exc:
                self._safe_stop(f"controller_error:{type(exc).__name__}")
                raise

    def _process_rl_residual(self, step: dict) -> None:
        """Run RL residual adapter during linear motion segments if enabled."""
        if self.rl_adapter is None:
            return
        action = step.get("action", "")
        if action not in {"FORWARD", "BACKWARD"}:
            return

        try:
            target_ticks = float(step.get("target_ticks", 0))
            if target_ticks > 0 and self._latest_encoders:
                completed = min(
                    abs(float(self._latest_encoders["left"])),
                    abs(float(self._latest_encoders["right"])),
                )
            else:
                completed = 0.0
                target_ticks = 1.0

            odom = self._latest_odometry or {}
            us = self._latest_ultrasonic or {}
            front_us = us.get("front")
            if front_us is None:
                front_us = us.get("center", -1.0)

            result = self.rl_adapter.step(
                is_forward=(action == "FORWARD"),
                completed_distance=completed,
                target_distance=target_ticks,
                target_heading_deg=0.0,
                fused_heading_deg=float(odom.get("heading_fused_deg", 0.0)),
                left_distance_cm=float(odom.get("left_cm", 0.0)),
                right_distance_cm=float(odom.get("right_cm", 0.0)),
                front_ultrasonic_cm=float(front_us),
                current_action=action,
            )

            if result and getattr(result, "apply_to_motor", False):
                pwm = int(getattr(result, "residual_pwm", 0))
                send_rl = getattr(self.serial, "send_rl_correction", None)
                if callable(send_rl):
                    send_rl(pwm)
        except Exception as exc:
            logger.warning("RL adapter evaluation failed: %s", exc)

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
        self._step_start_at = self._clock()

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

        if step.get("target_seconds", 0.0) > 0.0 and step["action"] in {"FORWARD", "BACKWARD"}:
            self._step_deadline = self._clock() + step["target_seconds"]
            self.state = GridState.MOVING
            self._send_action(step["action"])
            return

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
        if self._step_deadline is None:
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

        step = self._current_step()
        # For BACKWARD movement, front ultrasonic is not in the direction of motion;
        # reverse safety relies on corridor clearing and segment timeout.
        if step is not None and step.get("action") == "BACKWARD":
            return True

        # For FORWARD motion, check single front ultrasonic sensor
        front_val = readings.get("front")
        if front_val is None and "center" in readings:
            front_val = readings["center"]

        if front_val is not None and float(front_val) < self.obstacle_distance_cm:
            self._safe_stop("center_obstacle")
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
        self.state = GridState.STOPPED
        self.stop_reason = reason
