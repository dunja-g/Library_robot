"""Thread-safe Raspberry Pi to Arduino Mega serial communication."""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Callable

import serial


logger = logging.getLogger(__name__)


class SerialBridgeError(RuntimeError):
    """Raised when the serial connection cannot be opened."""


class SerialBridge:
    """Expose the Arduino text protocol as safe controller methods."""

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        timeout: float = 0.5,
        startup_delay: float = 2.0,
        serial_factory: Callable[..., Any] | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ):
        if baudrate <= 0 or timeout <= 0 or startup_delay < 0:
            raise ValueError("Invalid serial timing or baudrate configuration")

        factory = serial_factory or serial.Serial
        self._lock = threading.RLock()
        self._closed = False
        try:
            self.ser = factory(port=port, baudrate=baudrate, timeout=timeout)
            if startup_delay:
                sleep(startup_delay)
            reset_input = getattr(self.ser, "reset_input_buffer", None)
            if callable(reset_input):
                reset_input()
        except (serial.SerialException, OSError) as exc:
            raise SerialBridgeError(f"Unable to open Arduino on {port}: {exc}") from exc

        self.port = port
        self.baudrate = baudrate
        logger.info("SerialBridge ready on %s at %d baud", port, baudrate)

    def _write_locked(self, command: str) -> bool:
        if self._closed:
            logger.error("Cannot send %s: serial bridge is closed", command)
            return False
        try:
            self.ser.write(f"{command}\n".encode("ascii"))
            self.ser.flush()
            return True
        except (serial.SerialException, OSError) as exc:
            logger.error("Failed to send %s: %s", command, exc)
            return False

    def _send(self, command: str) -> bool:
        with self._lock:
            return self._write_locked(command)

    def send_forward(self) -> bool:
        return self._send("FORWARD")

    def send_backward(self) -> bool:
        return self._send("BACKWARD")

    def send_rotate_left(self) -> bool:
        return self._send("ROTATE_LEFT")

    def send_rotate_right(self) -> bool:
        return self._send("ROTATE_RIGHT")

    def send_turn_left(self, degrees: float | None = None) -> bool:
        """Request a self-terminating IMU turn (defaults to 90 degrees)."""
        return self._send("TURN_LEFT" if degrees is None else f"TURN_LEFT:{degrees:.1f}")

    def send_turn_right(self, degrees: float | None = None) -> bool:
        """Request a self-terminating IMU turn (defaults to 90 degrees)."""
        return self._send("TURN_RIGHT" if degrees is None else f"TURN_RIGHT:{degrees:.1f}")

    def send_turn_uturn(self, degrees: float | None = None) -> bool:
        """Request a self-terminating IMU turn (defaults to 180 degrees)."""
        return self._send("TURN_UTURN" if degrees is None else f"TURN_UTURN:{degrees:.1f}")

    @staticmethod
    def parse_turn_status(line: str) -> str | None:
        if not line.startswith("TURN:"):
            return None
        status = line[5:].strip()
        return status if status in {"IDLE", "ACTIVE", "DONE", "ERROR"} else None

    def get_turn_status(self, response_lines: int = 3) -> str | None:
        """Return the state of a self-terminating MPU6500 turn."""
        if response_lines <= 0:
            raise ValueError("response_lines must be positive")
        with self._lock:
            if not self._write_locked("TURN_STATUS"):
                return None
            for _ in range(response_lines):
                try:
                    raw = self.ser.readline()
                except (serial.SerialException, OSError) as exc:
                    logger.error("Failed to read turn status: %s", exc)
                    return None
                line = raw.decode("ascii", errors="ignore").strip()
                status = self.parse_turn_status(line)
                if status is not None:
                    return status
        return None

    def send_stop(self) -> bool:
        return self._send("STOP")

    def reset_encoders(self) -> bool:
        """Reset both Mega encoder counters before a motion segment."""
        return self._send("ENC_RESET")

    @staticmethod
    def parse_ultrasonic(line: str) -> dict[str, float] | None:
        """Parse ``US:left,center,right`` and reject unsafe values."""
        if not line.startswith("US:"):
            return None
        parts = line[3:].split(",")
        if len(parts) != 3:
            return None
        try:
            values = [float(part) for part in parts]
        except ValueError:
            return None
        if not all(math.isfinite(value) and value >= 0 for value in values):
            return None
        return dict(zip(("left", "center", "right"), values))

    def get_ultrasonic(self, response_lines: int = 3) -> dict[str, float] | None:
        """Request all three sensors, ignoring unrelated status lines."""
        if response_lines <= 0:
            raise ValueError("response_lines must be positive")
        with self._lock:
            if not self._write_locked("CHECK"):
                return None
            for _ in range(response_lines):
                try:
                    raw = self.ser.readline()
                except (serial.SerialException, OSError) as exc:
                    logger.error("Failed to read ultrasonic response: %s", exc)
                    return None
                line = raw.decode("ascii", errors="ignore").strip()
                readings = self.parse_ultrasonic(line)
                if readings is not None:
                    return readings
                if line:
                    logger.debug("Ignoring Arduino status line: %s", line)
        logger.warning("No valid ultrasonic response received")
        return None

    @staticmethod
    def parse_encoders(line: str) -> dict[str, int] | None:
        """Parse ``ENC:left,right`` tick counters."""
        if not line.startswith("ENC:"):
            return None
        parts = line[4:].split(",")
        if len(parts) != 2:
            return None
        try:
            values = [int(part) for part in parts]
        except ValueError:
            return None
        return dict(zip(("left", "right"), values))

    def get_encoders(self, response_lines: int = 3) -> dict[str, int] | None:
        """Request an atomic snapshot of left and right encoder ticks."""
        if response_lines <= 0:
            raise ValueError("response_lines must be positive")
        with self._lock:
            if not self._write_locked("ENCODER"):
                return None
            for _ in range(response_lines):
                try:
                    raw = self.ser.readline()
                except (serial.SerialException, OSError) as exc:
                    logger.error("Failed to read encoder response: %s", exc)
                    return None
                line = raw.decode("ascii", errors="ignore").strip()
                readings = self.parse_encoders(line)
                if readings is not None:
                    return readings
                if line:
                    logger.debug("Ignoring Arduino status line: %s", line)
        logger.warning("No valid encoder response received")
        return None

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._write_locked("STOP")
            try:
                self.ser.close()
            except (serial.SerialException, OSError) as exc:
                logger.warning("Failed to close serial port cleanly: %s", exc)
            finally:
                self._closed = True

    def __enter__(self) -> "SerialBridge":
        return self

    def __exit__(self, *_args) -> None:
        self.close()
