"""Single-producer Pi Camera capture and shared MJPEG streaming."""

from __future__ import annotations

import threading
import time
from typing import Callable, Protocol

import cv2
import numpy as np


class CameraError(RuntimeError):
    """Raised when the camera cannot start or produce a valid frame."""


class CameraBackend(Protocol):
    """Small backend contract that makes camera code hardware-testable."""

    def start(self) -> None: ...

    def capture_array(self) -> np.ndarray: ...

    def stop(self) -> None: ...


class _Picamera2Backend:
    def __init__(self, width: int, height: int, fps: int):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise CameraError(
                "picamera2 is not installed. Run this module on Raspberry Pi "
                "OS or inject a CameraBackend for offline tests."
            ) from exc

        self._camera = Picamera2()
        configuration = self._camera.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={"FrameRate": fps},
            buffer_count=4,
        )
        self._camera.configure(configuration)

    def start(self) -> None:
        self._camera.start()

    def capture_array(self) -> np.ndarray:
        # Picamera2's RGB888 memory layout is already B, G, R for OpenCV.
        # Despite the format name, converting RGB->BGR here would swap twice.
        return self._camera.capture_array("main")

    def stop(self) -> None:
        self._camera.stop()
        self._camera.close()


class Camera:
    """Capture BGR frames once and share the newest frame with all consumers.

    ``backend`` can wrap the team's existing cat-recognition camera pipeline.
    It only needs ``start()``, ``capture_array()`` and ``stop()`` methods.
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 20,
        stream_fps: int = 15,
        jpeg_quality: int = 75,
        backend: CameraBackend | None = None,
        auto_start: bool = True,
    ):
        if width <= 0 or height <= 0 or fps <= 0 or stream_fps <= 0:
            raise ValueError(
                "width, height, fps and stream_fps must all be positive"
            )
        if not 1 <= jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")

        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.stream_fps = min(int(stream_fps), self.fps)
        self.jpeg_quality = int(jpeg_quality)
        self._backend = backend or _Picamera2Backend(width, height, fps)
        self._condition = threading.Condition()
        self._stop_event = threading.Event()
        self._capture_thread: threading.Thread | None = None
        self._started = False
        self._closed = False
        self._latest_frame: np.ndarray | None = None
        self._latest_sequence = 0
        self._latest_captured_at = 0.0
        self._capture_error: str | None = None
        self._capture_count = 0
        self._fps_window_started_at = time.monotonic()
        self._fps_window_count = 0
        self._measured_fps = 0.0
        self._stream_clients = 0

        if auto_start:
            self.start()

    def start(self) -> None:
        with self._condition:
            if self._closed:
                raise CameraError("A closed camera cannot be restarted")
            if not self._started:
                self._backend.start()
                self._started = True
                self._stop_event.clear()
                self._capture_thread = threading.Thread(
                    target=self._capture_loop,
                    name="library-camera-capture",
                    daemon=True,
                )
                self._capture_thread.start()

    @staticmethod
    def _normalise_frame(frame: np.ndarray) -> np.ndarray:
        if not isinstance(frame, np.ndarray):
            raise CameraError("Camera backend returned a non-array frame")
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0:
            raise CameraError(f"Invalid camera frame shape: {frame.shape}")
        if frame.dtype != np.uint8:
            frame = np.clip(frame, 0, 255).astype(np.uint8)
        return frame

    def _capture_loop(self) -> None:
        period = 1.0 / self.fps
        next_capture_at = time.monotonic()
        while not self._stop_event.is_set():
            try:
                frame = self._normalise_frame(self._backend.capture_array())
                captured_at = time.monotonic()
                with self._condition:
                    self._latest_frame = frame.copy()
                    self._latest_sequence += 1
                    self._latest_captured_at = captured_at
                    self._capture_error = None
                    self._capture_count += 1
                    self._fps_window_count += 1
                    elapsed = captured_at - self._fps_window_started_at
                    if elapsed >= 1.0:
                        self._measured_fps = self._fps_window_count / elapsed
                        self._fps_window_started_at = captured_at
                        self._fps_window_count = 0
                    self._condition.notify_all()
            except Exception as exc:
                with self._condition:
                    self._capture_error = str(exc)
                    self._condition.notify_all()

            next_capture_at += period
            delay = next_capture_at - time.monotonic()
            if delay < -period:
                next_capture_at = time.monotonic()
                delay = 0.0
            self._stop_event.wait(max(0.0, delay))

    def get_frame(self, timeout: float = 2.0) -> np.ndarray:
        """Return a defensive copy of the latest BGR uint8 frame."""
        deadline = time.monotonic() + timeout
        with self._condition:
            if not self._started or self._closed:
                raise CameraError("Camera is not running")
            while self._latest_frame is None and not self._closed:
                if self._capture_error:
                    raise CameraError(
                        f"Camera produced no frame: {self._capture_error}"
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    detail = (
                        f": {self._capture_error}" if self._capture_error else ""
                    )
                    raise CameraError(f"Camera produced no frame{detail}")
                self._condition.wait(remaining)
            if self._latest_frame is None:
                raise CameraError("Camera is not running")
            return self._latest_frame.copy()

    def _wait_for_new_frame(
        self, previous_sequence: int, timeout: float
    ) -> tuple[np.ndarray | None, int]:
        deadline = time.monotonic() + timeout
        with self._condition:
            while (
                self._latest_sequence <= previous_sequence
                and not self._closed
                and not self._stop_event.is_set()
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, previous_sequence
                self._condition.wait(remaining)
            if self._latest_frame is None or self._closed:
                return None, previous_sequence
            return self._latest_frame.copy(), self._latest_sequence

    def generate_mjpeg(
        self,
        frame_provider: Callable[[], np.ndarray | None] | None = None,
        jpeg_quality: int | None = None,
        max_fps: int | None = None,
    ):
        """Yield Flask-compatible multipart JPEG frames.

        The default provider waits for the shared capture thread's newest
        frame. A custom provider remains available for annotated frames.
        """
        quality = self.jpeg_quality if jpeg_quality is None else jpeg_quality
        stream_fps = self.stream_fps if max_fps is None else max_fps
        if stream_fps <= 0:
            raise ValueError("max_fps must be positive")
        if not 1 <= quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, int(quality)]
        min_interval = 1.0 / min(int(stream_fps), self.fps)
        previous_sequence = 0
        next_emit_at = time.monotonic()

        with self._condition:
            self._stream_clients += 1
        try:
            while not self._stop_event.is_set():
                if frame_provider is None:
                    frame, previous_sequence = self._wait_for_new_frame(
                        previous_sequence, timeout=1.0
                    )
                else:
                    frame = frame_provider()
                if frame is None:
                    self._stop_event.wait(0.02)
                    continue

                delay = next_emit_at - time.monotonic()
                if delay > 0 and self._stop_event.wait(delay):
                    break
                next_emit_at = time.monotonic() + min_interval

                success, jpeg = cv2.imencode(".jpg", frame, encode_params)
                if not success:
                    raise CameraError("OpenCV failed to encode the camera frame")
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Cache-Control: no-store\r\n\r\n"
                    + jpeg.tobytes()
                    + b"\r\n"
                )
        except (GeneratorExit, BrokenPipeError, ConnectionResetError):
            return
        finally:
            with self._condition:
                self._stream_clients = max(0, self._stream_clients - 1)

    def get_stats(self) -> dict:
        """Return lightweight capture health for the web status endpoint."""
        with self._condition:
            age_ms = (
                None
                if not self._latest_captured_at
                else max(0.0, (time.monotonic() - self._latest_captured_at) * 1000)
            )
            return {
                "status": (
                    "ERROR"
                    if self._capture_error and self._latest_frame is None
                    else "OK"
                    if self._latest_frame is not None
                    else "STARTING"
                ),
                "capture_fps": round(self._measured_fps, 1),
                "target_fps": self.fps,
                "stream_fps": self.stream_fps,
                "frame_age_ms": None if age_ms is None else round(age_ms),
                "clients": self._stream_clients,
                "frames_captured": self._capture_count,
                "error": self._capture_error,
            }

    def stop(self) -> None:
        with self._condition:
            if self._closed:
                return
            self._stop_event.set()
            self._condition.notify_all()
            capture_thread = self._capture_thread

        if capture_thread is not None:
            capture_thread.join(timeout=2.0)

        try:
            if self._started:
                self._backend.stop()
        finally:
            with self._condition:
                self._started = False
                self._closed = True
                self._condition.notify_all()

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, *_args) -> None:
        self.stop()
