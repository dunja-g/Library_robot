"""Thread-safe Pi Camera capture with shared frame buffer and MJPEG streaming.

A single background thread captures frames at the configured FPS.  All
consumers — the control loop, the web video feed, any number of browser
clients — read the latest cached frame without triggering additional
hardware captures or JPEG re-encodes.
"""

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
            from picamera2 import Picamera2   # type: ignore[import-untyped]
        except ImportError as exc:
            raise CameraError(
                "picamera2 is not installed. Run this module on Raspberry Pi "
                "OS or inject a CameraBackend for offline tests."
            ) from exc

        self._camera = Picamera2()
        configuration = self._camera.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={"FrameRate": fps},
            buffer_count=2,
        )
        self._camera.configure(configuration)

    def start(self) -> None:
        self._camera.start()

    def capture_array(self) -> np.ndarray:
        return self._camera.capture_array("main")

    def stop(self) -> None:
        self._camera.stop()
        self._camera.close()


class Camera:
    """Capture loop → latest-frame cache → shared consumers.

    ``backend`` can wrap the team's existing cat-recognition pipeline.
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 20,
        jpeg_quality: int = 75,
        backend: CameraBackend | None = None,
        auto_start: bool = True,
    ):
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError("width, height and fps must all be positive")
        if not 1 <= jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be 1–100")

        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.jpeg_quality = int(jpeg_quality)
        self._backend = backend or _Picamera2Backend(width, height, fps)

        # Shared state — single writer (capture thread), many readers
        self._lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._latest_jpeg: bytes | None = None
        self._capture_error: str | None = None
        self._frame_ready = threading.Event()
        self._started = False
        self._closed = False

        # Background capture thread
        self._capture_thread: threading.Thread | None = None
        self._frame_interval = 1.0 / self.fps

        if auto_start:
            self.start()

    # ----------------------------------------------------------------
    # Lifecycle
    # ----------------------------------------------------------------
    def start(self) -> None:
        with self._lock:
            if self._closed:
                raise CameraError("A closed camera cannot be restarted")
            if self._started:
                return
            self._backend.start()
            self._started = True

        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="camera-capture"
        )
        self._capture_thread.start()

    def stop(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._started = False
            self._closed = True
            self._frame_ready.set()

        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)

        try:
            self._backend.stop()
        except Exception:
            pass

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, *_args) -> None:
        self.stop()

    # ----------------------------------------------------------------
    # Background capture
    # ----------------------------------------------------------------
    def _capture_loop(self) -> None:
        """Single writer: capture at ~fps Hz, cache frame + pre-encode JPEG."""
        while True:
            with self._lock:
                if not self._started:
                    break
            loop_start = time.monotonic()

            try:
                raw = self._backend.capture_array()
            except Exception as exc:
                with self._lock:
                    self._capture_error = str(exc)
                self._frame_ready.set()
                time.sleep(0.05)
                continue

            if not isinstance(raw, np.ndarray) or raw.ndim != 3 or raw.shape[2] != 3 or raw.size == 0:
                with self._lock:
                    self._capture_error = "Camera backend returned an invalid frame"
                self._frame_ready.set()
                time.sleep(0.05)
                continue
            if raw.dtype != np.uint8:
                raw = np.clip(raw, 0, 255).astype(np.uint8)

            frame = raw.copy()

            # Pre-encode JPEG once — all MJPEG consumers share this buffer
            success, jpeg_bytes = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            if not success:
                time.sleep(0.05)
                continue
            jpeg = jpeg_bytes.tobytes()

            with self._lock:
                self._latest_frame = frame
                self._latest_jpeg = jpeg
                self._capture_error = None
            self._frame_ready.set()

            # Sleep so we hit the target frame rate
            elapsed = time.monotonic() - loop_start
            sleep_for = self._frame_interval - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    # ----------------------------------------------------------------
    # Consumer API
    # ----------------------------------------------------------------
    def get_frame(self, timeout: float = 2.0) -> np.ndarray:
        """Wait for and return a defensive copy of the latest cached frame."""
        with self._lock:
            if not self._started or self._closed:
                raise CameraError("Camera is not running")
            frame = self._latest_frame
        if frame is None:
            self._frame_ready.wait(timeout)
        with self._lock:
            if not self._started or self._closed:
                raise CameraError("Camera is not running")
            if self._latest_frame is None:
                detail = f": {self._capture_error}" if self._capture_error else ""
                raise CameraError(f"Camera produced no frame{detail}")
            return self._latest_frame.copy()

    def get_latest_jpeg(self) -> bytes | None:
        """Return pre-encoded JPEG bytes (shared, don't mutate)."""
        with self._lock:
            return self._latest_jpeg

    def generate_mjpeg(
        self,
        frame_provider: Callable[[], np.ndarray | None] | None = None,
        jpeg_quality: int | None = None,
    ):
        """Yield Flask-compatible multipart JPEG frames.

        All browser clients share the same pre-encoded JPEG so the camera
        is only captured once per frame.  The loop is throttled to ``self.fps``.
        """
        quality = (
            jpeg_quality
            if jpeg_quality is not None
            else self.jpeg_quality
        )
        if not 1 <= quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")

        frame_interval = 1.0 / self.fps

        while True:
            loop_start = time.monotonic()

            if frame_provider is None:
                with self._lock:
                    if not self._started:
                        break
                    jpeg = self._latest_jpeg
            else:
                frame = frame_provider()
                if frame is None:
                    time.sleep(0.02)
                    continue
                success, jpeg_array = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
                )
                if not success:
                    raise CameraError("OpenCV failed to encode the camera frame")
                jpeg = jpeg_array.tobytes()

            if jpeg is None:
                time.sleep(0.02)
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + jpeg
                + b"\r\n"
            )

            elapsed = time.monotonic() - loop_start
            sleep_for = frame_interval - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)
