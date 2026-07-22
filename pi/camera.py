"""Thread-safe Pi Camera capture and MJPEG streaming."""

from __future__ import annotations

import threading
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
    """Capture BGR frames and expose an MJPEG byte generator.

    ``backend`` can wrap the team's existing cat-recognition camera pipeline.
    It only needs ``start()``, ``capture_array()`` and ``stop()`` methods.
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 20,
        backend: CameraBackend | None = None,
        auto_start: bool = True,
    ):
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError("width, height and fps must all be positive")

        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self._backend = backend or _Picamera2Backend(width, height, fps)
        self._lock = threading.Lock()
        self._started = False
        self._closed = False

        if auto_start:
            self.start()

    def start(self) -> None:
        with self._lock:
            if self._closed:
                raise CameraError("A closed camera cannot be restarted")
            if not self._started:
                self._backend.start()
                self._started = True

    def get_frame(self) -> np.ndarray:
        """Return a defensive copy of the latest BGR uint8 frame."""
        with self._lock:
            if not self._started or self._closed:
                raise CameraError("Camera is not running")
            frame = self._backend.capture_array()

        if not isinstance(frame, np.ndarray):
            raise CameraError("Camera backend returned a non-array frame")
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0:
            raise CameraError(f"Invalid camera frame shape: {frame.shape}")
        if frame.dtype != np.uint8:
            frame = np.clip(frame, 0, 255).astype(np.uint8)
        return frame.copy()

    def generate_mjpeg(
        self,
        frame_provider: Callable[[], np.ndarray] | None = None,
        jpeg_quality: int = 85,
    ):
        """Yield Flask-compatible multipart JPEG frames.

        A controller can pass ``get_latest_frame`` as ``frame_provider`` so
        the web stream does not capture from the physical camera twice.
        """
        if not 1 <= jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        provider = frame_provider or self.get_frame
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)]

        while True:
            frame = provider()
            success, jpeg = cv2.imencode(".jpg", frame, encode_params)
            if not success:
                raise CameraError("OpenCV failed to encode the camera frame")
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + jpeg.tobytes()
                + b"\r\n"
            )

    def stop(self) -> None:
        with self._lock:
            if self._closed:
                return
            try:
                if self._started:
                    self._backend.stop()
            finally:
                self._started = False
                self._closed = True

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, *_args) -> None:
        self.stop()
