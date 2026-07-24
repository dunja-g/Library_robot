"""Background QR code scanner using the robot's existing camera pipeline."""
from __future__ import annotations

import threading
import time
import logging
from typing import Callable

import cv2

logger = logging.getLogger(__name__)


class QRScanner:
    """Continuously scans frames from a frame provider for QR codes.
    
    Args:
        frame_provider: callable that returns the latest BGR numpy frame (or None).
        on_detect: callback called with the decoded QR string whenever a new code is found.
        scan_interval: seconds between scans (default 0.3s - don't hammer the CPU).
        cooldown: seconds to ignore repeat detections of the same code (default 5.0s).
    """
    
    def __init__(
        self,
        frame_provider: Callable,
        on_detect: Callable[[str], None],
        scan_interval: float = 0.3,
        cooldown: float = 5.0,
    ):
        self._frame_provider = frame_provider
        self._on_detect = on_detect
        self._scan_interval = scan_interval
        self._cooldown = cooldown
        self._detector = cv2.QRCodeDetector()
        self._last_code: str | None = None
        self._last_detected_at: float = 0.0
        self._running = False
        self._thread: threading.Thread | None = None
    
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="qr-scanner")
        self._thread.start()
        logger.info("QR scanner started")
    
    def stop(self) -> None:
        self._running = False
    
    def _loop(self) -> None:
        while self._running:
            try:
                frame = self._frame_provider()
                if frame is not None:
                    self._scan_frame(frame)
            except Exception:
                logger.exception("QR scanner error")
            time.sleep(self._scan_interval)
    
    def _scan_frame(self, frame) -> None:
        # 帧校验：防止 OpenCV convexHull 崩溃 + 空帧
        if frame is None or frame.size == 0 or frame.ndim < 2:
            return
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        except Exception:
            return
        try:
            data, _, _ = self._detector.detectAndDecode(gray)
        except Exception:
            return
        if not data:
            return
        now = time.monotonic()
        # Apply cooldown to prevent spamming the same code
        if data == self._last_code and (now - self._last_detected_at) < self._cooldown:
            return
        self._last_code = data
        self._last_detected_at = now
        logger.info("QR code detected: %s", data)
        self._on_detect(data)
