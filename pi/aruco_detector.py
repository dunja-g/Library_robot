"""OpenCV ArUco detection and display annotations."""

from __future__ import annotations

import logging
import time
from typing import Any

import cv2
import numpy as np

logger = logging.getLogger(__name__)


def _apply_sensitive_detector_params(parameters: cv2.aruco.DetectorParameters) -> None:
    """Loosen thresholds so smaller or lower-contrast markers are still found."""
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 10
    parameters.minMarkerPerimeterRate = 0.02
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.polygonalApproxAccuracyRate = 0.05
    parameters.minCornerDistanceRate = 0.01
    parameters.minDistanceToBorder = 1
    parameters.minOtsuStdDev = 1.0
    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX


class ArucoDetector:
    def __init__(
        self,
        dictionary_id: int | None = None,
        min_area_px: float = 0.0,
        *,
        enhance_vision: bool = True,
        clahe_clip_limit: float = 3.0,
        clahe_tile_grid: int = 8,
        upscale_factor: float = 2.0,
        candidate_min_area_px: float = 900.0,
        candidate_max_area_px: float = 120000.0,
    ):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "OpenCV ArUco is unavailable. Install opencv-contrib-python."
            )

        if min_area_px < 0:
            raise ValueError("min_area_px must be non-negative")
        if clahe_clip_limit <= 0:
            raise ValueError("clahe_clip_limit must be positive")
        if clahe_tile_grid <= 0:
            raise ValueError("clahe_tile_grid must be positive")
        if upscale_factor < 1.0:
            raise ValueError("upscale_factor must be at least 1.0")
        if candidate_min_area_px < 0:
            raise ValueError("candidate_min_area_px must be non-negative")
        if candidate_max_area_px <= 0:
            raise ValueError("candidate_max_area_px must be positive")

        self.min_area_px = float(min_area_px)
        self.enhance_vision = bool(enhance_vision)
        self.clahe_clip_limit = float(clahe_clip_limit)
        self.clahe_tile_grid = int(clahe_tile_grid)
        self.upscale_factor = float(upscale_factor)
        self.candidate_min_area_px = float(candidate_min_area_px)
        self.candidate_max_area_px = float(candidate_max_area_px)
        self._timing_started = time.monotonic()
        self._timing_calls = 0
        self._timing_total_ms = 0.0
        self._timing_max_ms = 0.0
        self._clahe = cv2.createCLAHE(
            clipLimit=self.clahe_clip_limit,
            tileGridSize=(self.clahe_tile_grid, self.clahe_tile_grid),
        )

        dictionary_id = (
            cv2.aruco.DICT_5X5_50
            if dictionary_id is None
            else dictionary_id
        )
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.parameters = cv2.aruco.DetectorParameters()
        if self.enhance_vision:
            _apply_sensitive_detector_params(self.parameters)
        self._detector = (
            cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            if hasattr(cv2.aruco, "ArucoDetector")
            else None
        )

    @staticmethod
    def _validate_frame(frame: np.ndarray) -> None:
        if not isinstance(frame, np.ndarray):
            raise TypeError("frame must be a numpy array")
        if frame.size == 0 or frame.ndim not in (2, 3):
            raise ValueError("frame must be a non-empty grayscale or RGB image")
        if frame.ndim == 3 and frame.shape[2] not in (3, 4):
            raise ValueError("colour frames must contain 3 or 4 channels")

    def _to_gray(self, frame: np.ndarray) -> np.ndarray:
        if frame.ndim == 2:
            return frame
        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2GRAY)
        return cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    def _enhance_gray(self, gray: np.ndarray) -> np.ndarray:
        """Boost local contrast and sharpen edges for ArUco binarisation."""
        equalized = self._clahe.apply(gray)
        blurred = cv2.GaussianBlur(equalized, (0, 0), sigmaX=0.8)
        sharpened = cv2.addWeighted(equalized, 1.4, blurred, -0.4, 0)
        return np.clip(sharpened, 0, 255).astype(np.uint8)

    def _detection_variants(self, frame: np.ndarray) -> list[tuple[np.ndarray, float]]:
        """Return ``(image, scale)`` passes; ``scale`` maps points back to ``frame``."""
        gray = self._to_gray(frame)
        if not self.enhance_vision:
            return [(gray, 1.0)]

        enhanced = self._enhance_gray(gray)
        variants: list[tuple[np.ndarray, float]] = [(gray, 1.0), (enhanced, 1.0)]
        adaptive = cv2.adaptiveThreshold(
            enhanced,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            11,
            2,
        )
        variants.append((adaptive, 1.0))

        # Distant markers span too few pixels for the corner refinement to
        # lock on, so give them a dedicated magnified pass.
        if self.upscale_factor > 1.0:
            upscaled = cv2.resize(
                enhanced,
                None,
                fx=self.upscale_factor,
                fy=self.upscale_factor,
                interpolation=cv2.INTER_CUBIC,
            )
            variants.append((upscaled, self.upscale_factor))
        return variants

    def _run_detector(self, gray: np.ndarray):
        if self._detector is not None:
            return self._detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(  # OpenCV contrib < 4.7
            gray,
            self.dictionary,
            parameters=self.parameters,
        )

    @staticmethod
    def _quad(raw_corners: Any, scale: float) -> np.ndarray:
        points = np.asarray(raw_corners, dtype=np.float32).reshape(4, 2)
        return points / scale if scale != 1.0 else points

    def _describe(self, points: np.ndarray) -> dict[str, Any]:
        center = points.mean(axis=0)
        return {
            "center_x": int(round(float(center[0]))),
            "center_y": int(round(float(center[1]))),
            "area": float(abs(cv2.contourArea(points))),
            "corners": points.copy(),
        }

    def _detect_on_gray(
        self, gray: np.ndarray, scale: float = 1.0
    ) -> list[dict[str, Any]]:
        corners, ids, _rejected = self._run_detector(gray)
        if ids is None:
            return []

        detections: list[dict[str, Any]] = []
        for marker_id, raw_corners in zip(ids.flatten(), corners):
            detection = self._describe(self._quad(raw_corners, scale))
            if detection["area"] < self.min_area_px:
                continue
            detection["id"] = int(marker_id)
            detections.append(detection)
        return detections

    @staticmethod
    def _valid_candidate_shape(points: np.ndarray) -> bool:
        """Reject concave and strongly non-square quads before navigation sees them."""
        contour = np.asarray(points, dtype=np.float32).reshape(4, 1, 2)
        if not cv2.isContourConvex(contour):
            return False
        edges = [
            float(np.linalg.norm(points[(index + 1) % 4] - points[index]))
            for index in range(4)
        ]
        width = (edges[0] + edges[2]) / 2.0
        height = (edges[1] + edges[3]) / 2.0
        shorter = min(width, height)
        return shorter > 0 and max(width, height) / shorter <= 1.8

    def _record_timing(self, elapsed_ms: float) -> None:
        """Periodically report detector cost without logging every control tick."""
        self._timing_calls += 1
        self._timing_total_ms += elapsed_ms
        self._timing_max_ms = max(self._timing_max_ms, elapsed_ms)
        now = time.monotonic()
        if now - self._timing_started < 10.0:
            return
        logger.info(
            "ArUco detection timing: calls=%d avg_ms=%.1f max_ms=%.1f passes=%d",
            self._timing_calls,
            self._timing_total_ms / self._timing_calls,
            self._timing_max_ms,
            4 if self.enhance_vision and self.upscale_factor > 1.0 else (
                3 if self.enhance_vision else 1
            ),
        )
        self._timing_started = now
        self._timing_calls = 0
        self._timing_total_ms = 0.0
        self._timing_max_ms = 0.0

    def detect(self, frame: np.ndarray) -> list[dict[str, Any]]:
        """Return marker ID, centre, polygon area and corners for each marker."""
        started = time.perf_counter()
        self._validate_frame(frame)
        merged: dict[int, dict[str, Any]] = {}
        for gray, scale in self._detection_variants(frame):
            for detection in self._detect_on_gray(gray, scale):
                marker_id = int(detection["id"])
                current = merged.get(marker_id)
                if current is None or float(detection["area"]) > float(
                    current["area"]
                ):
                    merged[marker_id] = detection
        detections = list(merged.values())
        self._record_timing((time.perf_counter() - started) * 1000.0)
        return detections

    def detect_candidates(self, frame: np.ndarray) -> list[dict[str, Any]]:
        """Return marker-shaped quads whose ID could not be decoded.

        These are the near-misses that let the robot keep steering toward a
        marker it can see but not yet read.
        """
        started = time.perf_counter()
        self._validate_frame(frame)
        decoded: list[dict[str, Any]] = []
        candidates: list[dict[str, Any]] = []
        for gray, scale in self._detection_variants(frame):
            corners, ids, rejected = self._run_detector(gray)
            if ids is not None:
                for marker_id, raw_corners in zip(ids.flatten(), corners):
                    marker = self._describe(self._quad(raw_corners, scale))
                    marker["id"] = int(marker_id)
                    decoded.append(marker)
            if rejected is not None:
                for raw_corners in rejected:
                    points = self._quad(raw_corners, scale)
                    candidate = self._describe(points)
                    if not (
                        self.candidate_min_area_px
                        <= candidate["area"]
                        <= self.candidate_max_area_px
                    ):
                        continue
                    if self._valid_candidate_shape(points):
                        candidates.append(candidate)
        candidates = self._deduplicate(candidates)
        filtered = [
            candidate
            for candidate in candidates
            if not any(
                abs(candidate["center_x"] - marker["center_x"]) <= 12
                and abs(candidate["center_y"] - marker["center_y"]) <= 12
                for marker in decoded
            )
        ]
        self._record_timing((time.perf_counter() - started) * 1000.0)
        return filtered

    @staticmethod
    def _deduplicate(candidates: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """Collapse the same quad found by several passes into one entry."""
        unique: list[dict[str, Any]] = []
        for candidate in sorted(
            candidates, key=lambda item: float(item["area"]), reverse=True
        ):
            duplicate = any(
                abs(candidate["center_x"] - kept["center_x"]) <= 10
                and abs(candidate["center_y"] - kept["center_y"]) <= 10
                for kept in unique
            )
            if not duplicate:
                unique.append(candidate)
        return unique

    def detect_target(
        self, frame: np.ndarray, target_id: int
    ) -> dict[str, Any] | None:
        """Return the requested marker detection, or ``None`` if absent."""
        matches = [
            item for item in self.detect(frame) if item["id"] == target_id
        ]
        return max(matches, key=lambda item: item["area"], default=None)

    def draw(
        self,
        frame: np.ndarray,
        detections: list[dict[str, Any]],
    ) -> np.ndarray:
        """Return an annotated copy without modifying the input frame."""
        self._validate_frame(frame)
        annotated = frame.copy()
        if annotated.ndim == 2:
            annotated = cv2.cvtColor(annotated, cv2.COLOR_GRAY2BGR)
        elif annotated.shape[2] == 4:
            annotated = cv2.cvtColor(annotated, cv2.COLOR_BGRA2BGR)

        for detection in detections:
            points = np.asarray(detection["corners"], dtype=np.int32).reshape(4, 2)
            center = (int(detection["center_x"]), int(detection["center_y"]))
            marker_id = int(detection["id"])
            cv2.polylines(annotated, [points], True, (0, 255, 0), 2)
            cv2.circle(annotated, center, 4, (0, 0, 255), -1)
            text_origin = (int(points[0][0]), max(18, int(points[0][1]) - 8))
            cv2.putText(
                annotated,
                f"ID {marker_id}",
                text_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        return annotated
