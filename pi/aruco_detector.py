"""OpenCV ArUco detection and display annotations."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


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

        self.min_area_px = float(min_area_px)
        self.enhance_vision = bool(enhance_vision)
        self.clahe_clip_limit = float(clahe_clip_limit)
        self.clahe_tile_grid = int(clahe_tile_grid)
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
            raise ValueError("frame must be a non-empty grayscale or BGR image")
        if frame.ndim == 3 and frame.shape[2] not in (3, 4):
            raise ValueError("colour frames must contain 3 or 4 channels")

    def _to_gray(self, frame: np.ndarray) -> np.ndarray:
        if frame.ndim == 2:
            return frame
        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _enhance_gray(self, gray: np.ndarray) -> np.ndarray:
        """Boost local contrast and sharpen edges for ArUco binarisation."""
        equalized = self._clahe.apply(gray)
        blurred = cv2.GaussianBlur(equalized, (0, 0), sigmaX=0.8)
        sharpened = cv2.addWeighted(equalized, 1.4, blurred, -0.4, 0)
        return np.clip(sharpened, 0, 255).astype(np.uint8)

    def _detection_variants(self, frame: np.ndarray) -> list[np.ndarray]:
        gray = self._to_gray(frame)
        if not self.enhance_vision:
            return [gray]

        enhanced = self._enhance_gray(gray)
        variants = [gray, enhanced]
        adaptive = cv2.adaptiveThreshold(
            enhanced,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            11,
            2,
        )
        variants.append(adaptive)
        return variants

    def _detect_on_gray(self, gray: np.ndarray) -> list[dict[str, Any]]:
        if self._detector is not None:
            corners, ids, _rejected = self._detector.detectMarkers(gray)
        else:  # OpenCV contrib < 4.7
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters,
            )

        if ids is None:
            return []

        detections: list[dict[str, Any]] = []
        for marker_id, raw_corners in zip(ids.flatten(), corners):
            points = np.asarray(raw_corners, dtype=np.float32).reshape(4, 2)
            center = points.mean(axis=0)
            area = float(abs(cv2.contourArea(points)))
            if area < self.min_area_px:
                continue
            detections.append(
                {
                    "id": int(marker_id),
                    "center_x": int(round(float(center[0]))),
                    "center_y": int(round(float(center[1]))),
                    "area": area,
                    "corners": points.copy(),
                }
            )
        return detections

    def detect(self, frame: np.ndarray) -> list[dict[str, Any]]:
        """Return marker ID, centre, polygon area and corners for each marker."""
        self._validate_frame(frame)
        merged: dict[int, dict[str, Any]] = {}
        for gray in self._detection_variants(frame):
            for detection in self._detect_on_gray(gray):
                marker_id = int(detection["id"])
                current = merged.get(marker_id)
                if current is None or float(detection["area"]) > float(
                    current["area"]
                ):
                    merged[marker_id] = detection
        return list(merged.values())

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
