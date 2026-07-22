"""OpenCV ArUco detection and display annotations."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


class ArucoDetector:
    def __init__(
        self,
        dictionary_id: int | None = None,
        min_area_px: float = 0.0,
    ):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "OpenCV ArUco is unavailable. Install opencv-contrib-python."
            )

        if min_area_px < 0:
            raise ValueError("min_area_px must be non-negative")
        self.min_area_px = float(min_area_px)
        dictionary_id = (
            cv2.aruco.DICT_5X5_50
            if dictionary_id is None
            else dictionary_id
        )
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.parameters = cv2.aruco.DetectorParameters()
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

    def detect(self, frame: np.ndarray) -> list[dict[str, Any]]:
        """Return marker ID, centre, polygon area and corners for each marker."""
        self._validate_frame(frame)
        if frame.ndim == 2:
            gray = frame
        elif frame.shape[2] == 4:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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
