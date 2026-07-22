"""Offline and Raspberry Pi diagnostics for ArUco camera detection."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import cv2

try:  # Supports both ``python -m pi...`` and ``python pi/...py``.
    from .aruco_detector import ArucoDetector
    from .camera import Camera
    from .navigation_config import NavigationConfig
except ImportError:  # pragma: no cover - exercised on Raspberry Pi as a script
    from aruco_detector import ArucoDetector
    from camera import Camera
    from navigation_config import NavigationConfig


def serialise_detections(detections: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Remove NumPy-only values so a detection report can be JSON encoded."""
    return [
        {
            "id": int(item["id"]),
            "center_x": int(item["center_x"]),
            "center_y": int(item["center_y"]),
            "area": round(float(item["area"]), 2),
            "corners": [
                [round(float(x), 2), round(float(y), 2)]
                for x, y in item["corners"]
            ],
        }
        for item in detections
    ]


def inspect_image(
    image_path: str | Path,
    detector: ArucoDetector,
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    """Detect one saved image and optionally write an annotated copy."""
    image_path = Path(image_path)
    frame = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if frame is None:
        raise FileNotFoundError(f"Unable to read image: {image_path}")

    detections = detector.detect(frame)
    if output_path is not None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), detector.draw(frame, detections)):
            raise OSError(f"Unable to write annotated image: {output_path}")

    return {
        "source": str(image_path),
        "frame_size": [int(frame.shape[1]), int(frame.shape[0])],
        "detections": serialise_detections(detections),
    }


def inspect_live_camera(
    seconds: float,
    detector: ArucoDetector,
    camera: Camera,
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    """Measure camera FPS and marker detection rate over a short interval."""
    if seconds <= 0:
        raise ValueError("seconds must be positive")

    started = time.monotonic()
    frame_count = 0
    frames_with_markers = 0
    seen_ids: set[int] = set()
    latest_annotated = None

    while time.monotonic() - started < seconds:
        frame = camera.get_frame()
        detections = detector.detect(frame)
        frame_count += 1
        if detections:
            frames_with_markers += 1
            seen_ids.update(item["id"] for item in detections)
        latest_annotated = detector.draw(frame, detections)

    elapsed = max(time.monotonic() - started, 1e-9)
    if output_path is not None and latest_annotated is not None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), latest_annotated):
            raise OSError(f"Unable to write camera snapshot: {output_path}")

    return {
        "seconds": round(elapsed, 3),
        "frames": frame_count,
        "measured_fps": round(frame_count / elapsed, 2),
        "marker_detection_rate": round(
            frames_with_markers / frame_count if frame_count else 0.0,
            4,
        ),
        "seen_ids": sorted(seen_ids),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, help="Inspect one saved image")
    source.add_argument(
        "--live-seconds",
        type=float,
        help="Capture from Pi Camera for this many seconds",
    )
    parser.add_argument("--output", type=Path, help="Save an annotated image")
    parser.add_argument("--min-area", type=float, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = NavigationConfig.from_env()
    min_area = config.min_marker_area_px if args.min_area is None else args.min_area
    detector = ArucoDetector(min_area_px=min_area)

    if args.image is not None:
        report = inspect_image(args.image, detector, args.output)
    else:
        camera = Camera(
            width=config.camera_width,
            height=config.camera_height,
            fps=config.camera_fps,
        )
        try:
            report = inspect_live_camera(
                args.live_seconds,
                detector,
                camera,
                args.output,
            )
        finally:
            camera.stop()
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
