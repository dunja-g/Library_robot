import cv2
import numpy as np

from aruco_codes.generate_markers import create_marker
from pi.aruco_detector import ArucoDetector


def as_bgr(image):
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def test_detects_generated_marker_with_correct_id_and_center():
    detector = ArucoDetector()
    frame = as_bgr(create_marker(3, image_size=300, border_px=30))

    detection = detector.detect_target(frame, 3)

    assert detection is not None
    assert abs(detection["center_x"] - 150) <= 2
    assert abs(detection["center_y"] - 150) <= 2
    assert detection["area"] > 0
    assert detection["corners"].shape == (4, 2)


def test_detects_multiple_markers_and_filters_target():
    detector = ArucoDetector()
    canvas = np.full((360, 680), 255, dtype=np.uint8)
    canvas[40:300, 30:290] = create_marker(1, 260, 25)
    canvas[40:300, 390:650] = create_marker(4, 260, 25)

    detections = detector.detect(as_bgr(canvas))

    assert {item["id"] for item in detections} == {1, 4}
    assert detector.detect_target(as_bgr(canvas), 4)["center_x"] > 400
    assert detector.detect_target(as_bgr(canvas), 2) is None


def test_empty_frame_contains_no_detections():
    frame = np.full((480, 640, 3), 255, dtype=np.uint8)
    assert ArucoDetector().detect(frame) == []


def test_draw_returns_annotated_copy():
    detector = ArucoDetector()
    frame = as_bgr(create_marker(2, image_size=300, border_px=30))
    original = frame.copy()

    annotated = detector.draw(frame, detector.detect(frame))

    assert np.array_equal(frame, original)
    assert not np.array_equal(annotated, original)

