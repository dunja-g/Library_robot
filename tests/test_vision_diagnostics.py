import json

import cv2

from aruco_codes.generate_markers import create_marker
from pi.aruco_detector import ArucoDetector
from pi.vision_diagnostics import inspect_image, serialise_detections


def test_inspect_image_writes_annotation_and_json_safe_report(tmp_path):
    source = tmp_path / "marker.png"
    output = tmp_path / "annotated.png"
    cv2.imwrite(str(source), create_marker(4, 300, 30))

    report = inspect_image(source, ArucoDetector(), output)

    assert report["detections"][0]["id"] == 4
    assert output.exists()
    json.dumps(report)


def test_serialise_detections_handles_detector_corner_arrays():
    marker = cv2.cvtColor(create_marker(1, 300, 30), cv2.COLOR_GRAY2BGR)
    detections = ArucoDetector().detect(marker)
    report = serialise_detections(detections)

    assert report[0]["id"] == 1
    assert len(report[0]["corners"]) == 4
