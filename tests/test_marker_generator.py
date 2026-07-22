import cv2
import numpy as np
import pytest

from aruco_codes.generate_markers import create_marker, save_markers


def test_create_marker_has_expected_shape_and_white_border():
    image = create_marker(0, image_size=300, border_px=20)

    assert image.shape == (300, 300)
    assert image.dtype == np.uint8
    assert np.all(image[:20, :] == 255)
    assert set(np.unique(image)) == {0, 255}


def test_save_markers_writes_requested_ids(tmp_path):
    paths = save_markers(tmp_path, marker_ids=[1, 4])

    assert [path.name for path in paths] == ["marker_1.png", "marker_4.png"]
    assert all(cv2.imread(str(path), cv2.IMREAD_GRAYSCALE) is not None for path in paths)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"marker_id": -1},
        {"marker_id": 0, "image_size": 0},
        {"marker_id": 0, "image_size": 100, "border_px": 50},
    ],
)
def test_create_marker_rejects_invalid_dimensions(kwargs):
    with pytest.raises(ValueError):
        create_marker(**kwargs)

