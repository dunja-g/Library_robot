import numpy as np
import pytest

from pi.camera import Camera, CameraError


class FakeBackend:
    def __init__(self, frame):
        self.frame = frame
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def capture_array(self):
        return self.frame

    def stop(self):
        self.stopped = True


def test_camera_captures_defensive_bgr_copy():
    source = np.zeros((48, 64, 3), dtype=np.uint8)
    backend = FakeBackend(source)
    camera = Camera(64, 48, backend=backend)

    frame = camera.get_frame()
    frame[0, 0] = 255

    assert backend.started
    assert np.all(source[0, 0] == 0)
    camera.stop()
    assert backend.stopped


def test_mjpeg_generator_produces_multipart_jpeg():
    backend = FakeBackend(np.zeros((48, 64, 3), dtype=np.uint8))
    camera = Camera(64, 48, backend=backend)

    chunk = next(camera.generate_mjpeg())

    assert chunk.startswith(b"--frame\r\nContent-Type: image/jpeg")
    assert b"\xff\xd8" in chunk
    camera.stop()


def test_camera_rejects_invalid_backend_frame():
    camera = Camera(64, 48, backend=FakeBackend(None))
    with pytest.raises(CameraError):
        camera.get_frame()


def test_stopped_camera_cannot_capture():
    camera = Camera(64, 48, backend=FakeBackend(np.zeros((48, 64, 3))))
    camera.stop()
    with pytest.raises(CameraError):
        camera.get_frame()
