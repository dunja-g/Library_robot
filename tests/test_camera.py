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


def test_stream_client_is_released_when_generator_closes():
    backend = FakeBackend(np.zeros((48, 64, 3), dtype=np.uint8))
    camera = Camera(64, 48, backend=backend)
    stream = camera.generate_mjpeg()

    next(stream)
    assert camera.get_metrics()["stream_clients"] == 1

    stream.close()
    assert camera.get_metrics()["stream_clients"] == 0
    camera.stop()


def test_camera_metrics_report_single_shared_capture_pipeline():
    backend = FakeBackend(np.zeros((48, 64, 3), dtype=np.uint8))
    camera = Camera(64, 48, fps=15, backend=backend)

    camera.get_frame()
    metrics = camera.get_metrics()

    assert metrics["running"] is True
    assert metrics["target_fps"] == 15
    assert metrics["frame_sequence"] >= 1
    assert metrics["frame_age_ms"] >= 0
    assert metrics["stream_clients"] == 0
    camera.stop()


def test_camera_rejects_invalid_backend_frame():
    camera = Camera(64, 48, backend=FakeBackend(None))
    with pytest.raises(CameraError):
        camera.get_frame()


def test_stopped_camera_cannot_capture():
    camera = Camera(64, 48, backend=FakeBackend(np.zeros((48, 64, 3))))
    camera.stop()


def test_mjpeg_waits_until_controller_has_a_frame():
    backend = FakeBackend(np.zeros((48, 64, 3), dtype=np.uint8))
    camera = Camera(64, 48, backend=backend)
    frames = iter([None, np.zeros((48, 64, 3), dtype=np.uint8)])

    chunk = next(camera.generate_mjpeg(lambda: next(frames)))

    assert chunk.startswith(b"--frame")
    camera.stop()
    with pytest.raises(CameraError):
        camera.get_frame()
