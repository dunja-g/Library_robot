import numpy as np
import pytest
import time

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


def test_multiple_streams_share_one_background_capture_source():
    class CountingBackend(FakeBackend):
        def __init__(self, frame):
            super().__init__(frame)
            self.capture_count = 0

        def capture_array(self):
            self.capture_count += 1
            return self.frame

    backend = CountingBackend(np.zeros((48, 64, 3), dtype=np.uint8))
    camera = Camera(64, 48, fps=20, stream_fps=15, backend=backend)
    first_stream = camera.generate_mjpeg()
    second_stream = camera.generate_mjpeg()

    assert next(first_stream).startswith(b"--frame")
    assert next(second_stream).startswith(b"--frame")
    stats = camera.get_stats()

    assert stats["clients"] == 2
    assert stats["frames_captured"] == backend.capture_count
    first_stream.close()
    second_stream.close()
    camera.stop()


def test_camera_stats_report_fresh_shared_frame():
    camera = Camera(
        64,
        48,
        fps=20,
        backend=FakeBackend(np.zeros((48, 64, 3), dtype=np.uint8)),
    )
    camera.get_frame()
    time.sleep(0.01)
    stats = camera.get_stats()

    assert stats["status"] == "OK"
    assert stats["target_fps"] == 20
    assert stats["frame_age_ms"] >= 0
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
