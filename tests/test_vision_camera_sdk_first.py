import numpy as np

from src.bbia_sim.bbia_vision import BBIAVision


class _Camera:
    def __init__(self, mode: str = "get_image") -> None:
        self.mode = mode
        self.calls = 0

    def get_image(self):
        self.calls += 1
        # RGB 640x480
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def capture(self):
        self.calls += 1
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def read(self):
        self.calls += 1
        return True, np.zeros((480, 640, 3), dtype=np.uint8)


class _Media:
    def __init__(self, camera: _Camera) -> None:
        self.camera = camera


class _RobotApi:
    def __init__(self, camera: _Camera) -> None:
        self.media = _Media(camera)


def test_scan_environment_uses_sdk_camera_get_image():
    cam = _Camera("get_image")
    robot = _RobotApi(cam)
    vision = BBIAVision(robot_api=robot)
    result = vision.scan_environment()
    assert "timestamp" in result
    assert result["source"] in {"camera_sdk", "simulation"}
    assert cam.calls >= 1
