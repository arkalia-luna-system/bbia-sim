"""Tests de non-regression pour RobotAPI."""

from bbia_sim.robot_api import RobotAPI


class _DummyRobot(RobotAPI):
    def __init__(self) -> None:
        super().__init__()
        self.is_connected = True
        self._joints: dict[str, float] = {"yaw_body": 0.0, "pitch_head": 0.0}
        self.step_calls = 0
        self.last_set: tuple[str, float] | None = None

    def connect(self) -> bool:
        self.is_connected = True
        return True

    def disconnect(self) -> bool:
        self.is_connected = False
        return True

    def get_available_joints(self) -> list[str]:
        return list(self._joints.keys())

    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        if joint_name not in self._joints:
            return False
        self._joints[joint_name] = position
        self.last_set = (joint_name, position)
        return True

    def get_joint_pos(self, joint_name: str) -> float | None:
        return self._joints.get(joint_name)

    def step(self) -> bool:
        self.step_calls += 1
        return True

    def emergency_stop(self) -> bool:
        return True


def test_run_behavior_rejects_non_positive_duration() -> None:
    robot = _DummyRobot()
    assert robot.run_behavior("greeting", duration=0) is False
    assert robot.run_behavior("greeting", duration=-1) is False


def test_run_behavior_handles_very_short_duration_without_crash() -> None:
    robot = _DummyRobot()
    assert robot.run_behavior("greeting", duration=0.01) is True
    assert robot.step_calls >= 1


def test_set_sleeping_pose_falls_back_when_goto_target_not_implemented() -> None:
    robot = _DummyRobot()
    # _DummyRobot herite goto_target() de RobotAPI -> NotImplementedError.
    assert robot.set_sleeping_pose(duration=0.1) is True
    assert robot.last_set is not None
    assert robot.last_set[0] == "yaw_body"
