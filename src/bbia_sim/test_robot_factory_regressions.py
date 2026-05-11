"""Tests de non-regression pour RobotFactory."""

from bbia_sim.robot_factory import RobotFactory


class _FakeQuery:
    def __init__(self, default):
        self.default = default

    def __str__(self) -> str:
        return "Query(default)"


def test_normalize_backend_type_handles_query_like_object() -> None:
    value = RobotFactory._normalize_backend_type(_FakeQuery("reachy_mini"))
    assert value == "reachy_mini"


def test_get_backend_info_accepts_query_like_object() -> None:
    info = RobotFactory.get_backend_info(_FakeQuery("mujoco"))
    assert info["name"] == "MuJoCo Simulator"


def test_create_multi_backend_default_excludes_auto(monkeypatch) -> None:
    calls: list[str] = []

    def _fake_create_backend(backend_type: str, **kwargs):
        calls.append(backend_type)
        return object()

    monkeypatch.setattr(
        RobotFactory, "create_backend", staticmethod(_fake_create_backend)
    )
    created = RobotFactory.create_multi_backend()

    assert "auto" not in calls
    assert "auto" not in created
    assert {"mujoco", "reachy", "reachy_mini"}.issubset(set(calls))
