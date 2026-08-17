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


def test_reachy_mini_backend_sdk_surface_matches_pollen() -> None:
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

    required = [
        "goto_target",
        "set_target",
        "look_at_image",
        "look_at_world",
        "wake_up",
        "goto_sleep",
        "enable_motors",
        "disable_motors",
        "enable_gravity_compensation",
        "set_automatic_body_yaw",
        "start_recording",
        "stop_recording",
        "play_move",
        "start_head_tracking",
        "stop_head_tracking",
        "get_tracked_face",
        "enable_wobbling",
        "disable_wobbling",
        "cancel_move",
        "release_media",
        "acquire_media",
        "imu",
        "media",
        "media_released",
    ]
    missing = [name for name in required if not hasattr(ReachyMiniBackend, name)]
    assert missing == []


def test_reachy_mini_backend_connection_defaults_match_factory() -> None:
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connection_mode == "localhost_only"
    assert backend.robot_name == "reachy_mini"
    assert backend.host == "reachy-mini.local"
    assert backend.port == 8000
    assert backend.automatic_body_yaw is True


def test_play_move_accepts_official_sound_argument() -> None:
    import inspect

    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

    params = inspect.signature(ReachyMiniBackend.play_move).parameters
    assert "sound" in params
    assert params["sound"].default is True
