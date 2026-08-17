"""Surface SDK Reachy Mini alignée sur pollen-robotics/reachy_mini."""

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


def test_backend_exposes_official_sdk_methods() -> None:
    required = [
        "goto_target",
        "set_target",
        "look_at_image",
        "look_at_world",
        "wake_up",
        "goto_sleep",
        "enable_motors",
        "disable_motors",
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
        "set_automatic_body_yaw",
    ]
    missing = [name for name in required if not hasattr(ReachyMiniBackend, name)]
    assert missing == []


def test_backend_pollen_connection_defaults() -> None:
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connection_mode == "localhost_only"
    assert backend.robot_name == "reachy_mini"
    assert backend.host == "reachy-mini.local"
    assert backend.port == 8000
    assert backend.automatic_body_yaw is True


def test_head_tracking_is_safe_in_simulation() -> None:
    backend = ReachyMiniBackend(use_sim=True)
    backend.connect()
    backend.start_head_tracking(weight=0.5)
    assert backend.get_tracked_face() is None
    backend.stop_head_tracking()
    backend.cancel_move()
    backend.enable_wobbling()
    backend.disable_wobbling()
    backend.set_automatic_body_yaw(True)
    assert backend.imu is None
    backend.disconnect()


def test_play_move_signature_matches_pollen() -> None:
    import inspect

    params = inspect.signature(ReachyMiniBackend.play_move).parameters
    assert "sound" in params
    assert params["sound"].default is True
