"""Régressions anticipations issues/GHSA Pollen (suite courte)."""

from __future__ import annotations

import asyncio
from typing import cast
from unittest.mock import MagicMock

import pytest
from fastapi import WebSocket

from bbia_sim.daemon.app import safe_media_upload
from bbia_sim.daemon.config import Settings, settings
from bbia_sim.daemon.ws.keepalive import (
    get_ws_keepalive_timeout,
    receive_text_with_keepalive,
    uvicorn_ws_keepalive_kwargs,
)
from bbia_sim.head_pose_guard import (
    clamp_head_pitch_roll,
    clamp_head_pose_matrix,
    get_head_pitch_roll_limit_rad,
)
from bbia_sim.head_tracking_guard import (
    clamp_look_direction_rad,
    clamp_look_image_uv,
    should_skip_head_tracking,
)
from bbia_sim.media_guard import (
    is_dangerous_media_backend,
    looks_like_shell_amixer,
    looks_like_usb_reachy_audio,
    refuse_shell_audio_command,
    resolve_media_backend,
)
from bbia_sim.motors_settle_guard import (
    await_motors_settle_if_needed,
    get_motors_settle_seconds,
    mark_motors_enabled,
    reset_motors_settle_state,
)
from bbia_sim.play_move_guard import get_play_move_initial_goto_seconds
from bbia_sim.sdk_connection import (
    call_with_reconnect,
    get_sdk_connect_timeout,
    get_sdk_heartbeat_timeout,
    get_sdk_reconnect_attempts,
)
from bbia_sim.sleep_guard import (
    clamp_yaw_amplitude,
    prepare_goto_sleep,
    sleep_hold_torque_requested,
)


def test_ws_keepalive_timeout_configurable(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pollen #1494 : timeout WS configurable via BBIA_WS_KEEPALIVE_TIMEOUT."""
    monkeypatch.setenv("BBIA_WS_KEEPALIVE_TIMEOUT", "45")
    fresh = Settings()
    assert fresh.ws_keepalive_timeout == 45.0


def test_uvicorn_ws_kwargs_wired_to_settings(monkeypatch: pytest.MonkeyPatch) -> None:
    """#1494 : uvicorn reçoit ws_ping_timeout depuis settings (pas orphelin)."""
    monkeypatch.setattr(settings, "ws_keepalive_timeout", 45.0)
    kwargs = uvicorn_ws_keepalive_kwargs()
    assert kwargs["ws_ping_timeout"] == 45.0
    assert kwargs["ws_ping_interval"] == 22.5
    assert get_ws_keepalive_timeout() == 45.0


def test_uvicorn_ws_kwargs_disable_when_nonpositive(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(settings, "ws_keepalive_timeout", 0.0)
    kwargs = uvicorn_ws_keepalive_kwargs()
    assert kwargs["ws_ping_timeout"] is None
    assert kwargs["ws_ping_interval"] is None


@pytest.mark.asyncio
async def test_receive_text_with_keepalive_times_out(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """#1494 : receive applicatif borné par le timeout settings."""

    class _SilentWS:
        async def receive_text(self) -> str:
            await asyncio.sleep(3600)
            return "never"

    monkeypatch.setattr(settings, "ws_keepalive_timeout", 0.05)
    # Py3.10: asyncio.TimeoutError n'est pas encore alias de builtins.TimeoutError
    with pytest.raises((TimeoutError, asyncio.TimeoutError)):
        await receive_text_with_keepalive(cast(WebSocket, _SilentWS()))


def test_sound_allowlist_accepts_wav_magic() -> None:
    content = b"RIFF" + b"\x00" * 8 + b"WAVEfmt "
    name = safe_media_upload.validate_sound_payload("beep.wav", content)
    assert name == "beep.wav"


def test_no_bluetooth_cmd_surface_in_bbia() -> None:
    """GHSA-vg3x / GHSA-993g : BBIA ne doit pas embarquer un handler CMD_/PIN_."""
    import bbia_sim

    # Smoke : le package principal ne réexporte pas de bluetooth_service
    assert not hasattr(bbia_sim, "bluetooth_service")
    assert not hasattr(bbia_sim, "BluetoothCommandService")


def test_sdk_connect_timeout_and_reconnect_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """#1494 suite : timeout + reconnect SDK configurables."""
    monkeypatch.setenv("BBIA_SDK_CONNECT_TIMEOUT", "7.5")
    monkeypatch.setenv("BBIA_SDK_RECONNECT_ATTEMPTS", "4")
    monkeypatch.setenv("BBIA_SDK_HEARTBEAT_TIMEOUT", "3.5")
    assert get_sdk_connect_timeout() == 7.5
    assert get_sdk_reconnect_attempts() == 4
    assert get_sdk_heartbeat_timeout() == 3.5
    fresh = Settings()
    assert fresh.sdk_connect_timeout == 7.5
    assert fresh.sdk_reconnect_attempts == 4


def test_sdk_reconnect_retries_then_succeeds(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """#1494 / #1466 : call_with_reconnect réessaie sur TimeoutError."""
    monkeypatch.setenv("BBIA_SDK_RECONNECT_ATTEMPTS", "3")
    monkeypatch.setenv("BBIA_SDK_RECONNECT_DELAY", "0")
    calls = {"n": 0}

    def factory() -> str:
        calls["n"] += 1
        if calls["n"] < 3:
            raise TimeoutError("relay down")
        return "ok"

    assert call_with_reconnect(factory, label="test") == "ok"
    assert calls["n"] == 3


def test_sdk_reconnect_exhausts(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("BBIA_SDK_RECONNECT_ATTEMPTS", "2")
    monkeypatch.setenv("BBIA_SDK_RECONNECT_DELAY", "0")

    def factory() -> str:
        raise ConnectionError("refused")

    with pytest.raises(ConnectionError):
        call_with_reconnect(factory, label="test-fail")


def test_media_guard_forces_no_media_when_disabled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """#1507/#1525 : BBIA_DISABLE_AUDIO force no_media."""
    monkeypatch.setenv("BBIA_DISABLE_AUDIO", "1")
    assert resolve_media_backend("default") == "no_media"
    assert resolve_media_backend("gstreamer") == "no_media"


def test_media_guard_refuses_dangerous_backends(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("BBIA_DISABLE_AUDIO", raising=False)
    assert is_dangerous_media_backend("gstreamer")
    assert is_dangerous_media_backend("alsa_amixer")
    assert resolve_media_backend("audiomixer") == "no_media"
    assert resolve_media_backend("no_media") == "no_media"


def test_media_guard_detects_usb_reachy_audio(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """#1507 : détection/refus clair du chemin USB Reachy Mini Audio."""
    monkeypatch.delenv("BBIA_DISABLE_AUDIO", raising=False)
    assert looks_like_usb_reachy_audio("reachy_mini_audio")
    assert looks_like_usb_reachy_audio("usb_xmos")
    assert not looks_like_usb_reachy_audio("sounddevice")
    assert resolve_media_backend("reachy_mini_audio") == "no_media"
    assert resolve_media_backend("pulse_usb") == "no_media"


def test_refuse_shell_amixer() -> None:
    """#1521 : pas de shell amixer."""
    assert looks_like_shell_amixer("amixer set Master 50%")
    assert looks_like_shell_amixer("gst-launch-1.0 audiomixer")
    with pytest.raises(ValueError, match="amixer"):
        refuse_shell_audio_command("amixer sset PCM 80%", context="test")


def test_head_tracking_skip_in_sim_by_default(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """#1453/#1451/#1450 : défaut sûr skip en sim."""
    monkeypatch.setenv("BBIA_HEAD_TRACKING_SIM", "0")
    assert should_skip_head_tracking(use_sim=True) is True
    monkeypatch.setenv("BBIA_HEAD_TRACKING_SIM", "1")
    assert should_skip_head_tracking(use_sim=True) is False
    assert should_skip_head_tracking(use_sim=False) is False


def test_head_tracking_clamps() -> None:
    yaw, pitch = clamp_look_direction_rad(2.0, -2.0)
    assert abs(yaw) <= 0.35
    assert abs(pitch) <= 0.35
    u, v = clamp_look_image_uv(9999, -5)
    assert u == 640
    assert v == 0


def test_yaw_and_sleep_guards(monkeypatch: pytest.MonkeyPatch) -> None:
    """#1479/#1478 : clamp yaw + prepare_goto_sleep soft."""
    monkeypatch.setenv("BBIA_YAW_SAFE_AMPLITUDE", "0.2")
    assert clamp_yaw_amplitude(1.0) == pytest.approx(0.2)
    monkeypatch.setenv("BBIA_SLEEP_HOLD_TORQUE", "0")
    assert sleep_hold_torque_requested() is False
    report = prepare_goto_sleep(None, use_sim=True)
    assert report["sim"] is True
    assert "simulation_or_no_robot" in report["warnings"]

    robot = MagicMock()
    robot.get_current_joint_positions.return_value = ([0.0] * 12, [0.0, 0.0])
    monkeypatch.setenv("BBIA_SLEEP_HOLD_TORQUE", "1")
    report2 = prepare_goto_sleep(robot, use_sim=False)
    assert report2["hold_torque_applied"] is True
    robot.enable_motors.assert_called()


def test_backend_head_tracking_noop_in_sim(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Backend : start_head_tracking no-op en sim (pas d'appel SDK)."""
    monkeypatch.delenv("BBIA_ALLOW_HARDWARE", raising=False)
    monkeypatch.setenv("BBIA_HEAD_TRACKING_SIM", "0")
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

    backend = ReachyMiniBackend(use_sim=True, media_backend="gstreamer")
    assert backend.media_backend == "no_media"
    backend.connect()
    fake = MagicMock()
    backend.robot = fake  # type: ignore[assignment]
    backend.use_sim = True
    backend.start_head_tracking(weight=0.5)
    fake.start_head_tracking.assert_not_called()
    backend.disconnect()


def test_certifi_floor_declared_in_pyproject() -> None:
    """Alignement Pollen 1.13 : certifi>=2026.10.19 dans pyproject."""
    from pathlib import Path

    text = Path("pyproject.toml").read_text(encoding="utf-8")
    assert "certifi>=2026.10.19" in text
    assert "reachy-mini==1.13.0" in text


def test_head_pitch_roll_clamp_40deg(monkeypatch: pytest.MonkeyPatch) -> None:
    """#1449 : clamp soft ±40° pitch/roll (doc Pollen absente du SDK)."""
    monkeypatch.delenv("BBIA_HEAD_PITCH_ROLL_LIMIT_DEG", raising=False)
    lim = get_head_pitch_roll_limit_rad()
    assert lim is not None
    assert abs(lim - __import__("math").radians(40)) < 1e-9
    p, r = clamp_head_pitch_roll(50.0, -55.0, degrees=True)
    assert p == pytest.approx(40.0)
    assert r == pytest.approx(-40.0)
    monkeypatch.setenv("BBIA_HEAD_PITCH_ROLL_LIMIT_DEG", "0")
    assert get_head_pitch_roll_limit_rad() is None
    p2, r2 = clamp_head_pitch_roll(50.0, -55.0, degrees=True)
    assert p2 == 50.0 and r2 == -55.0


def test_motors_settle_after_enable(monkeypatch: pytest.MonkeyPatch) -> None:
    """#1430 : settle soft après enable_motors avant set_target."""
    monkeypatch.setenv("BBIA_MOTORS_SETTLE_S", "0.05")
    reset_motors_settle_state()
    assert get_motors_settle_seconds() == pytest.approx(0.05)
    assert await_motors_settle_if_needed() == 0.0  # pas encore d'enable
    mark_motors_enabled()
    waited = await_motors_settle_if_needed(context="test")
    assert waited > 0.0
    # deuxième appel : fenêtre déjà écoulée
    assert await_motors_settle_if_needed() == 0.0
    monkeypatch.setenv("BBIA_MOTORS_SETTLE_S", "0")
    reset_motors_settle_state()
    mark_motors_enabled()
    assert await_motors_settle_if_needed() == 0.0


def test_head_pose_matrix_clamp_pitch_roll(monkeypatch: pytest.MonkeyPatch) -> None:
    """#1449 : matrices 4×4 hors create_head_pose aussi clampées."""
    import math

    import numpy as np
    from scipy.spatial.transform import Rotation as R

    monkeypatch.setenv("BBIA_HEAD_PITCH_ROLL_LIMIT_DEG", "40")
    # pitch 55° > 40°
    pose = np.eye(4, dtype=np.float64)
    pose[:3, :3] = R.from_euler("xyz", [0.0, math.radians(55.0), 0.0]).as_matrix()
    pose[0, 3] = 0.01  # translation préservée
    out = clamp_head_pose_matrix(pose)
    assert out is not pose
    roll, pitch, yaw = R.from_matrix(np.asarray(out)[:3, :3]).as_euler("xyz")
    assert abs(pitch) <= math.radians(40.0) + 1e-9
    assert abs(roll) <= math.radians(40.0) + 1e-9
    assert abs(float(np.asarray(out)[0, 3]) - 0.01) < 1e-12


def test_head_pose_matrix_clamp_disabled(monkeypatch: pytest.MonkeyPatch) -> None:
    import math

    import numpy as np
    from scipy.spatial.transform import Rotation as R

    monkeypatch.setenv("BBIA_HEAD_PITCH_ROLL_LIMIT_DEG", "0")
    pose = np.eye(4, dtype=np.float64)
    pose[:3, :3] = R.from_euler("xyz", [0.0, math.radians(55.0), 0.0]).as_matrix()
    out = clamp_head_pose_matrix(pose)
    assert out is pose


def test_play_move_initial_goto_default_safe(monkeypatch: pytest.MonkeyPatch) -> None:
    """#1417 : défaut soft 1.0 s (workaround waiting→yes1)."""
    monkeypatch.delenv("BBIA_PLAY_MOVE_INITIAL_GOTO_S", raising=False)
    assert get_play_move_initial_goto_seconds() == pytest.approx(1.0)
    assert get_play_move_initial_goto_seconds(None) == pytest.approx(1.0)
    # override explicite 0.0 autorisé (caller assume le risque)
    assert get_play_move_initial_goto_seconds(0.0) == pytest.approx(0.0)
    monkeypatch.setenv("BBIA_PLAY_MOVE_INITIAL_GOTO_S", "0")
    assert get_play_move_initial_goto_seconds() == pytest.approx(0.0)
    monkeypatch.setenv("BBIA_PLAY_MOVE_INITIAL_GOTO_S", "2.5")
    assert get_play_move_initial_goto_seconds() == pytest.approx(2.5)


def test_backend_play_move_applies_initial_goto_guard(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """#1417 : ReachyMiniBackend.play_move ne force plus 0.0 par défaut."""
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

    monkeypatch.setenv("BBIA_PLAY_MOVE_INITIAL_GOTO_S", "1.0")
    backend = ReachyMiniBackend()
    backend.is_connected = True
    robot = MagicMock()
    backend.robot = robot
    backend.play_move(MagicMock(name="move"))
    assert robot.play_move.called
    args, _kwargs = robot.play_move.call_args
    # play_move(move, play_frequency, duration, sound=...)
    assert len(args) >= 3
    assert args[2] == pytest.approx(1.0)
