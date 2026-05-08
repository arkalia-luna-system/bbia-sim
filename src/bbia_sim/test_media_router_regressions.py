"""Tests de non-regression pour le routeur media."""

import pytest

from bbia_sim.daemon.app.routers import media


def test_clamp_unit_interval_handles_invalid_and_bounds() -> None:
    assert media._clamp_unit_interval(-1, 0.5) == 0.0
    assert media._clamp_unit_interval(2, 0.5) == 1.0
    assert media._clamp_unit_interval("bad", 0.4) == 0.4


class _DummySpeaker:
    volume = 1.8
    active = 1


class _DummyMicrophone:
    volume = -0.5
    active = 0


class _DummyCamera:
    enabled = "yes"


class _DummyMedia:
    speaker = _DummySpeaker()
    microphone = _DummyMicrophone()
    camera = _DummyCamera()


@pytest.mark.asyncio
async def test_get_media_status_clamps_robot_values(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(media, "_get_robot_media", lambda: _DummyMedia())
    status = await media.get_media_status()
    assert status.speaker_volume == 1.0
    assert status.microphone_volume == 0.0
    assert status.speaker_active is True
    assert status.microphone_active is False
    assert status.camera_enabled is True
