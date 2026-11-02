"""Régressions hygiène télémétrie + ecosystem (joint honoré)."""

from __future__ import annotations

import pytest


@pytest.mark.asyncio
async def test_telemetry_info_uses_settings_frequency(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from bbia_sim.daemon import config as cfg
    from bbia_sim.daemon.ws import telemetry as tel

    monkeypatch.setattr(cfg.settings, "telemetry_frequency", 5.0, raising=False)
    info = await tel.get_telemetry_info()
    assert info["frequency_hz"] == 5.0
    assert info["frequency"] == "5Hz"


@pytest.mark.asyncio
async def test_apply_emotion_honors_joint_query(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from bbia_sim.daemon.app.routers import ecosystem as eco

    class _Emo:
        emotions: dict[str, dict[str, object]] = {"happy": {}}
        current_emotion = "neutral"
        emotion_intensity = 0.0

    monkeypatch.setattr(eco, "_bbia_emotions_cls", lambda: _Emo)
    resp = await eco.apply_emotion(
        emotion="happy", intensity=0.4, duration=1.0, joint="stewart_1"
    )
    assert resp.joints_affected == ["stewart_1"]
    assert resp.emotion == "happy"
