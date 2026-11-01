"""Régressions anticipations issues/GHSA Pollen (suite courte)."""

from __future__ import annotations

from bbia_sim.daemon.app import safe_media_upload
from bbia_sim.daemon.config import Settings


def test_ws_keepalive_timeout_configurable(monkeypatch) -> None:
    """Pollen #1494 : timeout WS configurable via BBIA_WS_KEEPALIVE_TIMEOUT."""
    monkeypatch.setenv("BBIA_WS_KEEPALIVE_TIMEOUT", "45")
    settings = Settings()
    assert settings.ws_keepalive_timeout == 45.0


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
