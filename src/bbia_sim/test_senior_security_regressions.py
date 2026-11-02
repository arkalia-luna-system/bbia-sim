"""Régressions senior-review : aliases env, token prod, yaw API, HF pip, presets."""

from __future__ import annotations

from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import pytest
from fastapi import HTTPException
from fastapi.security import HTTPAuthorizationCredentials

from bbia_sim.daemon.app.http_auth import verify_token
from bbia_sim.daemon.app.routers import presets as presets_mod
from bbia_sim.daemon.config import DEFAULT_DEV_API_TOKEN, Settings, settings
from bbia_sim.daemon.ws.auth import ws_auth_rejected, ws_token_is_valid
from bbia_sim.sleep_guard import clamp_yaw_amplitude


def _creds(token: str) -> HTTPAuthorizationCredentials:
    return HTTPAuthorizationCredentials(scheme="Bearer", credentials=token)


def test_settings_accepts_bbia_token_and_env_aliases(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("BBIA_API_TOKEN", raising=False)
    monkeypatch.delenv("BBIA_ENVIRONMENT", raising=False)
    monkeypatch.setenv("BBIA_TOKEN", "alias-secret-token")
    monkeypatch.setenv("BBIA_ENV", "prod")
    cfg = Settings(_env_file=None)  # type: ignore[call-arg]
    assert cfg.api_token == "alias-secret-token"  # nosec B105 — fixture test
    assert cfg.environment == "prod"
    assert cfg.is_production() is True


def test_settings_canonical_env_wins_over_alias(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("BBIA_API_TOKEN", "canonical-token")
    monkeypatch.setenv("BBIA_TOKEN", "alias-token")
    monkeypatch.setenv("BBIA_ENVIRONMENT", "dev")
    monkeypatch.setenv("BBIA_ENV", "prod")
    cfg = Settings(_env_file=None)  # type: ignore[call-arg]
    assert cfg.api_token == "canonical-token"  # nosec B105 — fixture test
    assert cfg.environment == "dev"


def test_require_secure_production_token_rejects_default(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("BBIA_ENVIRONMENT", "prod")
    monkeypatch.setenv("BBIA_API_TOKEN", DEFAULT_DEV_API_TOKEN)
    monkeypatch.delenv("BBIA_TOKEN", raising=False)
    cfg = Settings(_env_file=None)  # type: ignore[call-arg]
    with pytest.raises(RuntimeError, match="Production refusée"):
        cfg.require_secure_production_token()


def test_verify_token_rejects_empty(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(settings, "api_token", "secret-token")
    monkeypatch.setattr(settings, "environment", "dev")
    with pytest.raises(HTTPException) as exc:
        verify_token(_creds(""))
    assert exc.value.status_code == 401


def test_verify_token_prod_insecure_returns_503(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(settings, "environment", "prod")
    monkeypatch.setattr(settings, "api_token", DEFAULT_DEV_API_TOKEN)
    with pytest.raises(HTTPException) as exc:
        verify_token(_creds(DEFAULT_DEV_API_TOKEN))
    assert exc.value.status_code == 503


def test_ws_rejects_default_token_in_prod(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(settings, "environment", "prod")
    monkeypatch.setattr(settings, "api_token", DEFAULT_DEV_API_TOKEN)
    assert ws_token_is_valid(DEFAULT_DEV_API_TOKEN) is False
    assert ws_auth_rejected(DEFAULT_DEV_API_TOKEN) is True
    assert ws_auth_rejected("anything") is True


def test_set_target_body_yaw_clamps(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("BBIA_YAW_SAFE_AMPLITUDE", "0.25")
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

    backend = ReachyMiniBackend(use_sim=True, media_backend="no_media")
    backend.is_connected = True
    fake = MagicMock()
    backend.robot = fake  # type: ignore[assignment]
    backend.set_target_body_yaw(1.0)
    fake.set_target_body_yaw.assert_called_once()
    sent = float(fake.set_target_body_yaw.call_args[0][0])
    assert sent == pytest.approx(0.25)
    assert sent == clamp_yaw_amplitude(1.0)


@pytest.mark.asyncio
async def test_hf_pip_install_skipped_by_default(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.delenv("BBIA_HF_PIP_INSTALL", raising=False)
    from bbia_sim.daemon.app import hf_app_installer as hf_mod
    from bbia_sim.daemon.app.hf_app_installer import HFAppInstaller

    installer = HFAppInstaller(apps_dir=tmp_path)
    pip_calls: list[str] = []

    async def _boom(_req: Path) -> None:
        pip_calls.append("pip")
        raise AssertionError("pip ne doit pas tourner par défaut")

    monkeypatch.setattr(installer, "_install_dependencies", _boom)

    def _fake_subprocess_run(*_a: Any, **_k: Any) -> Any:
        # Simule un clone réussi : crée le dossier cible + requirements
        # argv: git clone <url> <path>
        dest = Path(_a[0][3]) if _a and len(_a[0]) > 3 else tmp_path / "demo"
        dest.mkdir(parents=True, exist_ok=True)
        (dest / "requirements.txt").write_text("requests==2.0.0\n", encoding="utf-8")

        class _R:
            returncode = 0
            stderr = ""
            stdout = ""

        return _R()

    async def _to_thread(fn: Any, *a: Any, **k: Any) -> Any:
        return fn(*a, **k)

    monkeypatch.setattr(hf_mod.subprocess, "run", _fake_subprocess_run)
    monkeypatch.setattr(hf_mod.asyncio, "to_thread", _to_thread)

    result = await installer.install_app("user/demo", "demo")
    assert result["status"] == "installed"
    assert result.get("pip_install") is False
    assert pip_calls == []


def test_preset_name_allowlist_rejects_traversal(tmp_path: Path) -> None:
    monkeypatch_dir = tmp_path / "presets"
    monkeypatch_dir.mkdir()
    old = presets_mod.PRESETS_DIR
    presets_mod.PRESETS_DIR = monkeypatch_dir
    try:
        with pytest.raises(HTTPException) as exc:
            presets_mod._safe_preset_path("../evil")
        assert exc.value.status_code == 400
        with pytest.raises(HTTPException):
            presets_mod._safe_preset_path("a/b")
        with pytest.raises(HTTPException):
            presets_mod._safe_preset_path("bad name")
        path = presets_mod._safe_preset_path("happy")
        assert path.name == "happy.json"
        assert path.parent == Path(monkeypatch_dir).resolve()
    finally:
        presets_mod.PRESETS_DIR = old
