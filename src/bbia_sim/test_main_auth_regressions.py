"""Tests de non-regression auth HTTP — via ``http_auth`` (sans import ``main``/Jinja)."""

from __future__ import annotations

import pytest
from fastapi import HTTPException
from fastapi.security import HTTPAuthorizationCredentials

from bbia_sim.daemon.app.http_auth import verify_token
from bbia_sim.daemon.config import settings


def _creds(token: str) -> HTTPAuthorizationCredentials:
    return HTTPAuthorizationCredentials(scheme="Bearer", credentials=token)


def test_verify_token_accepts_valid_token(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(settings, "api_token", "secret-token")
    assert verify_token(_creds("secret-token")) == "secret-token"


def test_verify_token_rejects_invalid_token(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(settings, "api_token", "secret-token")
    with pytest.raises(HTTPException) as exc:
        verify_token(_creds("bad-token"))
    assert exc.value.status_code == 401
    assert exc.value.headers == {"WWW-Authenticate": "Bearer"}


def test_verify_token_logs_masked_token(
    monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    monkeypatch.setattr(settings, "api_token", "secret-token")
    with pytest.raises(HTTPException):
        verify_token(_creds("bad-token"))
    assert "bad-token" not in caplog.text


def test_main_reexports_verify_token_from_http_auth() -> None:
    """Garde le câblage ``main`` → ``http_auth`` (sans exécuter l'app)."""
    import ast
    from pathlib import Path

    src = Path(__file__).resolve().parent / "daemon" / "app" / "main.py"
    tree = ast.parse(src.read_text(encoding="utf-8"))
    found = False
    for node in tree.body:
        if (
            isinstance(node, ast.ImportFrom)
            and node.module == "bbia_sim.daemon.app.http_auth"
        ):
            names = {a.name for a in node.names}
            if "verify_token" in names:
                found = True
                break
    assert found, "main.py doit importer verify_token depuis http_auth"
