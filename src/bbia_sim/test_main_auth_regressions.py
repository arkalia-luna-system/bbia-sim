"""Tests de non-regression pour l'authentification HTTP de main.py.

Évite le hang ExFAT : stub ``jinja2`` avant import de ``main`` (Jinja2Templates).
"""

from __future__ import annotations

import sys
from types import ModuleType

import pytest
from fastapi import HTTPException
from fastapi.security import HTTPAuthorizationCredentials


def _ensure_jinja2_stub() -> None:
    if "jinja2" in sys.modules:
        return
    stub = ModuleType("jinja2")
    stub.Environment = object  # type: ignore[attr-defined]
    stub.BaseLoader = object  # type: ignore[attr-defined]
    stub.TemplateNotFound = type("TemplateNotFound", (Exception,), {})  # type: ignore[attr-defined]
    stub.select_autoescape = lambda *a, **k: None  # type: ignore[attr-defined]
    sys.modules["jinja2"] = stub


_ensure_jinja2_stub()

from bbia_sim.daemon.app.main import verify_token  # noqa: E402
from bbia_sim.daemon.config import settings  # noqa: E402


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
