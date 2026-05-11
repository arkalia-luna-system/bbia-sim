"""Tests de non-regression pour l'authentification HTTP de main.py."""

import pytest
from fastapi import HTTPException
from fastapi.security import HTTPAuthorizationCredentials

from bbia_sim.daemon.app.main import verify_token
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
