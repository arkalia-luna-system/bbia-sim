#!/usr/bin/env python3
"""Tests unitaires légers du garde matériel (sans import ReachyMiniBackend / SDK)."""

from __future__ import annotations

from pathlib import Path

import pytest

from bbia_sim.hardware_guard import (
    ALLOW_HARDWARE_ENV,
    is_hardware_allowed,
    require_hardware_allowed,
    resolve_use_sim,
)


@pytest.mark.unit
@pytest.mark.fast
def test_is_hardware_allowed_default_false(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(ALLOW_HARDWARE_ENV, raising=False)
    assert is_hardware_allowed() is False


@pytest.mark.unit
@pytest.mark.fast
@pytest.mark.parametrize("value", ["1", "true", "YES", "on"])
def test_is_hardware_allowed_truthy(
    monkeypatch: pytest.MonkeyPatch, value: str
) -> None:
    monkeypatch.setenv(ALLOW_HARDWARE_ENV, value)
    assert is_hardware_allowed() is True


@pytest.mark.unit
@pytest.mark.fast
def test_resolve_use_sim_forces_sim_without_flag(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv(ALLOW_HARDWARE_ENV, raising=False)
    assert resolve_use_sim(False, context="test") is True
    assert resolve_use_sim(True, context="test") is True


@pytest.mark.unit
@pytest.mark.fast
def test_resolve_use_sim_allows_hardware_with_flag(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv(ALLOW_HARDWARE_ENV, "1")
    assert resolve_use_sim(False, context="test") is False


@pytest.mark.unit
@pytest.mark.fast
def test_require_hardware_allowed_raises(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv(ALLOW_HARDWARE_ENV, raising=False)
    with pytest.raises(PermissionError, match=ALLOW_HARDWARE_ENV):
        require_hardware_allowed(action="test action")


@pytest.mark.unit
@pytest.mark.fast
def test_package_init_does_not_construct_reachy_mini() -> None:
    """src/bbia_sim/__init__.py ne doit pas instancier ReachyMini."""
    init_src = Path(__file__).resolve().parents[1] / "src" / "bbia_sim" / "__init__.py"
    content = init_src.read_text(encoding="utf-8")
    assert "ReachyMini(" not in content
    assert "goto_target" not in content
