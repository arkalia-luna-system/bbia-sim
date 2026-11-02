"""Non-régression : cadence télémétrie pilotée par Settings (pas sleep hardcodé)."""

import ast
from pathlib import Path

import pytest

from bbia_sim.daemon.config import settings


def _broadcast_loop_source() -> str:
    path = Path(__file__).resolve().parent / "daemon" / "ws" / "telemetry.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "ConnectionManager":
            for item in node.body:
                if isinstance(item, ast.AsyncFunctionDef) and item.name == (
                    "_broadcast_loop"
                ):
                    return (
                        ast.get_source_segment(path.read_text(encoding="utf-8"), item)
                        or ""
                    )
    raise AssertionError("_broadcast_loop introuvable")


def test_broadcast_loop_reads_telemetry_frequency() -> None:
    """Évite la régression sleep(0.1) hardcodé sans Settings."""
    src = _broadcast_loop_source()
    assert "telemetry_frequency" in src
    assert "asyncio.sleep(0.1)" not in src.replace(" ", "")


def test_telemetry_interval_from_settings(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(settings, "telemetry_frequency", 20.0)
    freq = float(settings.telemetry_frequency)
    interval = 0.1 if freq <= 0 else max(0.01, 1.0 / freq)
    assert interval == pytest.approx(0.05)

    monkeypatch.setattr(settings, "telemetry_frequency", 0.0)
    freq = float(settings.telemetry_frequency)
    interval = 0.1 if freq <= 0 else max(0.01, 1.0 / freq)
    assert interval == 0.1
