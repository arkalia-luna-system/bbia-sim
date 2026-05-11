"""Tests de non-regression sur la sanitization des erreurs state router."""

import pytest

from bbia_sim.daemon.app.routers import state


@pytest.mark.asyncio
async def test_start_simulation_error_message_is_sanitized(monkeypatch: pytest.MonkeyPatch) -> None:
    class _FailingService:
        async def start_simulation(self, headless: bool = True) -> bool:
            raise RuntimeError("raw-internal-error")

    monkeypatch.setattr(state, "simulation_service", _FailingService())

    payload = await state.start_simulation()

    assert payload["status"] == "error"
    assert payload["message"] == "Erreur interne lors du démarrage de la simulation"
    assert "raw-internal-error" not in str(payload)


@pytest.mark.asyncio
async def test_stop_simulation_error_message_is_sanitized(monkeypatch: pytest.MonkeyPatch) -> None:
    class _FailingService:
        async def stop_simulation(self) -> None:
            raise RuntimeError("raw-internal-error")

    monkeypatch.setattr(state, "simulation_service", _FailingService())

    payload = await state.stop_simulation()

    assert payload["status"] == "error"
    assert payload["message"] == "Erreur interne lors de l'arrêt de la simulation"
    assert "raw-internal-error" not in str(payload)
