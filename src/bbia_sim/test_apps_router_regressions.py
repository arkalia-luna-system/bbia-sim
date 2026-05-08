"""Tests de non-regression pour le routeur apps."""

from typing import Any

import pytest
from fastapi import HTTPException

from bbia_sim.daemon.app.routers import apps


def test_append_job_log_caps_history() -> None:
    job: dict[str, Any] = {"logs": []}
    for idx in range(apps._MAX_JOB_LOGS + 25):
        apps._append_job_log(job, f"log-{idx}")

    logs = list(job["logs"])
    assert len(logs) == apps._MAX_JOB_LOGS
    assert logs[0] == "log-25"
    assert logs[-1] == f"log-{apps._MAX_JOB_LOGS + 24}"


@pytest.mark.asyncio
async def test_install_app_rejects_empty_name() -> None:
    with pytest.raises(HTTPException) as exc:
        await apps.install_app({"name": "   ", "source_kind": "local"})
    assert exc.value.status_code == 422


@pytest.mark.asyncio
async def test_remove_app_rejects_empty_name() -> None:
    with pytest.raises(HTTPException) as exc:
        await apps.remove_app("   ")
    assert exc.value.status_code == 422
