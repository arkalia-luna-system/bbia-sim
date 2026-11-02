"""Non-régression : kill port n'accepte que des PID numériques."""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import pytest

from bbia_sim.daemon.app.__main__ import kill_processes_on_port


def test_kill_processes_ignores_non_numeric_pids(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[list[str]] = []

    monkeypatch.setattr(
        "bbia_sim.daemon.app.__main__.shutil.which",
        lambda name: f"/bin/{name}",
    )

    def _run(cmd: list[str], **_kwargs: Any) -> MagicMock:
        if cmd[0].endswith("lsof"):
            m = MagicMock()
            m.returncode = 0
            m.stdout = "1234\nbad;rm\n5678\n"
            return m
        calls.append(cmd)
        m = MagicMock()
        m.returncode = 0
        m.stdout = ""
        return m

    monkeypatch.setattr("bbia_sim.daemon.app.__main__.subprocess.run", _run)
    monkeypatch.setattr("bbia_sim.daemon.app.__main__.time.sleep", lambda _s: None)

    killed = kill_processes_on_port(9999)
    assert killed >= 1
    for cmd in calls:
        # kill path + signal + pid
        assert cmd[-1].isdigit(), cmd
        assert cmd[-1] in {"1234", "5678"}
