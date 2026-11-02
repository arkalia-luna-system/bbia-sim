"""Non-régression découpage HF : ``hf_guards`` + banque de réponses."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from bbia_sim import bbia_hf_response_bank as bank
from bbia_sim.hf_guards import (
    HF_PIP_INSTALL_ENV,
    hf_pip_install_enabled,
    is_valid_hf_space_id,
    normalize_hf_app_name,
    validate_hf_space_id,
)


@pytest.mark.parametrize(
    "space_id",
    [
        "user/demo",
        "Org-Name/my_space.v2",
        "a/b",
    ],
)
def test_validate_hf_space_id_accepts_allowlisted(space_id: str) -> None:
    assert is_valid_hf_space_id(space_id) is True
    assert validate_hf_space_id(space_id) == space_id


@pytest.mark.parametrize(
    "bad_id",
    [
        "",
        "noslash",
        "../evil/space",
        "user/../evil",
        "user/space.git;rm",
        "https://evil.example/x",
        "user/space with space",
        "user/space\n",
        "user@host/space",
        None,
        123,
    ],
)
def test_validate_hf_space_id_rejects_injected(bad_id: object) -> None:
    assert is_valid_hf_space_id(bad_id) is False
    with pytest.raises(ValueError, match="HF Space ID invalide"):
        validate_hf_space_id(bad_id)


def test_normalize_hf_app_name() -> None:
    assert normalize_hf_app_name("demo-app") == "demo-app"
    with pytest.raises(ValueError):
        normalize_hf_app_name("../evil")
    with pytest.raises(ValueError):
        normalize_hf_app_name("a/b")


def test_hf_pip_install_default_off(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(HF_PIP_INSTALL_ENV, raising=False)
    assert hf_pip_install_enabled() is False
    assert hf_pip_install_enabled({}) is False


@pytest.mark.parametrize("raw", ["1", "true", "YES", "on"])
def test_hf_pip_install_opt_in(
    monkeypatch: pytest.MonkeyPatch,
    raw: str,
) -> None:
    monkeypatch.setenv(HF_PIP_INSTALL_ENV, raw)
    assert hf_pip_install_enabled() is True


def test_response_bank_stable_and_reexported() -> None:
    assert len(bank.SUFFIX_POOL) >= 4
    assert len(bank._expert_quality_padding) >= 10
    assert all(isinstance(s, str) and len(s) >= 30 for s in bank.SUFFIX_POOL)
    assert isinstance(bank.SAFE_FALLBACK, str) and len(bank.SAFE_FALLBACK) >= 20

    # Imports stables sans charger torch : AST sur le god-file
    hf_path = Path(__file__).resolve().parent / "bbia_huggingface.py"
    src = hf_path.read_text(encoding="utf-8")
    tree = ast.parse(src)
    bank_imported = False
    for node in tree.body:
        if isinstance(node, ast.ImportFrom):
            names = {alias.name for alias in node.names}
            if "bbia_hf_response_bank" in names:
                bank_imported = True
    assert bank_imported, "bbia_huggingface doit importer bbia_hf_response_bank"
    assert "SAFE_FALLBACK: str =" in src
    assert "SUFFIX_POOL: list[str] =" in src
