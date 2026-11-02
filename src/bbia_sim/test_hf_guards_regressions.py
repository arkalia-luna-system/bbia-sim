"""Non-régression découpage HF : ``hf_guards`` + banque + chat tools."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from bbia_sim import bbia_hf_chat_tools as chat_tools
from bbia_sim import bbia_hf_response_bank as bank
from bbia_sim import bbia_hf_tool_orchestration as orch
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
    chat_tools_imported = False
    orch_imported = False
    for node in tree.body:
        if isinstance(node, ast.ImportFrom):
            names = {alias.name for alias in node.names}
            if "bbia_hf_response_bank" in names:
                bank_imported = True
            if "bbia_hf_chat_tools" in names:
                chat_tools_imported = True
            if "bbia_hf_tool_orchestration" in names:
                orch_imported = True
    assert bank_imported, "bbia_huggingface doit importer bbia_hf_response_bank"
    assert chat_tools_imported, "bbia_huggingface doit importer bbia_hf_chat_tools"
    assert orch_imported, "bbia_huggingface doit importer bbia_hf_tool_orchestration"
    assert "SAFE_FALLBACK: str =" in src
    assert "SUFFIX_POOL: list[str] =" in src
    assert "TOOL_KEYWORD_PATTERNS =" in src
    assert "extract_angle =" in src


def test_chat_tools_helpers_stable_and_reexported() -> None:
    assert "move_head" in chat_tools.TOOL_KEYWORD_PATTERNS
    assert "play_emotion" in chat_tools.TOOL_NLP_DESCRIPTIONS
    assert len(chat_tools.TOOL_NLP_DESCRIPTIONS) >= 4
    assert chat_tools.extract_angle("tourne de 30 degrés") == 30.0
    assert chat_tools.extract_intensity("légèrement à gauche") == 0.2
    assert chat_tools.extract_angle("pi/4 radians") is not None
    assert (
        chat_tools.format_tool_result_text(
            {"status": "success", "detail": "ok"}, "dance"
        )
        == "✅ ok"
    )

    # Réexports sans charger torch : AST + présence des aliases
    hf_path = Path(__file__).resolve().parent / "bbia_huggingface.py"
    src = hf_path.read_text(encoding="utf-8")
    assert "TOOL_KEYWORD_PATTERNS = _hf_chat_tools.TOOL_KEYWORD_PATTERNS" in src
    assert "extract_angle = _hf_chat_tools.extract_angle" in src
    assert "extract_intensity = _hf_chat_tools.extract_intensity" in src
    assert "cast(float | None, extract_angle(message))" in src
    assert "cast(float | None, extract_intensity(message))" in src
    assert "detect_and_execute_tools = _hf_orch.detect_and_execute_tools" in src
    assert "_hf_orch.detect_and_execute_tools(" in src


def test_tool_orchestration_keyword_dance() -> None:
    class _State:
        _sentence_model: object | None = None
        _use_nlp_detection = False

    calls: list[tuple[str, dict[str, object]]] = []

    def _exec(name: str, params: dict[str, object]) -> dict[str, object]:
        calls.append((name, params))
        return {"status": "success", "detail": f"{name} ok"}

    text = orch.detect_and_execute_tools(
        "fais danser le robot",
        state=_State(),
        execute_tool=_exec,
        tools_available=True,
    )
    assert text == "✅ dance ok"
    assert calls and calls[0][0] == "dance"
    assert calls[0][1].get("move_name") == "happy_dance"


def test_tool_orchestration_move_head_params() -> None:
    params = orch.build_tool_params(
        "move_head",
        "tourne la tête à gauche de 30 degrés",
        "tourne la tête à gauche de 30 degrés",
        use_ner_extraction=True,
    )
    assert params["direction"] == "left"
    assert params["intensity"] == pytest.approx(30.0 / 90.0)
    kw = orch.build_tool_params(
        "move_head",
        "tourne la tête à gauche de 30 degrés",
        "tourne la tête à gauche de 30 degrés",
        use_ner_extraction=False,
    )
    assert kw["intensity"] == 0.5


def test_tool_orchestration_no_tools() -> None:
    class _State:
        _sentence_model: object | None = None
        _use_nlp_detection = False

    assert (
        orch.detect_and_execute_tools(
            "danse",
            state=_State(),
            execute_tool=lambda *_a, **_k: None,
            tools_available=False,
        )
        is None
    )
