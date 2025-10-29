#!/usr/bin/env python3
"""Tests rapides de sélection/fallback des backends IA (TTS/STT/LLM)."""

import pytest

from bbia_sim.ai_backends import (
    EchoLLM,
    KittenTTSTTS,
    Pyttsx3TTS,
    get_llm_backend,
    get_stt_backend,
    get_tts_backend,
    select_backends,
)


@pytest.mark.unit
@pytest.mark.fast
def test_tts_selection_env_kitten(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("BBIA_TTS_BACKEND", "kitten")
    tts = get_tts_backend()
    assert isinstance(tts, KittenTTSTTS)


@pytest.mark.unit
@pytest.mark.fast
def test_tts_selection_fallback(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("BBIA_TTS_BACKEND", "pyttsx3")
    tts = get_tts_backend()
    assert isinstance(tts, Pyttsx3TTS)


@pytest.mark.unit
@pytest.mark.fast
def test_llm_selection_default() -> None:
    llm = get_llm_backend()
    assert isinstance(llm, EchoLLM)
    out = llm.generate("Bonjour", max_tokens=5)
    assert len(out) <= 5


@pytest.mark.unit
@pytest.mark.fast
def test_stt_selection_default() -> None:
    stt = get_stt_backend()
    assert stt.transcribe_wav("dummy.wav") == ""


@pytest.mark.unit
@pytest.mark.fast
@pytest.mark.parametrize(
    "env_name,expected_cls",
    [
        ("kitten", KittenTTSTTS),
        ("pyttsx3", Pyttsx3TTS),
        ("unknown", KittenTTSTTS),  # fallback
    ],
)
def test_tts_selection_matrix(
    monkeypatch: pytest.MonkeyPatch, env_name: str, expected_cls: type
) -> None:
    monkeypatch.setenv("BBIA_TTS_BACKEND", env_name)
    tts = get_tts_backend()
    assert isinstance(tts, expected_cls)


@pytest.mark.unit
@pytest.mark.fast
def test_tts_selection_kokoro_neutts(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("BBIA_TTS_BACKEND", "kokoro")
    tts1 = get_tts_backend()
    assert hasattr(tts1, "synthesize_to_wav")
    monkeypatch.setenv("BBIA_TTS_BACKEND", "neutts")
    tts2 = get_tts_backend()
    assert hasattr(tts2, "synthesize_to_wav")


@pytest.mark.unit
@pytest.mark.fast
def test_select_backends_returns_all(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("BBIA_TTS_BACKEND", "kitten")
    monkeypatch.setenv("BBIA_STT_BACKEND", "whisper")
    monkeypatch.setenv("BBIA_LLM_BACKEND", "llama.cpp")
    backends = select_backends()
    assert set(backends.keys()) == {"tts", "stt", "llm"}
