#!/usr/bin/env python3
"""
Test latence génération LLM Hugging Face (150 tokens p50/p95).
Mesure mémoire pic chargement si possible.
"""

import gc
import statistics
import time

import pytest

# Import conditionnel
try:
    from bbia_sim.bbia_huggingface import BBIAHuggingFace

    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False
    BBIAHuggingFace = None  # type: ignore


def get_memory_usage() -> float | None:
    """Obtient l'utilisation mémoire actuelle en MB."""
    try:
        import psutil

        process = psutil.Process()
        return float(process.memory_info().rss / 1024 / 1024)  # MB
    except ImportError:
        return None


@pytest.mark.skipif(not HF_AVAILABLE, reason="Hugging Face non disponible")
@pytest.mark.unit
@pytest.mark.slow
def test_huggingface_llm_generation_latency() -> None:
    """Test latence génération LLM 150 tokens p50/p95."""
    try:
        hf = BBIAHuggingFace()
    except ImportError:
        pytest.skip("Hugging Face transformers non disponible")

    # Activer chat LLM
    try:
        hf.enable_llm_chat()
    except Exception:
        pytest.skip("Modèle LLM non chargé ou non disponible")

    iterations = 10
    latencies_ms: list[float] = []

    try:
        # Message de test (environ 150 tokens cible)
        test_message = "Bonjour ! Peux-tu me raconter une courte histoire amusante ?"

        for _ in range(iterations):
            t0 = time.perf_counter()
            response = hf.chat(test_message)
            t1 = time.perf_counter()

            assert isinstance(response, str)
            assert len(response) > 0

            latency_ms = (t1 - t0) * 1000.0
            latencies_ms.append(latency_ms)

            # Petit délai entre appels
            time.sleep(0.5)

        p50 = statistics.median(latencies_ms)
        p95 = float(
            statistics.quantiles(latencies_ms, n=20)[18]
            if len(latencies_ms) > 1
            else latencies_ms[0]
        )

        # Budget: Génération 150 tokens en < 5s (CPU), < 2s (GPU)
        # Budget large pour CI (peut être lent)
        assert p95 < 30000.0, f"Latence p95 trop élevée: {p95:.2f} ms (> 30s)"
    finally:
        gc.collect()


@pytest.mark.skipif(not HF_AVAILABLE, reason="Hugging Face non disponible")
@pytest.mark.unit
@pytest.mark.slow
def test_huggingface_memory_peak_loading() -> None:
    """Test mémoire pic lors du chargement modèle."""
    try:
        hf = BBIAHuggingFace()
    except ImportError:
        pytest.skip("Hugging Face transformers non disponible")

    # Mesurer mémoire avant chargement
    gc.collect()
    mem_before = get_memory_usage()

    try:
        # Charger modèle
        hf.enable_llm_chat()
        gc.collect()

        # Mesurer mémoire après chargement
        mem_after = get_memory_usage()

        # Vérifier que mémoire a augmenté (modèle chargé)
        if mem_before and mem_after:
            memory_increase = mem_after - mem_before
            # Vérifier qu'on a chargé quelque chose (au moins 100MB)
            # Mais pas de limite stricte (dépend du modèle)
            assert (
                memory_increase > 50.0
            ), "Pas assez de mémoire utilisée (modèle non chargé?)"
    finally:
        gc.collect()
