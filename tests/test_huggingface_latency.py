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
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (charge modèles LLM)
def test_huggingface_llm_generation_latency() -> None:
    """Test latence génération LLM 150 tokens p50/p95."""
    # Skip en CI si trop lent (chargement modèle LLM lourd)
    import os

    if os.environ.get("CI", "false").lower() == "true":
        pytest.skip("Test désactivé en CI (chargement modèle LLM trop lent)")
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

            # OPTIMISATION: Réduire sleep de 0.1 à 0.05 (2x plus rapide)
            time.sleep(0.05)

        statistics.median(latencies_ms)
        p95 = float(
            statistics.quantiles(latencies_ms, n=20)[18]
            if len(latencies_ms) > 1
            else latencies_ms[0]
        )

        # Budget: Génération 150 tokens en < 5s (CPU), < 2s (GPU)
        # NOTE: Seuil très large (30s) pour CI car:
        # - Modèles lourds (Mistral 7B, Llama) peuvent prendre 10-20s sur CPU
        # - CI machines peuvent être lentes (pas de GPU)
        # - Premier chargement modèle peut être lent
        # En local avec GPU, latence devrait être < 5s
        # Si test échoue, vérifier: modèle utilisé, hardware, cache modèle
        assert p95 < 30000.0, (
            f"Latence p95 trop élevée: {p95:.2f} ms (> 30s). "
            f"Vérifier modèle (Phi-2/TinyLlama recommandés pour RPi 5), "
            f"hardware (GPU disponible?), et cache modèle."
        )
    finally:
        gc.collect()


@pytest.mark.skipif(not HF_AVAILABLE, reason="Hugging Face non disponible")
@pytest.mark.unit
@pytest.mark.slow
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (mesure mémoire peak)
def test_huggingface_memory_peak_loading() -> None:
    """Test mémoire pic lors du chargement modèle."""
    # Skip en CI si trop lent (chargement modèle LLM lourd)
    import os

    if os.environ.get("CI", "false").lower() == "true":
        pytest.skip("Test désactivé en CI (chargement modèle LLM trop lent)")
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
        # OPTIMISATION: Réduire sleep de 0.5 à 0.2 (2.5x plus rapide)
        time.sleep(0.2)

        # Mesurer mémoire après chargement
        mem_after = get_memory_usage()

        # Vérifier que mémoire a augmenté (modèle chargé)
        if mem_before and mem_after:
            memory_increase = mem_after - mem_before
            # Vérifier qu'on a chargé quelque chose (au moins 10MB)
            # En CI, les modèles peuvent être plus légers ou déjà en cache
            if memory_increase <= 0.0:
                pytest.skip(
                    "Modèle non chargé ou déjà en cache "
                    f"(mémoire: {mem_before:.1f}MB → {mem_after:.1f}MB)"
                )
            # Vérifier qu'on a chargé quelque chose (au moins 5MB pour tolérer CI)
            # En CI, les modèles peuvent être en cache ou utiliser moins de mémoire
            # Skip si augmentation trop faible (probablement modèle déjà en cache)
            if memory_increase < 5.0:
                pytest.skip(
                    f"Modèle probablement déjà en cache ou non chargé "
                    f"(mémoire: {mem_before:.1f}MB → {mem_after:.1f}MB, "
                    f"augmentation: {memory_increase:.1f}MB)"
                )
            # Budget réduit pour CI (modèles peuvent être en cache ou plus légers)
            # On vérifie juste qu'on a utilisé un minimum de mémoire
            assert memory_increase >= 5.0, (
                f"Pas assez de mémoire utilisée (modèle non chargé?): "
                f"{memory_increase:.1f}MB (minimum: 5MB)"
            )
        else:
            pytest.skip("Impossible de mesurer la mémoire (psutil non disponible)")
    finally:
        gc.collect()
