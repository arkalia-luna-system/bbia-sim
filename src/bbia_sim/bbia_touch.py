#!/usr/bin/env python3
"""Module de détection tactile pour BBIA-SIM.

Issue #251: Add proper support for touch detection.

Ce module détecte les interactions tactiles (tap, caress) via l'analyse audio
des microphones du robot. Utilise l'analyse de fréquences et patterns audio
pour identifier les contacts physiques.

Usage:
    from src.bbia_sim import bbia_touch

    touch_detector = bbia_touch.BBIATouchDetection()
    if touch_detector.detect_tap():
        logger.info("Tap détecté!")
"""

import logging
import os
from collections import deque
from typing import Any

import numpy as np

from bbia_sim.utils.enum_compat import StrEnum

logger = logging.getLogger(__name__)

# Vérifier si sounddevice est disponible
try:
    import sounddevice as sd

    SOUNDDEVICE_AVAILABLE = True
except ImportError:
    SOUNDDEVICE_AVAILABLE = False
    logger.warning("sounddevice non disponible (détection tactile désactivée)")


class TouchType(StrEnum):
    """Types d'interactions tactiles détectées."""

    TAP = "tap"  # Tap rapide
    CARESS = "caress"  # Caresse lente
    PAT = "pat"  # Tape ferme
    NONE = "none"  # Aucune interaction


class BBIATouchDetection:
    """Détecteur d'interactions tactiles via analyse audio.

    Analyse les patterns audio des microphones pour détecter:
    - Tap: Impact rapide et court
    - Caress: Mouvement lent et continu
    - Pat: Impact ferme et prolongé

    Issue #251: Implémentation détection tactile acoustique.
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        buffer_duration: float = 0.5,
        tap_threshold: float = 0.3,
        caress_threshold: float = 0.15,
    ) -> None:
        """Initialise le détecteur tactile.

        Args:
            sample_rate: Fréquence d'échantillonnage audio (Hz)
            buffer_duration: Durée du buffer d'analyse (secondes)
            tap_threshold: Seuil pour détecter un tap (amplitude normalisée)
            caress_threshold: Seuil pour détecter une caresse (amplitude normalisée)

        """
        # Initialiser attributs même si désactivé (pour tests)
        self.sample_rate = sample_rate
        self.buffer_duration = buffer_duration
        self.buffer_size = int(sample_rate * buffer_duration)
        self.tap_threshold = tap_threshold
        self.caress_threshold = caress_threshold
        self.audio_buffer: deque[float] = deque(maxlen=self.buffer_size)

        if not SOUNDDEVICE_AVAILABLE:
            logger.warning(
                "⚠️  Détection tactile désactivée (sounddevice non disponible)",
            )
            self.enabled = False
            return

        # Vérifier flag d'environnement
        if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
            logger.debug("Détection tactile désactivée (BBIA_DISABLE_AUDIO=1)")
            self.enabled = False
            return

        self.enabled = True

        logger.info(
            "✅ BBIATouchDetection initialisé "
            f"(sample_rate={sample_rate}, buffer={buffer_duration}s)",
        )

    def _record_audio_chunk(self, duration: float = 0.1) -> np.ndarray | None:
        """Enregistre un chunk audio pour analyse.

        Args:
            duration: Durée d'enregistrement (secondes)

        Returns:
            Array audio ou None si erreur

        """
        if not self.enabled or not SOUNDDEVICE_AVAILABLE:
            return None

        try:
            samples = int(self.sample_rate * duration)
            audio = sd.rec(
                samples,
                samplerate=self.sample_rate,
                channels=1,
                dtype=np.float32,
            )
            sd.wait()
            result = audio.flatten()
            # Type narrowing pour mypy - flatten() retourne toujours ndarray
            result_array: np.ndarray[np.float32, Any] = np.asarray(
                result,
                dtype=np.float32,
            )
            return result_array
        except Exception as e:
            logger.debug("Erreur enregistrement audio pour détection tactile: %s", e)
            return None

    def _analyze_audio_pattern(self, audio: np.ndarray) -> dict[str, Any]:
        """Analyse un pattern audio pour détecter interactions tactiles.

        Args:
            audio: Array audio à analyser

        Returns:
            Dictionnaire avec type d'interaction et métriques

        """
        # Correction: utiliser size pour arrays numpy (évite erreur "truth value ambiguous")
        if audio.size == 0:
            return {"type": TouchType.NONE, "confidence": 0.0, "max_amplitude": 0.0}

        # Normaliser l'audio
        audio_normalized = audio / (np.max(np.abs(audio)) + 1e-10)

        # Calculer métriques
        max_amplitude = float(np.max(np.abs(audio_normalized)))
        mean_amplitude = float(np.mean(np.abs(audio_normalized)))
        std_amplitude = float(np.std(audio_normalized))

        # Analyser fréquences (FFT)
        fft = np.fft.fft(audio_normalized)
        freqs = np.fft.fftfreq(len(audio_normalized), 1.0 / self.sample_rate)
        power = np.abs(fft)

        # Fréquences caractéristiques des interactions tactiles
        # Tap: fréquences hautes (impact rapide)
        # Caress: fréquences basses (mouvement lent)
        low_freq_power = float(np.sum(power[freqs < 500]))
        high_freq_power = float(np.sum(power[freqs > 2000]))

        # Détecter type d'interaction
        touch_type = TouchType.NONE
        confidence = 0.0

        # Tap: amplitude élevée, fréquences hautes dominantes
        if max_amplitude > self.tap_threshold and high_freq_power > low_freq_power:
            touch_type = TouchType.TAP
            confidence = min(max_amplitude * 2.0, 1.0)

        # Caress: amplitude modérée, fréquences basses dominantes
        elif (
            mean_amplitude > self.caress_threshold
            and low_freq_power > high_freq_power * 2
        ):
            touch_type = TouchType.CARESS
            confidence = min(mean_amplitude * 3.0, 1.0)

        # Pat: amplitude très élevée, fréquences mixtes
        elif max_amplitude > self.tap_threshold * 1.5:
            touch_type = TouchType.PAT
            confidence = min(max_amplitude * 1.5, 1.0)

        return {
            "type": touch_type,
            "confidence": confidence,
            "max_amplitude": max_amplitude,
            "mean_amplitude": mean_amplitude,
            "std_amplitude": std_amplitude,
            "low_freq_power": low_freq_power,
            "high_freq_power": high_freq_power,
        }

    def detect_tap(self, duration: float = 0.1) -> bool:
        """Détecte un tap rapide.

        Args:
            duration: Durée d'analyse (secondes)

        Returns:
            True si tap détecté, False sinon

        """
        if not self.enabled:
            return False

        audio = self._record_audio_chunk(duration)
        if audio is None:
            return False

        analysis = self._analyze_audio_pattern(audio)
        is_tap = analysis["type"] == TouchType.TAP and analysis["confidence"] > 0.5
        return bool(is_tap)

    def detect_caress(self, duration: float = 0.3) -> bool:
        """Détecte une caresse lente.

        Args:
            duration: Durée d'analyse (secondes, plus long pour caresse)

        Returns:
            True si caresse détectée, False sinon

        """
        if not self.enabled:
            return False

        audio = self._record_audio_chunk(duration)
        if audio is None:
            return False

        analysis = self._analyze_audio_pattern(audio)
        is_caress = (
            analysis["type"] == TouchType.CARESS and analysis["confidence"] > 0.4
        )
        return bool(is_caress)

    def detect_touch(self, duration: float = 0.2) -> dict[str, Any]:
        """Détecte toute interaction tactile.

        Args:
            duration: Durée d'analyse (secondes)

        Returns:
            Dictionnaire avec type d'interaction et métriques détaillées

        """
        if not self.enabled:
            return {
                "type": TouchType.NONE,
                "confidence": 0.0,
                "enabled": False,
            }

        audio = self._record_audio_chunk(duration)
        if audio is None:
            return {
                "type": TouchType.NONE,
                "confidence": 0.0,
                "enabled": False,
            }

        analysis = self._analyze_audio_pattern(audio)
        analysis["enabled"] = True
        return analysis

    def is_enabled(self) -> bool:
        """Vérifie si la détection tactile est activée.

        Returns:
            True si activée, False sinon

        """
        return self.enabled


def create_touch_detector(
    sample_rate: int = 16000,
    buffer_duration: float = 0.5,
) -> BBIATouchDetection:
    """Factory pour créer un détecteur tactile.

    Args:
        sample_rate: Fréquence d'échantillonnage
        buffer_duration: Durée du buffer

    Returns:
        Instance BBIATouchDetection

    """
    return BBIATouchDetection(
        sample_rate=sample_rate,
        buffer_duration=buffer_duration,
    )


if __name__ == "__main__":
    # Test simple
    logging.basicConfig(level=logging.INFO)
    detector = BBIATouchDetection()

    if detector.is_enabled():
        logger.info("✅ Détection tactile activée")
        logger.info("💡 Tapez sur le robot pour tester...")
        logger.info("⏹️  Appuyez sur Ctrl+C pour arrêter")

        try:
            while True:
                touch = detector.detect_touch()
                if touch["type"] != TouchType.NONE:
                    logger.info(
                        "👆 Interaction détectée: %s (confiance: %.2f)",
                        touch["type"],
                        touch["confidence"],
                    )
        except KeyboardInterrupt:
            logger.info("\n✅ Test terminé")
    else:
        logger.warning("⚠️  Détection tactile désactivée")
