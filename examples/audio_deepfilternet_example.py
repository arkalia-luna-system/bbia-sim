#!/usr/bin/env python3
"""Exemple d'utilisation DeepFilterNet pour réduction bruit moteur.

Issue #135: Add sound processing and sound usage example.

Cet exemple montre comment utiliser DeepFilterNet pour réduire le bruit
des moteurs lors de l'enregistrement audio.

Usage:
    python examples/audio_deepfilternet_example.py
"""

import logging
import os
from pathlib import Path

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Vérifier si DeepFilterNet est disponible
try:
    import deepfilternet as df
    import numpy as np
    import soundfile as sf

    DEEPFILTERNET_AVAILABLE = True
except ImportError:
    DEEPFILTERNET_AVAILABLE = False
    logger.warning(
        "⚠️  DeepFilterNet non disponible. " "Installez avec: pip install deepfilternet"
    )


def reduce_noise_with_deepfilternet(
    input_audio: str | Path, output_audio: str | Path
) -> bool:
    """Réduit le bruit d'un fichier audio avec DeepFilterNet.

    Args:
        input_audio: Chemin fichier audio d'entrée
        output_audio: Chemin fichier audio de sortie (bruit réduit)

    Returns:
        True si succès, False sinon
    """
    if not DEEPFILTERNET_AVAILABLE:
        logger.error("❌ DeepFilterNet non disponible")
        return False

    try:
        input_path = Path(input_audio)
        output_path = Path(output_audio)

        if not input_path.exists():
            logger.error("❌ Fichier audio introuvable: %s", input_path)
            return False

        logger.info("🎤 Chargement audio: %s", input_path)
        audio, sample_rate = sf.read(str(input_path))

        # DeepFilterNet attend un signal mono
        if len(audio.shape) > 1:
            audio = np.mean(audio, axis=1)

        logger.info("🔧 Réduction bruit avec DeepFilterNet...")
        # Appliquer DeepFilterNet
        enhanced_audio = df.enhance(audio, sr=sample_rate)

        # Sauvegarder audio amélioré
        output_path.parent.mkdir(parents=True, exist_ok=True)
        sf.write(str(output_path), enhanced_audio, sample_rate)
        logger.info("✅ Audio amélioré sauvegardé: %s", output_path)

        return True
    except Exception as e:
        logger.exception("❌ Erreur réduction bruit: %s", e)
        return False


def record_and_enhance_audio(output_file: str | Path, duration: float = 3.0) -> bool:
    """Enregistre audio depuis microphone et applique DeepFilterNet.

    Args:
        output_file: Chemin fichier de sortie
        duration: Durée enregistrement en secondes

    Returns:
        True si succès, False sinon
    """
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        logger.warning("⚠️  Audio désactivé (BBIA_DISABLE_AUDIO=1)")
        return False

    try:
        from src.bbia_sim import bbia_audio

        # Enregistrer audio brut
        temp_file = Path("/tmp/bbia_audio_raw.wav")
        logger.info("🎤 Enregistrement audio (%ss)...", duration)
        if not bbia_audio.enregistrer_audio(str(temp_file), duree=int(duration)):
            logger.error("❌ Échec enregistrement audio")
            return False

        # Appliquer DeepFilterNet
        logger.info("🔧 Application DeepFilterNet pour réduction bruit...")
        if not reduce_noise_with_deepfilternet(temp_file, output_file):
            logger.error("❌ Échec réduction bruit")
            return False

        logger.info("✅ Audio enregistré et amélioré: %s", output_file)
        return True
    except Exception as e:
        logger.exception("❌ Erreur enregistrement/amélioration: %s", e)
        return False


def main() -> None:
    """Fonction principale."""
    if not DEEPFILTERNET_AVAILABLE:
        logger.error(
            "❌ DeepFilterNet non disponible. "
            "Installez avec: pip install deepfilternet"
        )
        logger.info("💡 DeepFilterNet réduit le bruit des moteurs dans l'audio")
        return

    logger.info("🎵 Exemple DeepFilterNet - Réduction bruit moteur")
    logger.info("=" * 60)

    # Exemple 1: Améliorer fichier audio existant
    example_audio = Path("assets/voice") / "sample.wav"
    if example_audio.exists():
        output_enhanced = Path("assets/voice") / "sample_enhanced.wav"
        logger.info("\n📁 Exemple 1: Amélioration fichier existant")
        if reduce_noise_with_deepfilternet(example_audio, output_enhanced):
            logger.info("✅ Fichier amélioré: %s", output_enhanced)

    # Exemple 2: Enregistrer et améliorer en temps réel
    logger.info("\n🎤 Exemple 2: Enregistrement + amélioration")
    output_file = Path("assets/voice") / "recorded_enhanced.wav"
    if record_and_enhance_audio(output_file, duration=3.0):
        logger.info("✅ Audio enregistré et amélioré: %s", output_file)

    logger.info("\n" + "=" * 60)
    logger.info("✅ Exemple terminé!")


if __name__ == "__main__":
    main()
