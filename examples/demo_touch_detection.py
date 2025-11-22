#!/usr/bin/env python3
"""Démonstration détection tactile BBIA-SIM.

Issue #251: Add proper support for touch detection.

Ce script démontre la détection d'interactions tactiles (tap, caress, pat)
via l'analyse audio des microphones du robot.

Usage:
    python examples/demo_touch_detection.py
"""

import logging
import time

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

try:
    from src.bbia_sim import bbia_touch

    TOUCH_AVAILABLE = True
except ImportError:
    TOUCH_AVAILABLE = False
    logger.error("❌ Module bbia_touch non disponible")


def main() -> None:
    """Fonction principale."""
    if not TOUCH_AVAILABLE:
        logger.error("❌ Détection tactile non disponible")
        return

    logger.info("👆 Démonstration Détection Tactile BBIA-SIM")
    logger.info("=" * 60)

    # Créer détecteur tactile
    detector = bbia_touch.BBIATouchDetection()

    if not detector.is_enabled():
        logger.warning("⚠️  Détection tactile désactivée")
        logger.info("💡 Activez avec: export BBIA_DISABLE_AUDIO=0")
        return

    logger.info("✅ Détection tactile activée")
    logger.info("💡 Tapez ou caressez le robot pour tester...")
    logger.info("⏹️  Appuyez sur Ctrl+C pour arrêter\n")

    interaction_count = 0
    tap_count = 0
    caress_count = 0
    pat_count = 0

    try:
        while True:
            # Détecter interaction
            touch = detector.detect_touch(duration=0.2)

            if touch["type"] != bbia_touch.TouchType.NONE:
                interaction_count += 1
                touch_type = touch["type"]
                confidence = touch["confidence"]

                # Compter par type
                if touch_type == bbia_touch.TouchType.TAP:
                    tap_count += 1
                    emoji = "👆"
                elif touch_type == bbia_touch.TouchType.CARESS:
                    caress_count += 1
                    emoji = "🤲"
                elif touch_type == bbia_touch.TouchType.PAT:
                    pat_count += 1
                    emoji = "✋"
                else:
                    emoji = "👋"

                logger.info(
                    f"{emoji} Interaction #{interaction_count}: {touch_type.value} "
                    f"(confiance: {confidence:.2f}, "
                    f"amplitude: {touch.get('max_amplitude', 0):.3f})"
                )

            time.sleep(0.1)  # Éviter CPU 100%

    except KeyboardInterrupt:
        logger.info("\n" + "=" * 60)
        logger.info("📊 Statistiques:")
        logger.info(f"   Total interactions: {interaction_count}")
        logger.info(f"   Taps: {tap_count}")
        logger.info(f"   Caresses: {caress_count}")
        logger.info(f"   Pats: {pat_count}")
        logger.info("✅ Démonstration terminée")


if __name__ == "__main__":
    main()
