#!/usr/bin/env python3
"""Démo BBIAVoiceAdvanced - Synthèse vocale avancée.

Démonstration du module de synthèse vocale avancée avec contrôle pitch/émotion.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.bbia_voice_advanced import BBIAVoiceAdvanced


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo BBIAVoiceAdvanced")
    parser.add_argument(
        "--text",
        default="Bonjour, je suis BBIA, votre robot compagnon.",
        help="Texte à synthétiser",
    )
    parser.add_argument(
        "--emotion",
        default="happy",
        choices=["happy", "sad", "excited", "calm", "neutral"],
        help="Émotion à appliquer",
    )
    parser.add_argument("--pitch", type=float, help="Pitch personnalisé (-1.0 à 1.0)")
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("🎤 Démo BBIAVoiceAdvanced - Synthèse vocale avancée")
    print(f"   • Texte : {args.text[:50]}...")
    print(f"   • Émotion : {args.emotion}")
    if args.pitch is not None:
        print(f"   • Pitch : {args.pitch}")
    print(f"   • Backend : {args.backend}")

    try:
        # Créer backend
        if args.backend == "mujoco":
            backend = MuJoCoBackend()
        else:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend = ReachyMiniBackend()

        backend.connect()
        print("✅ Backend connecté")

        # Créer module voix avancée
        voice_advanced = BBIAVoiceAdvanced(robot_api=backend)
        print("✅ BBIAVoiceAdvanced créé")

        # Synthétiser texte
        print(f"\n🔊 Synthèse vocale avec émotion '{args.emotion}'...")
        if args.pitch is not None:
            voice_advanced.say(args.text, emotion=args.emotion, pitch=args.pitch)
        else:
            voice_advanced.say(args.text, emotion=args.emotion)
        print("✅ Synthèse vocale terminée")

        print("\n✅ Démo terminée avec succès")
        return 0

    except ImportError as e:
        print(f"❌ Dépendances manquantes : {e}")
        print("💡 Coqui TTS optionnel, fallback pyttsx3 activé")
        return 1
    except Exception as e:
        print(f"❌ Erreur : {e}")
        import traceback

        traceback.print_exc()
        return 1
    finally:
        backend.disconnect()


if __name__ == "__main__":
    sys.exit(main())
