#!/usr/bin/env python3
"""Démo BBIAEmotionRecognition - Reconnaissance émotions humaines.

Démonstration du module de reconnaissance d'émotions faciales et vocales.
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo BBIAEmotionRecognition")
    parser.add_argument(
        "--mode",
        choices=["facial", "vocal", "multimodal"],
        default="facial",
        help="Mode de reconnaissance",
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--device",
        default="auto",
        choices=["auto", "cpu", "cuda"],
        help="Device pour modèles ML",
    )

    args = parser.parse_args()

    print("😊 Démo BBIAEmotionRecognition - Reconnaissance émotions")
    print(f"   • Mode : {args.mode}")
    print(f"   • Device : {args.device}")

    try:
        # Créer module
        emotion_recognition = BBIAEmotionRecognition(device=args.device)
        print("✅ BBIAEmotionRecognition créé")

        # 1. Reconnaissance faciale
        if args.mode == "facial":
            print("\n📸 Analyse émotion faciale...")
            # Simuler une image (dans version réelle, utiliser webcam)
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            result = emotion_recognition.analyze_facial_emotion(image)
            print(f"   Émotion détectée : {result.get('emotion', 'N/A')}")
            print(f"   Confiance : {result.get('confidence', 0.0):.2f}")

        # 2. Reconnaissance vocale
        elif args.mode == "vocal":
            print("\n🎤 Analyse émotion vocale...")
            # Simuler un texte (dans version réelle, utiliser audio)
            text = "Je suis très heureux aujourd'hui !"
            result = emotion_recognition.analyze_vocal_emotion(text)
            print(f"   Émotion détectée : {result.get('emotion', 'N/A')}")
            print(f"   Confiance : {result.get('confidence', 0.0):.2f}")

        # 3. Multimodal
        elif args.mode == "multimodal":
            print("\n🎭 Analyse multimodale (faciale + vocale)...")
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            text = "Je suis surpris !"
            result = emotion_recognition.analyze_multimodal_emotion(image, text)
            print(f"   Émotion détectée : {result.get('emotion', 'N/A')}")
            print(f"   Confiance : {result.get('confidence', 0.0):.2f}")

        print("\n✅ Démo terminée avec succès")
        return 0

    except ImportError as e:
        print(f"❌ Dépendances manquantes : {e}")
        print("💡 Installez avec: pip install mediapipe torch transformers")
        return 1
    except Exception as e:
        print(f"❌ Erreur : {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
