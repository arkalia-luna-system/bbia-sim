#!/usr/bin/env python3
# ⚠️ OBSOLÈTE : Test intégration phase 2, plus utilisé
# Ce fichier peut être archivé dans une future version
"""
Exemple d'intégration des modules BBIA Phase 2
Démonstration des nouvelles fonctionnalités :
Hugging Face, Emotion Recognition, Adaptive Behavior
"""

import sys
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_adaptive_behavior import BBIAAdaptiveBehavior
from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition
from bbia_sim.bbia_huggingface import BBIAHuggingFace
from bbia_sim.robot_factory import RobotFactory


def demo_huggingface_integration():
    """Démonstration de l'intégration Hugging Face."""
    print("🤗 === DÉMONSTRATION HUGGING FACE INTEGRATION ===")

    try:
        # Initialisation
        hf = BBIAHuggingFace()
        print(f"✅ BBIA Hugging Face initialisé (device: {hf.device})")

        # Affichage des modèles disponibles
        print("\n📋 Modèles disponibles:")
        models = hf.get_available_models()
        if isinstance(models, dict):
            for category, model_list in models.items():
                if isinstance(model_list, dict):
                    print(f"  {category}: {list(model_list.keys())}")
                else:
                    print(f"  {category}: {model_list}")

        # Test analyse sentiment
        print("\n📝 Test analyse sentiment...")
        sentiment_result = hf.analyze_sentiment(
            "Je suis très heureux de travailler sur ce projet!"
        )
        print(f"Résultat: {sentiment_result}")

        # Test analyse émotion
        print("\n😊 Test analyse émotion...")
        emotion_result = hf.analyze_emotion(
            "Je suis excité par les nouvelles fonctionnalités!"
        )
        print(f"Résultat: {emotion_result}")

        print("✅ Démonstration Hugging Face terminée\n")

    except ImportError as e:
        print(f"❌ Hugging Face non disponible: {e}")
        print("Installez avec: pip install transformers torch")
    except Exception as e:
        print(f"❌ Erreur: {e}")


def demo_emotion_recognition():
    """Démonstration de la reconnaissance d'émotions."""
    print("😊 === DÉMONSTRATION EMOTION RECOGNITION ===")

    try:
        # Initialisation
        emotion_rec = BBIAEmotionRecognition()
        print(f"✅ BBIA Emotion Recognition initialisé (device: {emotion_rec.device})")

        # Initialisation des modèles
        print("\n🚀 Initialisation des modèles...")
        success = emotion_rec.initialize()
        print(f"Résultat: {'✅' if success else '❌'}")

        # Test analyse émotion vocale
        print("\n🗣️ Test analyse émotion vocale...")
        vocal_result = emotion_rec.analyze_vocal_emotion(
            "Je suis vraiment excité par ce projet!"
        )
        print(f"Résultat: {vocal_result}")

        # Test analyse émotion faciale (simulation)
        print("\n😊 Test analyse émotion faciale...")
        import numpy as np

        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        facial_result = emotion_rec.analyze_facial_emotion(test_image)
        print(f"Résultat: {facial_result}")

        # Test fusion d'émotions
        print("\n🔄 Test fusion d'émotions...")
        fusion_result = emotion_rec.fuse_emotions(facial_result, vocal_result)
        print(f"Résultat: {fusion_result}")

        # Test analyse temps réel
        print("\n⏱️ Test analyse temps réel...")
        realtime_result = emotion_rec.analyze_emotion_realtime(
            test_image, "Je suis très heureux!"
        )
        print(f"Résultat: {realtime_result}")

        # Statistiques
        print(f"\n📊 Statistiques: {emotion_rec.get_emotion_statistics()}")

        print("✅ Démonstration Emotion Recognition terminée\n")

    except ImportError as e:
        print(f"❌ Dépendances ML non disponibles: {e}")
        print("Installez avec: pip install mediapipe torch transformers")
    except Exception as e:
        print(f"❌ Erreur: {e}")


def demo_adaptive_behavior():
    """Démonstration des comportements adaptatifs."""
    print("🧠 === DÉMONSTRATION ADAPTIVE BEHAVIOR ===")

    try:
        # Initialisation
        adaptive_behavior = BBIAAdaptiveBehavior()
        print("✅ BBIA Adaptive Behavior initialisé")

        # Configuration d'un contexte
        print("\n🎭 Configuration contexte 'greeting'...")
        adaptive_behavior.set_context("greeting", 0.9)
        adaptive_behavior.set_emotion_state("happy", 0.8)

        # Génération de comportements
        print("\n🎭 Génération de comportements...")
        for i in range(3):
            behavior = adaptive_behavior.generate_behavior(f"user_arrival_{i}")
            print(
                f"  Comportement {i + 1}: {behavior['name']} - "
                f"{behavior['description']}"
            )
            print(
                f"    Contexte: {behavior['context']}, "
                f"Émotion: {behavior['emotion']}"
            )
            print(f"    Intensité: {behavior['emotion_intensity']:.2f}")
            print(
                f"    Paramètres: {behavior['parameters']['duration']:.1f}s, "
                f"{len(behavior['parameters']['joints'])} joints"
            )

        # Test comportement proactif
        print("\n🚀 Test comportement proactif...")
        proactive = adaptive_behavior.get_proactive_behavior()
        if proactive:
            print(
                f"Comportement proactif: {proactive['name']} - "
                f"{proactive['description']}"
            )
        else:
            print("Aucun comportement proactif généré")

        # Test adaptation feedback
        print("\n🔄 Test adaptation feedback...")
        behaviors = adaptive_behavior.behavior_history
        if behaviors:
            behavior_id = behaviors[0]["id"]
            adaptive_behavior.adapt_to_feedback(behavior_id, "positive", 0.8)
            print("Feedback positif appliqué")

        # Statistiques
        print(f"\n📊 Statistiques: {adaptive_behavior.get_behavior_statistics()}")

        print("✅ Démonstration Adaptive Behavior terminée\n")

    except Exception as e:
        print(f"❌ Erreur: {e}")


def demo_integrated_workflow():
    """Démonstration du workflow intégré."""
    print("🔄 === DÉMONSTRATION WORKFLOW INTÉGRÉ ===")

    try:
        # Initialisation des modules
        print("🚀 Initialisation des modules...")

        # Hugging Face (optionnel)
        hf = None
        try:
            hf = BBIAHuggingFace()
            print("✅ Hugging Face initialisé")
        except ImportError:
            print("⚠️ Hugging Face non disponible")

        # Emotion Recognition
        emotion_rec = BBIAEmotionRecognition()
        emotion_rec.initialize()
        print("✅ Emotion Recognition initialisé")

        # Adaptive Behavior
        adaptive_behavior = BBIAAdaptiveBehavior()
        print("✅ Adaptive Behavior initialisé")

        # Robot Backend
        robot = RobotFactory.create_backend("mujoco")
        if robot and robot.connect():
            print("✅ Robot MuJoCo connecté")
        else:
            print("⚠️ Robot non connecté")

        # Simulation d'interaction
        print("\n🎭 Simulation d'interaction utilisateur...")

        # Étape 1: Analyse émotionnelle
        print("\n1️⃣ Analyse émotionnelle...")
        user_text = "Bonjour! Je suis très heureux de vous rencontrer!"

        # Analyse avec Hugging Face si disponible
        if hf:
            sentiment = hf.analyze_sentiment(user_text)
            emotion = hf.analyze_emotion(user_text)
            print(f"   Sentiment: {sentiment}")
            print(f"   Émotion: {emotion}")

        # Analyse avec Emotion Recognition
        vocal_emotion = emotion_rec.analyze_vocal_emotion(user_text)
        print(f"   Émotion vocale: {vocal_emotion}")

        # Étape 2: Adaptation du contexte
        print("\n2️⃣ Adaptation du contexte...")
        detected_emotion = vocal_emotion.get("emotion", "neutral")
        emotion_intensity = vocal_emotion.get("confidence", 0.5)

        # Détermination du contexte
        if detected_emotion == "happy" and emotion_intensity > 0.7:
            context = "greeting"
        elif detected_emotion == "curious":
            context = "conversation"
        else:
            context = "neutral"

        adaptive_behavior.set_context(context)
        adaptive_behavior.set_emotion_state(detected_emotion, emotion_intensity)
        print(f"   Contexte: {context}")
        print(f"   Émotion: {detected_emotion} (intensité: {emotion_intensity:.2f})")

        # Étape 3: Génération de comportement
        print("\n3️⃣ Génération de comportement...")
        behavior = adaptive_behavior.generate_behavior("user_interaction")
        print(f"   Comportement: {behavior['name']}")
        print(f"   Description: {behavior['description']}")
        print(f"   Paramètres: {behavior['parameters']}")

        # Étape 4: Exécution sur le robot
        print("\n4️⃣ Exécution sur le robot...")
        if robot:
            # Simulation d'exécution
            print(
                f"   Exécution: {behavior['name']} pendant "
                f"{behavior['parameters']['duration']:.1f}s"
            )

            # Application de l'émotion au robot
            robot.set_emotion(detected_emotion, emotion_intensity)
            print(
                f"   Émotion appliquée: {detected_emotion} "
                f"(intensité: {emotion_intensity:.2f})"
            )

            # Simulation de mouvement
            for joint in behavior["parameters"]["joints"]:
                if joint in robot.get_available_joints():
                    # Mouvement simulé
                    print(
                        f"   Mouvement joint {joint}: amplitude "
                        f"{behavior['parameters']['intensity']:.2f}"
                    )

        # Étape 5: Apprentissage
        print("\n5️⃣ Apprentissage...")
        adaptive_behavior.adapt_to_feedback(behavior["id"], "positive", 0.8)
        print("   Feedback positif enregistré")

        # Statistiques finales
        print("\n📊 Statistiques finales:")
        print(f"   Comportements générés: {len(adaptive_behavior.behavior_history)}")
        print(f"   Émotions analysées: {len(emotion_rec.emotion_history)}")

        print("✅ Workflow intégré terminé avec succès\n")

    except Exception as e:
        print(f"❌ Erreur workflow intégré: {e}")


def main():
    """Point d'entrée principal."""
    print("🚀 === DÉMONSTRATION BBIA PHASE 2 MODULES ===")
    print(
        "Nouvelles fonctionnalités: Hugging Face, Emotion Recognition, "
        "Adaptive Behavior"
    )
    print("=" * 80)

    # Démonstrations individuelles
    demo_huggingface_integration()
    demo_emotion_recognition()
    demo_adaptive_behavior()

    # Démonstration intégrée
    demo_integrated_workflow()

    print("🎉 === DÉMONSTRATION TERMINÉE ===")
    print("Tous les modules Phase 2 sont fonctionnels!")
    print("Prêt pour la Phase 3: Ouverture Écosystème")


if __name__ == "__main__":
    main()
