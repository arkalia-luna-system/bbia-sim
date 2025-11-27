#!/usr/bin/env python3
"""Démonstration de toutes les capacités du projet BBIA.

Ce script utilise toutes les fonctionnalités disponibles pour maximiser
l'utilisation des capacités du projet.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# Imports de toutes les capacités
import numpy as np

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.bbia_adaptive_behavior import BBIAAdaptiveBehavior
from bbia_sim.bbia_adaptive_learning import BBIAAdaptiveLearning
from bbia_sim.bbia_behavior import BBIABehaviorManager
from bbia_sim.bbia_chat import BBIAChat
from bbia_sim.bbia_emotions import BBIAEmotions
from bbia_sim.bbia_idle_animations import (
    BBIABreathingAnimation,
    BBIAPoseTransitionManager,
    BBIAVocalTremor,
    BBIIdleAnimationManager,
)
from bbia_sim.bbia_memory import BBIAMemory, append_record
from bbia_sim.bbia_tools import BBIATools
from bbia_sim.bbia_vision import BBIAVision
from bbia_sim.bbia_voice import dire_texte
from bbia_sim.daemon.app.backend_adapter import BackendAdapter
from bbia_sim.pose_detection import BBIAPoseDetection, create_pose_detector
from bbia_sim.troubleshooting import TroubleshootingChecker, check_all
from bbia_sim.utils.types import as_any_pose, clamp_joint_position


def demo_emotions() -> None:
    """Démonstration du module émotions."""
    print("\n=== DÉMONSTRATION ÉMOTIONS ===")
    emotions = BBIAEmotions()
    emotions.set_emotion("happy", 0.8)
    emotions.set_emotion("curious", 0.6)
    emotions.random_emotion()
    emotions.emotional_response("compliment")
    emotions.blend_emotions("happy", "excited", 0.5)
    stats = emotions.get_emotion_stats()
    print(f"Statistiques: {stats}")


def demo_vision() -> None:
    """Démonstration du module vision."""
    print("\n=== DÉMONSTRATION VISION ===")
    vision = BBIAVision()
    vision.start_async_scanning()
    vision.scan_environment_async()
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    result = vision.scan_environment_from_image(image)
    print(f"Résultat scan: {result}")
    vision.stop_async_scanning()


def demo_voice() -> None:
    """Démonstration du module voix."""
    print("\n=== DÉMONSTRATION VOIX ===")
    dire_texte("Bonjour, je suis BBIA")


def demo_chat() -> None:
    """Démonstration du module chat."""
    print("\n=== DÉMONSTRATION CHAT ===")
    try:
        chat = BBIAChat()
        response = chat.chat("Bonjour")
        print(f"Réponse: {response}")
    except Exception as e:
        print(f"Chat non disponible: {e}")


def demo_behaviors() -> None:
    """Démonstration des comportements."""
    print("\n=== DÉMONSTRATION COMPORTEMENTS ===")
    backend = MuJoCoBackend()
    backend.connect()
    manager = BBIABehaviorManager(robot_api=backend)
    manager.execute_behavior("wake_up")
    manager.clear_saved_moves()
    backend.disconnect()


def demo_adaptive_learning() -> None:
    """Démonstration de l'apprentissage adaptatif."""
    print("\n=== DÉMONSTRATION APPRENTISSAGE ADAPTATIF ===")
    learning = BBIAAdaptiveLearning()
    learning.learn_preference("user_1", "voice_speed", "fast")
    learning.remember_interaction("user_1", "greeting", "happy")
    prefs = learning.get_preferences("user_1")
    patterns = learning.get_patterns("user_1")
    adapted = learning.adapt_behavior("user_1", "conversation")
    print(f"Préférences: {prefs}, Patterns: {patterns}, Adapté: {adapted}")


def demo_adaptive_behavior() -> None:
    """Démonstration du comportement adaptatif."""
    print("\n=== DÉMONSTRATION COMPORTEMENT ADAPTATIF ===")
    behavior = BBIAAdaptiveBehavior()
    behavior.execute_behavior({})


def demo_memory() -> None:
    """Démonstration de la mémoire."""
    print("\n=== DÉMONSTRATION MÉMOIRE ===")
    memory = BBIAMemory()
    memory.load_learnings()
    append_record({"test": "data"})


def demo_tools() -> None:
    """Démonstration des outils."""
    print("\n=== DÉMONSTRATION OUTILS ===")
    tools = BBIATools()
    tools_list = tools.get_tools()
    print(f"Outils disponibles: {len(tools_list)}")


def demo_idle_animations() -> None:
    """Démonstration des animations idle."""
    print("\n=== DÉMONSTRATION ANIMATIONS IDLE ===")
    backend = MuJoCoBackend()
    backend.connect()
    manager = BBIIdleAnimationManager(backend)  # noqa: F841
    breathing = BBIABreathingAnimation(backend)  # noqa: F841
    pose_transitions = BBIAPoseTransitionManager(backend)  # noqa: F841
    vocal_tremor = BBIAVocalTremor(backend)  # noqa: F841
    print("Animations idle initialisées")
    backend.disconnect()


def demo_pose_detection() -> None:
    """Démonstration de la détection de pose."""
    print("\n=== DÉMONSTRATION DÉTECTION DE POSE ===")
    detector = BBIAPoseDetection()
    detector2 = create_pose_detector()  # noqa: F841
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    result = detector.detect_pose(image)
    print(f"Résultat détection: {result}")


def demo_troubleshooting() -> None:
    """Démonstration du troubleshooting."""
    print("\n=== DÉMONSTRATION TROUBLESHOOTING ===")
    checker = TroubleshootingChecker()  # noqa: F841
    results = check_all()
    print(f"Résultats: {results['summary']}")


def demo_backend_adapter() -> None:
    """Démonstration du backend adapter."""
    print("\n=== DÉMONSTRATION BACKEND ADAPTER ===")
    backend = MuJoCoBackend()
    backend.connect()
    adapter = BackendAdapter(backend)
    adapter.connect_if_needed()
    pose = adapter.get_present_head_pose()
    yaw = adapter.get_present_body_yaw()
    antennas = adapter.get_present_antenna_joint_positions()
    status = adapter.get_status()  # noqa: F841
    adapter.close()
    backend.disconnect()
    print(f"Pose: {pose.shape}, Yaw: {yaw}, Antennas: {antennas.shape}")


def demo_utils() -> None:
    """Démonstration des utilitaires."""
    print("\n=== DÉMONSTRATION UTILITAIRES ===")
    clamped = clamp_joint_position("head_yaw", 2.0)
    print(f"Position clampée: {clamped}")
    pose_array = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    pose = as_any_pose(pose_array, use_matrix=False)
    print(f"Pose créée: {pose}")


def main() -> None:
    """Fonction principale."""
    print("🚀 DÉMONSTRATION COMPLÈTE DE TOUTES LES CAPACITÉS BBIA")
    print("=" * 60)

    try:
        demo_emotions()
        demo_vision()
        demo_voice()
        demo_chat()
        demo_behaviors()
        demo_adaptive_learning()
        demo_adaptive_behavior()
        demo_memory()
        demo_tools()
        demo_idle_animations()
        demo_pose_detection()
        demo_troubleshooting()
        demo_backend_adapter()
        demo_utils()

        print("\n✅ Toutes les démonstrations terminées avec succès !")
    except Exception as e:
        print(f"\n❌ Erreur lors de la démonstration: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
