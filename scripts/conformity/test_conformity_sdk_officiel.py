#!/usr/bin/env python3
"""🧪 TEST CONFORMITÉ SDK OFFICIEL REACHY-MINI
Validation complète de la conformité avec le SDK officiel reachy_mini
Basé sur les spécifications officielles de Pollen Robotics (décembre 2024)
"""

import sys
import time
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory


def test_sdk_availability():
    """Test la disponibilité du SDK officiel."""
    print("🔍 Test disponibilité SDK officiel...")

    try:
        from reachy_mini import ReachyMini

        print("✅ Module reachy_mini importé avec succès")
        print("✅ Classe ReachyMini disponible")

        # Vérifier que la classe existe et ses méthodes principales
        # (Ce sont des méthodes d'instance, pas du module)
        instance_methods = [
            "wake_up",
            "goto_sleep",
            "get_current_joint_positions",
            "set_target_head_pose",
            "set_target_body_yaw",
            "look_at_world",
            "get_current_head_pose",
            "look_at_image",
            "goto_target",
            "enable_motors",
            "disable_motors",
        ]

        print("\n📋 Méthodes d'instance ReachyMini:")
        all_available = True
        for method in instance_methods:
            if hasattr(ReachyMini, method):
                print(f"   ✅ {method}")
            else:
                print(f"   ❌ {method} (manquante)")
                all_available = False

        # Vérifier les utilitaires du module
        try:
            from reachy_mini import utils

            _ = utils  # Rendre l'import utilisé
            print("\n✅ Utilitaires SDK disponibles: reachy_mini.utils")
        except ImportError:
            print("\n⚠️  Utilitaires SDK partiellement disponibles")

        return all_available

    except ImportError as e:
        print(f"❌ SDK officiel non disponible: {e}")
        print("💡 Installez avec: pip install reachy-mini")
        return False


def test_backend_conformity():
    """Test la conformité du backend avec le SDK officiel."""
    print("\n🧪 Test conformité backend...")

    try:
        # Créer le backend
        robot = RobotFactory.create_backend("reachy_mini")
        print("✅ Backend reachy_mini créé")

        # Test des joints disponibles
        joints = robot.get_available_joints()
        expected_joints = {
            "yaw_body",
            "stewart_1",
            "stewart_2",
            "stewart_3",
            "stewart_4",
            "stewart_5",
            "stewart_6",
            "left_antenna",
            "right_antenna",
        }

        if set(joints) == expected_joints:
            print("✅ Joints conformes au SDK officiel")
        else:
            print(f"❌ Joints non conformes: {set(joints) - expected_joints}")

        # Test des méthodes API
        robot.is_connected = True  # Mode simulation

        # Test émotions
        emotions = ["happy", "sad", "neutral", "excited", "curious", "calm"]
        for emotion in emotions:
            success = robot.set_emotion(emotion, 0.5)
            if not success:
                print(f"❌ Émotion {emotion} échouée")
                return False
        print("✅ Toutes les émotions fonctionnent")

        # Test mouvements
        test_joints = ["yaw_body", "stewart_1", "stewart_2"]
        for joint in test_joints:
            success = robot.set_joint_pos(joint, 0.1)
            if not success:
                print(f"❌ Mouvement {joint} échoué")
                return False
        print("✅ Tous les mouvements fonctionnent")

        # Test look_at
        success = robot.look_at(0.1, 0.2, 0.3)
        if not success:
            print("❌ Look_at échoué")
            return False
        print("✅ Look_at fonctionne")

        # Test comportements
        behaviors = ["wake_up", "goto_sleep", "nod"]
        for behavior in behaviors:
            success = robot.run_behavior(behavior, 1.0)
            if not success:
                print(f"❌ Comportement {behavior} échoué")
                return False
        print("✅ Tous les comportements fonctionnent")

        return True

    except Exception as e:
        print(f"❌ Erreur conformité backend: {e}")
        return False


def test_api_compatibility():
    """Test la compatibilité avec l'API officielle."""
    print("\n🔌 Test compatibilité API...")

    try:
        robot = RobotFactory.create_backend("reachy_mini")
        robot.is_connected = True

        # Test télémétrie
        telemetry = robot.get_telemetry()
        required_fields = [
            "step_count",
            "elapsed_time",
            "current_emotion",
            "is_connected",
        ]

        for field in required_fields:
            if field not in telemetry:
                print(f"❌ Champ télémétrie {field} manquant")
                return False
        print("✅ Télémétrie conforme")

        # Test limites de sécurité
        if hasattr(robot, "safe_amplitude_limit"):
            limit = robot.safe_amplitude_limit
            if limit <= 0.3:  # Limite recommandée
                print(f"✅ Limite amplitude sécurisée: {limit}")
            else:
                print(f"⚠️ Limite amplitude élevée: {limit}")
        else:
            print("❌ Limite amplitude non définie")
            return False

        return True

    except Exception as e:
        print(f"❌ Erreur compatibilité API: {e}")
        return False


def test_performance():
    """Test les performances du backend."""
    print("\n⚡ Test performances...")

    try:
        robot = RobotFactory.create_backend("reachy_mini")
        robot.is_connected = True

        # Test latence
        start_time = time.time()
        for _ in range(100):
            robot.set_joint_pos("yaw_body", 0.1)
            robot.get_joint_pos("yaw_body")
        end_time = time.time()

        avg_latency = (end_time - start_time) / 100 * 1000  # ms
        print(f"📊 Latence moyenne: {avg_latency:.2f}ms")

        if avg_latency < 10:  # < 10ms acceptable
            print("✅ Performances excellentes")
        elif avg_latency < 50:  # < 50ms acceptable
            print("✅ Performances bonnes")
        else:
            print("⚠️ Performances à améliorer")

        return True

    except Exception as e:
        print(f"❌ Erreur test performances: {e}")
        return False


def main():
    """Point d'entrée principal."""
    print("🧪 TEST CONFORMITÉ SDK OFFICIEL REACHY-MINI")
    print("=" * 60)
    print("📅 Basé sur les spécifications officielles (décembre 2024)")
    print("=" * 60)

    tests = [
        ("Disponibilité SDK", test_sdk_availability),
        ("Conformité Backend", test_backend_conformity),
        ("Compatibilité API", test_api_compatibility),
        ("Performances", test_performance),
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        try:
            if test_func():
                passed += 1
                print(f"✅ {test_name}: PASSÉ")
            else:
                print(f"❌ {test_name}: ÉCHOUÉ")
        except Exception as e:
            print(f"❌ {test_name}: ERREUR - {e}")

    print(f"\n{'='*60}")
    print("📊 RÉSULTATS FINAUX")
    print(f"Tests passés: {passed}/{total} ({passed/total:.1%})")

    if passed == total:
        print("🎉 CONFORMITÉ 100% AVEC LE SDK OFFICIEL !")
        print("✅ Votre projet est prêt pour le robot Reachy-Mini")
    else:
        print("⚠️ Des ajustements sont nécessaires")
        print("❌ Vérifiez les erreurs ci-dessus")

    print("\n🚀 Prochaines étapes:")
    print("1. Installer le SDK officiel: pip install reachy-mini")
    print("2. Tester avec robot physique")
    print("3. Développer nouveaux comportements")
    print("4. Intégrer modèles Hugging Face")


if __name__ == "__main__":
    main()
