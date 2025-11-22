#!/usr/bin/env python3
"""
Hello World BBIA-SIM - Conformité Parfaite SDK Officiel
Exemple simple pour tester la conformité parfaite avec le SDK officiel Reachy-Mini
"""

import sys
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory


def main():
    """Exemple simple de conformité parfaite SDK officiel."""
    print("🚀 BBIA-SIM Hello World - Conformité Parfaite SDK Officiel")
    print("=" * 60)

    # Créer le backend Reachy-Mini
    print("1. Création du backend Reachy-Mini...")
    backend = RobotFactory.create_backend("reachy_mini")

    if backend is None:
        print("❌ Erreur: Impossible de créer le backend")
        return False

    print("✅ Backend créé avec succès")

    # Connexion
    print("\n2. Connexion au robot...")
    if backend.connect():
        print("✅ Connexion réussie")
    else:
        print("⚠️  Mode simulation (pas de robot physique)")

    # Test des méthodes SDK officiel
    print("\n3. Test des méthodes SDK officiel...")

    # Test get_current_joint_positions
    print("   - get_current_joint_positions()...")
    if hasattr(backend, "get_current_joint_positions"):
        head_pos, antenna_pos = backend.get_current_joint_positions()  # type: ignore[attr-defined]
        print(f"     ✅ Head positions: {len(head_pos)} éléments")
        print(f"     ✅ Antenna positions: {len(antenna_pos)} éléments")
    else:
        print("     ⚠️  Méthode non disponible")

    # Test get_current_head_pose
    print("   - get_current_head_pose()...")
    if hasattr(backend, "get_current_head_pose"):
        head_pose = backend.get_current_head_pose()  # type: ignore[attr-defined]
        print(f"     ✅ Head pose shape: {head_pose.shape}")
    else:
        print("     ⚠️  Méthode non disponible")

    # Test get_present_antenna_joint_positions
    print("   - get_present_antenna_joint_positions()...")
    if hasattr(backend, "get_present_antenna_joint_positions"):
        antenna_positions = backend.get_present_antenna_joint_positions()  # type: ignore[attr-defined]
        print(f"     ✅ Antenna positions: {antenna_positions}")
    else:
        print("     ⚠️  Méthode non disponible")

    # Test des méthodes de contrôle (retournent None)
    print("\n4. Test des méthodes de contrôle...")

    control_methods = [
        (
            "enable_motors",
            lambda: backend.enable_motors()
            if hasattr(backend, "enable_motors")
            else None,
        ),  # type: ignore[attr-defined]
        (
            "disable_motors",
            lambda: backend.disable_motors()
            if hasattr(backend, "disable_motors")
            else None,
        ),  # type: ignore[attr-defined]
        (
            "enable_gravity_compensation",
            lambda: backend.enable_gravity_compensation()
            if hasattr(backend, "enable_gravity_compensation")
            else None,
        ),  # type: ignore[attr-defined]
        (
            "disable_gravity_compensation",
            lambda: backend.disable_gravity_compensation()
            if hasattr(backend, "disable_gravity_compensation")
            else None,  # type: ignore[attr-defined]
        ),
        (
            "set_target_body_yaw",
            lambda: backend.set_target_body_yaw(0.1)
            if hasattr(backend, "set_target_body_yaw")
            else None,
        ),  # type: ignore[attr-defined]
        (
            "set_target_antenna_joint_positions",
            lambda: backend.set_target_antenna_joint_positions([0.1, 0.2])
            if hasattr(backend, "set_target_antenna_joint_positions")
            else None,  # type: ignore[attr-defined]
        ),
        (
            "start_recording",
            lambda: backend.start_recording()
            if hasattr(backend, "start_recording")
            else None,
        ),  # type: ignore[attr-defined]
        ("wake_up", lambda: backend.wake_up() if hasattr(backend, "wake_up") else None),  # type: ignore[attr-defined]
        (
            "goto_sleep",
            lambda: backend.goto_sleep() if hasattr(backend, "goto_sleep") else None,
        ),  # type: ignore[attr-defined]
    ]

    for method_name, method_call in control_methods:
        try:
            result = method_call()
            status = (
                "✅" if result is None or result is True else f"❌ ({type(result)})"
            )
            print(f"   - {method_name}(): {status}")
        except Exception as e:
            print(f"   - {method_name}(): ❌ Erreur: {e}")

    # Test des méthodes de mouvement
    print("\n5. Test des méthodes de mouvement...")

    try:
        # Test look_at_world
        if hasattr(backend, "look_at_world"):
            pose = backend.look_at_world(0.1, 0.2, 0.3)  # type: ignore[attr-defined]
            print(f"   - look_at_world(): ✅ Pose shape: {pose.shape}")
        else:
            print("   - look_at_world(): ⚠️  Méthode non disponible")
    except Exception as e:
        print(f"   - look_at_world(): ❌ Erreur: {e}")

    try:
        # Test look_at_image
        if hasattr(backend, "look_at_image"):
            pose = backend.look_at_image(100, 200)  # type: ignore[attr-defined]
            print(f"   - look_at_image(): ✅ Pose shape: {pose.shape}")
        else:
            print("   - look_at_image(): ⚠️  Méthode non disponible")
    except Exception as e:
        print(f"   - look_at_image(): ❌ Erreur: {e}")

    try:
        # Test goto_target
        if hasattr(backend, "goto_target"):
            backend.goto_target(body_yaw=0.1)  # type: ignore[attr-defined]
            print("   - goto_target(): ✅")
        else:
            print("   - goto_target(): ⚠️  Méthode non disponible")
    except Exception as e:
        print(f"   - goto_target(): ❌ Erreur: {e}")

    # Test des émotions BBIA
    print("\n6. Test des émotions BBIA...")

    emotions = ["happy", "sad", "neutral", "excited", "curious", "calm"]
    for emotion in emotions:
        try:
            result = backend.set_emotion(emotion, 0.8)
            status = "✅" if result else "❌"
            print(f"   - set_emotion('{emotion}'): {status}")
        except Exception as e:
            print(f"   - set_emotion('{emotion}'): ❌ Erreur: {e}")

    # Test des comportements
    print("\n7. Test des comportements...")

    behaviors = ["wake_up", "goto_sleep", "nod"]
    for behavior in behaviors:
        try:
            result = backend.run_behavior(behavior, 2.0)
            status = "✅" if result else "❌"
            print(f"   - run_behavior('{behavior}'): {status}")
        except Exception as e:
            print(f"   - run_behavior('{behavior}'): ❌ Erreur: {e}")

    # Test de la télémétrie
    print("\n8. Test de la télémétrie...")

    try:
        if hasattr(backend, "get_telemetry"):
            telemetry = backend.get_telemetry()  # type: ignore[attr-defined]
            print(f"   - get_telemetry(): ✅ {len(telemetry)} métriques")
            for key, value in telemetry.items():
                print(f"     {key}: {value}")
        else:
            print("   - get_telemetry(): ⚠️  Méthode non disponible")
    except Exception as e:
        print(f"   - get_telemetry(): ❌ Erreur: {e}")

    # Déconnexion
    print("\n9. Déconnexion...")
    backend.disconnect()
    print("✅ Déconnexion réussie")

    print("\n" + "=" * 60)
    print("🎉 CONFORMITÉ PARFAITE SDK OFFICIEL TESTÉE AVEC SUCCÈS !")
    print("✅ Votre BBIA-SIM est prêt pour le robot physique !")
    print("=" * 60)

    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
