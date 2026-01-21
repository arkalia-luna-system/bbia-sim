#!/usr/bin/env python3
"""Débloque les mouvements de la tête - Test et activation des moteurs.

Ce script teste et débloque les mouvements de la tête.
"""

import sys
import time

try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    print("❌ SDK reachy_mini non disponible")
    sys.exit(1)


def unlock_head_movement() -> None:
    """Débloque les mouvements de la tête."""
    print("🔓 DÉBLOCAGE MOUVEMENTS TÊTE")
    print("=" * 60)
    print()

    try:
        print("1️⃣ Connexion au robot...")
        robot = ReachyMini(
            media_backend="no_media",
            use_sim=False,
            localhost_only=True,
            timeout=5.0,
        )
        robot.__enter__()
        print("   ✅ Robot connecté")
        print()

        # Vérifier l'état des moteurs
        print("2️⃣ Vérification état des moteurs...")
        try:
            # D'abord désactiver puis réactiver pour forcer
            robot.disable_motors()
            print("   → Moteurs désactivés (reset)")
            time.sleep(1)

            # Activer les moteurs (pas compensation gravité pour permettre mouvement)
            robot.enable_motors()
            print("   ✅ Moteurs activés (mode normal - permet mouvement)")
            time.sleep(2)
        except Exception as e:
            print(f"   ⚠️  Erreur activation: {e}")
            print("   → Essai avec compensation gravité...")
            try:
                robot.enable_gravity_compensation()
                print("   ✅ Compensation gravité activée")
                time.sleep(2)
            except Exception as e2:
                print(f"   ❌ Erreur: {e2}")
        print()

        # Lire la position actuelle
        print("3️⃣ Position actuelle...")
        head_positions, _ = robot.get_current_joint_positions()
        print("   Positions:")
        for i in range(min(6, len(head_positions))):
            joint_name = f"stewart_{i+1}"
            pos = head_positions[i]
            print(f"   {joint_name:12s}: {pos:8.4f} rad ({pos*180/3.14159:6.2f}°)")
        print()

        # Test 1: Petit mouvement roll (gauche/droite) avec vérification
        print("4️⃣ Test mouvement Roll (gauche/droite)...")
        try:
            # Lire position avant
            pos_before, _ = robot.get_current_joint_positions()
            print(f"   Position avant: stewart_2 = {pos_before[1]*180/3.14159:.2f}°" if len(pos_before) >= 2 else "   Position avant: N/A")

            print("   → Mouvement +10° (plus grand pour être sûr)...")
            pose1 = create_head_pose(roll=10, degrees=True)
            robot.goto_target(head=pose1, duration=2.0)
            time.sleep(3.0)  # Attendre plus longtemps

            # Vérifier si ça a bougé
            pos_after1, _ = robot.get_current_joint_positions()
            if len(pos_before) >= 2 and len(pos_after1) >= 2:
                diff1 = abs(pos_after1[1] - pos_before[1]) * 180 / 3.14159
                print(f"   Position après: stewart_2 = {pos_after1[1]*180/3.14159:.2f}° (déplacement: {diff1:.2f}°)")
                if diff1 < 1.0:
                    print("   ⚠️  La tête n'a PAS bougé!")
                else:
                    print("   ✅ La tête a bougé")

            print("   → Mouvement -10°...")
            pose2 = create_head_pose(roll=-10, degrees=True)
            robot.goto_target(head=pose2, duration=2.0)
            time.sleep(3.0)

            print("   → Retour centre...")
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )
            robot.goto_target(head=neutral, duration=2.0)
            time.sleep(3.0)
            print("   ✅ Test Roll terminé")
        except Exception as e:
            print(f"   ❌ Erreur test Roll: {e}")
        print()

        # Test 2: Petit mouvement pitch (haut/bas)
        print("5️⃣ Test mouvement Pitch (haut/bas)...")
        try:
            print("   → Mouvement +5°...")
            pose1 = create_head_pose(pitch=5, degrees=True)
            robot.goto_target(head=pose1, duration=2.0)
            time.sleep(2.5)

            print("   → Mouvement -5°...")
            pose2 = create_head_pose(pitch=-5, degrees=True)
            robot.goto_target(head=pose2, duration=2.0)
            time.sleep(2.5)

            print("   → Retour centre...")
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )
            robot.goto_target(head=neutral, duration=2.0)
            time.sleep(2.5)
            print("   ✅ Test Pitch réussi")
        except Exception as e:
            print(f"   ❌ Erreur test Pitch: {e}")
        print()

        # Test 3: Petit mouvement yaw (rotation)
        print("6️⃣ Test mouvement Yaw (rotation)...")
        try:
            print("   → Mouvement +10°...")
            pose1 = create_head_pose(yaw=10, degrees=True)
            robot.goto_target(head=pose1, duration=2.0)
            time.sleep(2.5)

            print("   → Mouvement -10°...")
            pose2 = create_head_pose(yaw=-10, degrees=True)
            robot.goto_target(head=pose2, duration=2.0)
            time.sleep(2.5)

            print("   → Retour centre...")
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )
            robot.goto_target(head=neutral, duration=2.0)
            time.sleep(2.5)
            print("   ✅ Test Yaw réussi")
        except Exception as e:
            print(f"   ❌ Erreur test Yaw: {e}")
        print()

        # Vérifier la position finale
        print("7️⃣ Position finale...")
        head_positions_final, _ = robot.get_current_joint_positions()
        print("   Positions après tests:")
        for i in range(min(6, len(head_positions_final))):
            joint_name = f"stewart_{i+1}"
            pos = head_positions_final[i]
            print(f"   {joint_name:12s}: {pos:8.4f} rad ({pos*180/3.14159:6.2f}°)")
        print()

        robot.__exit__(None, None, None)

        print("=" * 60)
        print("✅ TESTS TERMINÉS")
        print("=" * 60)
        print()
        print("💡 Résultats:")
        print("   - Si tous les tests sont ✅ : Les mouvements fonctionnent!")
        print("   - Si des tests échouent ❌ : Problème avec certains axes")
        print()
        print("⚠️  Si la tête ne bouge toujours pas:")
        print("   1. Vérifier que les moteurs sont activés")
        print("   2. Redémarrer le daemon: sudo systemctl restart reachy-mini-daemon")
        print("   3. Redémarrer le robot (OFF/ON)")
        print()

    except Exception as e:
        print(f"❌ Erreur: {e}")
        import traceback
        traceback.print_exc()


def main() -> None:
    """Fonction principale."""
    print("🔓 DÉBLOCAGE MOUVEMENTS TÊTE")
    print("=" * 60)
    print()
    print("Ce script va tester les mouvements de la tête:")
    print("  - Roll (gauche/droite)")
    print("  - Pitch (haut/bas)")
    print("  - Yaw (rotation)")
    print()

    if not SDK_AVAILABLE:
        print("❌ SDK reachy_mini non disponible")
        sys.exit(1)

    unlock_head_movement()


if __name__ == "__main__":
    main()

