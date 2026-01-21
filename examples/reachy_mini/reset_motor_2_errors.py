#!/usr/bin/env python3
"""Réinitialise les erreurs du moteur 2 et corrige la tête de travers.

Ce script :
1. Lit les erreurs matérielles du moteur 2
2. Réinitialise les erreurs
3. Repositionne la tête en position neutre
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


def reset_motor_2_errors() -> None:
    """Réinitialise les erreurs du moteur 2."""
    print("🔧 RÉINITIALISATION MOTEUR 2 - Correction erreurs")
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

        # Désactiver puis réactiver les moteurs pour réinitialiser les erreurs
        print("2️⃣ Réinitialisation des moteurs...")
        try:
            # Désactiver les moteurs
            robot.disable_motors()
            print("   ✅ Moteurs désactivés")
            time.sleep(1)

            # Réactiver les moteurs
            robot.enable_motors()
            print("   ✅ Moteurs réactivés")
            time.sleep(1)
        except Exception as e:
            print(f"   ⚠️  Erreur réinitialisation: {e}")
        print()

        # Lire la position actuelle
        print("3️⃣ Position actuelle de la tête...")
        head_positions, antenna_positions = robot.get_current_joint_positions()
        print(f"   Positions tête: {len(head_positions)} joints")
        if len(head_positions) >= 2:
            stewart_2_pos = head_positions[1]
            print(f"   stewart_2: {stewart_2_pos:.4f} rad ({stewart_2_pos*180/3.14159:.2f}°)")
        print()

        # Repositionner la tête en position neutre
        print("4️⃣ Repositionnement de la tête en position neutre...")
        try:
            # Position neutre (tête droite)
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )

            print("   → Déplacement vers position neutre...")
            robot.goto_target(head=neutral, duration=3.0)
            time.sleep(3.5)
            print("   ✅ Tête repositionnée")
        except Exception as e:
            print(f"   ⚠️  Erreur repositionnement: {e}")
        print()

        # Vérifier la nouvelle position
        print("5️⃣ Vérification de la nouvelle position...")
        head_positions, _ = robot.get_current_joint_positions()
        if len(head_positions) >= 6:
            print("   Positions après correction:")
            for i in range(6):
                joint_name = f"stewart_{i+1}"
                pos = head_positions[i]
                print(f"   {joint_name:12s}: {pos:8.4f} rad ({pos*180/3.14159:6.2f}°)")
        print()

        # Essayer un petit mouvement du moteur 2 pour voir s'il répond
        print("6️⃣ Test de mouvement du moteur 2...")
        try:
            # Petit mouvement de test
            test_pose = create_head_pose(
                roll=5,  # Petit mouvement
                degrees=True
            )
            print("   → Petit mouvement de test...")
            robot.goto_target(head=test_pose, duration=1.0)
            time.sleep(1.5)

            # Retour neutre
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )
            robot.goto_target(head=neutral, duration=1.0)
            time.sleep(1.5)
            print("   ✅ Mouvement de test réussi")
        except Exception as e:
            print(f"   ⚠️  Erreur mouvement test: {e}")
            print("   → Le moteur 2 ne répond peut-être pas correctement")
        print()

        robot.__exit__(None, None, None)

        print("=" * 60)
        print("✅ RÉINITIALISATION TERMINÉE")
        print("=" * 60)
        print()
        print("💡 Vérifiez maintenant:")
        print("   1. Le moteur 2 clignote-t-il toujours en rouge?")
        print("   2. La tête est-elle droite?")
        print("   3. Les mouvements fonctionnent-ils?")
        print()
        print("Si le problème persiste:")
        print("   → Vérifiez le câblage du moteur 2")
        print("   → Vérifiez que le moteur peut bouger librement")
        print("   → Contactez le support Pollen Robotics")

    except Exception as e:
        print(f"❌ Erreur: {e}")
        import traceback
        traceback.print_exc()


def main() -> None:
    """Fonction principale."""
    print("🔧 RÉINITIALISATION MOTEUR 2 + CORRECTION TÊTE")
    print("=" * 60)
    print()
    print("Ce script va:")
    print("  1. Réinitialiser les erreurs du moteur 2")
    print("  2. Repositionner la tête en position neutre")
    print("  3. Tester le mouvement du moteur 2")
    print()

    if not SDK_AVAILABLE:
        print("❌ SDK reachy_mini non disponible")
        sys.exit(1)

    reset_motor_2_errors()


if __name__ == "__main__":
    main()

