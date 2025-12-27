#!/usr/bin/env python3
"""Force la tête à être vraiment droite en ajustant tous les stewart joints.

Ce script corrige la tête de travers en forçant tous les stewart joints à 0.
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


def force_head_straight() -> None:
    """Force la tête à être droite."""
    print("🔧 CORRECTION FORCÉE - Tête droite")
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

        # Lire la position actuelle
        print("2️⃣ Position actuelle...")
        head_positions, _ = robot.get_current_joint_positions()
        print("   Positions avant correction:")
        for i in range(min(6, len(head_positions))):
            joint_name = f"stewart_{i+1}"
            pos = head_positions[i]
            print(f"   {joint_name:12s}: {pos:8.4f} rad ({pos*180/3.14159:6.2f}°)")
        print()

        # Désactiver les moteurs pour permettre ajustement manuel si nécessaire
        print("3️⃣ Désactivation temporaire des moteurs...")
        robot.disable_motors()
        print("   ✅ Moteurs désactivés")
        print("   💡 Si la tête est vraiment bloquée mécaniquement,")
        print("      tu peux maintenant la déplacer manuellement vers le centre")
        print("      (attends 5 secondes)")
        time.sleep(5)
        print()

        # Réactiver les moteurs
        print("4️⃣ Réactivation des moteurs...")
        robot.enable_motors()
        print("   ✅ Moteurs réactivés")
        time.sleep(1)
        print()

        # Essayer plusieurs positions pour forcer la tête droite
        print("5️⃣ Correction de la tête (plusieurs tentatives)...")
        
        # Tentative 1: Position neutre standard
        print("   → Tentative 1: Position neutre standard...")
        try:
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )
            robot.goto_target(head=neutral, duration=3.0)
            time.sleep(3.5)
        except Exception as e:
            print(f"      ⚠️  Erreur: {e}")
        
        # Tentative 2: Petit mouvement pour débloquer
        print("   → Tentative 2: Petit mouvement pour débloquer...")
        try:
            # Petit mouvement roll
            test1 = create_head_pose(roll=5, degrees=True)
            robot.goto_target(head=test1, duration=1.5)
            time.sleep(2)
            
            test2 = create_head_pose(roll=-5, degrees=True)
            robot.goto_target(head=test2, duration=1.5)
            time.sleep(2)
            
            # Retour neutre
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )
            robot.goto_target(head=neutral, duration=2.0)
            time.sleep(2.5)
        except Exception as e:
            print(f"      ⚠️  Erreur: {e}")
        
        # Tentative 3: Petit mouvement pitch
        print("   → Tentative 3: Petit mouvement pitch...")
        try:
            test1 = create_head_pose(pitch=5, degrees=True)
            robot.goto_target(head=test1, duration=1.5)
            time.sleep(2)
            
            test2 = create_head_pose(pitch=-5, degrees=True)
            robot.goto_target(head=test2, duration=1.5)
            time.sleep(2)
            
            # Retour neutre
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )
            robot.goto_target(head=neutral, duration=2.0)
            time.sleep(2.5)
        except Exception as e:
            print(f"      ⚠️  Erreur: {e}")
        
        print("   ✅ Corrections terminées")
        print()

        # Vérifier la position finale
        print("6️⃣ Position finale...")
        head_positions, _ = robot.get_current_joint_positions()
        print("   Positions après correction:")
        for i in range(min(6, len(head_positions))):
            joint_name = f"stewart_{i+1}"
            pos = head_positions[i]
            print(f"   {joint_name:12s}: {pos:8.4f} rad ({pos*180/3.14159:6.2f}°)")
        
        # Calculer l'écart moyen
        if len(head_positions) >= 6:
            avg_pos = sum(head_positions[:6]) / 6
            max_dev = max(abs(p - avg_pos) for p in head_positions[:6])
            print()
            print(f"   Position moyenne: {avg_pos:.4f} rad ({avg_pos*180/3.14159:.2f}°)")
            print(f"   Écart max: {max_dev:.4f} rad ({max_dev*180/3.14159:.2f}°)")
            
            if max_dev < 0.1:  # Moins de 6° d'écart
                print("   ✅ Tête relativement droite")
            else:
                print("   ⚠️  Tête encore inclinée")
                print("      → Le problème peut être mécanique (calibration)")
        print()

        robot.__exit__(None, None, None)

        print("=" * 60)
        print("✅ CORRECTION TERMINÉE")
        print("=" * 60)
        print()
        print("💡 Si la tête est toujours de travers:")
        print("   1. C'est peut-être un problème de CALIBRATION/offset")
        print("   2. Les moteurs peuvent avoir des offsets différents")
        print("   3. Il faut peut-être recalibrer les offsets")
        print()
        print("   Solution: Contacter Pollen Robotics pour recalibration")
        print("   Formulaire: https://forms.gle/JdhMzadeCnbynw7Q6")
        print()

    except Exception as e:
        print(f"❌ Erreur: {e}")
        import traceback
        traceback.print_exc()


def main() -> None:
    """Fonction principale."""
    print("🔧 CORRECTION FORCÉE - Tête droite")
    print("=" * 60)
    print()
    print("Ce script va forcer la tête à être droite en:")
    print("  1. Désactivant/réactivant les moteurs")
    print("  2. Faisant plusieurs mouvements pour débloquer")
    print("  3. Repositionnant en position neutre")
    print()

    if not SDK_AVAILABLE:
        print("❌ SDK reachy_mini non disponible")
        sys.exit(1)

    force_head_straight()


if __name__ == "__main__":
    main()

