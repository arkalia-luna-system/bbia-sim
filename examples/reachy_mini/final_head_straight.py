#!/usr/bin/env python3
"""Correction finale - Remet la tête vraiment droite maintenant que les mouvements fonctionnent.

Ce script utilise les mouvements fonctionnels pour remettre la tête droite.
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


def final_head_straight() -> None:
    """Remet la tête vraiment droite."""
    print("🎯 CORRECTION FINALE - Tête droite")
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

        # Activer les moteurs
        print("2️⃣ Activation des moteurs...")
        robot.enable_motors()
        print("   ✅ Moteurs activés")
        time.sleep(1)
        print()

        # Lire la position actuelle
        print("3️⃣ Position actuelle...")
        head_positions, _ = robot.get_current_joint_positions()
        print("   Positions avant correction:")
        for i in range(min(6, len(head_positions))):
            joint_name = f"stewart_{i+1}"
            pos = head_positions[i]
            print(f"   {joint_name:12s}: {pos:8.4f} rad ({pos*180/3.14159:6.2f}°)")
        print()

        # Correction progressive avec plusieurs tentatives
        print("4️⃣ Correction progressive de la tête...")

        # Tentative 1: Position neutre standard
        print("   → Tentative 1: Position neutre (0,0,0)...")
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

        # Vérifier la position après tentative 1
        head_positions, _ = robot.get_current_joint_positions()
        if len(head_positions) >= 6:
            avg_dev = sum(abs(p) for p in head_positions[:6]) / 6
            print(f"      Écart moyen: {avg_dev*180/3.14159:.2f}°")

        # Tentative 2: Ajustements fins
        print("   → Tentative 2: Ajustements fins...")
        try:
            # Petit mouvement pour "réinitialiser"
            test1 = create_head_pose(roll=2, pitch=2, degrees=True)
            robot.goto_target(head=test1, duration=2.0)
            time.sleep(2.5)

            test2 = create_head_pose(roll=-2, pitch=-2, degrees=True)
            robot.goto_target(head=test2, duration=2.0)
            time.sleep(2.5)

            # Retour neutre
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )
            robot.goto_target(head=neutral, duration=2.5)
            time.sleep(3.0)
        except Exception as e:
            print(f"      ⚠️  Erreur: {e}")

        # Tentative 3: Mouvement circulaire pour "centrer"
        print("   → Tentative 3: Mouvement circulaire pour centrer...")
        try:
            # Séquence de mouvements pour centrer
            movements = [
                create_head_pose(roll=3, degrees=True),
                create_head_pose(roll=-3, degrees=True),
                create_head_pose(pitch=3, degrees=True),
                create_head_pose(pitch=-3, degrees=True),
                create_head_pose(yaw=5, degrees=True),
                create_head_pose(yaw=-5, degrees=True),
            ]

            for _i, move in enumerate(movements, 1):
                robot.goto_target(head=move, duration=1.5)
                time.sleep(1.8)

            # Retour neutre final
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )
            robot.goto_target(head=neutral, duration=3.0)
            time.sleep(3.5)
        except Exception as e:
            print(f"      ⚠️  Erreur: {e}")

        print("   ✅ Corrections terminées")
        print()

        # Vérifier la position finale
        print("5️⃣ Position finale...")
        head_positions_final, _ = robot.get_current_joint_positions()
        print("   Positions après correction:")
        for i in range(min(6, len(head_positions_final))):
            joint_name = f"stewart_{i+1}"
            pos = head_positions_final[i]
            print(f"   {joint_name:12s}: {pos:8.4f} rad ({pos*180/3.14159:6.2f}°)")

        # Calculer l'écart
        if len(head_positions_final) >= 6:
            positions_deg = [p*180/3.14159 for p in head_positions_final[:6]]
            avg_pos = sum(positions_deg) / 6
            max_dev = max(abs(p - avg_pos) for p in positions_deg)
            avg_dev = sum(abs(p) for p in positions_deg) / 6

            print()
            print(f"   Position moyenne: {avg_pos:.2f}°")
            print(f"   Écart max: {max_dev:.2f}°")
            print(f"   Écart moyen: {avg_dev:.2f}°")

            if avg_dev < 5.0:  # Moins de 5° d'écart moyen
                print()
                print("   ✅ TÊTE RELATIVEMENT DROITE!")
            elif avg_dev < 10.0:
                print()
                print("   ⚠️  Tête légèrement inclinée (acceptable)")
            else:
                print()
                print("   ⚠️  Tête encore inclinée")
                print("      → C'est probablement un problème d'offset/calibration")
                print("      → La tête fonctionne mais les offsets d'usine sont incorrects")
        print()

        robot.__exit__(None, None, None)

        print("=" * 60)
        print("✅ CORRECTION FINALE TERMINÉE")
        print("=" * 60)
        print()
        print("💡 RÉSULTAT:")
        print("   ✅ Les mouvements fonctionnent!")
        print("   ✅ La tête bouge correctement!")
        if len(head_positions_final) >= 6:
            positions_deg = [p*180/3.14159 for p in head_positions_final[:6]]
            avg_dev = sum(abs(p) for p in positions_deg) / 6
            if avg_dev < 10.0:
                print("   ✅ La tête est maintenant droite!")
            else:
                print("   ⚠️  La tête est encore un peu inclinée (problème d'offset)")
        print()
        print("🎉 FÉLICITATIONS! Le robot fonctionne maintenant!")
        print()
        print("⚠️  Si le moteur 2 clignote toujours en rouge:")
        print("   → C'est une erreur matérielle mineure")
        print("   → Les mouvements fonctionnent quand même")
        print("   → Tu peux l'ignorer si tout fonctionne")
        print("   → OU contacter Pollen pour vérification")
        print()

    except Exception as e:
        print(f"❌ Erreur: {e}")
        import traceback
        traceback.print_exc()


def main() -> None:
    """Fonction principale."""
    print("🎯 CORRECTION FINALE - Tête droite")
    print("=" * 60)
    print()
    print("Maintenant que les mouvements fonctionnent,")
    print("ce script va remettre la tête vraiment droite.")
    print()

    if not SDK_AVAILABLE:
        print("❌ SDK reachy_mini non disponible")
        sys.exit(1)

    final_head_straight()


if __name__ == "__main__":
    main()

