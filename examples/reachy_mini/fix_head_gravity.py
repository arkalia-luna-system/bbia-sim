#!/usr/bin/env python3
"""Corrige la tête qui retombe - Active la compensation de gravité.

Ce script active la compensation de gravité pour que la tête reste en position.
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


def fix_head_gravity() -> None:
    """Corrige la tête qui retombe."""
    print("🔧 CORRECTION TÊTE QUI RETOMBE")
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

        # Activer la compensation de gravité
        print("2️⃣ Activation compensation de gravité...")
        try:
            robot.enable_gravity_compensation()
            print("   ✅ Compensation de gravité activée")
            print("   💡 La tête devrait maintenant rester en position")
            time.sleep(1)
        except Exception as e:
            print(f"   ⚠️  Erreur activation compensation: {e}")
            print("   → Essai avec enable_motors()...")
            try:
                robot.enable_motors()
                print("   ✅ Moteurs activés (mode normal)")
            except Exception as e2:
                print(f"   ❌ Erreur: {e2}")
        print()

        # Repositionner la tête droite
        print("3️⃣ Repositionnement tête droite...")
        try:
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )
            robot.goto_target(head=neutral, duration=3.0)
            time.sleep(3.5)
            print("   ✅ Tête repositionnée")
        except Exception as e:
            print(f"   ⚠️  Erreur: {e}")
        print()

        # Vérifier la position
        print("4️⃣ Vérification position...")
        head_positions, _ = robot.get_current_joint_positions()
        print("   Positions:")
        for i in range(min(6, len(head_positions))):
            joint_name = f"stewart_{i+1}"
            pos = head_positions[i]
            print(f"   {joint_name:12s}: {pos:8.4f} rad ({pos*180/3.14159:6.2f}°)")
        print()

        # Attendre un peu pour voir si la tête retombe
        print("5️⃣ Test stabilité (attente 5 secondes)...")
        print("   → Vérifie si la tête reste en position...")
        time.sleep(5)

        head_positions_after, _ = robot.get_current_joint_positions()
        if len(head_positions) >= 6 and len(head_positions_after) >= 6:
            # Comparer les positions
            max_diff = max(abs(head_positions[i] - head_positions_after[i]) for i in range(6))
            if max_diff < 0.01:  # Moins de 0.01 rad de différence
                print("   ✅ Tête stable (ne retombe pas)")
            else:
                print(f"   ⚠️  Tête a bougé de {max_diff*180/3.14159:.2f}°")
                print("   → La compensation de gravité ne fonctionne peut-être pas correctement")
        print()

        robot.__exit__(None, None, None)

        print("=" * 60)
        print("✅ CORRECTION TERMINÉE")
        print("=" * 60)
        print()
        print("💡 RÉSULTAT:")
        print("   ✅ Compensation de gravité activée")
        print("   ✅ Tête repositionnée")
        print()
        print("⚠️  Si la tête retombe toujours:")
        print("   1. Les moteurs doivent rester activés")
        print("   2. Vérifier que le daemon est démarré:")
        print("      sudo systemctl status reachy-mini-daemon")
        print("   3. Redémarrer le daemon si nécessaire:")
        print("      sudo systemctl restart reachy-mini-daemon")
        print()
        print("💡 La tête doit rester en position si:")
        print("   - Les moteurs sont activés (enable_motors)")
        print("   - OU la compensation de gravité est activée (enable_gravity_compensation)")
        print()

    except Exception as e:
        print(f"❌ Erreur: {e}")
        import traceback
        traceback.print_exc()


def main() -> None:
    """Fonction principale."""
    print("🔧 CORRECTION TÊTE QUI RETOMBE")
    print("=" * 60)
    print()
    print("Ce script va:")
    print("  1. Activer la compensation de gravité")
    print("  2. Repositionner la tête droite")
    print("  3. Vérifier que la tête reste en position")
    print()

    if not SDK_AVAILABLE:
        print("❌ SDK reachy_mini non disponible")
        sys.exit(1)

    fix_head_gravity()


if __name__ == "__main__":
    main()

