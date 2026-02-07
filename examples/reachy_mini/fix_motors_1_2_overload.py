#!/usr/bin/env python3
"""Déblocage moteurs 1 et 2 (surcharge / clignotement) – une seule procédure.

À lancer UNE FOIS après avoir allumé le robot (daemon déjà démarré).
- Désactive les moteurs 5 s (ils se détendent, erreur surcharge peut se réinitialiser).
- Réactive avec compensation gravité si dispo, sinon mode normal.
- Envoie un mouvement TRÈS LENT (6 s) vers la position neutre uniquement.
Pas de mouvement brusque, pas d’angle fort.
Si timeout depuis le Mac : fix_motors_1_2_overload_ssh.py --robot-ip <IP>
"""

import sys
import time

try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False

def fix_motors_1_2_overload() -> bool:
    if not SDK_AVAILABLE:
        print("❌ SDK reachy_mini non disponible. Installe: pip install reachy-mini")
        return False

    print("🔧 DÉBLOCAGE MOTEURS 1 ET 2 (surcharge)")
    print("=" * 60)
    print("Connexion au robot (WiFi: localhost_only=False, timeout 60s)...")
    print("(Si timeout : utiliser fix_motors_1_2_overload_ssh.py --robot-ip <IP>)")
    print()

    try:
        robot = ReachyMini(
            media_backend="no_media",
            use_sim=False,
            localhost_only=False,
            timeout=60.0,
        )
        robot.__enter__()
        robot.get_current_joint_positions()
        print("✅ Robot connecté")
        print()

        # 1) Désactiver les moteurs → ils se détendent, erreur surcharge peut se clear
        print("1️⃣ Désactivation des moteurs (5 s)...")
        robot.disable_motors()
        time.sleep(5)
        print("   ✅ Moteurs désactivés")
        print()

        # 2) Réactiver avec compensation gravité si dispo
        print("2️⃣ Réactivation des moteurs...")
        try:
            robot.enable_gravity_compensation()
            print("   ✅ Compensation gravité activée")
        except Exception:
            robot.enable_motors()
            print("   ✅ Moteurs activés (mode normal)")
        time.sleep(2)
        print()

        # 3) Un SEUL mouvement très lent vers neutre (pas d’angle agressif)
        print("3️⃣ Mouvement lent vers position neutre (6 s)...")
        neutral = create_head_pose(
            x=0, y=0, z=0,
            roll=0, pitch=0, yaw=0,
            degrees=True, mm=True,
        )
        robot.goto_target(head=neutral, duration=6.0)
        time.sleep(6.5)
        print("   ✅ Mouvement terminé")
        print()

        robot.__exit__(None, None, None)
        print("=" * 60)
        print("✅ DÉBLOCAGE TERMINÉ")
        print("=" * 60)
        print("Relance le dashboard et teste. Si les moteurs 1 et 2 clignotent encore,")
        print("contacter Pollen (calibration / offsets).")
        return True

    except Exception as e:
        print(f"❌ Erreur: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    fix_motors_1_2_overload()
