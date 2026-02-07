#!/usr/bin/env python3
"""Déblocage moteurs 1 et 2 – exécuté sur le robot via SSH (évite le timeout Zenoh).

Si depuis le Mac tu as « Timeout while waiting for connection with the server »,
lance ce script : il copie et exécute la procédure directement sur le robot.

Usage (depuis le Mac, même réseau que le robot) :
    python3 examples/reachy_mini/fix_motors_1_2_overload_ssh.py --robot-ip 192.168.129.64
    python3 examples/reachy_mini/fix_motors_1_2_overload_ssh.py --robot-ip reachy-mini.local

Ou en SSH sur le robot :
    ssh pollen@<ROBOT_IP>
    python3 /chemin/vers/fix_motors_1_2_overload.py   # avec localhost_only=True sur le robot
"""

import argparse
import subprocess
import sys
from pathlib import Path

# Script Python à exécuter sur le robot (même logique que fix_motors_1_2_overload.py)
# Sur Reachy Wireless le daemon Zenoh écoute souvent sur l'IP WiFi, pas 127.0.0.1 → localhost_only=False
SCRIPT_CONTENT = '''#!/usr/bin/env python3
"""Déblocage moteurs 1 et 2 - Exécuté sur le robot."""
import os
import sys
import time

try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose
except ImportError:
    print("❌ SDK reachy_mini non disponible. Sur le robot: pip install reachy-mini")
    sys.exit(1)

# IP du robot (passée en env par le launcher) pour que Zenoh trouve le daemon sur le réseau local
ROBOT_IP = os.environ.get("REACHY_ROBOT_IP", "192.168.129.64")

print("🔧 DÉBLOCAGE MOTEURS 1 ET 2 (surcharge)")
print("=" * 60)
print("Connexion au daemon (réseau local, IP du robot)...")
print()

try:
    # localhost_only=False : sur Wireless le daemon écoute sur l'IP WiFi, pas 127.0.0.1
    robot = ReachyMini(
        media_backend="no_media",
        use_sim=False,
        localhost_only=False,
        timeout=45.0,
    )
    robot.__enter__()
    robot.get_current_joint_positions()
    print("✅ Robot connecté")
    print()

    print("1️⃣ Désactivation des moteurs (5 s)...")
    robot.disable_motors()
    time.sleep(5)
    print("   ✅ Moteurs désactivés")
    print()

    print("2️⃣ Réactivation des moteurs...")
    try:
        robot.enable_gravity_compensation()
        print("   ✅ Compensation gravité activée")
    except Exception:
        robot.enable_motors()
        print("   ✅ Moteurs activés (mode normal)")
    time.sleep(2)
    print()

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
    print("Si les moteurs 1 et 2 clignotent encore → calibration/offsets côté Pollen.")
    sys.exit(0)

except Exception as e:
    print(f"❌ Erreur: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
'''


def main() -> None:
    """Fonction principale."""
    parser = argparse.ArgumentParser(
        description="Déblocage moteurs 1 et 2 en exécutant le script sur le robot via SSH"
    )
    parser.add_argument(
        "--robot-ip",
        type=str,
        default="192.168.129.64",
        help="IP ou hostname du robot (défaut: 192.168.129.64)",
    )
    parser.add_argument(
        "--user",
        type=str,
        default="pollen",
        help="Utilisateur SSH (défaut: pollen)",
    )
    args = parser.parse_args()

    if args.robot_ip == "<ROBOT_IP>" or not args.robot_ip.strip():
        print("❌ Indique l'IP du robot: --robot-ip 192.168.129.64 (ou reachy-mini.local)")
        sys.exit(1)

    print("🔧 DÉBLOCAGE MOTEURS 1 ET 2 – Via SSH")
    print("=" * 60)
    print()
    print(f"Robot: {args.user}@{args.robot_ip}")
    print("Le script sera exécuté sur le robot (pas de timeout Zenoh).")
    print()

    script_file = Path("/tmp/fix_motors_1_2_overload_robot.py")
    script_file.write_text(SCRIPT_CONTENT)
    script_file.chmod(0o755)

    print("📤 Copie du script sur le robot...")
    try:
        scp_cmd = [
            "scp",
            str(script_file),
            f"{args.user}@{args.robot_ip}:/tmp/fix_motors_1_2_overload_robot.py",
        ]
        result = subprocess.run(scp_cmd, check=False, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"⚠️  Erreur copie (mot de passe SSH si besoin): {result.stderr or result.stdout}")
            print("   Copie manuelle: scp ... puis ssh ... python3 /tmp/fix_motors_1_2_overload_robot.py")
        else:
            print("   ✅ Copié")

        print("🚀 Exécution sur le robot...")
        ssh_cmd = [
            "ssh",
            f"{args.user}@{args.robot_ip}",
            f"REACHY_ROBOT_IP={args.robot_ip} python3 /tmp/fix_motors_1_2_overload_robot.py",
        ]
        result = subprocess.run(ssh_cmd, check=False)
        if result.returncode == 0:
            print("\n✅ Déblocage terminé avec succès.")
        else:
            print(f"\n⚠️  Le script sur le robot a retourné le code {result.returncode}")
    except FileNotFoundError:
        print("❌ Commandes scp/ssh introuvables. Exécute manuellement sur le robot:")
        print(f"   1. Copie le script sur le robot, puis:")
        print(f"   2. ssh {args.user}@{args.robot_ip}")
        print("   3. python3 /tmp/fix_motors_1_2_overload_robot.py")
        sys.exit(1)
    finally:
        if script_file.exists():
            script_file.unlink()


if __name__ == "__main__":
    main()
