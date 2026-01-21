#!/usr/bin/env python3
"""Script de correction tête penchée - Exécuté directement sur le robot via SSH.

Ce script se connecte au robot en SSH et exécute la correction directement sur le robot.
C'est plus fiable que de se connecter depuis le Mac via Zenoh.

Usage:
    python examples/reachy_mini/fix_head_tilted_ssh.py
    python examples/reachy_mini/fix_head_tilted_ssh.py --robot-ip <ROBOT_IP>
"""

import argparse
import subprocess
import sys
from pathlib import Path

# Script Python à exécuter sur le robot
SCRIPT_CONTENT = '''#!/usr/bin/env python3
"""Correction tête penchée - Exécuté sur le robot."""
import sys
import time
import numpy as np

try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose
except ImportError:
    print("❌ SDK reachy_mini non disponible sur le robot")
    sys.exit(1)

print("🔧 CORRECTION TÊTE PENCHÉE")
print("=" * 60)
print()

# Connexion au robot (localhost car on est sur le robot)
print("🔌 Connexion au robot (localhost)...")
try:
    robot = ReachyMini(media_backend="no_media", use_sim=False, localhost_only=True)
    robot.__enter__()

    # Tester connexion
    robot.get_current_joint_positions()
    print("✅ Robot connecté")
    print()

    # Correction de la position
    print("1️⃣ Correction position tête...")
    print("   Roll: +120° (remonte côté droit)")
    print("   Translation Z: +30mm (remonte tête)")
    print("   Pitch: -30° (évite qu'elle regarde le sol)")

    roll_correction = np.deg2rad(120.0)
    pitch_correction = np.deg2rad(-30.0)

    neutral_pose = create_head_pose(
        roll=roll_correction,
        pitch=pitch_correction,
        yaw=0.0,
        z=30.0,  # mm
        degrees=False,
        mm=True,
    )

    robot.goto_target(
        head=neutral_pose,
        antennas=[0.0, 0.0],
        duration=3.0,
        method="minjerk",
    )

    print("   ✅ Commande envoyée")
    print("   ⏳ Attente 4 secondes...")
    time.sleep(4)

    # Vérification
    print("\\n2️⃣ Vérification position...")
    head_positions, _ = robot.get_current_joint_positions()

    if len(head_positions) >= 12:
        stewart_values = [
            head_positions[1],  # stewart_1
            head_positions[3],  # stewart_2
            head_positions[5],  # stewart_3
            head_positions[7],  # stewart_4
            head_positions[9],  # stewart_5
            head_positions[11],  # stewart_6
        ]
    elif len(head_positions) >= 6:
        stewart_values = head_positions[:6]
    else:
        print("   ⚠️  Format inattendu")
        sys.exit(1)

    avg = sum(stewart_values) / len(stewart_values)
    variance = sum((v - avg) ** 2 for v in stewart_values) / len(stewart_values)
    std_dev = variance ** 0.5

    print(f"   Moyenne: {avg*180/np.pi:+.2f}°")
    print(f"   Écart-type: {std_dev*180/np.pi:+.2f}°")

    if std_dev < 0.3:
        print("   ✅ Tête équilibrée!")
    else:
        print("   ⚠️  Tête encore déséquilibrée")

    robot.__exit__(None, None, None)
    print("\\n✅ Correction terminée")

except Exception as e:
    print(f"❌ Erreur: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
'''


def main() -> None:
    """Fonction principale."""
    parser = argparse.ArgumentParser(
        description="Corrige la tête penchée en exécutant le script directement sur le robot"
    )
    parser.add_argument(
        "--robot-ip",
        type=str,
        default="<ROBOT_IP>",
        help="IP du robot (défaut: <ROBOT_IP>)",
    )
    parser.add_argument(
        "--user",
        type=str,
        default="pollen",
        help="Utilisateur SSH (défaut: pollen)",
    )

    args = parser.parse_args()

    print("🔧 CORRECTION TÊTE PENCHÉE - Via SSH")
    print("=" * 60)
    print()
    print(f"Robot: {args.user}@{args.robot_ip}")
    print("Le script sera exécuté directement sur le robot")
    print()

    # Créer un fichier temporaire avec le script
    script_file = Path("/tmp/fix_head_tilted_robot.py")
    script_file.write_text(SCRIPT_CONTENT)
    script_file.chmod(0o755)

    # Copier le script sur le robot et l'exécuter
    print("📤 Copie du script sur le robot...")
    try:
        # Copier via scp
        scp_cmd = [
            "scp",
            str(script_file),
            f"{args.user}@{args.robot_ip}:/tmp/fix_head_tilted_robot.py",
        ]
        result = subprocess.run(scp_cmd, check=False, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"⚠️  Erreur copie (peut nécessiter mot de passe): {result.stderr}")
            print("   Vous devrez copier manuellement le script")
            print(f"   Fichier: {script_file}")

        # Exécuter via SSH
        print("🚀 Exécution du script sur le robot...")
        ssh_cmd = [
            "ssh",
            f"{args.user}@{args.robot_ip}",
            "python3 /tmp/fix_head_tilted_robot.py",
        ]
        result = subprocess.run(ssh_cmd, check=False)

        if result.returncode == 0:
            print("\n✅ Correction terminée avec succès!")
        else:
            print(f"\n⚠️  Le script a retourné le code {result.returncode}")
            print("   Vérifiez les messages ci-dessus")

    except FileNotFoundError:
        print("❌ Commandes scp/ssh non trouvées")
        print("   Installez OpenSSH ou copiez/exécutez manuellement:")
        print(f"   1. Copier: {script_file} vers le robot")
        print(f"   2. SSH: ssh {args.user}@{args.robot_ip}")
        print("   3. Exécuter: python3 /tmp/fix_head_tilted_robot.py")
    except Exception as e:
        print(f"❌ Erreur: {e}")
        sys.exit(1)
    finally:
        # Nettoyer
        if script_file.exists():
            script_file.unlink()


if __name__ == "__main__":
    main()
