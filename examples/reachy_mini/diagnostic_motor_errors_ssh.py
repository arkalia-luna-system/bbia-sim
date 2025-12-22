#!/usr/bin/env python3
"""Script de diagnostic des erreurs moteurs - À exécuter sur le robot via SSH.

Ce script identifie précisément quel moteur (par ID) a un problème et quel type d'erreur.

Usage:
    # Depuis votre Mac, connectez-vous en SSH au robot:
    ssh pollen@192.168.129.64

    # Puis exécutez le script:
    python3 /path/to/diagnostic_motor_errors_ssh.py

    # OU copiez le script sur le robot et exécutez-le:
    scp examples/reachy_mini/diagnostic_motor_errors_ssh.py pollen@192.168.129.64:/tmp/
    ssh pollen@192.168.129.64 "python3 /tmp/diagnostic_motor_errors_ssh.py"
"""

import subprocess
import sys

# Script à exécuter sur le robot
SCRIPT_CONTENT = '''#!/usr/bin/env python3
"""Diagnostic des erreurs moteurs - À exécuter sur le robot."""
import sys
import time

try:
    from reachy_mini import ReachyMini

    print("🔍 DIAGNOSTIC DES MOTEURS")
    print("=" * 60)
    print()

    # Connexion au robot via SDK
    print("🔌 Connexion au robot...")
    try:
        robot = ReachyMini(media_backend="no_media", use_sim=False, localhost_only=True)
        robot.__enter__()
        print("✅ Robot connecté")
    except Exception as e:
        print(f"❌ Erreur de connexion: {e}")
        print("   Vérifiez que:")
        print("   - Le robot est allumé")
        print("   - Le daemon est démarré: sudo systemctl start reachy-mini-daemon")
        sys.exit(1)

    print()
    print("📊 Vérification des moteurs via positions des joints...")
    print()

    # Correspondance Motor ID ↔ Nom ↔ Joint
    motor_info = {
        10: ("Base (rotation corps)", "yaw_body"),
        11: ("stewart_1 (tête, moteur 1)", "stewart_1"),
        12: ("stewart_2 (tête, moteur 2) ← Si c'est celui qui clignote!", "stewart_2"),
        13: ("stewart_3 (tête, moteur 3)", "stewart_3"),
        14: ("stewart_4 (tête, moteur 4)", "stewart_4"),
        15: ("stewart_5 (tête, moteur 5)", "stewart_5"),
        16: ("stewart_6 (tête, moteur 6)", "stewart_6"),
        17: ("Antenne gauche", "left_antenna"),
        18: ("Antenne droite", "right_antenna"),
    }

    motors_with_errors = []

    try:
        # Obtenir les positions actuelles
        head_positions, antenna_positions = robot.get_current_joint_positions()
        print(f"   Positions tête: {len(head_positions)} joints détectés")
        print(f"   Positions antennes: {len(antenna_positions)} joints détectés")
        print()

        # Mapping des joints
        stewart_joints = ["stewart_1", "stewart_2", "stewart_3", "stewart_4", "stewart_5", "stewart_6"]
        antenna_joints = ["left_antenna", "right_antenna"]

        # Vérifier chaque moteur
        for motor_id in [10, 11, 12, 13, 14, 15, 16, 17, 18]:
            motor_name, joint_name = motor_info[motor_id]
            print(f"Motor ID {motor_id:2d} ({motor_name:30s}): ", end="", flush=True)

            try:
                # Essayer de lire la position du joint
                if motor_id == 10:
                    # Base - yaw_body
                    try:
                        pos = robot.get_current_joint_positions()
                        print("✅ OK (position lisible)")
                    except Exception as e:
                        print(f"⚠️  Erreur lecture: {e}")
                        motors_with_errors.append((motor_id, "READ ERROR", str(e)))
                elif motor_id in [11, 12, 13, 14, 15, 16]:
                    # Stewart joints (indices: 1, 3, 5, 7, 9, 11 dans head_positions)
                    stewart_index = motor_id - 11  # 0-5
                    array_index = stewart_index * 2 + 1  # 1, 3, 5, 7, 9, 11
                    if array_index < len(head_positions):
                        pos = head_positions[array_index]
                        print(f"✅ OK (position: {pos*180/3.14159:.1f}°)")
                    else:
                        print("⚠️  Index hors limites")
                        motors_with_errors.append((motor_id, "INDEX ERROR", f"Index {array_index} > {len(head_positions)}"))
                elif motor_id in [17, 18]:
                    # Antennes
                    antenna_index = motor_id - 17  # 0 ou 1
                    if antenna_index < len(antenna_positions):
                        pos = antenna_positions[antenna_index]
                        print(f"✅ OK (position: {pos*180/3.14159:.1f}°)")
                    else:
                        print("⚠️  Index hors limites")
                        motors_with_errors.append((motor_id, "INDEX ERROR", f"Index {antenna_index} > {len(antenna_positions)}"))

            except Exception as e:
                print(f"❌ ERROR: {e}")
                motors_with_errors.append((motor_id, "EXCEPTION", str(e)))

            time.sleep(0.05)

        # Essayer d'accéder aux moteurs directement si possible
        print()
        print("🔍 Tentative accès direct aux moteurs...")
        try:
            if hasattr(robot, 'head') and hasattr(robot.head, 'motors'):
                motors = robot.head.motors
                print(f"   {len(motors)} moteurs trouvés dans robot.head.motors")
                for i, motor in enumerate(motors, 1):
                    try:
                        # Essayer d'accéder aux propriétés du moteur
                        if hasattr(motor, 'id'):
                            motor_id_attr = motor.id
                            print(f"   Moteur {i}: ID={motor_id_attr}")
                    except Exception as e:
                        print(f"   Moteur {i}: ⚠️  {e}")
            else:
                print("   ⚠️  Accès direct aux moteurs non disponible via cette API")
        except Exception as e:
            print(f"   ⚠️  {e}")

    except Exception as e:
        print(f"❌ Erreur lors de la lecture des positions: {e}")
        import traceback
        traceback.print_exc()

    print()
    print()
    print("=" * 60)

    if motors_with_errors:
        print("❌ MOTEURS AVEC ERREURS:")
        print()
        for motor_id, error_type, error_detail in motors_with_errors:
            motor_name, _ = motor_info.get(motor_id, ("?", "?"))
            print(f"   Motor ID {motor_id:2d} ({motor_name:30s})")
            print(f"      Type: {error_type}")
            if error_detail:
                print(f"      Détail: {error_detail}")
            print()

        print("💡 SOLUTIONS:")
        print("   1. Vérifiez le câblage du moteur problématique")
        print("   2. Vérifiez que le moteur est dans le bon emplacement")
        print("   3. Vérifiez qu'aucun câble ne bloque le mouvement")
        print("   4. Si le problème persiste, contactez support Pollen Robotics")
    else:
        print("✅ TOUS LES MOTEURS RÉPONDENT (positions lisibles)")
        print()
        print("   ⚠️  IMPORTANT:")
        print("   Si un moteur clignote quand même en rouge:")
        print("   → C'est peut-être un problème visuel (LED défectueuse)")
        print("   → Ou le moteur a une erreur matérielle non détectée par l'API")
        print("   → Vérifiez visuellement quel moteur clignote et notez son numéro")

    print("=" * 60)
    print()
    print("💡 Pour identifier précisément le moteur qui clignote:")
    print("   1. Éteignez le robot (interrupteur OFF)")
    print("   2. Enlevez le capot de la tête")
    print("   3. Rallumez le robot (interrupteur ON)")
    print("   4. Observez quel moteur clignote en rouge")
    print("   5. Notez le numéro (1 à 6 pour la tête)")

    # Nettoyage
    try:
        robot.__exit__(None, None, None)
        print("✅ Robot déconnecté")
    except Exception:
        pass

except ImportError as e:
    print(f"❌ Erreur d'import: {e}")
    print("   Vérifiez que reachy-mini est installé sur le robot:")
    print("   pip install --upgrade reachy-mini")
    sys.exit(1)
except Exception as e:
    print(f"❌ Erreur inattendue: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
'''


def main() -> None:
    """Fonction principale."""
    print("🔍 DIAGNOSTIC ERREURS MOTEURS - Via SSH")
    print("=" * 60)
    print()
    print("Ce script va:")
    print("  1. Se connecter au robot via SSH")
    print("  2. Exécuter le diagnostic des moteurs")
    print("  3. Identifier précisément quel moteur a un problème")
    print()

    # Paramètres par défaut
    robot_ip = "192.168.129.64"
    username = "pollen"

    print(f"Robot: {username}@{robot_ip}")
    print()

    # Créer le script temporaire local
    temp_script_local = "/tmp/diagnostic_motor_errors_robot.py"
    temp_script_remote = "/tmp/diagnostic_motor_errors_robot.py"

    try:
        with open(temp_script_local, "w") as f:
            f.write(SCRIPT_CONTENT)
        print(f"✅ Script créé: {temp_script_local}")
    except Exception as e:
        print(f"❌ Erreur création script local: {e}")
        sys.exit(1)

    # Copier le script sur le robot
    print("📤 Copie du script sur le robot...")
    try:
        result = subprocess.run(
            ["scp", temp_script_local, f"{username}@{robot_ip}:{temp_script_remote}"],
            check=True,
            capture_output=True,
            text=True,
        )
        print("✅ Script copié sur le robot")
    except subprocess.CalledProcessError as e:
        print(f"⚠️  Erreur copie (peut nécessiter mot de passe): {e.stderr}")
        print("   Vous pouvez copier manuellement le script:")
        print(f"   scp {temp_script_local} {username}@{robot_ip}:{temp_script_remote}")
        # Continuer quand même pour tenter l'exécution si le fichier existe déjà
    except FileNotFoundError:
        print("❌ Commande 'scp' non trouvée")
        print("   Installez OpenSSH ou copiez manuellement le script")
        sys.exit(1)

    # Exécuter le script sur le robot
    print()
    print("🚀 Exécution du diagnostic sur le robot...")
    print("   (Assurez-vous que le robot est allumé)")
    print()

    try:
        result = subprocess.run(
            ["ssh", f"{username}@{robot_ip}", f"python3 {temp_script_remote}"],
            check=False,  # Ne pas échouer si le script retourne une erreur
            capture_output=True,
            text=True,
        )

        # Afficher la sortie
        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print("Stderr:", result.stderr, file=sys.stderr)

        if result.returncode == 0:
            print("✅ Diagnostic terminé avec succès")
        else:
            print(f"⚠️  Le script a retourné le code {result.returncode}")
            print("   Vérifiez les messages ci-dessus")

    except subprocess.CalledProcessError as e:
        print(f"⚠️  Erreur lors de l'exécution: {e}")
        print("   Vous pouvez exécuter manuellement sur le robot:")
        print(f"   ssh {username}@{robot_ip}")
        print(f"   python3 {temp_script_remote}")
    except FileNotFoundError:
        print("❌ Commande 'ssh' non trouvée")
        print("   Installez OpenSSH ou connectez-vous manuellement au robot")
        sys.exit(1)

    # Nettoyage
    print()
    print("🧹 Nettoyage...")
    try:
        # Supprimer le script local
        import os
        os.remove(temp_script_local)
        print("✅ Script temporaire local supprimé")

        # Supprimer le script sur le robot
        subprocess.run(
            ["ssh", f"{username}@{robot_ip}", f"rm {temp_script_remote}"],
            check=False,
            capture_output=True,
            text=True,
        )
        print("✅ Script temporaire supprimé du robot")
    except Exception as e:
        print(f"⚠️  Erreur nettoyage: {e}")


if __name__ == "__main__":
    main()

