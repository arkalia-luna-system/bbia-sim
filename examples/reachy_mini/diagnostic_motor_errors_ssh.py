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

import sys
import time
import subprocess
from pathlib import Path

# Script à exécuter sur le robot
SCRIPT_CONTENT = '''#!/usr/bin/env python3
"""Diagnostic des erreurs moteurs - À exécuter sur le robot."""
import time

try:
    from reachy_mini.io.motor_controller import MotorController
    from reachy_mini.io.serial_connection import SerialConnection
    
    print("🔍 DIAGNOSTIC DES MOTEURS")
    print("=" * 60)
    print()
    
    # Ports série possibles
    ports_to_try = ["/dev/ttyAMA3", "/dev/ttyUSB0", "/dev/ttyACM0"]
    controller = None
    port_used = None
    
    for port in ports_to_try:
        try:
            print(f"Tentative connexion sur {port}...")
            connection = SerialConnection(port=port, baudrate=1000000)
            controller = MotorController(connection=connection)
            port_used = port
            print(f"✅ Connecté sur {port}")
            break
        except Exception as e:
            print(f"   ❌ {port}: {e}")
            continue
    
    if controller is None:
        print("❌ Impossible de se connecter à aucun port série")
        print("   Vérifiez que le robot est allumé et que le daemon est arrêté")
        sys.exit(1)
    
    print()
    print("📊 Vérification des moteurs...")
    print()
    
    # Correspondance Motor ID ↔ Nom
    motor_names = {
        10: "Base (rotation corps)",
        11: "stewart_1 (tête, moteur 1)",
        12: "stewart_2 (tête, moteur 2) ← Si c'est celui qui clignote!",
        13: "stewart_3 (tête, moteur 3)",
        14: "stewart_4 (tête, moteur 4)",
        15: "stewart_5 (tête, moteur 5)",
        16: "stewart_6 (tête, moteur 6)",
        17: "Antenne gauche",
        18: "Antenne droite",
    }
    
    motors_with_errors = []
    
    for motor_id in [10, 11, 12, 13, 14, 15, 16, 17, 18]:
        try:
            print(f"Motor ID {motor_id:2d} ({motor_names.get(motor_id, '?'):30s}): ", end="", flush=True)
            
            # Ping le moteur
            try:
                result = controller.ping(motor_id)
                if not result:
                    print("❌ NO RESPONSE")
                    motors_with_errors.append((motor_id, "NO RESPONSE", None))
                    continue
            except Exception as e:
                print(f"❌ PING ERROR: {e}")
                motors_with_errors.append((motor_id, "PING ERROR", str(e)))
                continue
            
            # Lit les erreurs hardware
            try:
                errors = controller.read_hardware_error(motor_id)
                if errors:
                    print(f"⚠️  ERRORS: {errors}")
                    motors_with_errors.append((motor_id, "HARDWARE ERROR", errors))
                else:
                    print("✅ OK")
            except Exception as e:
                print(f"⚠️  Cannot read errors: {e}")
                # Moteur répond mais on ne peut pas lire les erreurs
                print("   (Moteur répond mais erreur de lecture)")
            
            time.sleep(0.1)
            
        except Exception as e:
            print(f"❌ ERROR: {e}")
            motors_with_errors.append((motor_id, "EXCEPTION", str(e)))
            time.sleep(0.1)
    
    print()
    print("=" * 60)
    
    if motors_with_errors:
        print("❌ MOTEURS AVEC ERREURS:")
        print()
        for motor_id, error_type, error_detail in motors_with_errors:
            print(f"   Motor ID {motor_id:2d} ({motor_names.get(motor_id, '?'):30s})")
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
        print("✅ TOUS LES MOTEURS SONT OK")
        print()
        print("   Si un moteur clignote quand même en rouge:")
        print("   → C'est peut-être un problème visuel (LED défectueuse)")
        print("   → Ou le moteur a une erreur temporaire qui se résout au redémarrage")
    
    print("=" * 60)
    
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
    print(f"📤 Copie du script sur le robot...")
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

