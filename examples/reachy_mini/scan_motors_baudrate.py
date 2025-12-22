#!/usr/bin/env python3
"""Script de scan automatique du bus Dynamixel - Détecte les moteurs avec mauvais baudrate.

Ce script scanne le bus Dynamixel à différents baudrates pour identifier les moteurs
qui sont encore configurés avec les paramètres d'usine (ID=1, baudrate 57,600) au lieu
de la configuration correcte (ID=11-18, baudrate 1,000,000).

Usage:
    # Scan automatique (détecte le port série)
    python examples/reachy_mini/scan_motors_baudrate.py

    # Scan avec port série spécifié
    python examples/reachy_mini/scan_motors_baudrate.py --serialport /dev/ttyAMA3

    # Scan + correction automatique
    python examples/reachy_mini/scan_motors_baudrate.py --auto-fix

Basé sur la solution de squirrel (Discord Pollen Robotics - 20/12/2025):
- Scan à 1,000,000 baud → trouve tous les moteurs correctement configurés
- Scan à 57,600 baud → trouve les moteurs avec paramètres d'usine (ID=1)
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

# Mapping Motor ID → Nom joint
MOTOR_ID_TO_JOINT = {
    10: "yaw_body",
    11: "stewart_1",
    12: "stewart_2",
    13: "stewart_3",  # ← Le moteur problématique dans ton cas
    14: "stewart_4",
    15: "stewart_5",
    16: "stewart_6",
    17: "left_antenna",
    18: "right_antenna",
}

# Mapping inverse: Joint → Motor ID attendu
JOINT_TO_MOTOR_ID = {v: k for k, v in MOTOR_ID_TO_JOINT.items()}

# Baudrates à tester
BAUDRATE_1M = 1_000_000  # Configuration correcte
BAUDRATE_57K = 57_600  # Paramètres d'usine (bug)

try:
    # Essayer d'importer depuis reachy_mini_motor_controller
    try:
        from reachy_mini_motor_controller import MotorsBus

        MOTOR_CONTROLLER_AVAILABLE = True
    except ImportError:
        # Essayer depuis reachy_mini.tools
        try:
            from reachy_mini.tools.motor_controller import MotorsBus

            MOTOR_CONTROLLER_AVAILABLE = True
        except ImportError:
            MOTOR_CONTROLLER_AVAILABLE = False

    # Imports SDK pour détection port série
    try:
        from reachy_mini.daemon.utils import find_serial_port

        SDK_AVAILABLE = True
    except ImportError:
        SDK_AVAILABLE = False
        find_serial_port = None

except ImportError as e:
    MOTOR_CONTROLLER_AVAILABLE = False
    SDK_AVAILABLE = False
    IMPORT_ERROR: str = str(e)


def scan_bus_at_baudrate(serialport: str, baudrate: int) -> list[int]:
    """Scanne le bus Dynamixel à un baudrate spécifique.

    Args:
        serialport: Port série du robot (e.g. /dev/ttyAMA3)
        baudrate: Baudrate à utiliser pour le scan

    Returns:
        Liste des IDs de moteurs trouvés
    """
    if not MOTOR_CONTROLLER_AVAILABLE:
        print("   ⚠️  Impossible de scanner (module motor_controller non disponible)")
        return []

    try:
        bus = MotorsBus(serialport, baudrate=baudrate)
        motors = bus.scan()
        return motors if motors else []
    except Exception as e:
        print(f"   ⚠️  Erreur scan à {baudrate} baud: {e}")
        return []


def diagnose_motors_via_sdk() -> dict[str, Any]:
    """Diagnostique les moteurs manquants via le SDK reachy_mini (méthode alternative).

    Cette méthode utilise get_current_joint_positions() pour détecter les moteurs manquants
    sans nécessiter motor_controller.

    Returns:
        Dictionnaire avec résultats du diagnostic
    """
    print("\n" + "=" * 60)
    print("🔍 DIAGNOSTIC MOTEURS - Via SDK reachy_mini")
    print("=" * 60)
    print()
    print("Méthode: Analyse des positions des joints pour détecter les moteurs manquants")
    print()

    try:
        from reachy_mini import ReachyMini

        print("1️⃣ Connexion au robot...")
        robot = ReachyMini(
            media_backend="no_media",
            use_sim=False,
            localhost_only=False,  # ← CRITIQUE pour connexion réseau (Wireless)
            timeout=5.0,
        )
        robot.__enter__()
        print("   ✅ Robot connecté")
        print()

        print("2️⃣ Lecture des positions des joints...")
        head_positions, antenna_positions = robot.get_current_joint_positions()
        print(f"   Positions tête: {len(head_positions)} joints détectés")
        print(f"   Positions antennes: {len(antenna_positions)} joints détectés")
        print()

        # Analyser les résultats
        expected_stewart_count = 6
        expected_antenna_count = 2

        # Détecter les joints manquants
        missing_joints = []
        detected_motors = []

        # Vérifier les joints stewart
        if len(head_positions) < expected_stewart_count:
            missing_count = expected_stewart_count - len(head_positions)
            print(f"⚠️  MOTEURS STEWART MANQUANTS: {missing_count}/{expected_stewart_count}")
            print(f"   Seulement {len(head_positions)} joints détectés au lieu de {expected_stewart_count}")
            print()
            print("   Joints manquants probables:")
            for i in range(len(head_positions) + 1, expected_stewart_count + 1):
                joint_name = f"stewart_{i}"
                motor_id = JOINT_TO_MOTOR_ID.get(joint_name, "?")
                missing_joints.append(joint_name)
                print(f"   - {joint_name} (Motor ID {motor_id})")
            print()
            print("   💡 CAUSE PROBABLE:")
            print("      → Moteur avec paramètres d'usine (ID=1, baudrate 57,600)")
            print("      → Au lieu de configuration correcte (ID=11-16, baudrate 1,000,000)")
            print()
        else:
            print(f"✅ Tous les joints stewart détectés ({len(head_positions)}/{expected_stewart_count})")
            for i in range(1, 7):
                motor_id = 10 + i  # 11-16
                detected_motors.append(motor_id)

        # Vérifier les antennes
        if len(antenna_positions) < expected_antenna_count:
            missing_count = expected_antenna_count - len(antenna_positions)
            print(f"⚠️  ANTENNES MANQUANTES: {missing_count}/{expected_antenna_count}")
            for i in range(len(antenna_positions), expected_antenna_count):
                joint_name = ["left_antenna", "right_antenna"][i]
                motor_id = JOINT_TO_MOTOR_ID.get(joint_name, "?")
                missing_joints.append(joint_name)
                print(f"   - {joint_name} (Motor ID {motor_id})")
            print()
        else:
            print(f"✅ Toutes les antennes détectées ({len(antenna_positions)}/{expected_antenna_count})")
            detected_motors.extend([17, 18])

        # Vérifier yaw_body (base)
        try:
            if hasattr(robot, "get_current_body_yaw"):
                robot.get_current_body_yaw()
                print("✅ yaw_body (base) détecté")
                detected_motors.append(10)
            else:
                print("⚠️  yaw_body: impossible de vérifier")
        except Exception:
            print("⚠️  yaw_body: erreur de lecture")

        robot.__exit__(None, None, None)

        # Convertir les joints manquants en IDs de moteurs
        missing_motors = [JOINT_TO_MOTOR_ID.get(j, None) for j in missing_joints]
        missing_motors = [m for m in missing_motors if m is not None]

        return {
            "motors_1m": detected_motors,
            "motors_57k": [],  # Impossible à détecter sans motor_controller
            "missing_motors": missing_motors,
            "wrong_baudrate_motors": missing_motors,  # Probablement le même problème
            "expected_motors": list(MOTOR_ID_TO_JOINT.keys()),
            "found_motors": detected_motors,
            "missing_joints": missing_joints,
        }

    except ImportError:
        print("❌ SDK reachy_mini non disponible")
        print("   Installez: pip install reachy-mini")
        return {
            "motors_1m": [],
            "motors_57k": [],
            "missing_motors": [],
            "wrong_baudrate_motors": [],
            "expected_motors": list(MOTOR_ID_TO_JOINT.keys()),
            "found_motors": [],
            "missing_joints": [],
        }
    except Exception as e:
        error_msg = str(e)
        print("❌ Erreur lors de la connexion au robot")
        print(f"   Détail: {error_msg}")
        print()
        print("💡 CAUSES POSSIBLES:")
        print()
        print("1️⃣ Le daemon n'est pas démarré sur le robot")
        print("   Solution (sur le robot via SSH):")
        print("      ssh pollen@192.168.129.64")
        print("      sudo systemctl start reachy-mini-daemon")
        print()
        print("2️⃣ Le robot n'est pas accessible depuis ce Mac")
        print("   Vérifiez:")
        print("      - Le robot est allumé (interrupteur ON)")
        print("      - Le robot est sur le même réseau WiFi")
        print("      - Le robot est accessible: ping 192.168.129.64")
        print()
        print("3️⃣ Le robot est en mode USB (Lite) et non connecté")
        print("   Vérifiez la connexion USB")
        print()
        print("💡 ALTERNATIVE: Utiliser le script officiel directement sur le robot")
        print("   ssh pollen@192.168.129.64")
        print("   reachy-mini-reflash-motors")
        print()
        return {
            "motors_1m": [],
            "motors_57k": [],
            "missing_motors": [],
            "wrong_baudrate_motors": [],
            "expected_motors": list(MOTOR_ID_TO_JOINT.keys()),
            "found_motors": [],
            "missing_joints": [],
            "connection_error": True,
        }


def diagnose_motors_baudrate(serialport: str | None = None) -> dict[str, Any]:
    """Diagnostique les moteurs avec mauvais baudrate.

    Args:
        serialport: Port série du robot (optionnel, utilisé seulement si motor_controller disponible)

    Returns:
        Dictionnaire avec résultats du diagnostic:
        - motors_1m: Liste des IDs trouvés à 1M baud
        - motors_57k: Liste des IDs trouvés à 57.6k baud
        - missing_motors: Liste des joints manquants
        - wrong_baudrate_motors: Liste des moteurs avec mauvais baudrate
    """
    # Si motor_controller n'est pas disponible, utiliser la méthode SDK
    if not MOTOR_CONTROLLER_AVAILABLE:
        print("\n⚠️  Module motor_controller non disponible")
        print("   Utilisation de la méthode alternative (SDK reachy_mini)")
        print()
        return diagnose_motors_via_sdk()

    # Méthode originale avec scan direct du bus
    print("\n" + "=" * 60)
    print("🔍 SCAN BUS DYNAMIXEL - Détection moteurs mal configurés")
    print("=" * 60)
    print()
    if serialport:
        print(f"Port série: {serialport}")
    else:
        print("Port série: (détection automatique)")
    print()

    # Scan à 1,000,000 baud (configuration correcte)
    print("1️⃣ Scan à 1,000,000 baud (configuration correcte)...")
    if serialport:
        motors_1m = scan_bus_at_baudrate(serialport, BAUDRATE_1M)
    else:
        motors_1m = []
        print("   ⚠️  Port série requis pour le scan direct")
    print(f"   ✅ Moteurs trouvés: {motors_1m if motors_1m else 'Aucun'}")
    time.sleep(0.5)

    # Scan à 57,600 baud (paramètres d'usine)
    print("\n2️⃣ Scan à 57,600 baud (paramètres d'usine)...")
    if serialport:
        motors_57k = scan_bus_at_baudrate(serialport, BAUDRATE_57K)
    else:
        motors_57k = []
        print("   ⚠️  Port série requis pour le scan direct")
    print(f"   {'⚠️  Moteurs trouvés: ' + str(motors_57k) if motors_57k else '✅ Aucun (normal)'}")

    # Analyser les résultats
    print("\n3️⃣ Analyse des résultats...")
    print()

    # Moteurs attendus (tous les IDs de 10 à 18)
    expected_motors = set(MOTOR_ID_TO_JOINT.keys())
    found_motors = set(motors_1m)

    # Moteurs manquants
    missing_motors = expected_motors - found_motors

    # Moteurs avec mauvais baudrate (trouvés à 57.6k mais pas à 1M)
    wrong_baudrate_motors = set(motors_57k) - found_motors

    # Afficher les résultats
    if missing_motors:
        print("❌ MOTEURS MANQUANTS (non détectés à 1M baud):")
        for motor_id in sorted(missing_motors):
            joint_name = MOTOR_ID_TO_JOINT.get(motor_id, "?")
            print(f"   - Motor ID {motor_id:2d} ({joint_name})")
        print()

    if wrong_baudrate_motors:
        print("⚠️  MOTEURS AVEC MAUVAIS BAUDRATE (détectés à 57.6k baud):")
        for motor_id in sorted(wrong_baudrate_motors):
            joint_name = MOTOR_ID_TO_JOINT.get(motor_id, "?")
            print(f"   - Motor ID {motor_id:2d} ({joint_name})")
            print("     → Probablement configuré avec ID=1 et baudrate 57,600")
            print(f"     → Doit être reconfiguré: ID={motor_id}, baudrate 1,000,000")
        print()

    if not missing_motors and not wrong_baudrate_motors:
        print("✅ TOUS LES MOTEURS SONT CORRECTEMENT CONFIGURÉS")
        print(f"   {len(found_motors)}/{len(expected_motors)} moteurs détectés à 1M baud")
        print()

    return {
        "motors_1m": motors_1m,
        "motors_57k": motors_57k,
        "missing_motors": list(missing_motors),
        "wrong_baudrate_motors": list(wrong_baudrate_motors),
        "expected_motors": list(expected_motors),
        "found_motors": list(found_motors),
    }


def fix_motor_baudrate(
    serialport: str, motor_id: int, old_id: int = 1, old_baudrate: int = 57600
) -> bool:
    """Corrige le baudrate d'un moteur spécifique.

    Args:
        serialport: Port série du robot
        motor_id: ID cible du moteur
        old_id: Ancien ID du moteur (par défaut 1)
        old_baudrate: Ancien baudrate (par défaut 57,600)

    Returns:
        True si succès, False sinon
    """
    print(f"\n🔧 Correction Motor ID {motor_id}...")

    try:
        # Importer les outils de configuration
        try:
            from importlib.resources import files

            import reachy_mini
            from reachy_mini.tools.setup_motor import setup_motor
            from reachy_mini.utils.hardware_config.parser import parse_yaml_config
        except ImportError:
            print("   ❌ Module reachy_mini.tools.setup_motor non disponible")
            print("   💡 Utilisez: reachy-mini-reflash-motors")
            return False

        # Charger la configuration hardware
        config_file_path = str(
            files(reachy_mini).joinpath("assets/config/hardware_config.yaml")
        )
        config = parse_yaml_config(config_file_path)

        # Trouver le nom du joint correspondant
        joint_name = MOTOR_ID_TO_JOINT.get(motor_id)
        if not joint_name:
            print(f"   ❌ Joint inconnu pour Motor ID {motor_id}")
            return False

        # Trouver la configuration du moteur
        if joint_name not in config.motors:
            print(f"   ❌ Moteur '{joint_name}' non trouvé dans la configuration")
            return False

        motor_config = config.motors[joint_name]

        print(f"   - Joint: {joint_name}")
        print(f"   - Ancien ID: {old_id} (paramètres d'usine)")
        print(f"   - Nouveau ID: {motor_id}")
        print(f"   - Ancien baudrate: {old_baudrate}")
        print(f"   - Nouveau baudrate: {BAUDRATE_1M}")

        # Reconfigurer le moteur
        setup_motor(
            motor_config,
            serialport,
            from_id=old_id,
            from_baudrate=old_baudrate,
            target_baudrate=BAUDRATE_1M,
        )

        print(f"   ✅ Motor ID {motor_id} reconfiguré avec succès")
        return True

    except Exception as e:
        print(f"   ❌ Erreur lors de la correction: {e}")
        import traceback

        traceback.print_exc()
        return False


def main() -> None:
    """Fonction principale."""
    parser = argparse.ArgumentParser(
        description="Scan automatique du bus Dynamixel - Détecte les moteurs avec mauvais baudrate",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  # Scan automatique (détecte le port série)
  python examples/reachy_mini/scan_motors_baudrate.py

  # Scan avec port série spécifié
  python examples/reachy_mini/scan_motors_baudrate.py --serialport /dev/ttyAMA3

  # Scan + correction automatique
  python examples/reachy_mini/scan_motors_baudrate.py --auto-fix

Note: Ce script nécessite:
  - Accès au port série (permissions)
  - SDK reachy_mini installé (pour correction)
  - Daemon arrêté: sudo systemctl stop reachy-mini-daemon
        """,
    )
    parser.add_argument(
        "--serialport",
        type=str,
        default=None,
        help="Port série du robot (e.g. /dev/ttyAMA3). Si non spécifié, détection automatique.",
    )
    parser.add_argument(
        "--auto-fix",
        action="store_true",
        help="Corrige automatiquement les moteurs avec mauvais baudrate",
    )

    args = parser.parse_args()

    print("🔍 SCAN BUS DYNAMIXEL - Reachy Mini")
    print("=" * 60)
    print()
    print("Ce script détecte les moteurs avec mauvais baudrate (bug décembre 2025)")
    print("Problème: Moteurs avec paramètres d'usine (ID=1, baudrate 57,600)")
    print("Solution: Reconfiguration vers ID correct, baudrate 1,000,000")
    print()

    # Vérifier les dépendances
    use_sdk_method = not MOTOR_CONTROLLER_AVAILABLE

    if use_sdk_method:
        print("⚠️  Module motor_controller non disponible")
        error_msg = globals().get("IMPORT_ERROR", "ImportError")
        print(f"   Erreur: {error_msg}")
        print()
        print("💡 Le script utilisera la méthode alternative (SDK reachy_mini)")
        print("   Cette méthode ne nécessite pas de port série")
        print()
    else:
        print("✅ Module motor_controller disponible")
        print("   Le script utilisera le scan direct du bus Dynamixel")
        print()

    # Détection du port série (seulement si motor_controller disponible)
    serialport = args.serialport
    if not use_sdk_method and serialport is None:
        if not SDK_AVAILABLE or find_serial_port is None:
            print("⚠️  Impossible de détecter automatiquement le port série")
            print("   Passage à la méthode alternative (SDK reachy_mini)")
            print()
            use_sdk_method = True
        else:
            print("🔍 Détection automatique du port série...")
            try:
                # Demander version (Lite ou Wireless)
                print("Quelle version de Reachy Mini utilisez-vous?")
                print("  1. Lite (USB)")
                print("  2. Wireless (WiFi)")
                choice = input("Choix (1 ou 2): ").strip()
                wireless_choice = choice == "2"

                ports = find_serial_port(wireless_version=wireless_choice)

                if len(ports) == 0:
                    print("⚠️  Aucun port série Reachy Mini trouvé")
                    print("   Passage à la méthode alternative (SDK reachy_mini)")
                    print()
                    use_sdk_method = True
                elif len(ports) > 1:
                    print(f"⚠️  Plusieurs ports trouvés: {ports}")
                    print("   Passage à la méthode alternative (SDK reachy_mini)")
                    print()
                    use_sdk_method = True
                else:
                    serialport = ports[0]
                    print(f"✅ Port série trouvé: {serialport}")
                    print()

            except Exception as e:
                print(f"⚠️  Erreur détection port série: {e}")
                print("   Passage à la méthode alternative (SDK reachy_mini)")
                print()
                use_sdk_method = True

    # Diagnostic
    if use_sdk_method:
        print("🔄 Utilisation de la méthode SDK (analyse des positions des joints)...")
        print()
        results = diagnose_motors_via_sdk()
    else:
        results = diagnose_motors_baudrate(serialport)

    # Correction automatique si demandée
    if args.auto_fix and results.get("wrong_baudrate_motors"):
        print("\n" + "=" * 60)
        print("🔧 CORRECTION AUTOMATIQUE")
        print("=" * 60)
        print()

        if use_sdk_method:
            print("⚠️  La correction automatique nécessite le port série")
            print("   La méthode SDK peut seulement détecter, pas corriger")
            print()
            print("💡 Pour corriger, vous devez:")
            print("   1. Utiliser le script officiel: reachy-mini-reflash-motors")
            print("   2. OU spécifier le port série: --serialport /dev/ttyAMA3")
            print("   3. OU reconfigurer manuellement via SSH")
            print()
        else:
            if not serialport:
                print("❌ Port série requis pour la correction")
                print("   Spécifiez avec --serialport /dev/ttyAMA3")
                sys.exit(1)

            print("⚠️  ATTENTION: Cette opération va reconfigurer les moteurs!")
            print("   Assurez-vous d'avoir:")
            print("   1. Arrêté le daemon: sudo systemctl stop reachy-mini-daemon")
            print("   2. Accès au port série")
            print()
            confirm = input("Continuer? (oui/non): ").strip().lower()
            if confirm not in ["oui", "o", "yes", "y"]:
                print("❌ Opération annulée")
                sys.exit(0)

            print()
            success_count = 0
            for motor_id in sorted(results["wrong_baudrate_motors"]):
                if fix_motor_baudrate(serialport, motor_id, old_id=1, old_baudrate=57600):
                    success_count += 1
                time.sleep(1)

            print("\n" + "=" * 60)
            if success_count == len(results["wrong_baudrate_motors"]):
                print("✅ TOUS LES MOTEURS CORRIGÉS")
            else:
                print(f"⚠️  {success_count}/{len(results['wrong_baudrate_motors'])} moteurs corrigés")
            print("=" * 60)
            print()
            print("💡 Prochaines étapes:")
            print("   1. Redémarrer le daemon: sudo systemctl start reachy-mini-daemon")
            print("   2. Relancer le scan pour vérifier: python examples/reachy_mini/scan_motors_baudrate.py")
            print("   3. Tester les mouvements de la tête")

    elif results.get("connection_error"):
        # Erreur de connexion déjà gérée dans diagnose_motors_via_sdk
        pass
    elif results.get("wrong_baudrate_motors") or results.get("missing_motors"):
        print("\n" + "=" * 60)
        print("💡 SOLUTION")
        print("=" * 60)
        print()

        if results.get("missing_joints"):
            print("❌ MOTEURS MANQUANTS DÉTECTÉS:")
            for joint in results["missing_joints"]:
                motor_id = JOINT_TO_MOTOR_ID.get(joint, "?")
                print(f"   - {joint} (Motor ID {motor_id})")
            print()

        print("Pour corriger, utilisez l'une de ces méthodes:")
        print()
        print("1️⃣ Script officiel (recommandé - à exécuter sur le robot):")
        print("   ssh pollen@192.168.129.64")
        print("   reachy-mini-reflash-motors")
        print()
        print("2️⃣ Script avec port série (nécessite accès SSH au robot):")
        if serialport:
            print(f"   python examples/reachy_mini/scan_motors_baudrate.py --serialport {serialport} --auto-fix")
        else:
            print("   # Sur le robot (SSH):")
            print("   sudo systemctl stop reachy-mini-daemon")
            print("   python3 examples/reachy_mini/scan_motors_baudrate.py --serialport /dev/ttyAMA3 --auto-fix")
        print()
        print("3️⃣ Reconfiguration manuelle via SSH (solution squirrel - Discord):")
        print("   ssh pollen@192.168.129.64")
        print("   sudo systemctl stop reachy-mini-daemon")
        print("   source /venvs/mini-daemon/bin/activate")
        print("   python3 -m reachy_mini.tools.setup_motor")
        print()
    else:
        print("\n" + "=" * 60)
        print("✅ TOUS LES MOTEURS SONT DÉTECTÉS")
        print("=" * 60)
        print()
        print("Aucun problème détecté avec les moteurs.")
        print("Si un moteur clignote quand même en rouge:")
        print("  → Vérifiez visuellement quel moteur clignote")
        print("  → Vérifiez le câblage")
        print("  → Redémarrez le robot (interrupteur OFF/ON)")
        print()


if __name__ == "__main__":
    main()

