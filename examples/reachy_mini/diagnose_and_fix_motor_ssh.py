#!/usr/bin/env python3
"""Script de diagnostic et correction moteur - À exécuter directement sur le robot via SSH.

Ce script combine le diagnostic et la correction en un seul script simple à exécuter
directement sur le robot via SSH. Basé sur la solution de squirrel (Discord Pollen Robotics).

Usage (sur le robot via SSH):
    ssh pollen@<ROBOT_IP>
    sudo systemctl stop reachy-mini-daemon
    source /venvs/mini-daemon/bin/activate
    python3 /path/to/diagnose_and_fix_motor_ssh.py

Ou depuis le Mac (si le script est copié sur le robot):
    ssh pollen@<ROBOT_IP> "sudo systemctl stop reachy-mini-daemon && source /venvs/mini-daemon/bin/activate && python3 diagnose_and_fix_motor_ssh.py"
"""

import sys
import time
from pathlib import Path

# Ajouter le chemin src si nécessaire
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

JOINT_TO_MOTOR_ID = {v: k for k, v in MOTOR_ID_TO_JOINT.items()}

BAUDRATE_1M = 1_000_000
BAUDRATE_57K = 57_600

try:
    from reachy_mini_motor_controller import MotorsBus

    MOTOR_CONTROLLER_AVAILABLE = True
except ImportError:
    try:
        from reachy_mini.tools.motor_controller import MotorsBus

        MOTOR_CONTROLLER_AVAILABLE = True
    except ImportError:
        MOTOR_CONTROLLER_AVAILABLE = False
        print("⚠️  Module motor_controller non disponible")
        print("   Installez: pip install reachy-mini-motor-controller")
        sys.exit(1)

try:
    from importlib.resources import files

    import reachy_mini
    from reachy_mini.tools.setup_motor import setup_motor
    from reachy_mini.utils.hardware_config.parser import parse_yaml_config

    TOOLS_AVAILABLE = True
except ImportError:
    TOOLS_AVAILABLE = False
    print("⚠️  Module reachy_mini.tools non disponible")
    print("   Installez: pip install reachy-mini")
    sys.exit(1)


def scan_bus_at_baudrate(serialport: str, baudrate: int) -> list[int]:
    """Scanne le bus Dynamixel à un baudrate spécifique."""
    try:
        bus = MotorsBus(serialport, baudrate=baudrate)
        motors = bus.scan()
        return motors if motors else []
    except Exception as e:
        print(f"   ⚠️  Erreur scan à {baudrate} baud: {e}")
        return []


def diagnose_motors(serialport: str = "/dev/ttyAMA3") -> dict:
    """Diagnostique les moteurs avec mauvais baudrate."""
    print("\n" + "=" * 60)
    print("🔍 DIAGNOSTIC MOTEURS - Scan Bus Dynamixel")
    print("=" * 60)
    print()

    print(f"1️⃣ Scan à 1,000,000 baud (configuration correcte)...")
    motors_1m = scan_bus_at_baudrate(serialport, BAUDRATE_1M)
    print(f"   ✅ Moteurs trouvés: {motors_1m if motors_1m else 'Aucun'}")
    time.sleep(0.5)

    print(f"\n2️⃣ Scan à 57,600 baud (paramètres d'usine)...")
    motors_57k = scan_bus_at_baudrate(serialport, BAUDRATE_57K)
    print(f"   {'⚠️  Moteurs trouvés: ' + str(motors_57k) if motors_57k else '✅ Aucun (normal)'}")

    # Analyser les résultats
    expected_motors = set(MOTOR_ID_TO_JOINT.keys())
    found_motors = set(motors_1m)
    missing_motors = expected_motors - found_motors
    wrong_baudrate_motors = set(motors_57k) - found_motors

    print("\n3️⃣ Analyse des résultats...")
    print()

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


def fix_motor(
    serialport: str, motor_id: int, old_id: int = 1, old_baudrate: int = 57600
) -> bool:
    """Corrige le baudrate d'un moteur spécifique."""
    print(f"\n🔧 Correction Motor ID {motor_id}...")

    try:
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
    print("🔧 DIAGNOSTIC ET CORRECTION MOTEUR - Reachy Mini")
    print("=" * 60)
    print()
    print("Ce script diagnostique et corrige les moteurs avec mauvais baudrate")
    print("(bug décembre 2025 - paramètres d'usine ID=1, baudrate 57,600)")
    print()

    serialport = "/dev/ttyAMA3"

    # Vérifier que le daemon est arrêté
    print("⚠️  IMPORTANT: Assurez-vous que le daemon est arrêté:")
    print("   sudo systemctl stop reachy-mini-daemon")
    print()
    input("Appuyez sur Entrée pour continuer...")

    # Diagnostic
    results = diagnose_motors(serialport)

    # Si des moteurs ont un mauvais baudrate, proposer la correction
    if results["wrong_baudrate_motors"]:
        print("\n" + "=" * 60)
        print("💡 CORRECTION DISPONIBLE")
        print("=" * 60)
        print()
        print(f"⚠️  {len(results['wrong_baudrate_motors'])} moteur(s) nécessite(nt) une correction")
        print()
        confirm = input("Voulez-vous corriger automatiquement? (oui/non): ").strip().lower()
        if confirm in ["oui", "o", "yes", "y"]:
            print()
            success_count = 0
            for motor_id in sorted(results["wrong_baudrate_motors"]):
                if fix_motor(serialport, motor_id, old_id=1, old_baudrate=57600):
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
            print("   2. Relancer le diagnostic pour vérifier")
            print("   3. Tester les mouvements de la tête")
        else:
            print("\n❌ Correction annulée")
    elif results["missing_motors"]:
        print("\n" + "=" * 60)
        print("⚠️  MOTEURS MANQUANTS DÉTECTÉS")
        print("=" * 60)
        print()
        print("Les moteurs manquants peuvent être:")
        print("  1. Avec mauvais baudrate (non détectés à 57.6k non plus)")
        print("  2. Défectueux physiquement")
        print("  3. Problème de câblage")
        print()
        print("💡 Solutions:")
        print("   1. Vérifier le câblage")
        print("   2. Redémarrer le robot (interrupteur OFF/ON)")
        print("   3. Utiliser: reachy-mini-reflash-motors")
        print("   4. Contacter support Pollen Robotics si le problème persiste")
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


if __name__ == "__main__":
    main()

