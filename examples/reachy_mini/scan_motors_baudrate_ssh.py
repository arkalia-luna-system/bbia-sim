#!/usr/bin/env python3
"""Version SSH du script de scan - À exécuter directement sur le robot.

Ce script est optimisé pour être exécuté sur le robot Reachy Mini via SSH.
Il utilise le SDK reachy_mini qui est déjà installé sur le robot.

Usage sur le robot:
    # Copier le script sur le robot
    scp examples/reachy_mini/scan_motors_baudrate_ssh.py pollen@<ROBOT_IP>:/tmp/

    # Se connecter au robot
    ssh pollen@<ROBOT_IP>

    # Exécuter le script
    python3 /tmp/scan_motors_baudrate_ssh.py
"""

import sys

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

try:
    from reachy_mini import ReachyMini

    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    print("❌ SDK reachy_mini non disponible")
    print("   Installez: pip install reachy-mini")
    sys.exit(1)


def diagnose_motors_via_sdk() -> dict:
    """Diagnostique les moteurs manquants via le SDK reachy_mini.

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
            "motors_57k": [],
            "missing_motors": missing_motors,
            "wrong_baudrate_motors": missing_motors,
            "expected_motors": list(MOTOR_ID_TO_JOINT.keys()),
            "found_motors": detected_motors,
            "missing_joints": missing_joints,
        }

    except Exception as e:
        error_msg = str(e)
        print("❌ Erreur lors de la connexion au robot")
        print(f"   Détail: {error_msg}")
        print()
        print("💡 CAUSES POSSIBLES:")
        print()
        print("1️⃣ Le daemon n'est pas démarré")
        print("   Solution:")
        print("      sudo systemctl start reachy-mini-daemon")
        print()
        print("2️⃣ Le robot n'est pas accessible")
        print("   Vérifiez que le robot est allumé (interrupteur ON)")
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


def main() -> None:
    """Fonction principale."""
    print("🔍 SCAN MOTEURS - Reachy Mini (Version SSH)")
    print("=" * 60)
    print()
    print("Ce script détecte les moteurs avec mauvais baudrate (bug décembre 2025)")
    print("Problème: Moteurs avec paramètres d'usine (ID=1, baudrate 57,600)")
    print("Solution: Reconfiguration vers ID correct, baudrate 1,000,000")
    print()

    if not SDK_AVAILABLE:
        print("❌ SDK reachy_mini non disponible")
        print("   Installez: pip install reachy-mini")
        sys.exit(1)

    # Diagnostic
    print("🔄 Utilisation de la méthode SDK (analyse des positions des joints)...")
    print()
    results = diagnose_motors_via_sdk()

    # Afficher les résultats
    if results.get("connection_error"):
        # Erreur de connexion déjà gérée
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
        print("1️⃣ Script officiel (recommandé):")
        print("   reachy-mini-reflash-motors")
        print()
        print("2️⃣ Reconfiguration manuelle (solution squirrel - Discord):")
        print("   sudo systemctl stop reachy-mini-daemon")
        print("   source /venvs/mini-daemon/bin/activate")
        print("   python3 -m reachy_mini.tools.setup_motor")
        print()
        print("3️⃣ Scan du bus à différents baudrates:")
        print("   sudo systemctl stop reachy-mini-daemon")
        print("   python3 << 'EOF'")
        print("   from reachy_mini_motor_controller import MotorsBus")
        print("   bus = MotorsBus('/dev/ttyAMA3', baudrate=1_000_000)")
        print("   motors_1M = bus.scan()")
        print("   print(f'Moteurs à 1M baud: {motors_1M}')")
        print("   bus = MotorsBus('/dev/ttyAMA3', baudrate=57_600)")
        print("   motors_57k = bus.scan()")
        print("   print(f'Moteurs à 57.6k baud: {motors_57k}')")
        print("   EOF")
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

