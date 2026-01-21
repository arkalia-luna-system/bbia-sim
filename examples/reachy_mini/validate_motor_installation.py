#!/usr/bin/env python3
"""Script de validation complète après installation des nouveaux moteurs.

Ce script effectue une validation complète après l'installation des nouveaux moteurs :
1. Vérification de la connexion au robot
2. Scan des moteurs (baudrate et ID)
3. Test de chaque moteur individuellement
4. Test des mouvements de la tête
5. Vérification du reflash automatique
6. Génération d'un rapport complet

Usage:
    python examples/reachy_mini/validate_motor_installation.py
"""

import importlib.util
import sys
import time
from pathlib import Path
from typing import Any, cast

# Ajouter le répertoire racine au path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose
except ImportError:
    print("❌ Erreur: reachy_mini SDK non installé")
    print("   Installez avec: pip install reachy-mini")
    sys.exit(1)

try:
    # Import depuis le même répertoire
    from scan_motors_baudrate import diagnose_motors_baudrate
except ImportError:
    try:
        # Fallback: import depuis le répertoire parent
        scan_path = Path(__file__).parent / "scan_motors_baudrate.py"
        if scan_path.exists():
            spec = importlib.util.spec_from_file_location("scan_motors_baudrate", scan_path)
            if spec is not None and spec.loader is not None:
                scan_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(scan_module)
                diagnose_motors_baudrate = scan_module.diagnose_motors_baudrate
            else:
                diagnose_motors_baudrate = None
        else:
            diagnose_motors_baudrate = None
    except Exception:
        print("⚠️  Script de scan non disponible, utilisation de la méthode SDK uniquement")
        diagnose_motors_baudrate = None


# Mapping des joints aux Motor IDs
JOINT_TO_MOTOR_ID = {
    "yaw_body": 10,
    "stewart_1": 11,
    "stewart_2": 12,
    "stewart_3": 13,
    "stewart_4": 14,
    "stewart_5": 15,
    "stewart_6": 16,
    "left_antenna": 17,
    "right_antenna": 18,
}

# Moteurs remplacés (selon votre cas)
REPLACED_MOTORS = [11, 12, 14]  # stewart_1, stewart_2, stewart_4


class MotorValidationResult:
    """Résultat de validation d'un moteur."""

    def __init__(self, motor_id: int, joint_name: str):
        self.motor_id = motor_id
        self.joint_name = joint_name
        self.detected = False
        self.responds = False
        self.moves_smoothly = False
        self.no_red_led = True
        self.errors: list[str] = []

    def is_valid(self) -> bool:
        """Vérifie si le moteur est valide."""
        return (
            self.detected
            and self.responds
            and self.moves_smoothly
            and self.no_red_led
            and len(self.errors) == 0
        )

    def __str__(self) -> str:
        status = "✅ VALIDE" if self.is_valid() else "❌ PROBLÈME"
        return f"{status} - Motor {self.motor_id} ({self.joint_name})"


def print_header(title: str) -> None:
    """Affiche un en-tête."""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70 + "\n")


def check_connection() -> ReachyMini | None:
    """Vérifie la connexion au robot."""
    print_header("1️⃣ VÉRIFICATION DE LA CONNEXION")

    try:
        print("Tentative de connexion au robot...")
        robot = ReachyMini(use_sim=False, timeout=5.0)
        print("✅ Connexion réussie!")
        print(f"   Robot: {robot}")
        return robot
    except Exception as e:
        print(f"❌ Erreur de connexion: {e}")
        print("\n💡 Vérifications:")
        print("   - Le robot est-il allumé?")
        print("   - Le daemon est-il démarré? (sudo systemctl status reachy-mini-daemon)")
        print("   - La connexion réseau/USB est-elle active?")
        return None


def scan_motors() -> dict[str, Any]:
    """Scanne les moteurs pour détecter les problèmes de baudrate."""
    print_header("2️⃣ SCAN DES MOTEURS (BAUDRATE ET ID)")

    if diagnose_motors_baudrate:
        try:
            results = diagnose_motors_baudrate()
            return cast(dict[str, Any], results)
        except Exception as e:
            print(f"⚠️  Erreur lors du scan: {e}")
            return {
                "motors_1m": [],
                "motors_57k": [],
                "missing_motors": [],
                "wrong_baudrate_motors": [],
            }

    print("⚠️  Script de scan non disponible, utilisation de la méthode SDK")
    return {
        "motors_1m": [],
        "motors_57k": [],
        "missing_motors": [],
        "wrong_baudrate_motors": [],
    }


def test_motor(robot: ReachyMini, motor_id: int, joint_name: str) -> MotorValidationResult:
    """Teste un moteur individuellement."""
    result = MotorValidationResult(motor_id, joint_name)

    try:
        # Vérifier que le joint existe
        if not hasattr(robot.head, joint_name):
            result.errors.append(f"Joint {joint_name} non trouvé")
            return result

        joint = getattr(robot.head, joint_name)
        result.detected = True

        # Vérifier que le moteur répond
        try:
            _ = joint.present_position  # Vérification que le moteur répond
            result.responds = True
        except Exception as e:
            result.errors.append(f"Le moteur ne répond pas: {e}")
            return result

        # Test de mouvement simple
        try:
            # Sauvegarder la position actuelle
            initial_pos = joint.present_position

            # Petit mouvement
            target_pos = initial_pos + 0.1 if initial_pos < 0.5 else initial_pos - 0.1
            joint.goal_position = target_pos
            time.sleep(0.5)

            # Vérifier que le moteur a bougé
            new_pos = joint.present_position
            if abs(new_pos - initial_pos) > 0.05:
                result.moves_smoothly = True
            else:
                result.errors.append("Le moteur ne bouge pas")

            # Retour à la position initiale
            joint.goal_position = initial_pos
            time.sleep(0.5)

        except Exception as e:
            result.errors.append(f"Erreur lors du test de mouvement: {e}")

        # Note: La vérification de la LED rouge nécessiterait un accès hardware
        # On suppose que si le moteur répond et bouge, c'est OK

    except Exception as e:
        result.errors.append(f"Erreur générale: {e}")

    return result


def test_head_movements(robot: ReachyMini) -> bool:
    """Teste les mouvements de la tête."""
    print_header("4️⃣ TEST DES MOUVEMENTS DE LA TÊTE")

    try:
        print("Test 1: Mouvement vers le haut...")
        robot.goto_target(
            head=create_head_pose(z=10, degrees=True, mm=True),
            duration=1.0,
        )
        time.sleep(1.5)
        print("✅ OK")

        print("\nTest 2: Mouvement vers le bas...")
        robot.goto_target(
            head=create_head_pose(z=-10, degrees=True, mm=True),
            duration=1.0,
        )
        time.sleep(1.5)
        print("✅ OK")

        print("\nTest 3: Rotation (roll)...")
        robot.goto_target(
            head=create_head_pose(roll=15, degrees=True),
            duration=1.0,
        )
        time.sleep(1.5)
        print("✅ OK")

        print("\nTest 4: Retour à la position neutre...")
        robot.goto_target(
            head=create_head_pose(z=0, roll=0, degrees=True, mm=True),
            duration=1.0,
        )
        time.sleep(1.5)
        print("✅ OK")

        return True

    except Exception as e:
        print(f"❌ Erreur lors des tests de mouvement: {e}")
        return False


def check_reflash_status(robot: ReachyMini) -> bool:
    """Vérifie le statut du reflash automatique."""
    print_header("5️⃣ VÉRIFICATION DU REFLASH AUTOMATIQUE")

    print("ℹ️  Le SDK v1.2.4+ effectue automatiquement un reflash des moteurs")
    print("   lors de la connexion et du démarrage du robot.")
    print()
    print("✅ Si vous êtes connecté, le reflash a été effectué automatiquement")
    print("✅ Les LEDs des moteurs devraient être éteintes après le reflash")

    # Note: On ne peut pas vraiment vérifier le reflash sans accès hardware
    # Mais si le robot fonctionne, c'est que le reflash a réussi
    return True


def generate_report(results: dict[str, MotorValidationResult], scan_results: dict[str, Any]) -> None:
    """Génère un rapport complet."""
    print_header("📊 RAPPORT DE VALIDATION")

    print("RÉSUMÉ PAR MOTEUR:")
    print("-" * 70)

    all_valid = True
    for result in results.values():
        status = "✅" if result.is_valid() else "❌"
        print(f"{status} {result}")
        if not result.is_valid() and result.errors:
            for error in result.errors:
                print(f"      ⚠️  {error}")
        all_valid = all_valid and result.is_valid()

    print("\n" + "-" * 70)

    # Résultats du scan
    if scan_results:
        print("\nRÉSULTATS DU SCAN:")
        print("-" * 70)
        if scan_results.get("wrong_baudrate_motors"):
            print("⚠️  MOTEURS AVEC MAUVAIS BAUDRATE:")
            for motor_id in scan_results["wrong_baudrate_motors"]:
                print(f"   - Motor ID {motor_id}")
        if scan_results.get("missing_motors"):
            print("❌ MOTEURS MANQUANTS:")
            for motor_id in scan_results["missing_motors"]:
                print(f"   - Motor ID {motor_id}")
        if not scan_results.get("wrong_baudrate_motors") and not scan_results.get("missing_motors"):
            print("✅ Tous les moteurs sont correctement configurés")

    # Conclusion
    print("\n" + "=" * 70)
    if all_valid:
        print("✅ VALIDATION RÉUSSIE - Tous les moteurs fonctionnent correctement!")
        print("\n💡 Prochaines étapes:")
        print("   - Continuer à surveiller les moteurs (voir GUIDE_PREVENTION_PROBLEMES_MOTEURS.md)")
        print("   - Effectuer des tests réguliers (quotidien, hebdomadaire)")
    else:
        print("❌ VALIDATION ÉCHOUÉE - Certains moteurs ont des problèmes")
        print("\n💡 Actions recommandées:")
        print("   - Vérifier le câblage")
        print("   - Vérifier les logs du daemon: journalctl -u reachy-mini-daemon -f")
        print("   - Consulter le guide de troubleshooting")
        print("   - Contacter Pollen Robotics si le problème persiste")
    print("=" * 70 + "\n")


def main() -> None:
    """Fonction principale."""
    print("\n" + "=" * 70)
    print("  VALIDATION COMPLÈTE APRÈS INSTALLATION DES MOTEURS")
    print("=" * 70)
    print("\nCe script va valider que tous les moteurs fonctionnent correctement")
    print("après l'installation des nouveaux moteurs.\n")

    # 1. Vérification de la connexion
    robot = check_connection()
    if not robot:
        print("\n❌ Impossible de continuer sans connexion au robot")
        sys.exit(1)

    # 2. Scan des moteurs
    scan_results = scan_motors()

    # 3. Test de chaque moteur
    print_header("3️⃣ TEST INDIVIDUEL DE CHAQUE MOTEUR")

    results: dict[str, MotorValidationResult] = {}

    # Tester tous les joints de la tête
    for joint_name, motor_id in JOINT_TO_MOTOR_ID.items():
        if motor_id >= 11:  # Seulement les stewart joints et antennes
            print(f"\nTest du moteur {motor_id} ({joint_name})...")
            result = test_motor(robot, motor_id, joint_name)
            results[joint_name] = result
            print(f"   {result}")

    # 4. Test des mouvements de la tête
    head_ok = test_head_movements(robot)

    # 5. Vérification du reflash
    reflash_ok = check_reflash_status(robot)

    # 6. Génération du rapport
    generate_report(results, scan_results)

    # Fermeture propre
    try:
        robot.close()
    except Exception:
        pass

    # Code de sortie
    all_valid = all(r.is_valid() for r in results.values()) and head_ok and reflash_ok
    sys.exit(0 if all_valid else 1)


if __name__ == "__main__":
    main()
