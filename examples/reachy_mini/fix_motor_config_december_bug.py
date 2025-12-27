#!/usr/bin/env python3
"""Script de correction du bug décembre 2025 - Tête penchée, moteur 1 non configuré.

Problème identifié sur le lot de décembre 2025:
- Tête penchée
- Moteur 1 ne bouge pas mais devient rigide quand alimenté
- Pas de LED rouge qui clignote
- Moteur reste avec paramètres d'usine (ID=1, baudrate 57,600) au lieu d'être préconfiguré

Solutions:
1. Utiliser le script reachy-mini-reflash-motors (branche 592)
2. Reconfigurer manuellement le moteur via SSH
3. Utiliser ce script qui appelle les outils du SDK officiel

Basé sur les solutions du Discord Pollen Robotics:
- robertodipizzamano: "Motor 1 issue fixed for me by running the 592 branch reachy-mini-reflash-motors script"
- squirrel: Reconfiguration manuelle via setup_motor
- Augustin (Pollen Team): Post épinglé "Head tilted, motor n°1 not moving, but get stiff when powered on, and doesn't blink red - SOLVED"

Usage:
    python examples/reachy_mini/fix_motor_config_december_bug.py
    python examples/reachy_mini/fix_motor_config_december_bug.py --serialport /dev/ttyAMA3
    python examples/reachy_mini/fix_motor_config_december_bug.py --motor-id 13 --old-id 1
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

try:
    # Essayer d'importer depuis reachy_mini.tools (branche 592 ou version dev)
    try:
        from reachy_mini.tools.setup_motor import (
            check_configuration,
            light_led_up,
            setup_motor,
        )

        TOOLS_AVAILABLE = True
    except ImportError:
        # Essayer depuis reachy_mini.tools directement
        try:
            import reachy_mini.tools
            from reachy_mini.tools.setup_motor import (
                check_configuration,
                light_led_up,
                setup_motor,
            )

            TOOLS_AVAILABLE = True
        except ImportError:
            TOOLS_AVAILABLE = False

    from importlib.resources import files

    import reachy_mini
    from reachy_mini.daemon.utils import find_serial_port
    from reachy_mini.utils.hardware_config.parser import parse_yaml_config

    SDK_AVAILABLE = True
    if not TOOLS_AVAILABLE:
        IMPORT_ERROR = "Module reachy_mini.tools non disponible (nécessite branche 592 ou version dev)"
except ImportError as e:
    SDK_AVAILABLE = False
    TOOLS_AVAILABLE = False
    IMPORT_ERROR = str(e)


def diagnose_motor_issue(serialport: str, motor_id: int = 13, old_id: int = 1) -> bool:
    """Diagnostique le problème du moteur 1 (lot décembre 2025).

    Args:
        serialport: Port série du robot (e.g. /dev/ttyAMA3)
        motor_id: ID cible du moteur (par défaut 13 pour stewart_1)
        old_id: Ancien ID du moteur (par défaut 1, paramètres d'usine)

    Returns:
        True si problème détecté, False sinon
    """
    print("\n" + "=" * 60)
    print("🔍 DIAGNOSTIC PROBLÈME MOTEUR (Lot Décembre 2025)")
    print("=" * 60)
    print()

    try:
        # Essayer de scanner le bus à différents baudrates
        print(f"1️⃣ Scan du bus Dynamixel sur {serialport}...")
        print(f"   - À 1,000,000 baud: recherche moteur ID {motor_id}")
        print(f"   - À 57,600 baud: recherche moteur ID {old_id} (paramètres d'usine)")

        # Essayer d'utiliser le script de scan automatique si disponible
        try:
            from examples.reachy_mini.scan_motors_baudrate import diagnose_motors_baudrate

            print("\n   🔍 Utilisation du scan automatique...")
            results = diagnose_motors_baudrate(serialport)

            if motor_id in results["wrong_baudrate_motors"]:
                print(f"   ⚠️  PROBLÈME DÉTECTÉ: Motor ID {motor_id} a un mauvais baudrate!")
                print(f"      → Trouvé à 57.6k baud mais pas à 1M baud")
                return True
            elif motor_id in results["missing_motors"]:
                print(f"   ⚠️  PROBLÈME DÉTECTÉ: Motor ID {motor_id} est manquant!")
                return True
            else:
                print(f"   ✅ Motor ID {motor_id} est correctement configuré")
                return False

        except ImportError:
            # Fallback: message informatif
            print("   ⚠️  Si le moteur répond à 57,600 baud avec ID=1, c'est le bug!")
            print("   ✅ Si le moteur répond à 1,000,000 baud avec ID=13, c'est OK")
            print("\n   💡 Pour un scan automatique, utilisez:")
            print("      python examples/reachy_mini/scan_motors_baudrate.py")
            return True

    except Exception as e:
        print(f"   ❌ Erreur diagnostic: {e}")
        return False


def fix_motor_configuration(
    serialport: str,
    motor_id: int = 13,
    old_id: int = 1,
    old_baudrate: int = 57600,
    target_baudrate: int = 1000000,
) -> bool:
    """Corrige la configuration du moteur mal configuré.

    Args:
        serialport: Port série du robot
        motor_id: ID cible du moteur (13 pour stewart_1)
        old_id: Ancien ID du moteur (1 = paramètres d'usine)
        old_baudrate: Ancien baudrate (57,600 = paramètres d'usine)
        target_baudrate: Baudrate cible (1,000,000 = configuration correcte)

    Returns:
        True si succès, False sinon
    """
    print("\n" + "=" * 60)
    print("🔧 CORRECTION CONFIGURATION MOTEUR")
    print("=" * 60)
    print()

    try:
        print("1️⃣ Arrêt du daemon Reachy Mini...")
        print(
            "   💡 Exécutez sur le robot (SSH): sudo systemctl stop reachy-mini-daemon"
        )
        print("   ⏳ Attente 3 secondes...")
        time.sleep(3)

        print("\n2️⃣ Configuration du moteur...")
        print(f"   Port série: {serialport}")
        print(f"   Ancien ID: {old_id} (paramètres d'usine)")
        print(f"   Nouveau ID: {motor_id} (configuration correcte)")
        print(f"   Ancien baudrate: {old_baudrate}")
        print(f"   Nouveau baudrate: {target_baudrate}")

        # Charger la configuration hardware
        config_file_path = str(
            files(reachy_mini).joinpath("assets/config/hardware_config.yaml")
        )
        config = parse_yaml_config(config_file_path)

        # Trouver la configuration du moteur stewart_1
        motor_name = "stewart_1"  # Le moteur problématique
        if motor_name not in config.motors:
            print(f"   ❌ Moteur '{motor_name}' non trouvé dans la configuration")
            return False

        motor_config = config.motors[motor_name]

        print(f"\n3️⃣ Reconfiguration du moteur '{motor_name}'...")
        print("   - Désactivation du couple")
        print(f"   - Changement baudrate: {old_baudrate} → {target_baudrate}")
        print(f"   - Changement ID: {old_id} → {motor_id}")

        # Utiliser setup_motor pour reconfigurer
        setup_motor(
            motor_config,
            serialport,
            from_id=old_id,  # ID actuel (paramètres d'usine)
            from_baudrate=old_baudrate,  # Baudrate actuel (paramètres d'usine)
            target_baudrate=target_baudrate,  # Baudrate cible
        )

        print("   ✅ Reconfiguration effectuée")

        print("\n4️⃣ Vérification de la configuration...")
        try:
            check_configuration(
                motor_config,
                serialport,
                baudrate=target_baudrate,
            )
            print("   ✅ Configuration vérifiée avec succès")
        except RuntimeError as e:
            print(f"   ❌ Vérification échouée: {e}")
            return False

        print("\n5️⃣ Test LED (allumage)...")
        light_led_up(
            serialport,
            motor_config.id,
            baudrate=target_baudrate,
        )
        print("   ✅ LED allumée - moteur répond correctement")

        print("\n6️⃣ Redémarrage du daemon...")
        print(
            "   💡 Exécutez sur le robot (SSH): sudo systemctl start reachy-mini-daemon"
        )
        print("   ⏳ Attente 5 secondes...")
        time.sleep(5)

        return True

    except Exception as e:
        print(f"   ❌ Erreur lors de la correction: {e}")
        import traceback

        traceback.print_exc()
        return False


def main() -> None:
    """Fonction principale."""
    parser = argparse.ArgumentParser(
        description="Corrige le bug décembre 2025: tête penchée, moteur 1 non configuré",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  # Détection automatique du port série
  python examples/reachy_mini/fix_motor_config_december_bug.py

  # Spécifier le port série manuellement
  python examples/reachy_mini/fix_motor_config_december_bug.py --serialport /dev/ttyAMA3

  # Spécifier l'ID du moteur problématique
  python examples/reachy_mini/fix_motor_config_december_bug.py --motor-id 13 --old-id 1

Note: Ce script nécessite:
  - Accès SSH au robot (pour arrêter/démarrer le daemon)
  - SDK reachy_mini installé
  - Permissions d'accès au port série
        """,
    )
    parser.add_argument(
        "--serialport",
        type=str,
        default=None,
        help="Port série du robot (e.g. /dev/ttyAMA3, /dev/ttyUSB0). "
        "Si non spécifié, tentative de détection automatique.",
    )
    parser.add_argument(
        "--motor-id",
        type=int,
        default=13,
        help="ID cible du moteur (par défaut 13 pour stewart_1)",
    )
    parser.add_argument(
        "--old-id",
        type=int,
        default=1,
        help="Ancien ID du moteur (par défaut 1, paramètres d'usine)",
    )
    parser.add_argument(
        "--diagnose-only",
        action="store_true",
        help="Uniquement diagnostiquer, ne pas corriger",
    )

    args = parser.parse_args()

    print("🔧 CORRECTION BUG DÉCEMBRE 2025 - Reachy Mini")
    print("=" * 60)
    print()
    print("Problème: Tête penchée, moteur 1 ne bouge pas mais devient rigide")
    print("Cause: Moteur avec paramètres d'usine (ID=1, baudrate 57,600)")
    print("Solution: Reconfiguration du moteur vers ID=13, baudrate 1,000,000")
    print()

    if not SDK_AVAILABLE:
        print("❌ SDK reachy_mini non disponible!")
        print(f"   Erreur: {IMPORT_ERROR}")
        print()
        print("💡 Solutions:")
        print("   1. Installer le SDK: pip install reachy-mini")
        print("   2. Utiliser le script officiel: reachy-mini-reflash-motors")
        print("   3. Reconfigurer manuellement via SSH (voir documentation)")
        sys.exit(1)

    if not TOOLS_AVAILABLE:
        print("⚠️  Module reachy_mini.tools non disponible!")
        print(f"   Erreur: {IMPORT_ERROR}")
        print()
        print("💡 Le module 'tools' est nécessaire pour reconfigurer les moteurs.")
        print(
            "   Il est disponible dans la branche 592 ou les versions de développement."
        )
        print()
        print("Solutions:")
        print("   1. Installer depuis la branche 592:")
        print(
            "      pip install git+https://github.com/pollen-robotics/reachy_mini.git@592"
        )
        print()
        print("   2. Utiliser le script officiel de reflash:")
        print("      python examples/reachy_mini/reflash_motors_simple.py")
        print()
        print("   3. Reconfigurer manuellement via SSH (voir documentation)")
        print()
        print("   4. Utiliser uniquement le diagnostic:")
        print(
            "      python examples/reachy_mini/fix_motor_config_december_bug.py --diagnose-only"
        )
        sys.exit(1)

    # Détection du port série
    serialport = args.serialport
    if serialport is None:
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
                print("❌ Aucun port série Reachy Mini trouvé")
                print("   Vérifiez la connexion USB et les permissions")
                print("   Ou spécifiez le port avec --serialport")
                sys.exit(1)
            elif len(ports) > 1:
                print(f"⚠️  Plusieurs ports trouvés: {ports}")
                print("   Spécifiez le port avec --serialport")
                sys.exit(1)

            serialport = ports[0]
            print(f"✅ Port série trouvé: {serialport}")

        except Exception as e:
            print(f"❌ Erreur détection port série: {e}")
            print("   Spécifiez le port avec --serialport")
            sys.exit(1)

    # Diagnostic
    diagnose_motor_issue(serialport, args.motor_id, args.old_id)

    if args.diagnose_only:
        print("\n✅ Diagnostic terminé (mode diagnostic uniquement)")
        return

    # Confirmation
    print("\n⚠️  ATTENTION: Cette opération va reconfigurer le moteur!")
    print("   Assurez-vous d'avoir:")
    print("   1. Arrêté le daemon: sudo systemctl stop reachy-mini-daemon")
    print("   2. Accès SSH au robot")
    print("   3. Permissions d'accès au port série")
    print()
    confirm = input("Continuer? (oui/non): ").strip().lower()
    if confirm not in ["oui", "o", "yes", "y"]:
        print("❌ Opération annulée")
        sys.exit(0)

    # Correction
    success = fix_motor_configuration(
        serialport,
        motor_id=args.motor_id,
        old_id=args.old_id,
    )

    if success:
        print("\n" + "=" * 60)
        print("✅ CORRECTION RÉUSSIE")
        print("=" * 60)
        print()
        print("💡 Prochaines étapes:")
        print("   1. Redémarrer le daemon: sudo systemctl start reachy-mini-daemon")
        print("   2. Tester les mouvements de la tête")
        print(
            "   3. Si nécessaire, ajuster mécaniquement le joint à la position neutre"
        )
        print("   4. Relancer le script fix_head_tilted.py pour corriger la position")
    else:
        print("\n" + "=" * 60)
        print("⚠️  CORRECTION PARTIELLE OU ÉCHOUÉE")
        print("=" * 60)
        print()
        print("💡 Solutions alternatives:")
        print(
            "   1. Utiliser le script officiel: reachy-mini-reflash-motors (branche 592)"
        )
        print("   2. Reconfigurer manuellement via SSH (voir documentation Discord)")
        print("   3. Contacter support Pollen Robotics")


if __name__ == "__main__":
    main()
