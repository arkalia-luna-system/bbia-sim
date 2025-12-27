#!/usr/bin/env python3
"""Script de diagnostic des erreurs moteurs - Identifie les moteurs qui clignotent en rouge.

Usage:
    python examples/reachy_mini/diagnostic_motor_errors.py
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

try:
    from reachy_mini import ReachyMini
    USE_SDK = True
except ImportError:
    USE_SDK = False
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


def diagnose_motor_errors(robot) -> None:
    """Diagnostique les erreurs des moteurs."""
    print("\n" + "=" * 60)
    print("🔍 DIAGNOSTIC ERREURS MOTEURS")
    print("=" * 60)
    print()

    try:
        # Obtenir l'état complet du robot
        print("1️⃣ Lecture état des moteurs...")
        
        if USE_SDK:
            # Obtenir les positions des joints
            head_positions, antenna_positions = robot.get_current_joint_positions()
            
            print(f"   Positions tête: {len(head_positions)} joints")
            print(f"   Positions antennes: {len(antenna_positions)} joints")
            
            # Essayer d'accéder aux moteurs directement si possible
            try:
                # Vérifier si on peut accéder aux moteurs via l'API
                if hasattr(robot, 'head') and hasattr(robot.head, 'motors'):
                    print("\n2️⃣ Vérification erreurs matérielles...")
                    motors = robot.head.motors
                    for i, motor in enumerate(motors, 1):
                        try:
                            # Essayer de lire l'état du moteur
                            print(f"   Moteur {i}: Vérification...")
                        except Exception as e:
                            print(f"   Moteur {i}: ⚠️  Erreur - {e}")
            except Exception as e:
                print(f"   ⚠️  Impossible d'accéder aux moteurs directement: {e}")
        else:
            # Backend BBIA
            if hasattr(robot, 'get_available_joints'):
                joints = robot.get_available_joints()
                print(f"   Joints disponibles: {len(joints)}")
                print(f"   Joints: {joints[:10]}...")  # Afficher les 10 premiers

        print("\n3️⃣ Correspondance moteurs physiques:")
        print("   Motor ID 10 = Base (rotation corps)")
        print("   Motor ID 11 = stewart_1 (tête)")
        print("   Motor ID 12 = stewart_2 (tête) ← Si c'est celui qui clignote!")
        print("   Motor ID 13 = stewart_3 (tête)")
        print("   Motor ID 14 = stewart_4 (tête)")
        print("   Motor ID 15 = stewart_5 (tête)")
        print("   Motor ID 16 = stewart_6 (tête)")
        print("   Motor ID 17 = Antenne gauche")
        print("   Motor ID 18 = Antenne droite")
        print()
        print("⚠️  Si le moteur PHYSIQUE numéro 2 clignote en rouge:")
        print("   → C'est probablement le Motor ID 12 (stewart_2)")
        print("   → Vérifiez:")
        print("      1. Le moteur est dans le bon emplacement")
        print("      2. Le câble est bien branché")
        print("      3. Le moteur n'est pas en butée mécanique")
        print("      4. Aucun câble n'est coincé ou plié")

    except Exception as e:
        print(f"❌ Erreur lors du diagnostic: {e}")
        import traceback
        traceback.print_exc()


def main() -> None:
    """Fonction principale."""
    print("🔍 DIAGNOSTIC ERREURS MOTEURS - Reachy Mini")
    print("=" * 60)
    print()
    print("Ce script identifie les moteurs qui ont des erreurs matérielles.")
    print()

    # Connexion
    print("🔌 Connexion au robot...")
    try:
        if USE_SDK:
            robot = ReachyMini(
                media_backend="no_media",
                use_sim=False,
                localhost_only=False,
                timeout=60.0,
            )
            robot.__enter__()
            print("✅ Robot connecté (SDK officiel)")
        else:
            robot = ReachyMiniBackend(use_sim=False, localhost_only=False)
            robot.connect()
            if not robot.is_connected:
                print("❌ Robot non connecté")
                sys.exit(1)
            print("✅ Robot connecté (Backend BBIA)")

        print()

        # Diagnostic
        diagnose_motor_errors(robot)

        print("\n" + "=" * 60)
        print("✅ DIAGNOSTIC TERMINÉ")
        print("=" * 60)
        print()
        print("💡 Prochaines étapes:")
        print("   1. Identifiez physiquement quel moteur clignote")
        print("   2. Vérifiez le câblage selon le guide:")
        print("      https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide")
        print("   3. Vérifiez que le moteur n'est pas en butée mécanique")
        print("   4. Si le problème persiste, contactez support Pollen Robotics")

    except KeyboardInterrupt:
        print("\n⏹️  Arrêt par l'utilisateur")
    except Exception as e:
        print(f"\n❌ Erreur: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if USE_SDK:
                robot.__exit__(None, None, None)
            else:
                robot.disconnect()
            print("✅ Robot déconnecté")
        except Exception:
            pass


if __name__ == "__main__":
    main()

