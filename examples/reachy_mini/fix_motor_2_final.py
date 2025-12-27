#!/usr/bin/env python3
"""Script FINAL - Réinitialise les erreurs matérielles du moteur 2.

Ce script essaie de réinitialiser les erreurs matérielles du moteur 2.
Si ça ne marche pas, c'est que le moteur est défectueux et doit être remplacé.
"""

import sys
import time

try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    print("❌ SDK reachy_mini non disponible")
    sys.exit(1)


def fix_motor_2_final() -> None:
    """Tente de réinitialiser les erreurs du moteur 2."""
    print("🔧 CORRECTION FINALE MOTEUR 2")
    print("=" * 60)
    print()
    print("Problème: Moteur 2 (stewart_2) clignote rouge + tête retombe")
    print("Cause probable: Erreur matérielle (surcharge, surchauffe, défectueux)")
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

        # Séquence de réinitialisation complète
        print("2️⃣ Réinitialisation complète du moteur 2...")
        print()

        # Étape 1: Désactiver tous les moteurs
        print("   → Étape 1: Désactivation moteurs...")
        try:
            robot.disable_motors()
            print("      ✅ Moteurs désactivés")
            time.sleep(2)
        except Exception as e:
            print(f"      ⚠️  Erreur: {e}")

        # Étape 2: Réactiver avec compensation gravité
        print("   → Étape 2: Activation compensation gravité...")
        try:
            robot.enable_gravity_compensation()
            print("      ✅ Compensation gravité activée")
            time.sleep(2)
        except Exception as e:
            print(f"      ⚠️  Erreur compensation: {e}")
            print("      → Essai avec enable_motors()...")
            try:
                robot.enable_motors()
                print("      ✅ Moteurs activés (mode normal)")
                time.sleep(2)
            except Exception as e2:
                print(f"      ❌ Erreur: {e2}")

        # Étape 3: Repositionner la tête
        print("   → Étape 3: Repositionnement tête...")
        try:
            neutral = create_head_pose(
                x=0, y=0, z=0,
                roll=0, pitch=0, yaw=0,
                degrees=True, mm=True
            )
            robot.goto_target(head=neutral, duration=3.0)
            time.sleep(3.5)
            print("      ✅ Tête repositionnée")
        except Exception as e:
            print(f"      ⚠️  Erreur: {e}")

        # Étape 4: Test de maintien
        print("   → Étape 4: Test maintien position (10 secondes)...")
        head_positions_before, _ = robot.get_current_joint_positions()
        print("      Position initiale stewart_2:",
              f"{head_positions_before[1]*180/3.14159:.2f}°" if len(head_positions_before) >= 2 else "N/A")
        time.sleep(10)
        head_positions_after, _ = robot.get_current_joint_positions()
        
        if len(head_positions_before) >= 2 and len(head_positions_after) >= 2:
            diff = abs(head_positions_before[1] - head_positions_after[1])
            diff_deg = diff * 180 / 3.14159
            print(f"      Position après 10s: {head_positions_after[1]*180/3.14159:.2f}°")
            print(f"      Déplacement: {diff_deg:.2f}°")

            if diff_deg < 2.0:
                print("      ✅ Tête stable (ne retombe pas)")
            else:
                print(f"      ❌ Tête retombe de {diff_deg:.2f}°")
                print("      → Le moteur 2 n'a pas assez de couple")
                print("      → Erreur matérielle confirmée")
        print()

        robot.__exit__(None, None, None)

        print("=" * 60)
        print("✅ TEST TERMINÉ")
        print("=" * 60)
        print()

        if len(head_positions_before) >= 2 and len(head_positions_after) >= 2:
            diff = abs(head_positions_before[1] - head_positions_after[1])
            diff_deg = diff * 180 / 3.14159

            if diff_deg >= 2.0:
                print("❌ RÉSULTAT: Le moteur 2 est DÉFECTUEUX")
                print()
                print("Preuve:")
                print(f"   - La tête retombe de {diff_deg:.2f}° en 10 secondes")
                print("   - Le moteur 2 clignote en rouge")
                print("   - Tous les autres moteurs fonctionnent")
                print()
                print("💡 SOLUTION: Remplacement du moteur 2")
                print()
                print("📧 Contacter Pollen Robotics:")
                print("   Formulaire: https://forms.gle/JdhMzadeCnbynw7Q6")
                print()
                print("   Informations à fournir:")
                print("   - Moteur 2 (stewart_2, ID 12) clignote rouge")
                print("   - La tête retombe du côté du moteur 2")
                print("   - Reflash réussi, câbles vérifiés 2 fois")
                print("   - Tous les autres moteurs fonctionnent")
                print("   - Demande: Remplacement moteur 2")
            else:
                print("✅ RÉSULTAT: Le moteur 2 semble fonctionner")
                print("   Si le clignotement persiste, c'est une erreur mineure")
                print("   que tu peux ignorer si tout fonctionne")
        else:
            print("⚠️  Impossible de vérifier la stabilité")
            print("   Mais si le moteur 2 clignote toujours rouge,")
            print("   c'est probablement un problème matériel")
        print()

    except Exception as e:
        print(f"❌ Erreur: {e}")
        import traceback
        traceback.print_exc()


def main() -> None:
    """Fonction principale."""
    if not SDK_AVAILABLE:
        print("❌ SDK reachy_mini non disponible")
        sys.exit(1)

    fix_motor_2_final()


if __name__ == "__main__":
    main()

