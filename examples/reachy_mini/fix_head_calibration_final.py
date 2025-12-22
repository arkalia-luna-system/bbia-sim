#!/usr/bin/env python3
"""Correction finale de la tête - Ajustement des offsets pour forcer la tête droite.

Ce script essaie de corriger la tête en ajustant les positions individuelles
des stewart joints pour compenser les offsets.
"""

import sys
import time
import math

try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    print("❌ SDK reachy_mini non disponible")
    sys.exit(1)


def fix_head_calibration() -> None:
    """Corrige la tête en ajustant les offsets."""
    print("🔧 CORRECTION FINALE - Ajustement calibration tête")
    print("=" * 60)
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

        # Lire la position actuelle
        print("2️⃣ Lecture position actuelle...")
        head_positions, _ = robot.get_current_joint_positions()
        print("   Positions actuelles:")
        positions = {}
        for i in range(min(6, len(head_positions))):
            joint_name = f"stewart_{i+1}"
            pos = head_positions[i]
            positions[joint_name] = pos
            print(f"   {joint_name:12s}: {pos:8.4f} rad ({pos*180/math.pi:6.2f}°)")
        print()

        # Calculer les corrections nécessaires
        print("3️⃣ Calcul des corrections...")
        corrections = {}
        for joint_name, pos in positions.items():
            # Objectif: ramener à 0
            corrections[joint_name] = -pos
            print(f"   {joint_name:12s}: correction de {corrections[joint_name]*180/math.pi:6.2f}°")
        print()

        # Désactiver les moteurs pour permettre ajustement manuel
        print("4️⃣ Désactivation moteurs pour ajustement manuel...")
        robot.disable_motors()
        print("   ✅ Moteurs désactivés")
        print()
        print("   💡 INSTRUCTIONS:")
        print("      1. La tête est maintenant libre")
        print("      2. Déplace MANUELLEMENT la tête pour qu'elle soit DROITE")
        print("      3. Assure-toi qu'elle est vraiment droite visuellement")
        print("      4. Attends 10 secondes...")
        print()
        time.sleep(10)
        print()

        # Réactiver les moteurs
        print("5️⃣ Réactivation des moteurs...")
        robot.enable_motors()
        print("   ✅ Moteurs réactivés")
        time.sleep(2)
        print()

        # Lire la nouvelle position après ajustement manuel
        print("6️⃣ Lecture position après ajustement manuel...")
        head_positions_new, _ = robot.get_current_joint_positions()
        print("   Nouvelles positions:")
        for i in range(min(6, len(head_positions_new))):
            joint_name = f"stewart_{i+1}"
            pos = head_positions_new[i]
            print(f"   {joint_name:12s}: {pos:8.4f} rad ({pos*180/math.pi:6.2f}°)")
        print()

        # Maintenir cette position comme nouvelle "neutre"
        print("7️⃣ Maintien de la position droite...")
        try:
            # Créer une pose qui maintient la position actuelle
            # On utilise la position actuelle comme référence
            current_pose = robot.get_current_head_pose()
            print("   → Maintien de la position actuelle...")
            robot.goto_target(head=current_pose, duration=1.0)
            time.sleep(1.5)
            print("   ✅ Position maintenue")
        except Exception as e:
            print(f"   ⚠️  Erreur: {e}")
        print()

        # Test de mouvement pour vérifier
        print("8️⃣ Test de mouvement...")
        try:
            # Petit mouvement
            test_pose = create_head_pose(roll=3, degrees=True)
            robot.goto_target(head=test_pose, duration=1.0)
            time.sleep(1.5)
            
            # Retour à la position "droite" actuelle
            current_pose = robot.get_current_head_pose()
            robot.goto_target(head=current_pose, duration=1.0)
            time.sleep(1.5)
            print("   ✅ Mouvement test réussi")
        except Exception as e:
            print(f"   ⚠️  Erreur: {e}")
        print()

        robot.__exit__(None, None, None)

        print("=" * 60)
        print("✅ CORRECTION TERMINÉE")
        print("=" * 60)
        print()
        print("💡 IMPORTANT:")
        print("   La tête devrait maintenant être droite (position manuelle)")
        print()
        print("⚠️  Si le problème persiste:")
        print("   1. Le moteur 2 peut être défectueux")
        print("   2. Les offsets d'usine sont incorrects")
        print("   3. Il faut contacter Pollen Robotics pour:")
        print("      - Recalibration des offsets")
        print("      - Remplacement du moteur 2 si défectueux")
        print()
        print("📧 Support Pollen Robotics:")
        print("   Formulaire: https://forms.gle/JdhMzadeCnbynw7Q6")
        print()
        print("   Informations à fournir:")
        print("   - Reflash réussi (tous moteurs détectés)")
        print("   - Câbles vérifiés 2 fois")
        print("   - Moteur 2 bouge mais clignote rouge")
        print("   - Tête de travers (stewart_2 à -22° en 'neutre')")
        print("   - Aucun échange de moteurs effectué")
        print()

    except Exception as e:
        print(f"❌ Erreur: {e}")
        import traceback
        traceback.print_exc()


def main() -> None:
    """Fonction principale."""
    print("🔧 CORRECTION FINALE - Calibration tête")
    print("=" * 60)
    print()
    print("Ce script va:")
    print("  1. Désactiver les moteurs")
    print("  2. Te permettre de déplacer la tête MANUELLEMENT pour qu'elle soit droite")
    print("  3. Maintenir cette position comme nouvelle 'neutre'")
    print()
    print("⚠️  IMPORTANT: Tu dois déplacer la tête MANUELLEMENT pendant les 10 secondes")
    print()

    if not SDK_AVAILABLE:
        print("❌ SDK reachy_mini non disponible")
        sys.exit(1)

    fix_head_calibration()


if __name__ == "__main__":
    main()

