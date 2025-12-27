#!/usr/bin/env python3
"""Diagnostic approfondi du moteur 2 (stewart_2) qui clignote en rouge.

Ce script diagnostique précisément pourquoi le moteur 2 clignote en rouge
après le reflash.
"""

import sys
import time

try:
    from reachy_mini import ReachyMini
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    print("❌ SDK reachy_mini non disponible")
    sys.exit(1)

# Mapping Motor ID → Nom
MOTOR_INFO = {
    10: ("yaw_body", "Base"),
    11: ("stewart_1", "Tête moteur 1"),
    12: ("stewart_2", "Tête moteur 2 ← PROBLÈME ICI"),
    13: ("stewart_3", "Tête moteur 3"),
    14: ("stewart_4", "Tête moteur 4"),
    15: ("stewart_5", "Tête moteur 5"),
    16: ("stewart_6", "Tête moteur 6"),
    17: ("left_antenna", "Antenne gauche"),
    18: ("right_antenna", "Antenne droite"),
}


def diagnose_motor_2() -> None:
    """Diagnostique le problème du moteur 2."""
    print("🔍 DIAGNOSTIC MOTEUR 2 (stewart_2) - Clignotement rouge")
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

        print("2️⃣ Lecture des positions des joints...")
        head_positions, antenna_positions = robot.get_current_joint_positions()
        print(f"   Positions tête: {len(head_positions)} joints")
        print(f"   Positions antennes: {len(antenna_positions)} joints")
        print()

        # Vérifier spécifiquement stewart_2 (index 1 dans le tableau)
        print("3️⃣ Vérification spécifique du moteur 2 (stewart_2)...")
        print()

        if len(head_positions) >= 2:
            stewart_2_index = 1  # stewart_2 est à l'index 1 (0=stewart_1, 1=stewart_2)
            stewart_2_position = head_positions[stewart_2_index]
            print(f"   Position stewart_2: {stewart_2_position:.4f} rad ({stewart_2_position*180/3.14159:.2f}°)")
            
            # Vérifier si la position est dans les limites
            stewart_2_limits = (-1.396263401595614, 1.2217304763958803)
            if stewart_2_limits[0] <= stewart_2_position <= stewart_2_limits[1]:
                print("   ✅ Position dans les limites")
            else:
                print(f"   ⚠️  Position HORS LIMITES!")
                print(f"      Limites: [{stewart_2_limits[0]:.4f}, {stewart_2_limits[1]:.4f}] rad")
        else:
            print("   ❌ Impossible de lire la position de stewart_2")
            print(f"      Seulement {len(head_positions)} joints détectés")
        print()

        # Essayer d'accéder directement au moteur si possible
        print("4️⃣ Accès direct au moteur 2...")
        try:
            if hasattr(robot, 'head') and hasattr(robot.head, 'motors'):
                motors = robot.head.motors
                if len(motors) > 1:
                    motor_2 = motors[1]  # Index 1 = stewart_2
                    print(f"   Moteur 2 trouvé: {motor_2}")
                    
                    # Essayer de lire l'état du moteur
                    if hasattr(motor_2, 'id'):
                        print(f"   ID du moteur: {motor_2.id}")
                    
                    # Essayer de lire les erreurs
                    if hasattr(motor_2, 'hardware_error'):
                        error = motor_2.hardware_error
                        print(f"   Erreur matérielle: {error}")
                        if error:
                            print("   ⚠️  ERREUR MATÉRIELLE DÉTECTÉE!")
                            print("      Causes possibles:")
                            print("      - Surcharge (overload)")
                            print("      - Surchauffe (overheating)")
                            print("      - Problème de connexion")
                            print("      - Moteur en butée mécanique")
                    
                    if hasattr(motor_2, 'present_position'):
                        pos = motor_2.present_position
                        print(f"   Position actuelle: {pos}")
                    
                    if hasattr(motor_2, 'goal_position'):
                        goal = motor_2.goal_position
                        print(f"   Position cible: {goal}")
                else:
                    print("   ⚠️  Moins de 2 moteurs trouvés")
            else:
                print("   ⚠️  Accès direct aux moteurs non disponible")
        except Exception as e:
            print(f"   ⚠️  Erreur accès moteur: {e}")
        print()

        # Vérifier tous les joints stewart pour voir lequel pose problème
        print("5️⃣ Comparaison de tous les joints stewart...")
        print()
        if len(head_positions) >= 6:
            for i in range(6):
                joint_name = f"stewart_{i+1}"
                motor_id = 11 + i
                motor_name, _ = MOTOR_INFO.get(motor_id, ("?", "?"))
                pos = head_positions[i]
                print(f"   {joint_name:12s} (ID {motor_id:2d}): {pos:8.4f} rad ({pos*180/3.14159:6.2f}°)")
        print()

        robot.__exit__(None, None, None)

    except Exception as e:
        print(f"❌ Erreur: {e}")
        import traceback
        traceback.print_exc()


def main() -> None:
    """Fonction principale."""
    print("🔴 DIAGNOSTIC MOTEUR 2 - Clignotement rouge")
    print("=" * 60)
    print()
    print("Problème: Le moteur 2 (stewart_2) clignote en rouge")
    print("Causes possibles:")
    print("  1. Moteur en butée mécanique")
    print("  2. Câble mal branché ou défectueux")
    print("  3. Moteur défectueux")
    print("  4. Position hors limites")
    print("  5. Surcharge ou surchauffe")
    print()

    if not SDK_AVAILABLE:
        print("❌ SDK reachy_mini non disponible")
        sys.exit(1)

    diagnose_motor_2()

    print("\n" + "=" * 60)
    print("💡 SOLUTIONS")
    print("=" * 60)
    print()
    print("1️⃣ Vérifier visuellement:")
    print("   - Le moteur 2 est-il en butée mécanique?")
    print("   - Y a-t-il un câble qui bloque le mouvement?")
    print("   - Le câble est-il bien branché?")
    print()
    print("2️⃣ Vérifier le câblage:")
    print("   - Le câble du moteur 2 est-il bien enfoncé?")
    print("   - Le câble n'est-il pas endommagé?")
    print("   - Le câble est-il dans le bon ordre (daisy-chain)?")
    print()
    print("3️⃣ Vérifier la position mécanique:")
    print("   - Le moteur peut-il bouger librement?")
    print("   - Y a-t-il une résistance anormale?")
    print()
    print("4️⃣ Si le problème persiste:")
    print("   - Contactez le support Pollen Robotics")
    print("   - Formulaire: https://forms.gle/JdhMzadeCnbynw7Q6")
    print()


if __name__ == "__main__":
    main()

