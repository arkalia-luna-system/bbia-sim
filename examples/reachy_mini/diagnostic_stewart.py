#!/usr/bin/env python3
"""Diagnostic des joints Stewart - Détection câbles manquants/mal branchés.

Ce script vérifie l'état des 6 joints stewart de la plateforme parallèle.
Si un câble est manquant ou mal branché, certains joints ne répondront pas.

Usage:
    python examples/reachy_mini/diagnostic_stewart.py
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


def format_rad_to_deg(rad: float) -> str:
    """Convertit radians en degrés avec formatage."""
    deg = rad * 180.0 / 3.141592653589793
    return f"{deg:+.2f}° ({rad:+.4f} rad)"


def check_stewart_joints(robot) -> dict[str, dict]:
    """Vérifie l'état des 6 joints stewart."""
    print("\n" + "=" * 60)
    print("🔍 DIAGNOSTIC JOINTS STEWART (Plateforme Parallèle)")
    print("=" * 60)
    print()

    results: dict[str, dict] = {}

    # Lire positions actuelles
    try:
        if USE_SDK:
            head_positions, _ = robot.get_current_joint_positions()
        else:
            head_positions, _ = robot.get_current_joint_positions()

        # Format SDK: head_positions peut être 12 éléments (indices impairs) ou 6 éléments
        # Extraire les 6 joints stewart
        if len(head_positions) >= 12:
            # Format 12 éléments: stewart aux indices impairs (1,3,5,7,9,11)
            stewart_values = [
                head_positions[1],  # stewart_1
                head_positions[3],  # stewart_2
                head_positions[5],  # stewart_3
                head_positions[7],  # stewart_4
                head_positions[9],  # stewart_5
                head_positions[11],  # stewart_6
            ]
        elif len(head_positions) >= 6:
            # Format 6 éléments: directement les stewart joints
            stewart_values = head_positions[:6]
        else:
            print("❌ Format de head_positions inattendu:", len(head_positions))
            return results

        # Limites officielles (en radians)
        limits = {
            "stewart_1": (-0.8378, 1.3963),
            "stewart_2": (-1.3963, 1.2217),
            "stewart_3": (-0.8378, 1.3963),
            "stewart_4": (-1.3963, 0.8378),
            "stewart_5": (-1.2217, 1.3963),
            "stewart_6": (-1.3963, 0.8378),
        }

        print("📊 Positions actuelles des joints:")
        print("-" * 60)

        for i, (joint_name, value) in enumerate(
            zip(
                [
                    "stewart_1",
                    "stewart_2",
                    "stewart_3",
                    "stewart_4",
                    "stewart_5",
                    "stewart_6",
                ],
                stewart_values,
                strict=True,
            )
        ):
            min_limit, max_limit = limits[joint_name]

            # Vérifier si dans les limites
            in_range = min_limit <= value <= max_limit
            # Vérifier si valeur anormale (toujours 0 ou très proche de 0 peut indiquer problème)
            is_zero = abs(value) < 0.001
            # Vérifier si valeur extrême (collée aux limites)
            at_limit = abs(value - min_limit) < 0.01 or abs(value - max_limit) < 0.01

            status = "✅ OK"
            if not in_range:
                status = "❌ HORS LIMITES"
            elif (
                is_zero and i > 0
            ):  # stewart_1 peut être à 0, mais pas les autres normalement
                status = "⚠️  SUSPECT (proche de 0)"
            elif at_limit:
                status = "⚠️  À LA LIMITE"

            results[joint_name] = {
                "value": value,
                "deg": format_rad_to_deg(value),
                "in_range": in_range,
                "is_zero": is_zero,
                "at_limit": at_limit,
                "status": status,
                "min_limit": min_limit,
                "max_limit": max_limit,
            }

            print(f"{joint_name:12} : {format_rad_to_deg(value):25} {status}")

        print("-" * 60)
        print()

        # Analyse des problèmes
        print("🔍 ANALYSE:")
        print("-" * 60)

        problems = []
        zeros = []
        out_of_range = []

        for joint_name, data in results.items():
            if not data["in_range"]:
                out_of_range.append(joint_name)
            if data["is_zero"]:
                zeros.append(joint_name)
            if data["status"].startswith("❌") or data["status"].startswith("⚠️"):
                problems.append(joint_name)

        if out_of_range:
            print(f"❌ Joints HORS LIMITES: {', '.join(out_of_range)}")
            print("   → Câble peut être mal branché ou moteur défectueux")
            print()

        if zeros:
            print(f"⚠️  Joints à ZÉRO (suspect): {', '.join(zeros)}")
            print("   → Câble peut être manquant ou non connecté")
            print()

        if problems:
            print(f"⚠️  PROBLÈMES DÉTECTÉS: {len(problems)} joint(s)")
            print("   → Vérifier:")
            print("     1. Câbles bien branchés entre bras 1-6 et contrôleur")
            print(
                "     2. Guide d'assemblage: https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide"
            )
            print("     3. Pas de câble manquant (normalement 6 câbles pour 6 joints)")
            print()
        else:
            print("✅ Tous les joints semblent OK")
            print("   → Si la tête reste penchée, peut être:")
            print("     1. Problème de calibration (faire calibration via app)")
            print("     2. Position initiale incorrecte")
            print("     3. Problème mécanique (vis, montage)")
            print()

        # Vérifier symétrie (si tête penchée, certains joints seront déséquilibrés)
        print("📐 VÉRIFICATION SYMÉTRIE:")
        print("-" * 60)

        # Calculer moyenne et écart-type
        values_list = [data["value"] for data in results.values()]
        avg = sum(values_list) / len(values_list)
        variance = sum((v - avg) ** 2 for v in values_list) / len(values_list)
        std_dev = variance**0.5

        print(f"Moyenne positions: {format_rad_to_deg(avg)}")
        print(f"Écart-type: {format_rad_to_deg(std_dev)}")

        if std_dev > 0.5:  # Écart-type > 0.5 rad (~28°)
            print("⚠️  DÉSÉQUILIBRE DÉTECTÉ - Tête probablement penchée")
            print("   → Certains joints travaillent plus que d'autres")
            print("   → Vérifier montage et câbles")
        else:
            print("✅ Positions équilibrées")

        print()

    except Exception as e:
        print(f"❌ Erreur lors de la lecture des joints: {e}")
        import traceback

        traceback.print_exc()
        return results

    return results


def test_movement(robot) -> None:
    """Test un petit mouvement pour voir si tous les joints répondent."""
    print("\n" + "=" * 60)
    print("🧪 TEST MOUVEMENT (vérification réponse joints)")
    print("=" * 60)
    print()

    try:
        import numpy as np

        if USE_SDK:
            from reachy_mini.utils import create_head_pose
        else:
            # Backend BBIA utilise aussi create_head_pose
            try:
                from reachy_mini.utils import create_head_pose
            except ImportError:
                # Fallback: créer pose manuellement
                def create_head_pose(
                    roll=0.0, pitch=0.0, yaw=0.0, degrees=False, mm=False
                ):
                    """Créer pose tête manuellement si SDK non disponible."""
                    if degrees:
                        roll = np.deg2rad(roll)
                        pitch = np.deg2rad(pitch)
                        yaw = np.deg2rad(yaw)
                    # Matrice de rotation simple (approximation)
                    pose = np.eye(4, dtype=np.float64)
                    # Rotation pitch (autour Y)
                    cp, sp = np.cos(pitch), np.sin(pitch)
                    # Rotation yaw (autour Z)
                    cy, sy = np.cos(yaw), np.sin(yaw)
                    # Rotation roll (autour X)
                    cr, sr = np.cos(roll), np.sin(roll)
                    # Matrice de rotation combinée (simplifiée)
                    pose[0, 0] = cy * cp
                    pose[0, 1] = cy * sp * sr - sy * cr
                    pose[0, 2] = cy * sp * cr + sy * sr
                    pose[1, 0] = sy * cp
                    pose[1, 1] = sy * sp * sr + cy * cr
                    pose[1, 2] = sy * sp * cr - cy * sr
                    pose[2, 0] = -sp
                    pose[2, 1] = cp * sr
                    pose[2, 2] = cp * cr
                    return pose

        # Position initiale
        print("1️⃣ Position initiale...")
        initial_pose = np.eye(4, dtype=np.float64)
        if USE_SDK:
            robot.goto_target(initial_pose, antennas=[0.0, 0.0], duration=2.0)
        else:
            robot.goto_target(initial_pose, antennas=[0.0, 0.0], duration=2.0)
        time.sleep(2.5)

        # Petit mouvement pitch (haut)
        print("2️⃣ Petit mouvement pitch UP (0.05 rad)...")
        head_pose = create_head_pose(roll=0.0, pitch=0.05, yaw=0.0, degrees=False)
        if USE_SDK:
            robot.goto_target(head_pose, antennas=[0.0, 0.0], duration=1.0)
        else:
            robot.goto_target(head_pose, antennas=[0.0, 0.0], duration=1.0)
        time.sleep(1.5)

        # Retour neutre
        print("3️⃣ Retour position neutre...")
        if USE_SDK:
            robot.goto_target(initial_pose, antennas=[0.0, 0.0], duration=1.0)
        else:
            robot.goto_target(initial_pose, antennas=[0.0, 0.0], duration=1.0)
        time.sleep(1.5)

        print("✅ Test mouvement terminé")
        print()

    except Exception as e:
        print(f"❌ Erreur lors du test mouvement: {e}")
        import traceback

        traceback.print_exc()


def main() -> None:
    """Fonction principale."""
    print("🔧 DIAGNOSTIC JOINTS STEWART - Reachy Mini")
    print("=" * 60)
    print()
    print("Ce script vérifie:")
    print("  • Positions actuelles des 6 joints stewart")
    print("  • Détection de câbles manquants/mal branchés")
    print("  • Symétrie et équilibre")
    print("  • Test de mouvement")
    print()
    print("⚠️  IMPORTANT: Si un câble manque entre bras 1-6,")
    print("   le joint correspondant ne répondra pas correctement.")
    print()

    # Connexion
    print("🔌 Connexion au robot...")
    try:
        if USE_SDK:
            # SDK officiel utilise context manager
            robot = ReachyMini(
                media_backend="no_media", use_sim=False, localhost_only=False
            )
            robot.__enter__()

            # Tester connexion en essayant de lire les joints
            try:
                robot.get_current_joint_positions()
                print("✅ Robot connecté (SDK officiel)")
            except Exception as e:
                print(f"❌ Erreur lors de la connexion: {e}")
                print("   Vérifiez que le daemon Reachy Mini est lancé")
                sys.exit(1)
        else:
            # Backend BBIA
            robot = ReachyMiniBackend(use_sim=False, localhost_only=False)
            robot.connect()

            if not robot.is_connected:
                print("❌ Robot non connecté")
                sys.exit(1)

            print("✅ Robot connecté (Backend BBIA)")

        print()

        # Attendre initialisation
        print("⏳ Attente 2 secondes pour initialisation...")
        time.sleep(2)

        # Diagnostic
        check_stewart_joints(robot)

        # Test mouvement
        test_movement(robot)

        # Re-diagnostic après mouvement
        print("\n" + "=" * 60)
        print("🔄 RE-DIAGNOSTIC APRÈS MOUVEMENT")
        print("=" * 60)
        _ = check_stewart_joints(robot)  # Re-vérifier après mouvement

        print("\n" + "=" * 60)
        print("✅ DIAGNOSTIC TERMINÉ")
        print("=" * 60)
        print()
        print("💡 Si des problèmes sont détectés:")
        print("   1. Vérifier guide d'assemblage officiel:")
        print(
            "      https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide"
        )
        print("   2. Vérifier que TOUS les 6 câbles sont bien branchés")
        print("   3. Vérifier que les câbles sont dans les bons connecteurs")
        print("   4. Si problème persiste, contacter support Pollen Robotics")
        print()

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
