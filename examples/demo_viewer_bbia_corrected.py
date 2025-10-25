#!/usr/bin/env python3
"""
Démo Viewer BBIA Corrigée - Script de démonstration 3D avec gestion des joints
Script de démonstration pour BBIA avec validation des joints et animation sécurisée
"""

import argparse
import math
import sys
import time
from pathlib import Path

import mujoco

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_emotions import BBIAEmotions


def analyze_joints(model):
    """Analyse tous les joints et les classe par sécurité."""
    safe_joints = []
    problematic_joints = []
    blocked_joints = []

    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_range = model.jnt_range[i]

        if joint_range[0] == joint_range[1]:  # Joint bloqué
            blocked_joints.append((name, joint_range))
        elif name == "yaw_body":  # Joint très sûr
            safe_joints.append((name, joint_range, "TRÈS SÛR"))
        elif "stewart" in name:  # Joints Stewart - risqués
            range_size = joint_range[1] - joint_range[0]
            if range_size > 2.0:
                problematic_joints.append((name, joint_range, "PROBLÉMATIQUE"))
            else:
                safe_joints.append((name, joint_range, "SÛR"))
        else:
            safe_joints.append((name, joint_range, "SÛR"))

    return safe_joints, problematic_joints, blocked_joints


def list_joints(model_path):
    """Affiche la liste des joints classés par sécurité."""
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        safe_joints, problematic_joints, blocked_joints = analyze_joints(model)

        print("🔍 ANALYSE DES JOINTS BBIA")
        print("=" * 50)
        print(f"📊 Total joints: {model.njnt}")
        print(f"✅ Joints sûrs: {len(safe_joints)}")
        print(f"⚠️  Joints problématiques: {len(problematic_joints)}")
        print(f"❌ Joints bloqués: {len(blocked_joints)}")

        print(f"\n✅ JOINTS SÛRS ({len(safe_joints)}):")
        for name, joint_range, reason in safe_joints:
            range_size = joint_range[1] - joint_range[0]
            print(
                f"   • {name:12} | [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] | {range_size:5.3f} rad | {reason}"
            )

        if problematic_joints:
            print(f"\n⚠️  JOINTS PROBLÉMATIQUES ({len(problematic_joints)}):")
            for name, joint_range, reason in problematic_joints:
                range_size = joint_range[1] - joint_range[0]
                print(
                    f"   • {name:12} | [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] | {range_size:5.3f} rad | {reason}"
                )

        print(f"\n❌ JOINTS BLOQUÉS ({len(blocked_joints)}):")
        for name, joint_range in blocked_joints:
            print(
                f"   • {name:12} | [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] | BLOQUÉ"
            )

        print("\n🎯 RECOMMANDATIONS:")
        print("   • Pour les démos: Utilisez SEULEMENT les joints sûrs")
        print("   • Joint le plus sûr: yaw_body (rotation du corps)")
        print("   • Évitez les joints problématiques en production")

        return True

    except Exception as e:
        print(f"❌ Erreur lors de l'analyse des joints: {e}")
        return False


def validate_joint(model, joint_name):
    """Valide qu'un joint existe et est sûr."""
    safe_joints, problematic_joints, blocked_joints = analyze_joints(model)

    # Vérifier si le joint existe
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id == -1:
        print(f"❌ Joint '{joint_name}' non trouvé")
        return False

    # Vérifier si le joint est sûr
    safe_joint_names = [name for name, _, _ in safe_joints]
    if joint_name not in safe_joint_names:
        print(f"❌ Joint '{joint_name}' non sûr")
        return False

    return True


def clamp_amplitude(amplitude, max_amplitude=0.3):
    """Clamp l'amplitude pour la sécurité."""
    if amplitude > max_amplitude:
        print(f"⚠️  Amplitude {amplitude} trop élevée, ajustée à {max_amplitude}")
        return max_amplitude
    return amplitude


def animate_joint(
    model, data, joint_name, duration, frequency=0.1, amplitude=0.2, headless=False
):
    """Anime un joint avec un mouvement sinusoïdal."""
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    joint_range = model.jnt_range[joint_id]

    # Clamp l'amplitude
    amplitude = clamp_amplitude(amplitude)

    print(f"🎮 Animation du joint: {joint_name}")
    print(f"📏 Limites: [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] rad")
    print(f"📊 Fréquence: {frequency} Hz")
    print(f"📊 Amplitude: {amplitude} rad")
    print(f"⏱️  Durée: {duration}s")
    print(f"🖥️  Mode: {'headless' if headless else 'graphique'}")

    # Initialiser BBIA Émotions
    emotions = BBIAEmotions()
    print(f"🎭 BBIA Émotions: {len(emotions.emotions)} émotions disponibles")

    # Configuration animation
    fps = 100
    total_steps = int(duration * fps)
    step_count = 0

    print("\n🚀 Démarrage animation...")
    start_time = time.time()

    try:
        for step in range(total_steps):
            # Calculer la position sinusoïdale
            t = step / total_steps
            position = amplitude * math.sin(2 * math.pi * frequency * t)

            # Appliquer la position au joint
            data.qpos[joint_id] = position

            # Step de simulation
            mujoco.mj_step(model, data)
            step_count += 1

            # Affichage périodique
            if step % (fps // 2) == 0:  # Toutes les 0.5s
                elapsed = time.time() - start_time
                print(
                    f"⏱️  {elapsed:.1f}s | Position: {position:.3f} rad | steps: {step_count}"
                )

    except KeyboardInterrupt:
        print("\n⏹️  Animation interrompue par l'utilisateur")

    end_time = time.time()
    actual_duration = end_time - start_time

    print("\n✅ Animation terminée")
    print(f"📊 Durée réelle: {actual_duration:.2f}s")
    print(f"📊 steps exécutés: {step_count}")
    print(f"📊 Fréquence moyenne: {step_count/actual_duration:.1f} Hz")

    if headless:
        print(f"Animation headless terminée avec {step_count} steps")

    return True


def main():
    parser = argparse.ArgumentParser(description="Démo Viewer BBIA Corrigée")
    parser.add_argument(
        "--xml",
        default="src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml",
        help="Chemin vers le modèle XML",
    )
    parser.add_argument(
        "--list-joints", action="store_true", help="Lister tous les joints"
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument("--duration", type=float, default=2.0, help="Durée en secondes")
    parser.add_argument("--joint", default="yaw_body", help="Joint à animer")
    parser.add_argument(
        "--frequency", type=float, default=0.1, help="Fréquence en Hz (SÉCURISÉ)"
    )
    parser.add_argument(
        "--amplitude", type=float, default=0.2, help="Amplitude en rad (SÉCURISÉ)"
    )

    args = parser.parse_args()

    # Vérifier que le fichier XML existe
    if not Path(args.xml).exists():
        print(f"❌ Fichier XML introuvable: {args.xml}")
        return 1

    try:
        # Charger le modèle
        model = mujoco.MjModel.from_xml_path(args.xml)
        data = mujoco.MjData(model)
        print(f"✅ Modèle chargé: {model.njnt} joints détectés")
    except Exception as e:
        print(f"❌ Erreur chargement modèle: {e}")
        return 1

    # Mode list-joints
    if args.list_joints:
        return 0 if list_joints(args.xml) else 1

    # Valider le joint
    if not validate_joint(model, args.joint):
        print("✅ Joints sûrs disponibles:")
        safe_joints, _, _ = analyze_joints(model)
        for name, _, reason in safe_joints:
            print(f"  • {name} ({reason})")
        return 1

    # Lancer l'animation
    success = animate_joint(
        model,
        data,
        args.joint,
        args.duration,
        args.frequency,
        args.amplitude,
        args.headless,
    )

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
