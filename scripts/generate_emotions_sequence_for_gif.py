#!/usr/bin/env python3
"""
Script pour générer séquence d'émotions pour GIF
Enchaîne automatiquement : happy (2s) → neutral (1s) → curious (2s) → excited (2s) → calm (1s)
Total : 8 secondes, prêt pour enregistrement écran

Configuration :
- Caméra orientée face au robot (azimuth configurable via --azimuth)
- Note : Le fond noir est une limitation de MuJoCo viewer (utiliser post-production pour fond pastel)
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory


def emotion_to_pose(
    emotion: str, intensity: float, step: int, total_steps: int
) -> float:
    """Convertit une émotion en position d'articulation."""
    t = step / total_steps

    emotion_patterns = {
        "happy": (
            lambda t: 0.12
            * math.sin(2 * math.pi * 0.1 * t)
            * (1 + 0.5 * math.sin(4 * math.pi * t))
        ),
        "sad": (
            lambda t: -0.12 * math.sin(2 * math.pi * 0.3 * t)
            - 0.04 * math.sin(6 * math.pi * t)
        ),
        "angry": (
            lambda t: 0.18 * math.sin(2 * math.pi * 0.8 * t)
            + 0.04 * math.sin(8 * math.pi * t)
        ),
        "surprised": (
            lambda t: 0.15 * math.sin(2 * math.pi * 0.2 * t) * math.cos(3 * math.pi * t)
        ),
        "neutral": (
            lambda t: 0.08 * math.sin(2 * math.pi * 0.1 * t)
            + 0.03 * math.sin(6 * math.pi * t)
        ),
        "curious": (
            lambda t: 0.10
            * math.sin(2 * math.pi * 0.15 * t)
            * (1 + 0.3 * math.sin(3 * math.pi * t))
        ),
        "excited": (
            lambda t: 0.16
            * math.sin(2 * math.pi * 0.12 * t)
            * (1 + 0.8 * math.sin(5 * math.pi * t))
        ),
        "calm": lambda t: 0.06 * math.sin(2 * math.pi * 0.08 * t),
    }

    base_movement = emotion_patterns.get(emotion, emotion_patterns["neutral"])(t)
    return base_movement * intensity


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Génère séquence d'émotions pour GIF")
    parser.add_argument(
        "--azimuth",
        type=float,
        default=90.0,
        help="Angle azimuth caméra (0=droite, 90=face, 180=gauche, 270=dos). Testez différentes valeurs!",
    )
    args = parser.parse_args()

    # Séquence d'émotions avec durées
    sequence = [
        ("happy", 2.0, 0.8),
        ("neutral", 1.0, 0.5),
        ("curious", 2.0, 0.7),
        ("excited", 2.0, 0.9),
        ("calm", 1.0, 0.6),
    ]

    # Créer le robot
    robot = RobotFactory.create_backend("mujoco")
    if not robot or not robot.connect():
        print("❌ Impossible de se connecter")
        return 1

    print("✅ Robot connecté")

    # Accéder au modèle et données MuJoCo directement
    if (
        not hasattr(robot, "model")
        or not hasattr(robot, "data")
        or robot.model is None
        or robot.data is None
    ):
        print("❌ Impossible d'accéder au modèle MuJoCo")
        return 1


    # Lancer viewer via le backend du robot
    print("✅ Lancement viewer MuJoCo...")
    if not robot.launch_viewer(passive=True):
        print("❌ Impossible de lancer le viewer")
        return 1

    # Attendre que le viewer soit prêt
    time.sleep(0.5)

    # Accéder au viewer du backend
    if not hasattr(robot, "viewer") or robot.viewer is None:
        print("❌ Viewer non disponible")
        return 1

    viewer = robot.viewer

    # Configurer caméra avec l'azimuth fourni en argument
    # 0° = côté droit, 90° = face, 180° = côté gauche, 270° = dos
    viewer.cam.azimuth = args.azimuth
    viewer.cam.elevation = -15.0  # Légèrement au-dessus
    viewer.cam.distance = 1.5
    viewer.cam.lookat[:] = [0.0, 0.0, 0.3]

    # Synchroniser pour appliquer la configuration de la caméra
    viewer.sync()
    time.sleep(0.2)  # Laisser le viewer se stabiliser

    print(f"✅ Caméra configurée (azimuth={args.azimuth}°)")
    print("💡 Si le robot est encore de côté, testez: --azimuth 90, 180, 270, ou -90")
    print(
        "⚠️  Fond noir : limitation de MuJoCo viewer (utiliser post-production vidéo pour fond pastel)"
    )

    print("\n🎬 DÉBUT SÉQUENCE (8 secondes)")
    print("📹 Préparez votre enregistrement d'écran maintenant !\n")
    time.sleep(2)  # Temps pour démarrer l'enregistrement

    fps = 10
    total_duration = sum(duration for _, duration, _ in sequence)

    print(f"⏱️  Durée totale : {total_duration}s\n")

    step_count = 0
    for emotion, duration, intensity in sequence:
        total_steps = int(duration * fps)
        print(f"🎭 {emotion.upper()} ({duration}s, intensité {intensity})")

        for step in range(total_steps):
            angle = emotion_to_pose(emotion, intensity, step, total_steps)
            robot.set_joint_pos("yaw_body", angle)
            robot.step()

            # Maintenir la configuration de la caméra à chaque frame
            viewer.cam.azimuth = args.azimuth
            viewer.cam.elevation = -15.0
            viewer.cam.distance = 1.5
            viewer.cam.lookat[:] = [0.0, 0.0, 0.3]

            # Mettre à jour la simulation
            viewer.sync()

            step_count += 1
            time.sleep(1.0 / fps)

    print(f"\n✅ Séquence terminée ({total_duration}s)")
    print("⏸️  Arrêtez votre enregistrement d'écran")
    print("💾 Puis convertissez la vidéo en GIF avec ffmpeg")
    print("\n💡 Pour le fond pastel : utilisez un filtre vidéo (chromakey/fond coloré)")
    time.sleep(2)

    # Fermer proprement le viewer (sans force kill qui peut causer des problèmes)
    print("\n🔄 Fermeture du viewer...")
    try:
        if hasattr(robot, "viewer") and robot.viewer is not None:
            # Fermeture simple et sûre
            try:
                if hasattr(robot.viewer, "close"):
                    robot.viewer.close()
                    time.sleep(0.3)  # Laisser le temps de fermer
                    print("✅ Viewer fermé")
            except AttributeError:
                # Si le viewer n'a pas de méthode close, il se fermera avec le contexte
                pass
            except Exception as e:
                print(f"⚠️  Viewer déjà fermé ou erreur mineure: {e}")
    except Exception as e:
        print(f"⚠️  Note: {e}")

    # Déconnecter le robot
    try:
        robot.disconnect()
        print("✅ Robot déconnecté")
    except Exception as e:
        print(f"⚠️  Note lors de la déconnexion: {e}")

    print("\n💡 Si une fenêtre MuJoCo reste ouverte, fermez-la avec Cmd+Q")

    return 0


if __name__ == "__main__":
    exit(main())
