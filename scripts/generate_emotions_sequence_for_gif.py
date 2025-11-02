#!/usr/bin/env python3
"""Script pour générer séquence d'émotions pour GIF
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
    emotion: str,
    intensity: float,
    step: int,
    total_steps: int,
) -> float:
    """Convertit une émotion en position d'articulation."""
    t = step / total_steps

    emotion_patterns = {
        # Happy : mouvement énergique et joyeux avec variation d'amplitude
        "happy": (
            lambda t: 0.15
            * math.sin(2 * math.pi * 0.12 * t)
            * (1 + 0.6 * math.sin(4 * math.pi * t))
            + 0.03 * math.cos(8 * math.pi * t)
        ),
        # Sad : mouvement lent vers le bas avec oscillation douce
        "sad": (
            lambda t: -0.10 * math.sin(2 * math.pi * 0.25 * t)
            - 0.05 * math.sin(6 * math.pi * t)
            - 0.02 * (1 - math.cos(2 * math.pi * 0.5 * t))
        ),
        # Angry : mouvement rapide et saccadé
        "angry": (
            lambda t: 0.20 * math.sin(2 * math.pi * 0.9 * t)
            + 0.06 * math.sin(10 * math.pi * t)
            + 0.04 * math.cos(15 * math.pi * t)
        ),
        # Surprised : mouvement brusque avec rebond
        "surprised": (
            lambda t: 0.18
            * math.sin(2 * math.pi * 0.18 * t)
            * math.cos(3 * math.pi * t)
            + 0.05 * math.exp(-5 * (t - 0.5) ** 2)  # Pics de surprise
        ),
        # Neutral : mouvement minimal et régulier
        "neutral": (
            lambda t: 0.06 * math.sin(2 * math.pi * 0.08 * t)
            + 0.02 * math.sin(6 * math.pi * t)
            + 0.01 * math.cos(12 * math.pi * t)
        ),
        # Curious : mouvement exploratoire avec variations
        "curious": (
            lambda t: 0.12
            * math.sin(2 * math.pi * 0.14 * t)
            * (1 + 0.4 * math.sin(3 * math.pi * t))
            + 0.04 * math.sin(7 * math.pi * t) * math.cos(5 * math.pi * t)
        ),
        # Excited : mouvement rapide et dynamique
        "excited": (
            lambda t: 0.18
            * math.sin(2 * math.pi * 0.15 * t)
            * (1 + 0.9 * math.sin(5 * math.pi * t))
            + 0.05 * math.sin(10 * math.pi * t)
            + 0.03 * math.cos(12 * math.pi * t)
        ),
        # Calm : mouvement très doux et lent
        "calm": (
            lambda t: 0.05 * math.sin(2 * math.pi * 0.06 * t)
            + 0.02 * math.sin(4 * math.pi * t) * math.exp(-0.5 * t)
        ),
    }

    base_movement = emotion_patterns.get(emotion, emotion_patterns["neutral"])(t)
    return base_movement * intensity


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Génère séquence d'émotions pour GIF")
    parser.add_argument(
        "--azimuth",
        type=float,
        default=180.0,
        help="Angle azimuth caméra (0=droite, 90=face, 180=face optimal, 270=dos). Défaut: 180",
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

    # Accéder au viewer IMMÉDIATEMENT et configurer la caméra AVANT le premier affichage
    if not hasattr(robot, "viewer") or robot.viewer is None:
        print("❌ Viewer non disponible")
        return 1

    viewer = robot.viewer

    # Configurer caméra IMMÉDIATEMENT (avant le premier sync) pour éviter la mauvaise première vue
    # 0° = côté droit, 90° = face, 180° = face optimal, 270° = dos
    viewer.cam.azimuth = args.azimuth
    viewer.cam.elevation = -15.0  # Légèrement au-dessus
    viewer.cam.distance = 1.2  # Rapproché de 20% (était 1.5, maintenant 1.2)
    viewer.cam.lookat[:] = [0.0, 0.0, 0.3]

    # Première synchronisation avec la bonne caméra déjà configurée
    viewer.sync()
    time.sleep(0.3)  # Laisser le viewer se stabiliser avec la bonne caméra

    print(f"✅ Caméra configurée (azimuth={args.azimuth}°, distance=1.2)")
    print("✅ Fond pastel configuré dans le modèle XML (skybox gradient)")
    print("💡 Si le robot est encore de côté, testez: --azimuth 90, 180, 270, ou -90")

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
            viewer.cam.distance = 1.2  # Rapproché de 20%
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
