#!/usr/bin/env python3
"""Demo séquences de mouvements pour Reachy Mini - BBIA-SIM.

Démonstration de différentes séquences de mouvements animés :
- Rotation yaw (gauche/droite)
- Rotation pitch (haut/bas)
- Rotation roll
- Translation verticale
- Animation antennes
- Mouvements combinés

Adapté du repo officiel pour BBIA-SIM.

Usage:
    python examples/reachy_mini/sequence.py
"""

import time

import numpy as np
from scipy.spatial.transform import Rotation as R

# Essayer d'utiliser le SDK officiel si disponible
try:
    from reachy_mini import ReachyMini

    USE_SDK = True
except ImportError:
    USE_SDK = False
    print("⚠️  SDK officiel non disponible, utilisation du backend BBIA")

if not USE_SDK:
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


def main() -> None:
    """Exécuter les séquences de mouvements."""
    print("🤖 Demo séquences de mouvements - Reachy Mini")
    print("=" * 50)

    if USE_SDK:
        with ReachyMini(media_backend="no_media", use_sim=True) as reachy_mini:
            reachy_mini.goto_target(np.eye(4), antennas=[0.0, 0.0], duration=1.0)
            print("✅ Position initiale atteinte")

            try:
                print("🎬 Démarrage séquences de mouvements...")
                print("   (Appuyez sur Ctrl+C pour arrêter)")

                while True:
                    pose = np.eye(4)
                    t0 = time.time()

                    # Séquence 1: Rotation Yaw (gauche/droite)
                    print("📐 Séquence 1: Rotation Yaw")
                    s = time.time()
                    while time.time() - s < 2.0:
                        t = time.time() - t0
                        euler_rot = np.array(
                            [0, 0.0, 0.7 * np.sin(2 * np.pi * 0.5 * t)]
                        )
                        rot_mat = R.from_euler(
                            "xyz", euler_rot, degrees=False
                        ).as_matrix()
                        pose[:3, :3] = rot_mat
                        reachy_mini.set_target(head=pose, antennas=[0, 0])
                        time.sleep(0.01)

                    # Séquence 2: Rotation Pitch (haut/bas)
                    print("📐 Séquence 2: Rotation Pitch")
                    s = time.time()
                    while time.time() - s < 2.0:
                        t = time.time() - t0
                        euler_rot = np.array([0, 0.3 * np.sin(2 * np.pi * 0.5 * t), 0])
                        rot_mat = R.from_euler(
                            "xyz", euler_rot, degrees=False
                        ).as_matrix()
                        pose[:3, :3] = rot_mat
                        reachy_mini.set_target(head=pose, antennas=[0, 0])
                        time.sleep(0.01)

                    # Séquence 3: Rotation Roll
                    print("📐 Séquence 3: Rotation Roll")
                    s = time.time()
                    while time.time() - s < 2.0:
                        t = time.time() - t0
                        euler_rot = np.array([0.3 * np.sin(2 * np.pi * 0.5 * t), 0, 0])
                        rot_mat = R.from_euler(
                            "xyz", euler_rot, degrees=False
                        ).as_matrix()
                        pose[:3, :3] = rot_mat
                        reachy_mini.set_target(head=pose, antennas=[0, 0])
                        time.sleep(0.01)

                    # Séquence 4: Translation verticale (Z)
                    print("📐 Séquence 4: Translation Z")
                    s = time.time()
                    while time.time() - s < 2.0:
                        t = time.time() - t0
                        pose = np.eye(4)
                        pose[:3, 3][2] += 0.025 * np.sin(2 * np.pi * 0.5 * t)
                        reachy_mini.set_target(head=pose, antennas=[0, 0])
                        time.sleep(0.01)

                    # Séquence 5: Animation antennes
                    print("📡 Séquence 5: Animation antennes")
                    s = time.time()
                    while time.time() - s < 2.0:
                        t = time.time() - t0
                        antennas = [
                            0.5 * np.sin(2 * np.pi * 0.5 * t),
                            -0.5 * np.sin(2 * np.pi * 0.5 * t),
                        ]
                        reachy_mini.set_target(head=pose, antennas=antennas)
                        time.sleep(0.01)

                    # Séquence 6: Mouvement combiné (translation XY circulaire)
                    print("🌀 Séquence 6: Mouvement combiné XY")
                    s = time.time()
                    while time.time() - s < 5.0:
                        t = time.time() - t0
                        pose[:3, 3] = [
                            0.015 * np.sin(2 * np.pi * 1.0 * t),
                            0.015 * np.sin(2 * np.pi * 1.0 * t + np.pi / 2),
                            0.0,
                        ]
                        reachy_mini.set_target(head=pose, antennas=[0, 0])
                        time.sleep(0.01)

                    # Retour position neutre
                    pose[:3, 3] = [0, 0, 0.0]
                    reachy_mini.set_target(head=pose, antennas=[0, 0])
                    time.sleep(0.5)

                    # Positions cibles fixes
                    print("🎯 Positions cibles fixes")
                    offsets: list[list[float]] = [[0.02, 0.02], [0.00, 0.02], [0.00, -0.02], [0, 0]]
                    for offset in offsets:
                        pose[:3, 3] = [offset[0], offset[1], 0.0]
                        if offset[0] == 0 and offset[1] != 0:
                            yaw = 0.5 if offset[1] > 0 else -0.5
                            euler_rot = np.array([0, 0, yaw])
                            rot_mat = R.from_euler(
                                "xyz", euler_rot, degrees=False
                            ).as_matrix()
                            pose[:3, :3] = rot_mat
                        reachy_mini.set_target(head=pose, antennas=[0, 0])
                        time.sleep(0.5)

                    time.sleep(2)

            except KeyboardInterrupt:
                print("\n⏹️  Arrêt des séquences")

    else:
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        try:
            backend.goto_target(np.eye(4), antennas=[0.0, 0.0], duration=1.0)
            print("✅ Position initiale atteinte")

            # Répéter les mêmes séquences avec backend BBIA
            # (Code similaire, mais utilise backend au lieu de reachy_mini)
            # ... (même logique que ci-dessus)

        except KeyboardInterrupt:
            print("\n⏹️  Arrêt des séquences")
        finally:
            backend.disconnect()

    print("✅ Demo terminée")


if __name__ == "__main__":
    main()
