#!/usr/bin/env python3
"""Demo minimale pour Reachy Mini - BBIA-SIM.

Exemple basique montrant comment utiliser goto_target et set_target avec les antennes.
Adapté du repo officiel pour BBIA-SIM.

Usage:
    python examples/reachy_mini/minimal_demo.py
"""

import time

import numpy as np

# Essayer d'utiliser le SDK officiel si disponible
try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose

    USE_SDK = True
except ImportError:
    USE_SDK = False
    print("⚠️  SDK officiel non disponible, utilisation du backend BBIA")

if not USE_SDK:
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


def main() -> None:
    """Demo minimale : mouvements tête et antennes."""
    print("🤖 Demo minimale Reachy Mini")
    print("=" * 50)

    if USE_SDK:
        # Utilisation directe du SDK officiel
        print("✅ Utilisation SDK officiel")
        with ReachyMini(media_backend="no_media", use_sim=True) as mini:
            mini.goto_target(create_head_pose(), antennas=[0.0, 0.0], duration=1.0)
            print("🎯 Mouvement initial vers position neutre")

            try:
                print("📡 Démarrage animation antennes et tête...")
                print("   (Appuyez sur Ctrl+C pour arrêter)")
                while True:
                    t = time.time()

                    # Animation des antennes (sinusoïde)
                    antennas_offset = np.deg2rad(20 * np.sin(2 * np.pi * 0.5 * t))
                    # Animation pitch de la tête (sinusoïde)
                    pitch = np.deg2rad(10 * np.sin(2 * np.pi * 0.5 * t))

                    # Créer pose de la tête avec pitch animé
                    head_pose = create_head_pose(
                        roll=0.0,
                        pitch=pitch,
                        yaw=0.0,
                        degrees=False,
                        mm=False,
                    )
                    # Appliquer mouvement
                    mini.set_target(
                        head=head_pose, antennas=(antennas_offset, antennas_offset)
                    )
                    time.sleep(0.02)  # ~50 Hz

            except KeyboardInterrupt:
                print("\n⏹️  Arrêt de l'animation")

    else:
        # Utilisation backend BBIA
        print("✅ Utilisation backend BBIA")
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        try:
            # Position initiale
            from reachy_mini.utils import create_head_pose

            backend.goto_target(
                create_head_pose(), antennas=[0.0, 0.0], duration=1.0
            )
            print("🎯 Mouvement initial vers position neutre")

            print("📡 Démarrage animation antennes et tête...")
            print("   (Appuyez sur Ctrl+C pour arrêter)")
            while True:
                t = time.time()

                antennas_offset = np.deg2rad(20 * np.sin(2 * np.pi * 0.5 * t))
                pitch = np.deg2rad(10 * np.sin(2 * np.pi * 0.5 * t))

                head_pose = create_head_pose(
                    roll=0.0,
                    pitch=pitch,
                    yaw=0.0,
                    degrees=False,
                    mm=False,
                )
                backend.set_target(
                    head=head_pose, antennas=(antennas_offset, antennas_offset)
                )
                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n⏹️  Arrêt de l'animation")
        finally:
            backend.disconnect()

    print("✅ Demo terminée")


if __name__ == "__main__":
    main()

