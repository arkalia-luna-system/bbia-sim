#!/usr/bin/env python3
"""Playground interpolation pour Reachy Mini - BBIA-SIM.

Démontre les différentes méthodes d'interpolation disponibles :
- linear: Interpolation linéaire
- minjerk: Minimum jerk (défaut, mouvement fluide)
- ease: Ease in/out
- cartoon: Style cartoon

Adapté du repo officiel pour BBIA-SIM.

Usage:
    python examples/reachy_mini/goto_interpolation_playground.py
"""

import numpy as np

# Essayer d'utiliser le SDK officiel si disponible
try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose
    from reachy_mini.utils.interpolation import InterpolationTechnique

    USE_SDK = True
except ImportError:
    USE_SDK = False
    print("⚠️  SDK officiel non disponible, utilisation du backend BBIA")
    InterpolationTechnique = None

if not USE_SDK:
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

    # Pour BBIA, utiliser les constantes d'interpolation si disponibles
    if InterpolationTechnique is None:
        # Fallback: définir manuellement
        from enum import Enum

        class InterpolationTechniqueLocal(Enum):  # Renommé pour éviter conflit
            LINEAR = "linear"
            MIN_JERK = "minjerk"
            EASE_IN_OUT = "ease"
            CARTOON = "cartoon"

        InterpolationTechnique = InterpolationTechniqueLocal  # type: ignore[assignment]


def main() -> None:
    """Tester les différentes méthodes d'interpolation."""
    print("🤖 Playground interpolation - Reachy Mini")
    print("=" * 50)

    if USE_SDK:
        with ReachyMini(media_backend="no_media", use_sim=True) as mini:
            try:
                print("🎯 Test de toutes les méthodes d'interpolation...\n")

                for method in InterpolationTechnique:
                    print(f"📊 Méthode: {method.value}")
                    print("-" * 30)

                    # Position neutre
                    pose = create_head_pose(x=0, y=0, z=0, yaw=0)
                    mini.goto_target(pose, duration=1.0, method=method)
                    print("  ✅ Position neutre atteinte")

                    # Mouvements avec cette méthode
                    for i in range(3):
                        pose = create_head_pose(
                            x=0.0, y=0.03, z=0, roll=5, yaw=-10, degrees=True
                        )
                        mini.goto_target(
                            pose,
                            antennas=np.deg2rad([-20, 20]),
                            duration=1.0,
                            method=method,
                        )
                        print(f"  ✅ Mouvement {i+1}/3 vers droite")

                        pose = create_head_pose(
                            x=0.0, y=-0.03, z=0, roll=-5, yaw=10, degrees=True
                        )
                        mini.goto_target(
                            pose,
                            antennas=np.deg2rad([20, -20]),
                            duration=1.0,
                            method=method,
                        )
                        print(f"  ✅ Mouvement {i+1}/3 vers gauche")

                    # Retour position neutre
                    pose = create_head_pose(x=0, y=0, z=0, yaw=0)
                    mini.goto_target(pose, duration=1.0, antennas=[0, 0], method=method)
                    print("  ✅ Retour position neutre\n")

            except KeyboardInterrupt:
                print("\n⏹️  Interrompu")

    else:
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        try:
            print("🎯 Test de toutes les méthodes d'interpolation...\n")

            for method in InterpolationTechnique:
                print(f"📊 Méthode: {method.value}")
                print("-" * 30)

                pose = create_head_pose(x=0, y=0, z=0, yaw=0)
                backend.goto_target(pose, duration=1.0, method=method)
                print("  ✅ Position neutre atteinte")

                # Répéter mouvements similaires
                for _i in range(3):
                    pose = create_head_pose(
                        x=0.0, y=0.03, z=0, roll=5, yaw=-10, degrees=True
                    )
                    backend.goto_target(
                        pose,
                        antennas=np.deg2rad([-20, 20]),
                        duration=1.0,
                        method=method,
                    )

                    pose = create_head_pose(
                        x=0.0, y=-0.03, z=0, roll=-5, yaw=10, degrees=True
                    )
                    backend.goto_target(
                        pose,
                        antennas=np.deg2rad([20, -20]),
                        duration=1.0,
                        method=method,
                    )

                pose = create_head_pose(x=0, y=0, z=0, yaw=0)
                backend.goto_target(pose, duration=1.0, antennas=[0, 0], method=method)
                print("  ✅ Retour position neutre\n")

        except KeyboardInterrupt:
            print("\n⏹️  Interrompu")
        finally:
            backend.disconnect()

    print("✅ Demo terminée")


if __name__ == "__main__":
    main()
