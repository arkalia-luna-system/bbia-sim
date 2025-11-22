#!/usr/bin/env python3
"""Démonstration de la détection de collision (Issue #183).

Ce script montre comment utiliser la méthode check_collision()
pour détecter les collisions dans la simulation MuJoCo.
"""

import sys
import time
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory


def main() -> None:
    """Démonstration détection collision."""
    print("🔍 Démonstration détection de collision (Issue #183)")
    print("=" * 60)

    # Créer le backend MuJoCo (collision check disponible uniquement en simulation)
    backend = RobotFactory.create_backend("mujoco")

    if backend is None:
        print("❌ Erreur: Impossible de créer le backend MuJoCo")
        return

    # Connexion
    print("\n1. Connexion à la simulation...")
    if backend.connect():
        print("✅ Connexion réussie")
    else:
        print("❌ Erreur: Impossible de se connecter à la simulation")
        return

    # Vérifier collision initiale
    print("\n2. Vérification collision initiale...")
    if hasattr(backend, "check_collision"):
        has_collision = backend.check_collision()  # type: ignore[attr-defined]
        print(f"   Collision détectée: {'Oui ⚠️' if has_collision else 'Non ✅'}")
    else:
        print("❌ Méthode check_collision() non disponible")
        backend.disconnect()
        return

    # Test mouvement normal (ne devrait pas causer collision)
    print("\n3. Test mouvement normal...")
    if hasattr(backend, "set_joint_pos"):
        backend.set_joint_pos("yaw_body", 0.2)  # type: ignore[attr-defined]
        backend.step()  # type: ignore[attr-defined]
        time.sleep(0.5)

        if hasattr(backend, "check_collision"):
            has_collision = backend.check_collision()  # type: ignore[attr-defined]
            print(
                f"   Collision après mouvement: {'Oui ⚠️' if has_collision else 'Non ✅'}"
            )

    # Test collision check continu
    print("\n4. Test collision check continu (5 vérifications)...")
    if hasattr(backend, "check_collision"):
        collision_count = 0
        for _ in range(5):
            has_collision = backend.check_collision()  # type: ignore[attr-defined]
            if has_collision:
                collision_count += 1
            time.sleep(0.2)
        print(f"   Collisions détectées: {collision_count}/5")

    # Déconnexion
    print("\n5. Déconnexion...")
    backend.disconnect()
    print("✅ Déconnexion réussie")

    print("\n" + "=" * 60)
    print("🎉 Démonstration terminée !")
    print("=" * 60)
    print(
        "\n💡 Note: La détection de collision est disponible uniquement en simulation MuJoCo."
    )


if __name__ == "__main__":
    main()
