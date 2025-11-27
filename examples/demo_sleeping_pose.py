#!/usr/bin/env python3
"""Démonstration de la pose de sommeil améliorée (Issue #410).

Ce script montre comment utiliser la méthode set_sleeping_pose()
pour mettre le robot en position de sommeil naturelle.
"""

import sys
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory


def main() -> None:
    """Démonstration pose de sommeil."""
    print("🌙 Démonstration pose de sommeil améliorée (Issue #410)")
    print("=" * 60)

    # Créer le backend
    backend = RobotFactory.create_backend("reachy_mini")

    if backend is None:
        print("❌ Erreur: Impossible de créer le backend")
        return

    # Connexion
    print("\n1. Connexion au robot...")
    if backend.connect():
        print("✅ Connexion réussie")
    else:
        print("⚠️  Mode simulation (pas de robot physique)")

    # Test pose de sommeil par défaut (2 secondes)
    print("\n2. Pose de sommeil par défaut (2 secondes)...")
    if hasattr(backend, "set_sleeping_pose"):
        result = backend.set_sleeping_pose()  # type: ignore[attr-defined]
        if result:
            print("✅ Pose de sommeil définie avec succès")
        else:
            print("⚠️  Pose de sommeil partielle ou simplifiée")
    else:
        print("❌ Méthode set_sleeping_pose() non disponible")

    # Test pose de sommeil avec durée personnalisée
    print("\n3. Pose de sommeil avec durée personnalisée (3 secondes)...")
    if hasattr(backend, "set_sleeping_pose"):
        result = backend.set_sleeping_pose(duration=3.0)  # type: ignore[attr-defined]
        if result:
            print("✅ Pose de sommeil définie avec succès (durée: 3s)")
        else:
            print("⚠️  Pose de sommeil partielle ou simplifiée")
    else:
        print("❌ Méthode set_sleeping_pose() non disponible")

    # Test collision check (Issue #183)
    print("\n4. Vérification collision...")
    if hasattr(backend, "check_collision"):
        has_collision = backend.check_collision()  # type: ignore[attr-defined]
        if has_collision:
            print("⚠️  Collision détectée")
        else:
            print("✅ Aucune collision détectée")
    else:
        print("⚠️  Méthode check_collision() non disponible (normal en simulation)")

    # Déconnexion
    print("\n5. Déconnexion...")
    backend.disconnect()
    print("✅ Déconnexion réussie")

    print("\n" + "=" * 60)
    print("🎉 Démonstration terminée !")
    print("=" * 60)


if __name__ == "__main__":
    main()
