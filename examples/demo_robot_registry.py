#!/usr/bin/env python3
"""Démonstration du registre multi-robots (Issue #30).

Ce script montre comment utiliser create_robot_registry()
pour créer un registre de robots disponibles.
"""

import sys
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory


def main() -> None:
    """Démonstration registre robots."""
    print("🤖 Démonstration registre multi-robots (Issue #30)")
    print("=" * 60)

    # Créer le registre de robots
    print("\n1. Création du registre de robots...")
    registry = RobotFactory.create_robot_registry()

    print("\n2. Informations du registre:")
    print(f"   Robot ID: {registry.get('robot_id', 'N/A')}")
    print(f"   Hostname: {registry.get('hostname', 'N/A')}")
    print(f"   Port: {registry.get('port', 'N/A')}")
    print(f"   Backends disponibles: {len(registry.get('backends_available', []))}")

    print("\n3. Backends disponibles:")
    for backend_name in registry.get("backends_available", []):
        print(f"   - {backend_name}")

    # Test création backend depuis registre
    print("\n4. Test création backend depuis registre...")
    available_backends = registry.get("backends_available", [])
    if available_backends:
        backend_name = available_backends[0]
        print(f"   Création backend: {backend_name}")
        backend = RobotFactory.create_backend(backend_name)

        if backend:
            print(f"   ✅ Backend {backend_name} créé avec succès")
            backend.disconnect()
        else:
            print(f"   ❌ Impossible de créer le backend {backend_name}")
    else:
        print("   ⚠️  Aucun backend disponible")

    print("\n" + "=" * 60)
    print("🎉 Démonstration terminée !")
    print("=" * 60)
    print("\n💡 Note: Le registre multi-robots est une infrastructure pour")
    print("   le support futur de plusieurs robots simultanés.")


if __name__ == "__main__":
    main()
