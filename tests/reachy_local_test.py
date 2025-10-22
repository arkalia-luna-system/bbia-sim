#!/usr/bin/env python3
"""
Test du simulateur local Reachy pour BBIA
Utilise le simulateur local au lieu du simulateur web
"""

import os
import sys

# Ajouter le répertoire src au path pour importer nos modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim.sim.simulator import MuJoCoSimulator


def test_local_simulation():
    """Test de la simulation locale Reachy"""
    print("🤖 Test de la simulation locale Reachy")
    print("=" * 50)

    # Créer l'instance du simulateur MuJoCo
    model_path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "src",
        "bbia_sim",
        "sim",
        "models",
        "reachy_mini.xml",
    )
    sim = MuJoCoSimulator(model_path)

    print("✨ Simulateur MuJoCo initialisé !")

    # Lancer la simulation en mode headless pour les tests
    sim.launch_simulation(headless=True, duration=10)

    print("🎉 Test de simulation locale terminé avec succès !")


if __name__ == "__main__":
    try:
        test_local_simulation()
    except KeyboardInterrupt:
        print("\n⚠️ Test interrompu par l'utilisateur")
    except Exception as e:
        print(f"\n❌ Erreur lors du test: {e}")
        import traceback

        traceback.print_exc()
