#!/usr/bin/env python3
"""Script de test pour les tests de service de simulation."""

import os
import subprocess
import sys


def run_command(cmd, description):
    """Exécuter une commande et afficher le résultat."""
    print(f"\n=== {description} ===")
    print(f"Commande: {' '.join(cmd)}")

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        print("✅ Succès")
        if result.stdout:
            print("Sortie:")
            print(result.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print("❌ Échec")
        if e.stdout:
            print("Sortie:")
            print(e.stdout)
        if e.stderr:
            print("Erreur:")
            print(e.stderr)
        return False


def main():
    """Fonction principale."""
    print("🧪 Exécution des tests de service de simulation")

    # Vérifier que nous sommes dans le bon répertoire
    if not os.path.exists("src/bbia_sim"):
        print("❌ Erreur: Répertoire src/bbia_sim non trouvé")
        print("Assurez-vous d'être dans le répertoire racine du projet")
        sys.exit(1)

    # Tests de service de simulation
    service_tests = [
        "tests/test_simulation_service.py"
    ]

    # Exécuter les tests de service de simulation
    print("\n🔧 Tests de service de simulation")
    success = run_command([
        "python", "-m", "pytest", "-v", "--cov=src/bbia_sim",
        "--cov-report=term-missing", "--cov-report=xml",
        "--cov-report=html", "-m", "not slow"
    ] + service_tests, "Tests de service de simulation avec couverture")

    # Résumé
    print("\n📊 Résumé")
    print(f"Tests de service de simulation: {'✅' if success else '❌'}")

    if success:
        print("\n🎉 Les tests de service de simulation passent!")
        print("📁 Rapport de couverture HTML généré dans htmlcov/")
        print("📄 Rapport de couverture XML généré dans coverage.xml")
        sys.exit(0)
    else:
        print("\n💥 Les tests de service de simulation ont échoué")
        sys.exit(1)


if __name__ == "__main__":
    main()
