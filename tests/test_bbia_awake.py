#!/usr/bin/env python3
"""Tests pour BBIA Awake."""

import os
import subprocess
import sys

# Ajouter le répertoire src au PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

# Étapes minimales attendues (variantes possibles selon séquence aléatoire)
# Note: Les séquences sont aléatoires donc on vérifie des patterns flexibles
STEP_PATTERNS = [
    "Lumière",  # "Lumière blanche faible" ou "Lumière douce qui illumine"
    "Mouvements",  # "Mouvements de tête" ou "Mouvements initiaux"
    "Première pensée",  # "Première pensée : '...'"
    "réveillé",  # "Complètement réveillé et prêt" ou variante
]
# Patterns optionnels qui peuvent être dans certaines séquences mais pas toutes :
OPTIONAL_PATTERNS = [
    "Halo",  # "Halo bleu apaisant" ou "Lumière bleue"
    "Respiration",  # "Respiration simulée" ou "Premier souffle"
    "Expression",  # "Expression : sourire" ou "Regard qui s'éveille"
]


def test_bbia_awake_sequence():
    """Test de la séquence de réveil BBIA."""
    # Chemin correct vers le script
    script_path = os.path.join(
        os.path.dirname(__file__), "..", "src", "bbia_sim", "bbia_awake.py"
    )

    result = subprocess.run(
        [sys.executable, script_path], capture_output=True, text=True, timeout=30
    )
    output = result.stdout

    # Vérifier que le script s'est exécuté sans erreur
    assert (
        result.returncode == 0
    ), f"Script failed with return code {result.returncode}: {result.stderr}"

    # Vérifier que les patterns essentiels sont présents (flexible pour séquences aléatoires)
    success = True
    missing_patterns = []
    for pattern in STEP_PATTERNS:
        if pattern.lower() not in output.lower():
            success = False
            missing_patterns.append(pattern)

    # Note: Les patterns optionnels peuvent ne pas tous être présents selon la séquence choisie
    # On ne vérifie pas les patterns optionnels car les séquences sont aléatoires

    assert (
        success
    ), f"Patterns essentiels manquants dans la séquence de réveil BBIA: {missing_patterns}"


if __name__ == "__main__":
    test_bbia_awake_sequence()
