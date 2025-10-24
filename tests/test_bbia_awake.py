#!/usr/bin/env python3
"""Tests pour BBIA Awake."""

import os
import subprocess
import sys

# Ajouter le répertoire src au PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

STEPS = [
    "Lumière blanche faible",
    "Lumière qui s'intensifie doucement",
    "Halo bleu",
    "Respiration simulée : inspiration",
    "Respiration simulée : expiration",
    "Léger son de démarrage",
    "Mouvements de tête lents",
    "Mouvements de bras légers",
    "Expression : sourire doux",
    "Première pensée : 'Je suis là, Athalia.'",
    "complètement réveillé et prêt",
]


def test_bbia_awake_sequence():
    """Test de la séquence de réveil BBIA."""
    # Chemin correct vers le script
    script_path = os.path.join(os.path.dirname(__file__), "..", "src", "bbia_sim", "bbia_awake.py")
    
    result = subprocess.run(
        [sys.executable, script_path], capture_output=True, text=True, timeout=30
    )
    output = result.stdout
    
    # Vérifier que le script s'est exécuté sans erreur
    assert result.returncode == 0, f"Script failed with return code {result.returncode}: {result.stderr}"
    
    # Vérifier que tous les steps sont présents
    success = True
    missing_steps = []
    for step in STEPS:
        if step not in output:
            success = False
            missing_steps.append(step)
    
    assert success, f"Steps manquants dans la séquence de réveil BBIA: {missing_steps}"


if __name__ == "__main__":
    test_bbia_awake_sequence()
