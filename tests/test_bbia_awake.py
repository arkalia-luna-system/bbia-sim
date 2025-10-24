import subprocess
import sys

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
    result = subprocess.run(
        [sys.executable, "src/bbia_sim/bbia_awake.py"], capture_output=True, text=True
    )
    output = result.stdout
    success = True
    for step in STEPS:
        if step in output:
            pass
        else:
            success = False
    if success:
        pass
    else:
        pass
    assert success, "La séquence de réveil BBIA n'est pas complète ou fidèle."


if __name__ == "__main__":
    test_bbia_awake_sequence()
