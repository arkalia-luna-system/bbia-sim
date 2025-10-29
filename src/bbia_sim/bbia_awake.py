#!/usr/bin/env python3
"""
BBIA Awake - Séquence de réveil optimisée pour Reachy Mini
Conforme au SDK officiel avec intelligence et variété
"""

import secrets
import time


def start_bbia_sim() -> None:
    """Démarre la séquence de réveil BBIA avec intelligence et variété."""
    # AMÉLIORATION INTELLIGENCE: Messages de réveil variés et expressifs
    wake_sequences = [
        [
            "Lumière blanche faible",
            "Lumière qui s'intensifie doucement",
            "Halo bleu apaisant",
            "Respiration simulée : inspiration lente",
            "Respiration simulée : expiration douce",
            "Léger son de démarrage harmonieux",
            "Mouvements de tête lents et fluides",
            "Expression : sourire doux et chaleureux",
        ],
        [
            "Éveil progressif...",
            "Systèmes de perception s'activent",
            "Lumière douce qui illumine",
            "Premier souffle artificiel",
            "Conscience qui émerge",
            "Mouvements initiaux délicats",
            "Regard qui s'éveille",
            "Présence rassurante",
        ],
        [
            "BBIA se réveille...",
            "Lumière bleue apaisante",
            "Systèmes en cours d'initialisation",
            "Premiers mouvements calibrés",
            "Expression neutre qui se forme",
            "Attention qui se concentre",
            "Prêt à interagir",
        ],
    ]

    # Sélectionner une séquence aléatoire pour variété
    selected_sequence = secrets.choice(wake_sequences)

    for step in selected_sequence:
        print(step)
        time.sleep(1)

    # AMÉLIORATION INTELLIGENCE: Messages de première pensée variés
    first_thoughts = [
        "Je suis là, Athalia.",
        "Me voilà ! Prêt à vous rencontrer.",
        "Bonjour ! Je viens de m'éveiller.",
        "Salut ! Je suis réveillé maintenant.",
        "Hey ! Je suis là, comment allez-vous ?",
        "Bonjour ! Prêt pour une belle journée.",
    ]

    first_thought = secrets.choice(first_thoughts)
    print(f"Première pensée : '{first_thought}'")
    time.sleep(1)

    # Message final varié
    completion_messages = [
        "Complètement réveillé et prêt",
        "Systèmes opérationnels, prêt à interagir",
        "Éveil complet, tous les systèmes sont prêts",
        "Réveil terminé, je suis prêt pour vous",
    ]
    print(secrets.choice(completion_messages))


if __name__ == "__main__":
    start_bbia_sim()
