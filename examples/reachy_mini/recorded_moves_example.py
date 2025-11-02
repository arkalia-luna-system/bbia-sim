#!/usr/bin/env python3
"""Demo enregistrement/replay de mouvements pour Reachy Mini - BBIA-SIM.

Démontre comment jouer les mouvements enregistrés depuis un dataset.

Adapté du repo officiel pour BBIA-SIM.

Usage:
    python examples/reachy_mini/recorded_moves_example.py
    python examples/reachy_mini/recorded_moves_example.py --library emotions
"""

import argparse

# Essayer d'utiliser le SDK officiel si disponible
try:
    from reachy_mini import ReachyMini
    from reachy_mini.motion.recorded_move import RecordedMove, RecordedMoves

    USE_SDK = True
except ImportError:
    USE_SDK = False
    print("⚠️  SDK officiel non disponible")
    print("💡 Les recorded moves nécessitent le SDK officiel reachy_mini")
    RecordedMoves = None


def main(dataset_path: str) -> None:
    """Connecter à Reachy et jouer les mouvements enregistrés."""
    if not USE_SDK:
        print("❌ SDK officiel requis pour recorded moves")
        print("💡 Installez avec: pip install reachy-mini")
        return

    try:
        recorded_moves = RecordedMoves(dataset_path)
        print(f"📚 Dataset chargé: {dataset_path}")
        print(f"📦 Nombre de mouvements: {len(recorded_moves.list_moves())}")
    except Exception as e:
        print(f"❌ Erreur chargement dataset: {e}")
        return

    print("🤖 Connexion à Reachy Mini...")
    with ReachyMini(use_sim=True, media_backend="no_media") as reachy:
        print("✅ Connexion réussie! Démarrage séquence...\n")

        try:
            while True:
                for move_name in recorded_moves.list_moves():
                    move: RecordedMove = recorded_moves.get(move_name)
                    print(f"▶️  Jouer mouvement: {move_name}")
                    if hasattr(move, "description"):
                        print(f"   📝 Description: {move.description}\n")

                    reachy.play_move(move, initial_goto_duration=1.0)

        except KeyboardInterrupt:
            print("\n⏹️  Séquence interrompue par l'utilisateur")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Démontrer et jouer tous les mouvements disponibles d'un dataset."
    )
    parser.add_argument(
        "-l",
        "--library",
        type=str,
        default="dance",
        choices=["dance", "emotions"],
        help="Bibliothèque de mouvements (dance ou emotions)",
    )

    args = parser.parse_args()

    dataset_path = (
        "pollen-robotics/reachy-mini-dances-library"
        if args.library == "dance"
        else "pollen-robotics/reachy-mini-emotions-library"
    )

    print("🤖 Demo recorded moves - Reachy Mini")
    print("=" * 50)
    main(dataset_path)
