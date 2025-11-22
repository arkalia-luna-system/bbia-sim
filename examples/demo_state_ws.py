#!/usr/bin/env python3
"""Démo WebSocket State - État complet via WebSocket.

Démonstration du WebSocket /api/state/ws/full pour l'état complet.
"""

import argparse
import asyncio
import sys
from pathlib import Path

import websockets

sys.path.insert(0, str(Path(__file__).parent.parent))


async def receive_state(uri: str, count: int = 5) -> int:
    """Reçoit les messages d'état via WebSocket."""
    try:
        print(f"🔌 Connexion WebSocket : {uri}")
        async with websockets.connect(uri) as websocket:
            print("✅ Connecté au WebSocket")
            print(f"\n📡 Réception de {count} messages d'état...\n")

            for i in range(count):
                message = await websocket.recv()
                print(f"Message {i+1}/{count}:")
                print(f"   {message[:200]}...")  # Afficher les 200 premiers caractères
                print()

            print("✅ Réception terminée")
            return 0

    except websockets.exceptions.ConnectionClosed:
        print("⚠️  Connexion fermée")
        return 1
    except Exception as e:
        print(f"❌ Erreur : {e}")
        import traceback

        traceback.print_exc()
        return 1


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo WebSocket State")
    parser.add_argument(
        "--count", type=int, default=5, help="Nombre de messages à recevoir"
    )
    parser.add_argument(
        "--url",
        default="ws://localhost:8000",
        help="URL WebSocket (ws://localhost:8000)",
    )

    args = parser.parse_args()

    uri = f"{args.url}/api/state/ws/full"

    print("📡 Démo WebSocket State - État complet")
    print(f"   • URI : {uri}")
    print(f"   • Messages : {args.count}")

    return asyncio.run(receive_state(uri, args.count))


if __name__ == "__main__":
    sys.exit(main())
