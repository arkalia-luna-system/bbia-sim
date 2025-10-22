#!/usr/bin/env python3
"""Exemple : souscription télémétrie WebSocket.

Usage:
    python examples/subscribe_telemetry.py --token TOKEN [--count N] [--url URL]

Exemple:
    python examples/subscribe_telemetry.py --token dev --count 5
"""

import argparse
import asyncio
import json
import sys
from pathlib import Path

import websockets

# Ajouter le répertoire parent au path pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent))


async def main():
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Souscription télémétrie WebSocket")
    parser.add_argument("--token", required=True, help="Token d'authentification")
    parser.add_argument(
        "--count", type=int, default=5, help="Nombre de messages (défaut: 5)"
    )
    parser.add_argument("--url", default="ws://localhost:8000", help="URL WebSocket")

    args = parser.parse_args()

    ws_url = f"{args.url}/ws/telemetry"

    try:
        print(f"📡 Connexion WebSocket: {ws_url}")
        print(f"📊 Réception de {args.count} messages de télémétrie...")

        async with websockets.connect(ws_url) as websocket:
            print("✅ Connexion établie")

            message_count = 0
            while message_count < args.count:
                try:
                    # Réception message
                    message = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                    data = json.loads(message)

                    message_count += 1

                    # Affichage données clés
                    robot_pos = data.get("robot", {}).get("position", {})
                    joints = data.get("joints", {})
                    sensors = data.get("sensors", {})

                    print(f"\n📨 Message {message_count}/{args.count}:")
                    print(
                        f"   Robot: x={robot_pos.get('x', 'N/A'):.3f}, y={robot_pos.get('y', 'N/A'):.3f}"
                    )
                    print(f"   Articulations: {len(joints)} joints")
                    print(f"   Batterie: {sensors.get('battery', 'N/A')}%")

                    # Envoi ping pour maintenir la connexion
                    if message_count % 2 == 0:
                        ping_msg = {"type": "ping"}
                        await websocket.send(json.dumps(ping_msg))

                except asyncio.TimeoutError:
                    print("⏰ Timeout - pas de message reçu")
                    break
                except json.JSONDecodeError:
                    print("⚠️ Message JSON invalide reçu")
                    continue

            print(f"\n✅ Réception terminée: {message_count} messages")
            return 0

    except websockets.exceptions.InvalidURI as e:
        print(f"❌ Erreur URI WebSocket: {e}")
        return 1
    except websockets.exceptions.WebSocketException as e:
        print(f"❌ Erreur WebSocket: {e}")
        return 1
    except Exception as e:
        print(f"❌ Erreur: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
