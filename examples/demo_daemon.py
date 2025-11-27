#!/usr/bin/env python3
"""Démo API Daemon - Contrôle du daemon.

Démonstration des endpoints /api/daemon/* pour contrôler le daemon.
"""

import argparse
import sys
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).parent.parent))


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo API Daemon")
    parser.add_argument(
        "--action",
        choices=["status", "start", "stop", "restart"],
        default="status",
        help="Action à effectuer",
    )
    parser.add_argument(
        "--wake-up", action="store_true", help="Réveiller le robot au démarrage"
    )
    parser.add_argument(
        "--goto-sleep",
        action="store_true",
        help="Mettre le robot en veille à l'arrêt",
    )
    parser.add_argument("--url", default="http://localhost:8000", help="URL de l'API")

    args = parser.parse_args()

    try:
        print("🔧 Démo API Daemon - Contrôle du daemon")
        print(f"   • Action : {args.action}")
        print(f"   • URL : {args.url}")

        # 1. Statut
        if args.action == "status":
            print("\n📊 Récupération statut daemon...")
            response = httpx.get(f"{args.url}/api/daemon/status")
            response.raise_for_status()
            status = response.json()
            print(f"   Statut : {status['status']}")
            print(f"   Simulation : {status['simulation_running']}")
            if status.get("error"):
                print(f"   Erreur : {status['error']}")

        # 2. Démarrer
        elif args.action == "start":
            print(f"\n🚀 Démarrage daemon (wake_up={args.wake_up})...")
            params = {"wake_up": args.wake_up}
            response = httpx.post(f"{args.url}/api/daemon/start", params=params)
            response.raise_for_status()
            result = response.json()
            print(f"   Statut : {result['status']}")
            print(f"   Message : {result['message']}")

        # 3. Arrêter
        elif args.action == "stop":
            print(f"\n🛑 Arrêt daemon (goto_sleep={args.goto_sleep})...")
            params = {"goto_sleep": args.goto_sleep}
            response = httpx.post(f"{args.url}/api/daemon/stop", params=params)
            response.raise_for_status()
            result = response.json()
            print(f"   Statut : {result['status']}")
            print(f"   Message : {result['message']}")

        # 4. Redémarrer
        elif args.action == "restart":
            print("\n🔄 Redémarrage daemon...")
            response = httpx.post(f"{args.url}/api/daemon/restart")
            response.raise_for_status()
            result = response.json()
            print(f"   Statut : {result['status']}")
            print(f"   Message : {result['message']}")

        print("\n✅ Démo terminée avec succès")
        return 0

    except httpx.HTTPStatusError as e:
        print(f"❌ Erreur HTTP {e.response.status_code}: {e.response.text}")
        return 1
    except httpx.RequestError as e:
        print(f"❌ Erreur réseau: {e}")
        return 1
    except Exception as e:
        print(f"❌ Erreur : {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
