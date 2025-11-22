#!/usr/bin/env python3
"""Démo API Sanity - Vérification statut et arrêt d'urgence.

Démonstration des endpoints /api/sanity/* pour vérifier le statut
et déclencher l'arrêt d'urgence.
"""

import argparse
import sys
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).parent.parent))


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo API Sanity")
    parser.add_argument(
        "--action",
        choices=["status", "emergency_stop"],
        default="status",
        help="Action à effectuer",
    )
    parser.add_argument("--url", default="http://localhost:8000", help="URL de l'API")

    args = parser.parse_args()

    try:
        print("🔍 Démo API Sanity - Vérification statut et arrêt d'urgence")
        print(f"   • Action : {args.action}")
        print(f"   • URL : {args.url}")

        # 1. Status
        if args.action == "status":
            print("\n📊 Vérification statut système...")
            response = httpx.get(f"{args.url}/api/sanity/status")
            response.raise_for_status()
            result = response.json()

            print(f"   • Timestamp : {result.get('timestamp', 'N/A')}")
            print(f"   • Simulation prête : {result.get('simulation_ready', False)}")

            reachy_status = result.get("reachy", {})
            print(f"   • Reachy connecté : {reachy_status.get('ok', False)}")
            if reachy_status.get("status"):
                status = reachy_status["status"]
                print(f"   • Joints disponibles : {status.get('available_joints', [])}")

            warnings = result.get("warnings", [])
            if warnings:
                print(f"   ⚠️ Avertissements : {warnings}")
            else:
                print("   ✅ Aucun avertissement")

        # 2. Emergency Stop
        elif args.action == "emergency_stop":
            print("\n🛑 Arrêt d'urgence...")
            response = httpx.post(f"{args.url}/api/sanity/emergency_stop")
            response.raise_for_status()
            result = response.json()

            if result.get("ok"):
                print("   ✅ Arrêt d'urgence déclenché avec succès")
                print(f"   • Timestamp : {result.get('ts', 'N/A')}")
            else:
                print(f"   ❌ Erreur : {result.get('error', 'Inconnue')}")
                return 1

        return 0

    except httpx.HTTPStatusError as e:
        print(f"❌ Erreur HTTP {e.response.status_code}: {e.response.text}")
        return 1
    except Exception as e:
        print(f"❌ Erreur : {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
