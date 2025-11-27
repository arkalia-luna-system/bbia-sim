#!/usr/bin/env python3
"""Démo API Motors - Contrôle des moteurs.

Démonstration des endpoints /api/motors/* pour contrôler les moteurs.
"""

import argparse
import sys
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).parent.parent))


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo API Motors")
    parser.add_argument("--token", default="dev", help="Token d'authentification")
    parser.add_argument(
        "--mode",
        choices=["enabled", "disabled", "gravity_compensation"],
        help="Mode à définir (optionnel)",
    )
    parser.add_argument("--url", default="http://localhost:8000", help="URL de l'API")

    args = parser.parse_args()

    headers = {"Authorization": f"Bearer {args.token}"}

    try:
        print("⚙️  Démo API Motors - Contrôle des moteurs")
        print(f"   • URL : {args.url}")

        # 1. Récupérer statut actuel
        print("\n📊 Récupération statut moteurs...")
        response = httpx.get(f"{args.url}/api/motors/status", headers=headers)
        response.raise_for_status()
        status = response.json()
        print(f"   Mode actuel : {status['mode']}")

        # 2. Changer mode si demandé
        if args.mode:
            print(f"\n🔄 Changement mode → {args.mode}...")
            response = httpx.post(
                f"{args.url}/api/motors/set_mode/{args.mode}", headers=headers
            )
            response.raise_for_status()
            result = response.json()
            print(f"   Résultat : {result['status']}")

            # Vérifier nouveau statut
            response = httpx.get(f"{args.url}/api/motors/status", headers=headers)
            response.raise_for_status()
            new_status = response.json()
            print(f"   Nouveau mode : {new_status['mode']}")

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
