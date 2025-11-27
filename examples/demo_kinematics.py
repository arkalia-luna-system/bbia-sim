#!/usr/bin/env python3
"""Démo API Kinematics - Informations cinématique.

Démonstration des endpoints /api/kinematics/* pour la cinématique.
"""

import argparse
import sys
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).parent.parent))


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo API Kinematics")
    parser.add_argument("--token", default="dev", help="Token d'authentification")
    parser.add_argument(
        "--endpoint",
        choices=["info", "urdf", "stl"],
        default="info",
        help="Endpoint à appeler",
    )
    parser.add_argument("--url", default="http://localhost:8000", help="URL de l'API")

    args = parser.parse_args()

    headers = {"Authorization": f"Bearer {args.token}"}

    try:
        print("🔬 Démo API Kinematics - Informations cinématique")
        print(f"   • Endpoint : {args.endpoint}")
        print(f"   • URL : {args.url}")

        # 1. Info cinématique
        if args.endpoint == "info":
            print("\n📊 Récupération informations cinématique...")
            response = httpx.get(f"{args.url}/api/kinematics/info", headers=headers)
            response.raise_for_status()
            info = response.json()
            print(f"   Moteur : {info['info']['engine']}")
            print(f"   Vérification collision : {info['info']['collision_check']}")

        # 2. URDF
        elif args.endpoint == "urdf":
            print("\n📄 Récupération URDF...")
            response = httpx.get(f"{args.url}/api/kinematics/urdf", headers=headers)
            response.raise_for_status()
            urdf = response.json()
            print(f"   URDF disponible : {'urdf' in urdf}")

        # 3. STL
        elif args.endpoint == "stl":
            print("\n📦 Récupération liste STL...")
            response = httpx.get(f"{args.url}/api/kinematics/stl", headers=headers)
            response.raise_for_status()
            stl_list = response.json()
            print(f"   Nombre de fichiers STL : {len(stl_list.get('stl_files', []))}")

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
