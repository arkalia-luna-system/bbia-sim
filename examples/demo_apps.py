#!/usr/bin/env python3
"""Démo API Apps - Gestion applications HuggingFace.

Démonstration des endpoints /api/apps/* pour gérer les apps HF.
"""

import argparse
import sys
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).parent.parent))


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo API Apps")
    parser.add_argument("--token", default="dev", help="Token d'authentification")
    parser.add_argument(
        "--action",
        choices=["list", "status", "install", "start", "stop"],
        default="list",
        help="Action à effectuer",
    )
    parser.add_argument("--app-name", help="Nom de l'application")
    parser.add_argument("--url", default="http://localhost:8000", help="URL de l'API")

    args = parser.parse_args()

    headers = {"Authorization": f"Bearer {args.token}"}

    try:
        print("📱 Démo API Apps - Gestion applications HuggingFace")
        print(f"   • Action : {args.action}")
        print(f"   • URL : {args.url}")

        # 1. Lister apps
        if args.action == "list":
            print("\n📋 Liste des applications disponibles...")
            response = httpx.get(f"{args.url}/api/apps", headers=headers)
            response.raise_for_status()
            apps = response.json()
            print(f"   Nombre d'apps : {len(apps.get('apps', []))}")
            for app in apps.get("apps", [])[:5]:  # Afficher les 5 premières
                print(f"   • {app.get('name', 'N/A')} - {app.get('status', 'N/A')}")

        # 2. Statut app
        elif args.action == "status":
            if not args.app_name:
                print("❌ --app-name requis")
                return 1
            print(f"\n📊 Statut application '{args.app_name}'...")
            response = httpx.get(
                f"{args.url}/api/apps/{args.app_name}/status", headers=headers
            )
            response.raise_for_status()
            status = response.json()
            print(f"   Statut : {status.get('status', 'N/A')}")
            print(f"   En cours : {status.get('running', False)}")

        # 3. Installer app
        elif args.action == "install":
            if not args.app_name:
                print("❌ --app-name requis")
                return 1
            print(f"\n📥 Installation application '{args.app_name}'...")
            response = httpx.post(
                f"{args.url}/api/apps/{args.app_name}/install", headers=headers
            )
            response.raise_for_status()
            result = response.json()
            print(f"   Résultat : {result.get('status', 'OK')}")

        # 4. Démarrer app
        elif args.action == "start":
            if not args.app_name:
                print("❌ --app-name requis")
                return 1
            print(f"\n🚀 Démarrage application '{args.app_name}'...")
            response = httpx.post(
                f"{args.url}/api/apps/{args.app_name}/start", headers=headers
            )
            response.raise_for_status()
            result = response.json()
            print(f"   Résultat : {result.get('status', 'OK')}")

        # 5. Arrêter app
        elif args.action == "stop":
            if not args.app_name:
                print("❌ --app-name requis")
                return 1
            print(f"\n🛑 Arrêt application '{args.app_name}'...")
            response = httpx.post(
                f"{args.url}/api/apps/{args.app_name}/stop", headers=headers
            )
            response.raise_for_status()
            result = response.json()
            print(f"   Résultat : {result.get('status', 'OK')}")

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
