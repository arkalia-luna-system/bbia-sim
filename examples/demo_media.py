#!/usr/bin/env python3
"""Démo API Media - Contrôle audio/vidéo.

Démonstration des endpoints /api/media/* pour contrôler audio/vidéo.
"""

import argparse
import sys
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).parent.parent))


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo API Media")
    parser.add_argument(
        "--action",
        choices=["volume", "camera"],
        default="volume",
        help="Action à effectuer",
    )
    parser.add_argument("--volume", type=float, default=0.5, help="Volume (0.0-1.0)")
    parser.add_argument(
        "--camera-enabled",
        type=bool,
        help="Activer/désactiver caméra (True/False)",
    )
    parser.add_argument("--url", default="http://localhost:8000", help="URL de l'API")

    args = parser.parse_args()

    try:
        print("🎵 Démo API Media - Contrôle audio/vidéo")
        print(f"   • Action : {args.action}")
        print(f"   • URL : {args.url}")

        # 1. Volume
        if args.action == "volume":
            print(f"\n🔊 Changement volume → {args.volume}...")
            data = {"volume": args.volume}
            response = httpx.post(f"{args.url}/development/api/media/volume", json=data)
            response.raise_for_status()
            result = response.json()
            print(f"   Résultat : {result.get('status', 'OK')}")

        # 2. Caméra
        elif args.action == "camera":
            if args.camera_enabled is None:
                print("❌ --camera-enabled requis (True/False)")
                return 1
            print(f"\n📷 Changement caméra → {args.camera_enabled}...")
            data = {"enabled": args.camera_enabled}
            response = httpx.post(
                f"{args.url}/development/api/media/camera/toggle", json=data
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
