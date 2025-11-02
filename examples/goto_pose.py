#!/usr/bin/env python3
"""Exemple : contrôle mouvement robot via API REST.

Usage:
    python examples/goto_pose.py --token TOKEN --joint JOINT_NAME --pos POSITION [--url URL]

Exemple:
    python examples/goto_pose.py --token dev --joint neck_yaw --pos 0.6
"""

import argparse
import sys
import time
from pathlib import Path

import httpx

# Ajouter le répertoire parent au path pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent))


def main():
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Contrôle mouvement robot via API")
    parser.add_argument("--token", required=True, help="Token d'authentification")
    parser.add_argument("--joint", required=True, help="Nom de l'articulation")
    parser.add_argument(
        "--pos", type=float, required=True, help="Position cible (radians)"
    )
    parser.add_argument("--url", default="http://localhost:8000", help="URL de l'API")

    args = parser.parse_args()

    headers = {"Authorization": f"Bearer {args.token}"}

    try:
        print(f"🎯 Contrôle articulation: {args.joint} → {args.pos} rad")

        # 1. Vérification état initial
        print("📊 État initial...")
        response = httpx.get(f"{args.url}/api/state/joints", headers=headers)
        response.raise_for_status()
        initial_state = response.json()
        print(f"   Position initiale: {initial_state['joints'].get(args.joint, 'N/A')}")

        # 2. Envoi commande mouvement
        print("🚀 Envoi commande mouvement...")
        motion_data = [{"joint_name": args.joint, "position": args.pos}]
        response = httpx.post(
            f"{args.url}/api/motion/joints", json=motion_data, headers=headers
        )
        response.raise_for_status()
        motion_result = response.json()
        print(f"   Statut: {motion_result['status']}")

        # 3. Attente et vérification état final
        print("⏳ Attente exécution...")
        time.sleep(1.0)

        response = httpx.get(f"{args.url}/api/state/joints", headers=headers)
        response.raise_for_status()
        final_state = response.json()
        final_pos = final_state["joints"].get(args.joint, "N/A")

        print(f"✅ Position finale: {final_pos}")
        print(
            f"📈 Succès: {motion_result['success_count']}/{motion_result['total_count']}"
        )

        return 0

    except httpx.HTTPStatusError as e:
        print(f"❌ Erreur HTTP {e.response.status_code}: {e.response.text}")
        return 1
    except httpx.RequestError as e:
        print(f"❌ Erreur réseau: {e}")
        return 1
    except Exception as e:
        print(f"❌ Erreur: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
