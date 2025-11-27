#!/usr/bin/env python3
"""Démo API Metrics - Métriques Prometheus.

Démonstration des endpoints /metrics/* pour les métriques.
"""

import argparse
import sys
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).parent.parent))


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo API Metrics")
    parser.add_argument(
        "--endpoint",
        choices=["healthz", "readyz", "health", "prometheus"],
        default="health",
        help="Endpoint à appeler",
    )
    parser.add_argument("--url", default="http://localhost:8000", help="URL de l'API")

    args = parser.parse_args()

    try:
        print("📊 Démo API Metrics - Métriques Prometheus")
        print(f"   • Endpoint : {args.endpoint}")
        print(f"   • URL : {args.url}")

        # 1. Healthz (liveness)
        if args.endpoint == "healthz":
            print("\n❤️  Health check (liveness)...")
            response = httpx.get(f"{args.url}/metrics/healthz")
            response.raise_for_status()
            print(f"   Statut : {response.status_code} OK")

        # 2. Readyz (readiness)
        elif args.endpoint == "readyz":
            print("\n✅ Readiness check...")
            response = httpx.get(f"{args.url}/metrics/readyz")
            response.raise_for_status()
            print(f"   Statut : {response.status_code} OK")

        # 3. Health détaillé
        elif args.endpoint == "health":
            print("\n🏥 Health check détaillé...")
            response = httpx.get(f"{args.url}/metrics/health")
            response.raise_for_status()
            health = response.json()
            print(f"   Statut : {health.get('status', 'N/A')}")
            if "services" in health:
                for service, status in health["services"].items():
                    print(f"   {service} : {status}")

        # 4. Prometheus
        elif args.endpoint == "prometheus":
            print("\n📈 Métriques Prometheus...")
            response = httpx.get(f"{args.url}/metrics/prometheus")
            response.raise_for_status()
            metrics = response.text
            lines = metrics.split("\n")[:10]  # Afficher les 10 premières lignes
            print("   Métriques (10 premières lignes):")
            for line in lines:
                if line.strip():
                    print(f"   {line}")

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
