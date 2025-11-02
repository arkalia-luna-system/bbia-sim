#!/usr/bin/env python3
"""Script de test pour l'API publique BBIA-SIM - Phase 3."""

import asyncio
import logging
import sys

import httpx

# Configuration du logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class BBIAAPITester:
    """Testeur pour l'API publique BBIA-SIM."""

    def __init__(self, base_url: str = "http://localhost:8000"):
        """Initialise le testeur.

        Args:
            base_url: URL de base de l'API

        """
        self.base_url = base_url
        self.client = httpx.AsyncClient(timeout=30.0)

    async def test_root_endpoint(self) -> bool:
        """Teste l'endpoint racine."""
        try:
            logger.info("🔍 Test de l'endpoint racine...")
            response = await self.client.get(f"{self.base_url}/")

            if response.status_code == 200:
                data = response.json()
                logger.info(f"✅ Endpoint racine OK: {data.get('message', 'N/A')}")
                logger.info(f"📊 Version: {data.get('version', 'N/A')}")
                return True
            logger.error(f"❌ Endpoint racine échoué: {response.status_code}")
            return False
        except Exception as e:
            logger.error(f"❌ Erreur endpoint racine: {e}")
            return False

    async def test_health_check(self) -> bool:
        """Teste l'endpoint de santé."""
        try:
            logger.info("🔍 Test de l'endpoint de santé...")
            response = await self.client.get(f"{self.base_url}/health")

            if response.status_code == 200:
                data = response.json()
                logger.info(f"✅ Santé OK: {data.get('status', 'N/A')}")
                return True
            logger.error(f"❌ Santé échouée: {response.status_code}")
            return False
        except Exception as e:
            logger.error(f"❌ Erreur santé: {e}")
            return False

    async def test_api_info(self) -> bool:
        """Teste l'endpoint d'informations API."""
        try:
            logger.info("🔍 Test de l'endpoint d'informations...")
            response = await self.client.get(f"{self.base_url}/api/info")

            if response.status_code == 200:
                data = response.json()
                logger.info(f"✅ Info API OK: {data.get('name', 'N/A')}")
                logger.info(f"📊 Phase: {data.get('phase', 'N/A')}")
                return True
            logger.error(f"❌ Info API échouée: {response.status_code}")
            return False
        except Exception as e:
            logger.error(f"❌ Erreur info API: {e}")
            return False

    async def test_ecosystem_capabilities(self) -> bool:
        """Teste l'endpoint des capacités de l'écosystème."""
        try:
            logger.info("🔍 Test des capacités de l'écosystème...")
            response = await self.client.get(
                f"{self.base_url}/api/ecosystem/capabilities",
            )

            if response.status_code == 200:
                data = response.json()
                logger.info(f"✅ Capacités OK: {data.get('model', 'N/A')}")
                logger.info(f"📊 Joints: {data.get('joints', 'N/A')}")
                logger.info(f"😊 Émotions: {len(data.get('emotions', []))}")
                logger.info(f"🎭 Comportements: {len(data.get('behaviors', []))}")
                return True
            logger.error(f"❌ Capacités échouées: {response.status_code}")
            return False
        except Exception as e:
            logger.error(f"❌ Erreur capacités: {e}")
            return False

    async def test_ecosystem_status(self) -> bool:
        """Teste l'endpoint de statut de l'écosystème."""
        try:
            logger.info("🔍 Test du statut de l'écosystème...")
            response = await self.client.get(f"{self.base_url}/api/ecosystem/status")

            if response.status_code == 200:
                data = response.json()
                logger.info(f"✅ Statut OK: {data.get('status', 'N/A')}")
                logger.info(f"🤖 Robot connecté: {data.get('robot_connected', 'N/A')}")
                logger.info(f"🎮 Simulation: {data.get('simulation_running', 'N/A')}")
                return True
            logger.error(f"❌ Statut échoué: {response.status_code}")
            return False
        except Exception as e:
            logger.error(f"❌ Erreur statut: {e}")
            return False

    async def test_available_emotions(self) -> bool:
        """Teste l'endpoint des émotions disponibles."""
        try:
            logger.info("🔍 Test des émotions disponibles...")
            response = await self.client.get(
                f"{self.base_url}/api/ecosystem/emotions/available",
            )

            if response.status_code == 200:
                data = response.json()
                emotions = data.get("emotions", [])
                logger.info(f"✅ Émotions OK: {len(emotions)} émotions disponibles")
                logger.info(
                    f"📊 Émotions: {', '.join(emotions[:5])}{'...' if len(emotions) > 5 else ''}",
                )
                return True
            logger.error(f"❌ Émotions échouées: {response.status_code}")
            return False
        except Exception as e:
            logger.error(f"❌ Erreur émotions: {e}")
            return False

    async def test_available_behaviors(self) -> bool:
        """Teste l'endpoint des comportements disponibles."""
        try:
            logger.info("🔍 Test des comportements disponibles...")
            response = await self.client.get(
                f"{self.base_url}/api/ecosystem/behaviors/available",
            )

            if response.status_code == 200:
                data = response.json()
                behaviors = data.get("behaviors", [])
                logger.info(
                    f"✅ Comportements OK: {len(behaviors)} comportements disponibles",
                )
                logger.info(
                    f"📊 Comportements: {', '.join(behaviors[:5])}{'...' if len(behaviors) > 5 else ''}",
                )
                return True
            logger.error(f"❌ Comportements échoués: {response.status_code}")
            return False
        except Exception as e:
            logger.error(f"❌ Erreur comportements: {e}")
            return False

    async def test_demo_modes(self) -> bool:
        """Teste l'endpoint des modes de démonstration."""
        try:
            logger.info("🔍 Test des modes de démonstration...")
            response = await self.client.get(
                f"{self.base_url}/api/ecosystem/demo/modes",
            )

            if response.status_code == 200:
                data = response.json()
                modes = data.get("demo_modes", {})
                logger.info(f"✅ Modes démo OK: {len(modes)} modes disponibles")
                logger.info(f"📊 Modes: {', '.join(modes.keys())}")
                return True
            logger.error(f"❌ Modes démo échoués: {response.status_code}")
            return False
        except Exception as e:
            logger.error(f"❌ Erreur modes démo: {e}")
            return False

    async def test_openapi_spec(self) -> bool:
        """Teste la spécification OpenAPI."""
        try:
            logger.info("🔍 Test de la spécification OpenAPI...")
            response = await self.client.get(f"{self.base_url}/openapi.json")

            if response.status_code == 200:
                data = response.json()
                logger.info(
                    f"✅ OpenAPI OK: {data.get('info', {}).get('title', 'N/A')}",
                )
                logger.info(f"📊 Version: {data.get('info', {}).get('version', 'N/A')}")
                logger.info(f"📊 Endpoints: {len(data.get('paths', {}))}")
                return True
            logger.error(f"❌ OpenAPI échoué: {response.status_code}")
            return False
        except Exception as e:
            logger.error(f"❌ Erreur OpenAPI: {e}")
            return False

    async def run_all_tests(self) -> dict[str, bool]:
        """Exécute tous les tests."""
        logger.info("🚀 Démarrage des tests de l'API publique BBIA-SIM")
        logger.info(f"📍 URL de base: {self.base_url}")

        tests = [
            ("Root Endpoint", self.test_root_endpoint),
            ("Health Check", self.test_health_check),
            ("API Info", self.test_api_info),
            ("Ecosystem Capabilities", self.test_ecosystem_capabilities),
            ("Ecosystem Status", self.test_ecosystem_status),
            ("Available Emotions", self.test_available_emotions),
            ("Available Behaviors", self.test_available_behaviors),
            ("Demo Modes", self.test_demo_modes),
            ("OpenAPI Spec", self.test_openapi_spec),
        ]

        results = {}
        passed = 0
        total = len(tests)

        for test_name, test_func in tests:
            logger.info(f"\n{'=' * 50}")
            logger.info(f"🧪 Test: {test_name}")
            logger.info(f"{'=' * 50}")

            try:
                result = await test_func()
                results[test_name] = result
                if result:
                    passed += 1
                    logger.info(f"✅ {test_name}: PASSÉ")
                else:
                    logger.error(f"❌ {test_name}: ÉCHOUÉ")
            except Exception as e:
                logger.error(f"❌ {test_name}: ERREUR - {e}")
                results[test_name] = False

        # Résumé final
        logger.info(f"\n{'=' * 50}")
        logger.info("📊 RÉSUMÉ DES TESTS")
        logger.info(f"{'=' * 50}")
        logger.info(f"✅ Tests passés: {passed}/{total}")
        logger.info(f"❌ Tests échoués: {total - passed}/{total}")
        logger.info(f"📊 Taux de réussite: {(passed / total) * 100:.1f}%")

        if passed == total:
            logger.info("🎉 TOUS LES TESTS SONT PASSÉS !")
        else:
            logger.warning("⚠️ Certains tests ont échoué")

        return results

    async def close(self):
        """Ferme le client HTTP."""
        await self.client.aclose()


async def main():
    """Point d'entrée principal."""
    import argparse

    parser = argparse.ArgumentParser(
        description="🧪 Testeur pour l'API publique BBIA-SIM",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation :

  # Test sur localhost:8000 (défaut)
  python scripts/test_public_api.py

  # Test sur une autre URL
  python scripts/test_public_api.py --url http://localhost:3000

  # Test avec logs détaillés
  python scripts/test_public_api.py --log-level debug
        """,
    )

    parser.add_argument(
        "--url",
        default="http://localhost:8000",
        help="URL de base de l'API (défaut: http://localhost:8000)",
    )
    parser.add_argument(
        "--log-level",
        choices=["debug", "info", "warning", "error"],
        default="info",
        help="Niveau de log (défaut: info)",
    )

    args = parser.parse_args()

    # Configuration du logging
    logging.getLogger().setLevel(getattr(logging, args.log_level.upper()))

    # Création du testeur
    tester = BBIAAPITester(args.url)

    try:
        # Exécution des tests
        results = await tester.run_all_tests()

        # Code de sortie
        if all(results.values()):
            sys.exit(0)  # Succès
        else:
            sys.exit(1)  # Échec
    finally:
        await tester.close()


if __name__ == "__main__":
    asyncio.run(main())
