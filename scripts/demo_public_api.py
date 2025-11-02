#!/usr/bin/env python3
"""Script de démonstration complète BBIA-SIM - Phase 3."""

import argparse
import asyncio
import logging
import sys
import time

import httpx

# Configuration du logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class BBIADemo:
    """Démonstration complète de l'API publique BBIA-SIM."""

    def __init__(self, base_url: str = "http://localhost:8000"):
        """Initialise la démonstration.

        Args:
            base_url: URL de base de l'API

        """
        self.base_url = base_url
        self.client = httpx.AsyncClient(timeout=30.0)

    async def check_api_status(self) -> bool:
        """Vérifie que l'API est accessible."""
        try:
            logger.info("🔍 Vérification de l'API...")
            response = await self.client.get(f"{self.base_url}/health")

            if response.status_code == 200:
                data = response.json()
                logger.info(f"✅ API accessible: {data.get('status', 'N/A')}")
                return True
            logger.error(f"❌ API non accessible: {response.status_code}")
            return False
        except Exception as e:
            logger.error(f"❌ Erreur de connexion: {e}")
            return False

    async def demo_ecosystem_info(self):
        """Démonstration des informations de l'écosystème."""
        logger.info("\n" + "=" * 60)
        logger.info("🌐 DÉMONSTRATION - INFORMATIONS ÉCOSYSTÈME")
        logger.info("=" * 60)

        try:
            # Informations générales
            response = await self.client.get(f"{self.base_url}/api/info")
            if response.status_code == 200:
                data = response.json()
                logger.info(f"📊 Nom: {data.get('name', 'N/A')}")
                logger.info(f"📊 Version: {data.get('version', 'N/A')}")
                logger.info(f"📊 Phase: {data.get('phase', 'N/A')}")

                features = data.get("features", [])
                logger.info(f"📊 Fonctionnalités ({len(features)}):")
                for feature in features:
                    logger.info(f"   • {feature}")

                robot_info = data.get("robot", {})
                logger.info(f"🤖 Robot: {robot_info.get('model', 'N/A')}")
                logger.info(f"🤖 Joints: {robot_info.get('joints', 'N/A')}")
                logger.info(f"🤖 Backends: {', '.join(robot_info.get('backends', []))}")

                bbia_info = data.get("bbia_modules", {})
                logger.info("🧠 BBIA Modules:")
                logger.info(f"   • Émotions: {bbia_info.get('emotions', 'N/A')}")
                logger.info(f"   • Comportements: {bbia_info.get('behaviors', 'N/A')}")
                logger.info(f"   • Vision: {bbia_info.get('vision', 'N/A')}")
                logger.info(f"   • Audio: {bbia_info.get('audio', 'N/A')}")

            # Capacités du robot
            response = await self.client.get(
                f"{self.base_url}/api/ecosystem/capabilities",
            )
            if response.status_code == 200:
                data = response.json()
                logger.info("\n🎯 Capacités du Robot:")
                logger.info(f"   • Modèle: {data.get('model', 'N/A')}")
                logger.info(f"   • Joints: {data.get('joints', 'N/A')}")
                logger.info(
                    f"   • Mode simulation: {data.get('simulation_mode', 'N/A')}",
                )

                emotions = data.get("emotions", [])
                logger.info(
                    f"   • Émotions ({len(emotions)}): {', '.join(emotions[:5])}{'...' if len(emotions) > 5 else ''}",
                )

                behaviors = data.get("behaviors", [])
                logger.info(
                    f"   • Comportements ({len(behaviors)}): {', '.join(behaviors[:5])}{'...' if len(behaviors) > 5 else ''}",
                )

                backends = data.get("backends", [])
                logger.info(f"   • Backends: {', '.join(backends)}")

        except Exception as e:
            logger.error(f"❌ Erreur démo écosystème: {e}")

    async def demo_emotions(self):
        """Démonstration des émotions BBIA."""
        logger.info("\n" + "=" * 60)
        logger.info("😊 DÉMONSTRATION - ÉMOTIONS BBIA")
        logger.info("=" * 60)

        try:
            # Récupération des émotions disponibles
            response = await self.client.get(
                f"{self.base_url}/api/ecosystem/emotions/available",
            )
            if response.status_code == 200:
                data = response.json()
                emotions = data.get("emotions", [])
                descriptions = data.get("descriptions", {})

                logger.info(f"📊 {len(emotions)} émotions disponibles:")
                for emotion in emotions[:8]:  # Afficher les 8 premières
                    desc = descriptions.get(emotion, "Description non disponible")
                    logger.info(f"   • {emotion}: {desc}")

                if len(emotions) > 8:
                    logger.info(f"   • ... et {len(emotions) - 8} autres émotions")

            # Test d'application d'émotion (simulation)
            logger.info("\n🎭 Test d'application d'émotion...")
            test_emotions = ["happy", "sad", "curious", "excited"]

            for emotion in test_emotions:
                try:
                    response = await self.client.post(
                        f"{self.base_url}/api/ecosystem/emotions/apply",
                        params={"emotion": emotion, "intensity": 0.7, "duration": 3.0},
                    )

                    if response.status_code == 200:
                        data = response.json()
                        logger.info(f"   ✅ {emotion}: {data.get('status', 'N/A')}")
                        logger.info(
                            f"      Joints affectés: {len(data.get('joints_affected', []))}",
                        )
                    else:
                        logger.warning(
                            f"   ⚠️ {emotion}: Échec (code {response.status_code})",
                        )

                    await asyncio.sleep(0.5)  # Pause entre les émotions

                except Exception as e:
                    logger.warning(f"   ⚠️ {emotion}: Erreur - {e}")

        except Exception as e:
            logger.error(f"❌ Erreur démo émotions: {e}")

    async def demo_behaviors(self):
        """Démonstration des comportements BBIA."""
        logger.info("\n" + "=" * 60)
        logger.info("🎭 DÉMONSTRATION - COMPORTEMENTS BBIA")
        logger.info("=" * 60)

        try:
            # Récupération des comportements disponibles
            response = await self.client.get(
                f"{self.base_url}/api/ecosystem/behaviors/available",
            )
            if response.status_code == 200:
                data = response.json()
                behaviors = data.get("behaviors", [])
                descriptions = data.get("descriptions", {})

                logger.info(f"📊 {len(behaviors)} comportements disponibles:")
                for behavior in behaviors:
                    desc = descriptions.get(behavior, "Description non disponible")
                    logger.info(f"   • {behavior}: {desc}")

            # Test d'exécution de comportement (simulation)
            logger.info("\n🎬 Test d'exécution de comportement...")
            test_behaviors = ["wake_up", "greeting", "nod"]

            for behavior in test_behaviors:
                try:
                    response = await self.client.post(
                        f"{self.base_url}/api/ecosystem/behaviors/execute",
                        params={
                            "behavior": behavior,
                            "intensity": 1.0,
                            "duration": 5.0,
                        },
                    )

                    if response.status_code == 200:
                        data = response.json()
                        logger.info(f"   ✅ {behavior}: {data.get('status', 'N/A')}")
                        logger.info(
                            f"      Durée estimée: {data.get('estimated_duration', 'N/A')}s",
                        )
                    else:
                        logger.warning(
                            f"   ⚠️ {behavior}: Échec (code {response.status_code})",
                        )

                    await asyncio.sleep(1.0)  # Pause entre les comportements

                except Exception as e:
                    logger.warning(f"   ⚠️ {behavior}: Erreur - {e}")

        except Exception as e:
            logger.error(f"❌ Erreur démo comportements: {e}")

    async def demo_modes(self):
        """Démonstration des modes de démonstration."""
        logger.info("\n" + "=" * 60)
        logger.info("🎮 DÉMONSTRATION - MODES DE DÉMONSTRATION")
        logger.info("=" * 60)

        try:
            # Récupération des modes disponibles
            response = await self.client.get(
                f"{self.base_url}/api/ecosystem/demo/modes",
            )
            if response.status_code == 200:
                data = response.json()
                modes = data.get("demo_modes", {})
                recommended = data.get("recommended", "N/A")

                logger.info("📊 Modes de démonstration disponibles:")
                logger.info(f"   • Mode recommandé: {recommended}")

                for mode_name, mode_info in modes.items():
                    logger.info(f"\n   🎯 {mode_name.upper()}:")
                    logger.info(f"      • Nom: {mode_info.get('name', 'N/A')}")
                    logger.info(
                        f"      • Description: {mode_info.get('description', 'N/A')}",
                    )
                    logger.info(f"      • Backend: {mode_info.get('backend', 'N/A')}")

                    features = mode_info.get("features", [])
                    logger.info(f"      • Fonctionnalités: {', '.join(features)}")

                    requirements = mode_info.get("requirements", [])
                    logger.info(f"      • Prérequis: {', '.join(requirements)}")

            # Test de démarrage de démo (simulation)
            logger.info("\n🚀 Test de démarrage de démonstration...")
            try:
                response = await self.client.post(
                    f"{self.base_url}/api/ecosystem/demo/start",
                    params={"mode": "simulation", "duration": 10.0, "emotion": "happy"},
                )

                if response.status_code == 200:
                    data = response.json()
                    logger.info(f"   ✅ Démo démarrée: {data.get('status', 'N/A')}")
                    logger.info(f"      Mode: {data.get('mode', 'N/A')}")
                    logger.info(f"      Durée: {data.get('duration', 'N/A')}s")
                    logger.info(f"      Message: {data.get('message', 'N/A')}")
                else:
                    logger.warning(f"   ⚠️ Démo: Échec (code {response.status_code})")

            except Exception as e:
                logger.warning(f"   ⚠️ Démo: Erreur - {e}")

        except Exception as e:
            logger.error(f"❌ Erreur démo modes: {e}")

    async def demo_documentation(self):
        """Démonstration de la documentation."""
        logger.info("\n" + "=" * 60)
        logger.info("📚 DÉMONSTRATION - DOCUMENTATION")
        logger.info("=" * 60)

        try:
            # Test de la spécification OpenAPI
            response = await self.client.get(f"{self.base_url}/openapi.json")
            if response.status_code == 200:
                data = response.json()
                info = data.get("info", {})
                paths = data.get("paths", {})

                logger.info("📊 Spécification OpenAPI:")
                logger.info(f"   • Titre: {info.get('title', 'N/A')}")
                logger.info(f"   • Version: {info.get('version', 'N/A')}")
                logger.info(
                    f"   • Description: {info.get('description', 'N/A')[:100]}...",
                )
                logger.info(f"   • Endpoints: {len(paths)}")

                # Afficher quelques endpoints
                logger.info("   • Exemples d'endpoints:")
                for _i, path in enumerate(list(paths.keys())[:5]):
                    logger.info(f"     - {path}")
                if len(paths) > 5:
                    logger.info(f"     - ... et {len(paths) - 5} autres")

            # Informations sur la documentation
            logger.info("\n📖 Documentation disponible:")
            logger.info(f"   • Swagger UI: {self.base_url}/docs")
            logger.info(f"   • ReDoc: {self.base_url}/redoc")
            logger.info(f"   • OpenAPI: {self.base_url}/openapi.json")
            logger.info("   • GitHub: https://github.com/arkalia-luna-system/bbia-sim")

        except Exception as e:
            logger.error(f"❌ Erreur démo documentation: {e}")

    async def run_complete_demo(self):
        """Exécute la démonstration complète."""
        logger.info("🚀 DÉMONSTRATION COMPLÈTE BBIA-SIM v1.2.0 - PHASE 3")
        logger.info("=" * 80)
        logger.info(f"📍 URL de base: {self.base_url}")
        logger.info(f"🕐 Démarrage: {time.strftime('%Y-%m-%d %H:%M:%S')}")

        # Vérification de l'API
        if not await self.check_api_status():
            logger.error("❌ Impossible de continuer - API non accessible")
            return False

        # Démonstrations
        await self.demo_ecosystem_info()
        await self.demo_emotions()
        await self.demo_behaviors()
        await self.demo_modes()
        await self.demo_documentation()

        # Résumé final
        logger.info("\n" + "=" * 80)
        logger.info("🎉 DÉMONSTRATION TERMINÉE")
        logger.info("=" * 80)
        logger.info(
            "✅ Toutes les fonctionnalités de l'API publique ont été démontrées",
        )
        logger.info("🌐 L'écosystème BBIA-SIM est prêt pour l'intégration")
        logger.info("📚 Consultez la documentation pour plus de détails")
        logger.info(f"🕐 Fin: {time.strftime('%Y-%m-%d %H:%M:%S')}")

        return True

    async def close(self):
        """Ferme le client HTTP."""
        await self.client.aclose()


async def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description="🎭 Démonstration complète de l'API publique BBIA-SIM",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation :

  # Démonstration complète sur localhost:8000 (défaut)
  python scripts/demo_public_api.py

  # Démonstration sur une autre URL
  python scripts/demo_public_api.py --url http://localhost:3000

  # Démonstration avec logs détaillés
  python scripts/demo_public_api.py --log-level debug
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

    # Création de la démonstration
    demo = BBIADemo(args.url)

    try:
        # Exécution de la démonstration
        success = await demo.run_complete_demo()

        # Code de sortie
        sys.exit(0 if success else 1)
    finally:
        await demo.close()


if __name__ == "__main__":
    asyncio.run(main())
