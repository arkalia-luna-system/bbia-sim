#!/usr/bin/env python3
"""Mode démo complet BBIA-SIM - Phase 3 : Écosystème Ouvert."""

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


class BBIADemoMode:
    """Mode démonstration complet pour BBIA-SIM Phase 3."""

    def __init__(self, base_url: str = "http://localhost:8000"):
        """Initialise le mode démo.

        Args:
            base_url: URL de base de l'API
        """
        self.base_url = base_url
        self.client = httpx.AsyncClient(timeout=30.0)
        self.demo_config = {
            "simulation": {
                "name": "Mode Simulation",
                "description": "Démonstration avec simulation MuJoCo",
                "backend": "mujoco",
                "features": ["3D Viewer", "Physique réaliste", "Contrôle complet"],
                "duration": 30.0,
                "emotions": ["happy", "curious", "excited", "sad"],
                "behaviors": ["wake_up", "greeting", "nod", "wave"],
            },
            "robot_real": {
                "name": "Mode Robot Réel",
                "description": "Démonstration avec robot Reachy Mini physique",
                "backend": "reachy_mini",
                "features": ["Robot physique", "SDK officiel", "Contrôle sécurisé"],
                "duration": 20.0,
                "emotions": ["happy", "curious"],
                "behaviors": ["wake_up", "greeting"],
            },
            "mixed": {
                "name": "Mode Mixte",
                "description": "Basculement entre simulation et robot réel",
                "backend": "auto",
                "features": ["Switch automatique", "Comparaison", "Tests"],
                "duration": 45.0,
                "emotions": ["happy", "sad", "curious", "excited"],
                "behaviors": ["wake_up", "greeting", "nod"],
            },
        }

    async def check_api_status(self) -> bool:
        """Vérifie que l'API est accessible."""
        try:
            logger.info("🔍 Vérification de l'API...")
            response = await self.client.get(f"{self.base_url}/health")

            if response.status_code == 200:
                data = response.json()
                logger.info(f"✅ API accessible: {data.get('status', 'N/A')}")
                return True
            else:
                logger.error(f"❌ API non accessible: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"❌ Erreur de connexion: {e}")
            return False

    async def get_ecosystem_info(self) -> dict | None:
        """Récupère les informations de l'écosystème."""
        try:
            response = await self.client.get(
                f"{self.base_url}/api/ecosystem/capabilities"
            )
            if response.status_code == 200:
                return response.json()
            return None
        except Exception as e:
            logger.error(f"❌ Erreur récupération écosystème: {e}")
            return None

    async def demo_mode_simulation(self) -> bool:
        """Démonstration en mode simulation."""
        logger.info("\n" + "=" * 60)
        logger.info("🎮 DÉMONSTRATION - MODE SIMULATION")
        logger.info("=" * 60)

        config = self.demo_config["simulation"]
        logger.info(f"📊 Configuration: {config['name']}")
        logger.info(f"📊 Description: {config['description']}")
        logger.info(f"📊 Backend: {config['backend']}")
        logger.info(f"📊 Durée: {config['duration']}s")

        try:
            # Démarrage de la démo
            logger.info("\n🚀 Démarrage de la démonstration simulation...")
            response = await self.client.post(
                f"{self.base_url}/api/ecosystem/demo/start",
                params={
                    "mode": "simulation",
                    "duration": config["duration"],
                    "emotion": "happy",
                },
            )

            if response.status_code == 200:
                data = response.json()
                logger.info(f"✅ Démo démarrée: {data.get('status', 'N/A')}")
                logger.info(f"📊 Message: {data.get('message', 'N/A')}")
            else:
                logger.warning(f"⚠️ Démo: Échec (code {response.status_code})")

            # Démonstration des émotions
            logger.info("\n😊 Démonstration des émotions BBIA...")
            for emotion in config["emotions"]:
                try:
                    response = await self.client.post(
                        f"{self.base_url}/api/ecosystem/emotions/apply",
                        params={"emotion": emotion, "intensity": 0.7, "duration": 3.0},
                    )

                    if response.status_code == 200:
                        data = response.json()
                        logger.info(f"   ✅ {emotion}: {data.get('status', 'N/A')}")
                        logger.info(
                            f"      Joints affectés: {len(data.get('joints_affected', []))}"
                        )
                    else:
                        logger.warning(
                            f"   ⚠️ {emotion}: Échec (code {response.status_code})"
                        )

                    await asyncio.sleep(1.0)  # Pause entre les émotions

                except Exception as e:
                    logger.warning(f"   ⚠️ {emotion}: Erreur - {e}")

            # Démonstration des comportements
            logger.info("\n🎭 Démonstration des comportements BBIA...")
            for behavior in config["behaviors"]:
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
                            f"      Durée estimée: {data.get('estimated_duration', 'N/A')}s"
                        )
                    else:
                        logger.warning(
                            f"   ⚠️ {behavior}: Échec (code {response.status_code})"
                        )

                    await asyncio.sleep(2.0)  # Pause entre les comportements

                except Exception as e:
                    logger.warning(f"   ⚠️ {behavior}: Erreur - {e}")

            logger.info("\n✅ Démonstration simulation terminée")
            return True

        except Exception as e:
            logger.error(f"❌ Erreur démo simulation: {e}")
            return False

    async def demo_mode_robot_real(self) -> bool:
        """Démonstration en mode robot réel."""
        logger.info("\n" + "=" * 60)
        logger.info("🤖 DÉMONSTRATION - MODE ROBOT RÉEL")
        logger.info("=" * 60)

        config = self.demo_config["robot_real"]
        logger.info(f"📊 Configuration: {config['name']}")
        logger.info(f"📊 Description: {config['description']}")
        logger.info(f"📊 Backend: {config['backend']}")
        logger.info(f"📊 Durée: {config['duration']}s")
        logger.info("⚠️ ATTENTION: Mode robot réel - Sécurité maximale")

        try:
            # Démarrage de la démo
            logger.info("\n🚀 Démarrage de la démonstration robot réel...")
            response = await self.client.post(
                f"{self.base_url}/api/ecosystem/demo/start",
                params={
                    "mode": "robot_real",
                    "duration": config["duration"],
                    "emotion": "happy",
                },
            )

            if response.status_code == 200:
                data = response.json()
                logger.info(f"✅ Démo démarrée: {data.get('status', 'N/A')}")
                logger.info(f"📊 Message: {data.get('message', 'N/A')}")
            else:
                logger.warning(f"⚠️ Démo: Échec (code {response.status_code})")

            # Démonstration sécurisée des émotions
            logger.info("\n😊 Démonstration sécurisée des émotions BBIA...")
            for emotion in config["emotions"]:
                try:
                    response = await self.client.post(
                        f"{self.base_url}/api/ecosystem/emotions/apply",
                        params={
                            "emotion": emotion,
                            "intensity": 0.3,  # Intensité réduite pour sécurité
                            "duration": 2.0,  # Durée réduite
                        },
                    )

                    if response.status_code == 200:
                        data = response.json()
                        logger.info(f"   ✅ {emotion}: {data.get('status', 'N/A')}")
                        logger.info(
                            f"      Joints affectés: {len(data.get('joints_affected', []))}"
                        )
                    else:
                        logger.warning(
                            f"   ⚠️ {emotion}: Échec (code {response.status_code})"
                        )

                    await asyncio.sleep(2.0)  # Pause plus longue pour sécurité

                except Exception as e:
                    logger.warning(f"   ⚠️ {emotion}: Erreur - {e}")

            # Démonstration sécurisée des comportements
            logger.info("\n🎭 Démonstration sécurisée des comportements BBIA...")
            for behavior in config["behaviors"]:
                try:
                    response = await self.client.post(
                        f"{self.base_url}/api/ecosystem/behaviors/execute",
                        params={
                            "behavior": behavior,
                            "intensity": 0.5,  # Intensité réduite pour sécurité
                            "duration": 3.0,  # Durée réduite
                        },
                    )

                    if response.status_code == 200:
                        data = response.json()
                        logger.info(f"   ✅ {behavior}: {data.get('status', 'N/A')}")
                        logger.info(
                            f"      Durée estimée: {data.get('estimated_duration', 'N/A')}s"
                        )
                    else:
                        logger.warning(
                            f"   ⚠️ {behavior}: Échec (code {response.status_code})"
                        )

                    await asyncio.sleep(3.0)  # Pause plus longue pour sécurité

                except Exception as e:
                    logger.warning(f"   ⚠️ {behavior}: Erreur - {e}")

            logger.info("\n✅ Démonstration robot réel terminée")
            logger.info("🛡️ Sécurité: Tous les mouvements ont été sécurisés")
            return True

        except Exception as e:
            logger.error(f"❌ Erreur démo robot réel: {e}")
            return False

    async def demo_mode_mixed(self) -> bool:
        """Démonstration en mode mixte."""
        logger.info("\n" + "=" * 60)
        logger.info("🔄 DÉMONSTRATION - MODE MIXTE")
        logger.info("=" * 60)

        config = self.demo_config["mixed"]
        logger.info(f"📊 Configuration: {config['name']}")
        logger.info(f"📊 Description: {config['description']}")
        logger.info(f"📊 Backend: {config['backend']}")
        logger.info(f"📊 Durée: {config['duration']}s")

        try:
            # Phase 1: Simulation
            logger.info("\n🎮 Phase 1: Démonstration Simulation")
            await self.demo_mode_simulation()

            await asyncio.sleep(2.0)  # Pause entre les phases

            # Phase 2: Robot réel (si disponible)
            logger.info("\n🤖 Phase 2: Démonstration Robot Réel")
            logger.info("⚠️ Note: Mode robot réel nécessite un robot physique connecté")

            # Test de disponibilité robot réel
            try:
                response = await self.client.get(
                    f"{self.base_url}/api/ecosystem/status"
                )
                if response.status_code == 200:
                    data = response.json()
                    robot_connected = data.get("robot_connected", False)

                    if robot_connected:
                        logger.info("✅ Robot réel détecté - Démonstration sécurisée")
                        await self.demo_mode_robot_real()
                    else:
                        logger.info("⚠️ Robot réel non détecté - Simulation uniquement")
                        logger.info(
                            "💡 Connectez un robot Reachy Mini pour tester le mode réel"
                        )
            except Exception as e:
                logger.warning(f"⚠️ Impossible de vérifier le statut robot: {e}")

            # Phase 3: Comparaison et analyse
            logger.info("\n📊 Phase 3: Comparaison et Analyse")
            logger.info("✅ Simulation: Fonctionnalités complètes, sécurité maximale")
            logger.info("✅ Robot réel: Contrôle physique, sécurité renforcée")
            logger.info("✅ Mode mixte: Basculement transparent entre les modes")

            logger.info("\n✅ Démonstration mode mixte terminée")
            return True

        except Exception as e:
            logger.error(f"❌ Erreur démo mode mixte: {e}")
            return False

    async def run_demo_mode(self, mode: str) -> bool:
        """Exécute le mode de démonstration spécifié."""
        logger.info("🚀 DÉMONSTRATION COMPLÈTE BBIA-SIM v1.2.0 - PHASE 3")
        logger.info("=" * 80)
        logger.info(f"📍 URL de base: {self.base_url}")
        logger.info(f"🎮 Mode: {mode}")
        logger.info(f"🕐 Démarrage: {time.strftime('%Y-%m-%d %H:%M:%S')}")

        # Vérification de l'API
        if not await self.check_api_status():
            logger.error("❌ Impossible de continuer - API non accessible")
            return False

        # Récupération des informations écosystème
        ecosystem_info = await self.get_ecosystem_info()
        if ecosystem_info:
            logger.info("\n🌐 Informations Écosystème:")
            logger.info(f"   • Modèle: {ecosystem_info.get('model', 'N/A')}")
            logger.info(f"   • Joints: {ecosystem_info.get('joints', 'N/A')}")
            logger.info(f"   • Émotions: {len(ecosystem_info.get('emotions', []))}")
            logger.info(
                f"   • Comportements: {len(ecosystem_info.get('behaviors', []))}"
            )
            logger.info(
                f"   • Backends: {', '.join(ecosystem_info.get('backends', []))}"
            )

        # Exécution du mode spécifié
        success = False
        if mode == "simulation":
            success = await self.demo_mode_simulation()
        elif mode == "robot_real":
            success = await self.demo_mode_robot_real()
        elif mode == "mixed":
            success = await self.demo_mode_mixed()
        else:
            logger.error(f"❌ Mode '{mode}' non reconnu")
            return False

        # Résumé final
        logger.info("\n" + "=" * 80)
        logger.info("🎉 DÉMONSTRATION TERMINÉE")
        logger.info("=" * 80)
        if success:
            logger.info("✅ Démonstration réussie")
            logger.info(f"🎮 Mode: {mode}")
            logger.info("🌐 L'écosystème BBIA-SIM est prêt pour l'intégration")
            logger.info("📚 Consultez la documentation pour plus de détails")
        else:
            logger.warning("⚠️ Démonstration partiellement réussie")
            logger.warning("🔧 Vérifiez la configuration et les logs")

        logger.info(f"🕐 Fin: {time.strftime('%Y-%m-%d %H:%M:%S')}")

        return success

    async def close(self):
        """Ferme le client HTTP."""
        await self.client.aclose()


async def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description="🎮 Mode démonstration complet BBIA-SIM Phase 3",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation :

  # Démonstration simulation (recommandée)
  python scripts/demo_mode_complete.py --mode simulation

  # Démonstration robot réel (nécessite robot physique)
  python scripts/demo_mode_complete.py --mode robot_real

  # Démonstration mode mixte (simulation + robot réel)
  python scripts/demo_mode_complete.py --mode mixed

  # Démonstration avec logs détaillés
  python scripts/demo_mode_complete.py --mode simulation --log-level debug
        """,
    )

    parser.add_argument(
        "--mode",
        choices=["simulation", "robot_real", "mixed"],
        default="simulation",
        help="Mode de démonstration (défaut: simulation)",
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
    demo = BBIADemoMode(args.url)

    try:
        # Exécution de la démonstration
        success = await demo.run_demo_mode(args.mode)

        # Code de sortie
        sys.exit(0 if success else 1)
    finally:
        await demo.close()


if __name__ == "__main__":
    asyncio.run(main())
