#!/usr/bin/env python3
"""
Démo BBIA réagit - Suivi de visage simulé
Démonstration de la connexion comportements → API → Simulateur

Ce script simule la détection d'un visage et fait suivre le robot
avec un mouvement oscillant doux du cou (neck_yaw).
"""

import argparse
import asyncio
import logging
import math
import time

import httpx

# Configuration du logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class BBIAFaceFollower:
    """Démonstrateur de suivi de visage BBIA."""

    def __init__(self, api_url: str, token: str):
        """Initialise le suiveur de visage.

        Args:
            api_url: URL de l'API BBIA-SIM
            token: Token d'authentification
        """
        self.api_url = api_url.rstrip("/")
        self.token = token
        self.headers = {"Authorization": f"Bearer {token}"}
        self.client = httpx.AsyncClient(timeout=10.0)
        self.is_running = False
        self.start_time = None

    async def check_api_health(self) -> bool:
        """Vérifie que l'API est accessible.

        Returns:
            True si l'API est accessible
        """
        try:
            response = await self.client.get(f"{self.api_url}/health")
            if response.status_code == 200:
                logger.info("✅ API BBIA-SIM accessible")
                return True
            else:
                logger.error(f"❌ API non accessible: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"❌ Erreur connexion API: {e}")
            return False

    async def get_initial_position(self) -> float:
        """Récupère la position initiale du cou.

        Returns:
            Position initiale en radians
        """
        try:
            response = await self.client.get(
                f"{self.api_url}/api/state/joints", headers=self.headers
            )
            if response.status_code == 200:
                data = response.json()
                neck_pos = data["joints"]["neck_yaw"]["position"]
                logger.info(f"📍 Position initiale du cou: {neck_pos:.3f} rad")
                return neck_pos
            else:
                logger.warning("⚠️ Impossible de récupérer la position initiale")
                return 0.0
        except Exception as e:
            logger.error(f"❌ Erreur récupération position: {e}")
            return 0.0

    async def move_neck(self, position: float) -> bool:
        """Déplace le cou à une position donnée.

        Args:
            position: Position cible en radians

        Returns:
            True si le mouvement a réussi
        """
        try:
            joint_data = [{"joint_name": "neck_yaw", "position": position}]
            response = await self.client.post(
                f"{self.api_url}/api/motion/joints",
                json=joint_data,
                headers=self.headers,
            )

            if response.status_code == 200:
                result = response.json()
                if result.get("success_count", 0) > 0:
                    logger.debug(f"🎯 Cou déplacé à {position:.3f} rad")
                    return True
                else:
                    logger.warning(f"⚠️ Échec mouvement cou vers {position:.3f}")
                    return False
            else:
                logger.error(f"❌ Erreur API mouvement: {response.status_code}")
                return False

        except Exception as e:
            logger.error(f"❌ Erreur mouvement cou: {e}")
            return False

    async def simulate_face_detection(self) -> bool:
        """Simule la détection d'un visage.

        Returns:
            True si un visage est "détecté"
        """
        # Simulation simple : visage détecté toutes les 2 secondes
        elapsed = time.time() - self.start_time
        return int(elapsed) % 2 == 0

    async def run_face_following(self, duration: float = 10.0) -> None:
        """Exécute le suivi de visage pendant la durée spécifiée.

        Args:
            duration: Durée du suivi en secondes
        """
        logger.info(f"🎭 Démarrage suivi de visage pour {duration}s")

        # Vérification API
        if not await self.check_api_health():
            logger.error("❌ API non accessible, arrêt du démo")
            return

        # Position initiale
        initial_pos = await self.get_initial_position()

        self.is_running = True
        self.start_time = time.time()

        # Paramètres du mouvement oscillant
        amplitude = 0.4  # ±0.4 radian
        frequency = 0.5  # 0.5 Hz (période de 2s)

        logger.info(
            f"🎯 Mouvement oscillant: amplitude={amplitude}rad, fréquence={frequency}Hz"
        )

        try:
            while self.is_running:
                elapsed = time.time() - self.start_time

                if elapsed >= duration:
                    logger.info("⏰ Durée atteinte, arrêt du suivi")
                    break

                # Simulation détection visage
                face_detected = await self.simulate_face_detection()

                if face_detected:
                    # Calcul position oscillante
                    angle = amplitude * math.sin(2 * math.pi * frequency * elapsed)
                    target_position = initial_pos + angle

                    # Application du mouvement
                    success = await self.move_neck(target_position)

                    if success:
                        logger.info(
                            f"👁️ Visage détecté! Cou à {target_position:.3f} rad"
                        )
                    else:
                        logger.warning("⚠️ Échec mouvement")
                else:
                    logger.debug("🔍 Recherche de visage...")

                # Attente avant prochaine itération
                await asyncio.sleep(0.5)

        except KeyboardInterrupt:
            logger.info("🛑 Arrêt demandé par l'utilisateur")
        except Exception as e:
            logger.error(f"❌ Erreur durant le suivi: {e}")
        finally:
            self.is_running = False

            # Retour à la position initiale
            logger.info("🏠 Retour à la position initiale")
            await self.move_neck(initial_pos)
            await asyncio.sleep(1.0)

    async def close(self) -> None:
        """Ferme les connexions."""
        await self.client.aclose()


async def main() -> None:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description="Démo BBIA réagit - Suivi de visage simulé",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation:
  python examples/behave_follow_face.py --token bbia-secret-key-dev
  python examples/behave_follow_face.py --url http://localhost:8000 --duration 15
        """,
    )

    parser.add_argument(
        "--url",
        default="http://localhost:8000",
        help="URL de l'API BBIA-SIM (défaut: http://localhost:8000)",
    )
    parser.add_argument(
        "--token",
        default="bbia-secret-key-dev",
        help="Token d'authentification (défaut: bbia-secret-key-dev)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Durée du suivi en secondes (défaut: 10.0)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Mode verbose pour plus de détails",
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    logger.info("🤖 Démo BBIA réagit - Suivi de visage")
    logger.info("=" * 50)
    logger.info(f"API URL: {args.url}")
    logger.info(f"Token: {args.token[:10]}...")
    logger.info(f"Durée: {args.duration}s")
    logger.info("=" * 50)

    # Création et exécution du suiveur
    follower = BBIAFaceFollower(args.url, args.token)

    try:
        await follower.run_face_following(args.duration)
        logger.info("✅ Démo terminée avec succès!")
    except Exception as e:
        logger.error(f"❌ Erreur durant la démo: {e}")
        return 1
    finally:
        await follower.close()

    return 0


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    exit(exit_code)
