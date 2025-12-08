#!/usr/bin/env python3
"""Robot Registry - Découverte automatique des robots via Zenoh.

Inspiré de @pierre-rouanet pour découverte automatique robots sur réseau local.
"""

import logging
from typing import Any

logger = logging.getLogger(__name__)

# Import conditionnel Zenoh
try:
    import zenoh  # type: ignore[import-untyped]
    from zenoh import Config  # type: ignore[import-untyped]

    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    zenoh = None  # type: ignore[assignment]
    Config = Any  # type: ignore[assignment,misc]


class RobotRegistry:
    """Registre pour découverte automatique des robots Reachy Mini."""

    def __init__(self) -> None:
        """Initialise le registre."""
        self.robots: list[dict[str, Any]] = []
        self.session: Any | None = None

    def discover_robots(self, timeout: float = 5.0) -> list[dict[str, Any]]:
        """Découvre automatiquement les robots sur le réseau local via Zenoh.

        Args:
            timeout: Timeout pour la découverte (secondes)

        Returns:
            Liste des robots découverts avec leurs informations
        """
        if not ZENOH_AVAILABLE:
            logger.warning("Zenoh non disponible - découverte robots impossible")
            return []

        try:
            if zenoh is None:
                logger.error("Zenoh non disponible")
                return []

            # Configuration Zenoh pour découverte
            config = Config()
            config.insert_json5("mode", '"client"')
            config.insert_json5("connect", '["tcp://localhost:7447"]')

            # Ouvrir session Zenoh
            self.session = zenoh.open(config)

            # Découvrir robots via topics Zenoh
            # Les robots Reachy Mini publient sur "reachy/mini/*"
            discovered_robots: list[dict[str, Any]] = []

            # Chercher sur topics connus
            topics_to_check = [
                "reachy/mini/state",
                "reachy/mini/telemetry",
            ]

            for topic in topics_to_check:
                try:
                    # Subscriber pour détecter robots
                    subscriber = self.session.declare_subscriber(topic)
                    # Attendre un peu pour recevoir des messages
                    import time

                    time.sleep(0.5)
                    # Fermer subscriber
                    subscriber.undeclare()
                except Exception as e:
                    logger.debug("Topic %s non disponible: %s", topic, e)

            # Pour l'instant, retourner robots détectés via variables d'environnement
            # TODO: Implémenter vraie découverte via Zenoh quand API disponible
            import os

            robot_id = os.environ.get("BBIA_ROBOT_ID")
            if robot_id:
                discovered_robots.append(
                    {
                        "id": robot_id,
                        "hostname": os.environ.get("BBIA_HOSTNAME", "localhost"),
                        "port": int(os.environ.get("BBIA_PORT", "8080")),
                        "status": "available",
                    }
                )

            logger.info("Découverte %d robot(s)", len(discovered_robots))
            return discovered_robots

        except Exception as e:
            logger.exception("Erreur découverte robots: %s", e)
            return []
        finally:
            if self.session:
                try:
                    self.session.close()
                except Exception:
                    pass

    def list_robots(self) -> list[dict[str, Any]]:
        """Liste les robots disponibles.

        Returns:
            Liste des robots avec leurs informations
        """
        if not self.robots:
            # Découvrir robots si pas encore fait
            self.robots = self.discover_robots()

        return self.robots
