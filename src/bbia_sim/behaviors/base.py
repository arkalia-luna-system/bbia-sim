#!/usr/bin/env python3
"""Classe de base pour tous les comportements BBIA.

Ce module définit l'interface commune que tous les comportements
doivent implémenter.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from ..robot_api import RobotAPI

# Création du dossier log si besoin (préférence utilisateur)
Path("log").mkdir(parents=True, exist_ok=True)

# Logger BBIA
logger = logging.getLogger("BBIA")
# Supprime tous les handlers existants
for h in list(logger.handlers):
    logger.removeHandler(h)
handler = logging.FileHandler("log/bbia.log", mode="a", encoding="utf-8")
formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)
logger.setLevel(logging.INFO)


class BBIABehavior:
    """Comportement de base pour BBIA.

    Tous les comportements doivent hériter de cette classe
    et implémenter les méthodes can_execute() et execute().
    """

    def __init__(
        self,
        name: str,
        description: str,
        robot_api: RobotAPI | None = None,
    ) -> None:
        """Initialise un comportement.

        Args:
            name: Nom du comportement
            description: Description du comportement
            robot_api: Instance RobotAPI pour contrôler le robot (optionnel)

        """
        self.name = name
        self.description = description
        self.is_active = False
        self.priority = 1  # 1-10, 10 étant le plus prioritaire
        self.robot_api = robot_api  # Référence au robot pour contrôler les mouvements

    def can_execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution (optionnel)

        Returns:
            True si le comportement peut être exécuté, False sinon

        """
        logger.info("Vérification d'exécution du comportement : %s", self.name)
        return True

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        """Exécute le comportement.

        Args:
            context: Contexte d'exécution (optionnel)

        Returns:
            True si l'exécution a réussi, False sinon

        """
        logger.info("Exécution du comportement : %s", self.name)
        return True

    def stop(self) -> None:
        """Arrête le comportement."""
        logger.info("Arrêt du comportement : %s", self.name)
        self.is_active = False
