#!/usr/bin/env python3
"""BBIA Adaptive Learning - Apprentissage adaptatif et mémoire à long terme.

Ce module permet à BBIA d'apprendre les préférences utilisateur,
d'adapter ses comportements selon le contexte, et de sauvegarder
une mémoire à long terme.
"""

import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from .robot_api import RobotAPI

# Création du dossier log si besoin
Path("log").mkdir(parents=True, exist_ok=True)

# Fichier de sauvegarde des préférences
PREFERENCES_FILE = Path("log/bbia_preferences.json")


class BBIAAdaptiveLearning:
    """Module d'apprentissage adaptatif pour BBIA.

    Permet à BBIA d'apprendre les préférences utilisateur et
    d'adapter ses comportements selon le contexte.
    """

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le module d'apprentissage adaptatif.

        Args:
            robot_api: Interface robotique (optionnel)

        """
        self.robot_api = robot_api
        self.user_preferences: dict[str, Any] = {}
        self.behavior_history: list[dict[str, Any]] = []
        self.context_patterns: dict[str, Any] = {}

        # Charger préférences sauvegardées
        self._load_preferences()

    def learn_preference(self, user_action: str, context: dict[str, Any]) -> None:
        """Apprend une préférence utilisateur.

        Args:
            user_action: Action de l'utilisateur (ex: "court", "rapide")
            context: Contexte de l'action (ex: {"response_length": "short"})

        """
        # Analyser action pour extraire préférence
        if "court" in user_action.lower() or "short" in user_action.lower():
            self.user_preferences["response_length"] = "short"
        elif "long" in user_action.lower() or "détails" in user_action.lower():
            self.user_preferences["response_length"] = "long"

        if "rapide" in user_action.lower() or "fast" in user_action.lower():
            self.user_preferences["response_speed"] = "fast"
        elif "lent" in user_action.lower() or "slow" in user_action.lower():
            self.user_preferences["response_speed"] = "slow"

        # Sauvegarder préférences
        self._save_preferences()

        logger.info("Préférence apprise: %s", self.user_preferences)

    def adapt_behavior(self, context: dict[str, Any]) -> dict[str, Any]:
        """Adapte un comportement selon le contexte et les préférences.

        Args:
            context: Contexte actuel

        Returns:
            Dictionnaire avec adaptations à appliquer

        """
        adaptations: dict[str, Any] = {}

        # Adapter selon préférences
        if self.user_preferences.get("response_length") == "short":
            adaptations["max_length"] = 100
        elif self.user_preferences.get("response_length") == "long":
            adaptations["max_length"] = 500

        if self.user_preferences.get("response_speed") == "fast":
            adaptations["duration"] = 0.5
        elif self.user_preferences.get("response_speed") == "slow":
            adaptations["duration"] = 2.0

        # Adapter selon contexte
        if context.get("time_of_day") == "morning":
            adaptations["emotion_intensity"] = 0.6  # Plus calme le matin
        elif context.get("time_of_day") == "evening":
            adaptations["emotion_intensity"] = 0.8  # Plus énergique le soir

        return adaptations

    def remember_interaction(
        self,
        behavior: str,
        context: dict[str, Any],
        result: str,
    ) -> None:
        """Mémorise une interaction pour apprentissage futur.

        Args:
            behavior: Nom du comportement exécuté
            context: Contexte de l'interaction
            result: Résultat (success, failure, etc.)

        """
        interaction = {
            "behavior": behavior,
            "context": context,
            "result": result,
            "timestamp": str(Path("log").stat().st_mtime),  # Approximation
        }

        self.behavior_history.append(interaction)

        # Garder seulement 100 dernières interactions
        if len(self.behavior_history) > 100:
            self.behavior_history = self.behavior_history[-100:]

        # Détecter patterns
        self._detect_patterns()

    def _detect_patterns(self) -> None:
        """Détecte des patterns dans l'historique des comportements."""
        # Analyser historique pour détecter patterns
        # Exemple : si utilisateur préfère toujours comportements rapides le matin
        # OPTIMISATION: Éviter double lookup avec variable temporaire
        morning_behaviors = [
            b
            for b in self.behavior_history
            if b.get("context", {}).get("time_of_day") == "morning"
        ]

        if len(morning_behaviors) > 5:
            # Pattern détecté : préférence pour comportements rapides le matin
            self.context_patterns["morning"] = {"preferred_speed": "fast"}

    def _load_preferences(self) -> None:
        """Charge les préférences depuis le fichier."""
        try:
            if PREFERENCES_FILE.exists():
                with open(PREFERENCES_FILE, encoding="utf-8") as f:
                    data = json.load(f)
                    self.user_preferences = data.get("preferences", {})
                    self.context_patterns = data.get("patterns", {})
                    logger.info("Préférences chargées depuis fichier")
        except (OSError, json.JSONDecodeError, KeyError) as e:
            logger.warning("Impossible de charger préférences: %s", e)

    def _save_preferences(self) -> None:
        """Sauvegarde les préférences dans un fichier."""
        try:
            data = {
                "preferences": self.user_preferences,
                "patterns": self.context_patterns,
            }
            with open(PREFERENCES_FILE, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            logger.info("Préférences sauvegardées")
        except (OSError, TypeError) as e:
            logger.warning("Impossible de sauvegarder préférences: %s", e)

    def get_preferences(self) -> dict[str, Any]:
        """Récupère les préférences utilisateur.

        Returns:
            Dictionnaire avec préférences

        """
        return self.user_preferences.copy()

    def get_patterns(self) -> dict[str, Any]:
        """Récupère les patterns détectés.

        Returns:
            Dictionnaire avec patterns

        """
        return self.context_patterns.copy()
