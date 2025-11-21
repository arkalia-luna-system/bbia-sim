#!/usr/bin/env python3
"""BBIA Weather Report Behavior - Rapport météo avec gestes expressifs.

Ce comportement permet à BBIA de donner un rapport météo avec des
mouvements expressifs selon les conditions (soleil = happy, pluie = sad),
et des recommandations (parapluie, etc.).
"""

import logging
import time
from typing import TYPE_CHECKING, Any

from .base import BBIABehavior

if TYPE_CHECKING:
    from ..robot_api import RobotAPI

logger = logging.getLogger(__name__)

# Import SDK officiel pour create_head_pose
try:
    from reachy_mini.utils import create_head_pose

    REACHY_MINI_UTILS_AVAILABLE = True
except ImportError:
    REACHY_MINI_UTILS_AVAILABLE = False
    create_head_pose = None


class WeatherReportBehavior(BBIABehavior):
    """Comportement de rapport météo avec gestes."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le comportement weather_report.

        Args:
            robot_api: Interface robotique pour contrôler le robot
        """
        super().__init__(
            name="weather_report",
            description="Rapport météo avec mouvements expressifs",
            robot_api=robot_api,
        )
        self.weather_data: dict[str, Any] | None = None

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si le comportement peut être exécuté
        """
        if not self.robot_api:
            logger.warning("WeatherReportBehavior: robot_api non disponible")
            return False
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le comportement weather_report.

        Args:
            context: Contexte d'exécution
                - 'city': Ville pour météo (optionnel)
                - 'weather_data': Données météo (optionnel, sinon récupération API)

        Returns:
            True si l'exécution a réussi
        """
        if not self.robot_api:
            return False

        city = context.get("city", "Paris")
        self.weather_data = context.get("weather_data")

        # Si pas de données, simuler (dans version réelle, utiliser API)
        if not self.weather_data:
            self.weather_data = self._simulate_weather(city)

        self.is_active = True

        try:
            self._report_weather()
            return True
        except Exception as e:
            logger.exception("Erreur lors du rapport météo: %s", e)
            return False
        finally:
            self.is_active = False

    def _simulate_weather(self, city: str) -> dict[str, Any]:
        """Simule des données météo (à remplacer par vraie API).

        Args:
            city: Ville

        Returns:
            Dictionnaire avec données météo simulées
        """
        # Simulation - dans version réelle, utiliser openweathermap API
        import random

        conditions = ["sunny", "rainy", "cloudy", "snowy"]
        condition = random.choice(conditions)  # nosec B311

        return {
            "city": city,
            "condition": condition,
            "temperature": random.randint(10, 25),  # nosec B311
            "humidity": random.randint(40, 80),  # nosec B311
        }

    def _report_weather(self) -> None:
        """Rapporte la météo avec mouvements expressifs."""
        if not self.weather_data:
            return

        city = self.weather_data.get("city", "Paris")
        condition = self.weather_data.get("condition", "sunny")
        temperature = self.weather_data.get("temperature", 20)

        # Déterminer émotion et mouvement selon condition
        weather_mapping = {
            "sunny": {
                "emotion": "happy",
                "movement": {"yaw": 0.0, "pitch": 0.15},
                "text": f"Le temps est ensoleillé à {city} !",
            },
            "rainy": {
                "emotion": "sad",
                "movement": {"yaw": 0.0, "pitch": -0.1},
                "text": f"Il pleut à {city} aujourd'hui.",
            },
            "cloudy": {
                "emotion": "neutral",
                "movement": {"yaw": 0.0, "pitch": 0.0},
                "text": f"Le temps est nuageux à {city}.",
            },
            "snowy": {
                "emotion": "excited",
                "movement": {"yaw": 0.1, "pitch": 0.1},
                "text": f"Il neige à {city} !",
            },
        }

        weather_info = weather_mapping.get(condition, weather_mapping["sunny"])

        # Introduction
        intro = f"Voici la météo pour {city}."
        self._speak_with_movement(intro, emotion="neutral")

        time.sleep(1.0)

        # Rapport principal
        report_text = (
            f"{weather_info['text']} La température est de {temperature} degrés."
        )
        # Cast explicite pour mypy
        emotion_str: str = str(weather_info["emotion"])
        movement_dict: dict[str, float] | None = (
            weather_info["movement"]
            if isinstance(weather_info["movement"], dict)
            else None
        )
        self._speak_with_movement(
            report_text,
            emotion=emotion_str,
            movement=movement_dict,
        )

        time.sleep(1.0)

        # Recommandation
        recommendation = self._get_recommendation(condition)
        if recommendation:
            self._speak_with_movement(recommendation, emotion="curious")

    def _get_recommendation(self, condition: str) -> str:
        """Obtient une recommandation selon la condition météo.

        Args:
            condition: Condition météo

        Returns:
            Recommandation
        """
        recommendations = {
            "sunny": "N'oubliez pas votre chapeau et de la crème solaire !",
            "rainy": "Pensez à prendre un parapluie !",
            "cloudy": "Le temps est variable, habillez-vous en conséquence.",
            "snowy": "Habillez-vous chaudement et faites attention à la route !",
        }
        return recommendations.get(condition, "")

    def _speak_with_movement(
        self,
        text: str,
        emotion: str = "neutral",
        movement: dict[str, float] | None = None,
    ) -> None:
        """Parle avec mouvement expressif.

        Args:
            text: Texte à dire
            emotion: Émotion à exprimer
            movement: Mouvement tête (yaw, pitch)
        """
        if not self.robot_api:
            return

        # Appliquer émotion
        try:
            from ..bbia_emotions import BBIAEmotions

            emotions_module = BBIAEmotions()
            emotions_module.set_emotion(emotion, intensity=0.6)
        except ImportError:
            pass

        # Appliquer mouvement
        if movement and REACHY_MINI_UTILS_AVAILABLE and create_head_pose:
            pose = create_head_pose(
                yaw=movement.get("yaw", 0.0),
                pitch=movement.get("pitch", 0.0),
                degrees=False,
            )
            self.robot_api.goto_target(head=pose, duration=1.5)

        # Parler
        try:
            from ..bbia_voice import dire_texte

            dire_texte(text, robot_api=self.robot_api)
        except ImportError:
            logger.info("[WEATHER] %s", text)

    def stop(self) -> None:
        """Arrête le comportement weather_report."""
        self.is_active = False
        self.weather_data = None
        logger.info("WeatherReportBehavior arrêté")
