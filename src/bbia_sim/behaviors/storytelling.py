#!/usr/bin/env python3
"""BBIA Storytelling Behavior - Raconter histoires avec mouvements expressifs.

Ce comportement permet à BBIA de raconter des histoires pré-enregistrées
avec des mouvements synchronisés, des émotions selon les scènes, et des
interactions avec l'utilisateur.
"""

import logging
import time
from typing import TYPE_CHECKING, Any

from .base import BBIABehavior

if TYPE_CHECKING:
    from bbia_sim.robot_api import RobotAPI

logger = logging.getLogger(__name__)

# Import SDK officiel pour create_head_pose
try:
    from reachy_mini.utils import create_head_pose

    REACHY_MINI_UTILS_AVAILABLE = True
except ImportError:
    REACHY_MINI_UTILS_AVAILABLE = False
    create_head_pose = None


class StorytellingBehavior(BBIABehavior):
    """Comportement de narration d'histoires avec mouvements expressifs."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le comportement storytelling.

        Args:
            robot_api: Interface robotique pour contrôler le robot

        """
        super().__init__(
            name="storytelling",
            description="Raconter histoires avec mouvements synchronisés",
            robot_api=robot_api,
        )
        self.stories = {
            "petit_chaperon_rouge": self._story_little_red_riding_hood,
            "trois_petits_cochons": self._story_three_little_pigs,
            "blanche_neige": self._story_snow_white,
        }
        self.current_story: str | None = None
        self.current_scene = 0

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution (peut contenir 'story' pour choisir l'histoire)

        Returns:
            True si le comportement peut être exécuté

        """
        if not self.robot_api:
            logger.warning("StorytellingBehavior: robot_api non disponible")
            return False
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le comportement storytelling.

        Args:
            context: Contexte d'exécution
                - 'story': Nom de l'histoire à raconter (optionnel)
                - 'interactive': Mode interactif avec questions (optionnel)

        Returns:
            True si l'exécution a réussi

        """
        if not self.robot_api:
            return False

        story_name = context.get("story", "petit_chaperon_rouge")
        interactive = context.get("interactive", False)

        if story_name not in self.stories:
            logger.warning("Histoire '%s' non trouvée", story_name)
            return False

        self.is_active = True
        self.current_story = story_name
        self.current_scene = 0

        try:
            story_func = self.stories[story_name]
            story_func(interactive=interactive)
            return True
        except Exception:
            logger.exception("Erreur lors de la narration")
            return False
        finally:
            self.is_active = False

    def _story_little_red_riding_hood(self, interactive: bool = False) -> None:
        """Raconte l'histoire du Petit Chaperon Rouge.

        Args:
            interactive: Mode interactif avec questions

        """
        scenes = [
            {
                "text": "Il était une fois une petite fille qui s'appelait le Petit Chaperon Rouge.",
                "emotion": "happy",
                "movement": {"yaw": 0.0, "pitch": 0.1},
            },
            {
                "text": "Sa grand-mère lui demanda d'apporter un panier de nourriture.",
                "emotion": "curious",
                "movement": {"yaw": 0.2, "pitch": 0.0},
            },
            {
                "text": "En chemin, elle rencontra le grand méchant loup !",
                "emotion": "fearful",
                "movement": {"yaw": -0.3, "pitch": -0.1},
            },
            {
                "text": "Mais le chasseur arriva à temps pour sauver la grand-mère.",
                "emotion": "happy",
                "movement": {"yaw": 0.0, "pitch": 0.1},
            },
        ]

        for i, scene in enumerate(scenes):
            if not self.is_active:
                break

            self.current_scene = i
            self._narrate_scene(scene, interactive=interactive)

            if interactive and i < len(scenes) - 1:
                # Pause pour question utilisateur
                time.sleep(2.0)

    def _story_three_little_pigs(self, interactive: bool = False) -> None:
        """Raconte l'histoire des Trois Petits Cochons.

        Args:
            interactive: Mode interactif avec questions

        """
        scenes = [
            {
                "text": "Il était une fois trois petits cochons qui décidèrent de construire leurs maisons.",
                "emotion": "excited",
                "movement": {"yaw": 0.0, "pitch": 0.1},
            },
            {
                "text": "Le premier construisit sa maison en paille, très rapidement.",
                "emotion": "happy",
                "movement": {"yaw": 0.2, "pitch": 0.0},
            },
            {
                "text": "Le loup souffla et détruisit la maison en paille !",
                "emotion": "fearful",
                "movement": {"yaw": -0.3, "pitch": -0.1},
            },
            {
                "text": "Mais la maison en briques résista, et les cochons furent sauvés !",
                "emotion": "happy",
                "movement": {"yaw": 0.0, "pitch": 0.1},
            },
        ]

        for i, scene in enumerate(scenes):
            if not self.is_active:
                break

            self.current_scene = i
            self._narrate_scene(scene, interactive=interactive)

            if interactive and i < len(scenes) - 1:
                time.sleep(2.0)

    def _story_snow_white(self, interactive: bool = False) -> None:
        """Raconte l'histoire de Blanche-Neige.

        Args:
            interactive: Mode interactif avec questions

        """
        scenes = [
            {
                "text": "Il était une fois une princesse nommée Blanche-Neige, la plus belle du royaume.",
                "emotion": "happy",
                "movement": {"yaw": 0.0, "pitch": 0.1},
            },
            {
                "text": "Sa belle-mère, la reine, était jalouse de sa beauté.",
                "emotion": "sad",
                "movement": {"yaw": -0.2, "pitch": -0.05},
            },
            {
                "text": "Blanche-Neige s'enfuit dans la forêt et rencontra les sept nains.",
                "emotion": "curious",
                "movement": {"yaw": 0.2, "pitch": 0.0},
            },
            {
                "text": "Le prince charmant la réveilla d'un baiser, et ils vécurent heureux.",
                "emotion": "happy",
                "movement": {"yaw": 0.0, "pitch": 0.1},
            },
        ]

        for i, scene in enumerate(scenes):
            if not self.is_active:
                break

            self.current_scene = i
            self._narrate_scene(scene, interactive=interactive)

            if interactive and i < len(scenes) - 1:
                time.sleep(2.0)

    def _narrate_scene(self, scene: dict[str, Any], interactive: bool = False) -> None:
        """Narrate une scène avec mouvement et émotion.

        Args:
            scene: Dictionnaire avec 'text', 'emotion', 'movement'
            interactive: Mode interactif

        """
        if not self.robot_api:
            return

        # Appliquer émotion
        emotion = scene.get("emotion", "neutral")
        try:
            from bbia_sim.bbia_emotions import BBIAEmotions

            emotions_module = BBIAEmotions()
            emotions_module.set_emotion(emotion, intensity=0.7)
        except ImportError:
            logger.warning("BBIAEmotions non disponible")

        # Appliquer mouvement
        movement = scene.get("movement", {})
        if REACHY_MINI_UTILS_AVAILABLE and create_head_pose:
            pose = create_head_pose(
                yaw=movement.get("yaw", 0.0),
                pitch=movement.get("pitch", 0.0),
                degrees=False,
            )
            self.robot_api.goto_target(head=pose, duration=1.5)

        # Narrer texte (via TTS si disponible)
        text = scene.get("text", "")
        try:
            from bbia_sim.bbia_voice import dire_texte

            dire_texte(text, robot_api=self.robot_api)
        except ImportError:
            logger.info("[STORYTELLING] %s", text)

        # Pause entre scènes
        time.sleep(1.0)

    def stop(self) -> None:
        """Arrête le comportement storytelling."""
        self.is_active = False
        self.current_story = None
        self.current_scene = 0
        logger.info("StorytellingBehavior arrêté")
