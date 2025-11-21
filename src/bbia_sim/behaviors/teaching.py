#!/usr/bin/env python3
"""BBIA Teaching Behavior - Mode éducatif interactif.

Ce comportement permet à BBIA d'enseigner des leçons (maths, sciences, etc.)
avec des mouvements explicatifs, des questions/réponses interactives,
et des encouragements selon la performance.
"""

import logging
import random
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


class TeachingBehavior(BBIABehavior):
    """Comportement d'enseignement interactif."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le comportement teaching.

        Args:
            robot_api: Interface robotique pour contrôler le robot
        """
        super().__init__(
            name="teaching",
            description="Mode éducatif interactif avec leçons et questions",
            robot_api=robot_api,
        )
        self.lessons = {
            "maths": self._lesson_maths,
            "sciences": self._lesson_sciences,
            "geographie": self._lesson_geographie,
        }
        self.current_lesson: str | None = None
        self.score = 0
        self.total_questions = 0

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si le comportement peut être exécuté
        """
        if not self.robot_api:
            logger.warning("TeachingBehavior: robot_api non disponible")
            return False
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le comportement teaching.

        Args:
            context: Contexte d'exécution
                - 'subject': Matière à enseigner (maths, sciences, etc.)
                - 'level': Niveau (beginner, intermediate, advanced)

        Returns:
            True si l'exécution a réussi
        """
        if not self.robot_api:
            return False

        subject = context.get("subject", "maths")
        level = context.get("level", "beginner")

        if subject not in self.lessons:
            logger.warning("Matière '%s' non trouvée", subject)
            return False

        self.is_active = True
        self.current_lesson = subject
        self.score = 0
        self.total_questions = 0

        try:
            lesson_func = self.lessons[subject]
            lesson_func(level=level)
            return True
        except Exception as e:
            logger.exception("Erreur lors de l'enseignement: %s", e)
            return False
        finally:
            self.is_active = False
            self._show_final_score()

    def _lesson_maths(self, level: str = "beginner") -> None:
        """Leçon de mathématiques.

        Args:
            level: Niveau de difficulté
        """
        if level == "beginner":
            questions = [
                {"question": "Combien font 2 + 2 ?", "answer": "4"},
                {"question": "Combien font 3 + 5 ?", "answer": "8"},
                {"question": "Combien font 10 - 3 ?", "answer": "7"},
            ]
        else:
            questions = [
                {"question": "Combien font 15 × 3 ?", "answer": "45"},
                {"question": "Combien font 20 ÷ 4 ?", "answer": "5"},
            ]

        self._teach_with_questions(questions, subject="maths")

    def _lesson_sciences(self, level: str = "beginner") -> None:
        """Leçon de sciences.

        Args:
            level: Niveau de difficulté
        """
        questions = [
            {
                "question": "Combien de planètes dans le système solaire ?",
                "answer": "8",
            },
            {
                "question": "Quel est l'élément chimique de symbole H ?",
                "answer": "hydrogène",
            },
        ]

        self._teach_with_questions(questions, subject="sciences")

    def _lesson_geographie(self, level: str = "beginner") -> None:
        """Leçon de géographie.

        Args:
            level: Niveau de difficulté
        """
        questions = [
            {"question": "Quelle est la capitale de la France ?", "answer": "paris"},
            {
                "question": "Quel est le plus grand océan ?",
                "answer": "pacifique",
            },
        ]

        self._teach_with_questions(questions, subject="geographie")

    def _teach_with_questions(
        self, questions: list[dict[str, str]], subject: str
    ) -> None:
        """Enseigne avec questions/réponses.

        Args:
            questions: Liste de questions avec réponses
            subject: Matière enseignée
        """
        # Introduction
        intro_text = f"Bonjour ! Aujourd'hui, nous allons apprendre le {subject}."
        self._speak_with_movement(intro_text, emotion="happy")

        # Questions
        for _i, qa in enumerate(questions):
            if not self.is_active:
                break

            question = qa["question"]
            correct_answer = qa["answer"].lower()

            # Poser question avec mouvement
            self._speak_with_movement(
                question, emotion="curious", movement={"yaw": 0.2, "pitch": 0.1}
            )

            # Attendre réponse (simulation - dans version réelle, utiliser STT)
            time.sleep(3.0)  # Simulation attente réponse

            # Vérifier réponse (simulation - toujours correcte pour l'instant)
            # Dans version réelle, utiliser STT pour capturer réponse
            is_correct = True  # Simulation

            self.total_questions += 1
            if is_correct:
                self.score += 1
                self._encourage(emotion="happy")
            else:
                self._encourage(emotion="sad", correct_answer=correct_answer)

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
            self.robot_api.goto_target(head=pose, duration=1.0)

        # Parler
        try:
            from ..bbia_voice import dire_texte

            dire_texte(text, robot_api=self.robot_api)
        except ImportError:
            logger.info("[TEACHING] %s", text)

    def _encourage(
        self, emotion: str = "happy", correct_answer: str | None = None
    ) -> None:
        """Encourage l'utilisateur selon performance.

        Args:
            emotion: Émotion à exprimer
            correct_answer: Réponse correcte si erreur
        """
        if emotion == "happy":
            messages = [
                "Excellent !",
                "Très bien !",
                "Bravo !",
                "Parfait !",
            ]
        else:
            messages = [
                f"Presque ! La bonne réponse est {correct_answer}.",
                f"Pas tout à fait. C'était {correct_answer}.",
            ]

        message = random.choice(messages)  # nosec B311
        self._speak_with_movement(message, emotion=emotion)

    def _show_final_score(self) -> None:
        """Affiche le score final."""
        if self.total_questions == 0:
            return

        percentage = (self.score / self.total_questions) * 100
        if percentage >= 80:
            emotion = "happy"
            message = (
                f"Excellent travail ! Tu as obtenu {self.score}/{self.total_questions}."
            )
        elif percentage >= 50:
            emotion = "neutral"
            message = f"Bien joué ! Tu as obtenu {self.score}/{self.total_questions}."
        else:
            emotion = "sad"
            message = f"Continue à pratiquer ! Tu as obtenu {self.score}/{self.total_questions}."

        self._speak_with_movement(message, emotion=emotion)

    def stop(self) -> None:
        """Arrête le comportement teaching."""
        self.is_active = False
        self.current_lesson = None
        logger.info("TeachingBehavior arrêté")
