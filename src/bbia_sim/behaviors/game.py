#!/usr/bin/env python3
"""BBIA Game Behavior - Jeux interactifs.

Ce comportement permet à BBIA de jouer des jeux interactifs avec l'utilisateur
comme pierre-papier-ciseaux, devine le nombre, etc., avec des réactions
selon le résultat et un système de score.
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


class GameBehavior(BBIABehavior):
    """Comportement de jeux interactifs."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le comportement game.

        Args:
            robot_api: Interface robotique pour contrôler le robot
        """
        super().__init__(
            name="game",
            description="Jeux interactifs avec réactions émotionnelles",
            robot_api=robot_api,
        )
        self.games = {
            "rock_paper_scissors": self._game_rock_paper_scissors,
            "guess_number": self._game_guess_number,
            "memory": self._game_memory,
        }
        self.current_game: str | None = None
        self.user_score = 0
        self.robot_score = 0

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si le comportement peut être exécuté
        """
        if not self.robot_api:
            logger.warning("GameBehavior: robot_api non disponible")
            return False
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le comportement game.

        Args:
            context: Contexte d'exécution
                - 'game': Nom du jeu (rock_paper_scissors, guess_number, memory)
                - 'rounds': Nombre de rounds (optionnel, défaut 3)

        Returns:
            True si l'exécution a réussi
        """
        if not self.robot_api:
            return False

        game_name = context.get("game", "rock_paper_scissors")
        rounds = context.get("rounds", 3)

        if game_name not in self.games:
            logger.warning(f"Jeu '{game_name}' non trouvé")
            return False

        self.is_active = True
        self.current_game = game_name
        self.user_score = 0
        self.robot_score = 0

        try:
            game_func = self.games[game_name]
            game_func(rounds=rounds)
            self._show_final_score()
            return True
        except Exception as e:
            logger.error(f"Erreur lors du jeu: {e}")
            return False
        finally:
            self.is_active = False

    def _game_rock_paper_scissors(self, rounds: int = 3) -> None:
        """Jeu pierre-papier-ciseaux.

        Args:
            rounds: Nombre de rounds
        """
        intro = "Jouons à pierre-papier-ciseaux ! Choisissez votre geste."
        self._speak_with_movement(intro, emotion="excited")

        choices = ["rock", "paper", "scissors"]
        gestures = {
            "rock": {"yaw": 0.0, "pitch": -0.1},
            "paper": {"yaw": 0.0, "pitch": 0.1},
            "scissors": {"yaw": 0.2, "pitch": 0.0},
        }

        for round_num in range(rounds):
            if not self.is_active:
                break

            # Choix robot
            robot_choice = random.choice(choices)
            gesture = gestures[robot_choice]

            # Afficher choix robot
            choice_text = f"Round {round_num+1}: J'ai choisi {robot_choice} !"
            self._speak_with_movement(
                choice_text,
                emotion="curious",
                movement=gesture,
            )

            # Attendre choix utilisateur (simulation - dans version réelle, utiliser vision)
            time.sleep(3.0)

            # Simuler résultat (dans version réelle, comparer avec geste utilisateur)
            user_choice = random.choice(choices)  # Simulation

            result = self._determine_winner(user_choice, robot_choice)
            self._react_to_result(result, round_num + 1)

    def _game_guess_number(self, rounds: int = 3) -> None:
        """Jeu devine le nombre.

        Args:
            rounds: Nombre de rounds
        """
        intro = "Jouons à devine le nombre ! Je pense à un nombre entre 1 et 10."
        self._speak_with_movement(intro, emotion="excited")

        for round_num in range(rounds):
            if not self.is_active:
                break

            number = random.randint(1, 10)
            self._speak_with_movement(
                f"Round {round_num+1}: Devinez mon nombre !",
                emotion="curious",
            )

            # Attendre réponse (simulation)
            time.sleep(3.0)

            # Simuler réponse (dans version réelle, utiliser STT)
            guess = random.randint(1, 10)  # Simulation

            if guess == number:
                self._react_to_result("win", round_num + 1)
            else:
                self._react_to_result("lose", round_num + 1)

    def _game_memory(self, rounds: int = 3) -> None:
        """Jeu de mémoire.

        Args:
            rounds: Nombre de rounds
        """
        intro = "Jouons à un jeu de mémoire ! Je vais vous montrer une séquence."
        self._speak_with_movement(intro, emotion="excited")

        for round_num in range(rounds):
            if not self.is_active:
                break

            # Créer séquence
            sequence_length = round_num + 2
            sequence = [
                random.choice(["left", "right", "up", "down"])
                for _ in range(sequence_length)
            ]

            # Afficher séquence
            self._speak_with_movement(
                f"Round {round_num+1}: Regardez bien la séquence !",
                emotion="curious",
            )

            for direction in sequence:
                movement_map = {
                    "left": {"yaw": -0.3, "pitch": 0.0},
                    "right": {"yaw": 0.3, "pitch": 0.0},
                    "up": {"yaw": 0.0, "pitch": 0.2},
                    "down": {"yaw": 0.0, "pitch": -0.2},
                }
                movement = movement_map.get(direction, {"yaw": 0.0, "pitch": 0.0})

                if REACHY_MINI_UTILS_AVAILABLE and create_head_pose:
                    pose = create_head_pose(
                        yaw=movement["yaw"],
                        pitch=movement["pitch"],
                        degrees=False,
                    )
                    if self.robot_api:
                        self.robot_api.goto_target(head=pose, duration=0.5)

                time.sleep(0.8)

            # Attendre réponse (simulation)
            time.sleep(3.0)

            # Simuler résultat (dans version réelle, comparer séquence utilisateur)
            is_correct = random.choice([True, False])  # Simulation

            if is_correct:
                self._react_to_result("win", round_num + 1)
            else:
                self._react_to_result("lose", round_num + 1)

    def _determine_winner(self, user: str, robot: str) -> str:
        """Détermine le gagnant pierre-papier-ciseaux.

        Args:
            user: Choix utilisateur
            robot: Choix robot

        Returns:
            'win', 'lose', ou 'tie'
        """
        if user == robot:
            return "tie"

        winning_combinations = {
            "rock": "scissors",
            "paper": "rock",
            "scissors": "paper",
        }

        if winning_combinations[user] == robot:
            return "win"
        return "lose"

    def _react_to_result(self, result: str, round_num: int) -> None:
        """Réagit au résultat du jeu.

        Args:
            result: 'win', 'lose', ou 'tie'
            round_num: Numéro du round
        """
        if result == "win":
            self.user_score += 1
            emotion = "happy"
            messages = [
                "Bravo ! Vous avez gagné ce round !",
                "Excellent ! Vous avez gagné !",
                "Félicitations ! Vous avez gagné !",
            ]
        elif result == "lose":
            self.robot_score += 1
            emotion = "sad"
            messages = [
                "J'ai gagné ce round !",
                "J'ai gagné !",
                "J'ai gagné ce round, bien joué quand même !",
            ]
        else:  # tie
            emotion = "neutral"
            messages = [
                "Égalité !",
                "Match nul !",
                "Égalité pour ce round !",
            ]

        import random

        message = random.choice(messages)
        self._speak_with_movement(message, emotion=emotion)

    def _show_final_score(self) -> None:
        """Affiche le score final."""
        if self.user_score > self.robot_score:
            emotion = "happy"
            message = f"Félicitations ! Vous avez gagné {self.user_score} à {self.robot_score} !"
        elif self.robot_score > self.user_score:
            emotion = "proud"
            message = f"J'ai gagné {self.robot_score} à {self.user_score} ! Bien joué !"
        else:
            emotion = "neutral"
            message = f"Match nul ! {self.user_score} partout !"

        self._speak_with_movement(message, emotion=emotion)

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
            emotions_module.set_emotion(emotion, intensity=0.7)
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
            logger.info(f"[GAME] {text}")

    def stop(self) -> None:
        """Arrête le comportement game."""
        self.is_active = False
        self.current_game = None
        self.user_score = 0
        self.robot_score = 0
        logger.info("GameBehavior arrêté")
