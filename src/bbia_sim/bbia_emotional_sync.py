#!/usr/bin/env python3
"""Module de synchronisation fine mouvements émotionnels ↔ parole.

Inspiré de reachy-mini-plugin (LAURA-agent) pour mouvements naturels
pendant conversation.

Fonctionnalités :
- Synchronisation fine : mouvements pendant la parole (pas avant/après)
- Timing adaptatif : mouvements selon rythme de la parole
- Micro-mouvements : petites animations pendant conversation
- Transitions fluides : passage d'une émotion à l'autre pendant parole
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from enum import Enum
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from bbia_sim.robot_api import RobotAPI

logger = logging.getLogger(__name__)


class ConversationState(Enum):
    """États conversationnels pour transitions naturelles."""

    IDLE = "idle"
    LISTENING = "listening"
    THINKING = "thinking"
    SPEAKING = "speaking"
    REACTING = "reacting"


class BBIAEmotionalSync:
    """Module de synchronisation fine mouvements émotionnels ↔ parole.

    Synchronise les mouvements émotionnels avec le timing de la parole
    pour des interactions plus naturelles.
    """

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        """Initialise le module de synchronisation.

        Args:
            robot_api: Interface RobotAPI pour contrôler le robot
        """
        self.robot_api = robot_api
        self.current_state = ConversationState.IDLE
        self.is_speaking = False
        self.sync_thread: threading.Thread | None = None
        self.stop_sync = threading.Event()

    def estimate_speech_duration(self, text: str, words_per_minute: int = 150) -> float:
        """Estime la durée de la parole en secondes.

        Args:
            text: Texte à prononcer
            words_per_minute: Vitesse de parole (mots/minute)

        Returns:
            Durée estimée en secondes
        """
        word_count = len(text.split())
        duration = (word_count / words_per_minute) * 60.0
        # Minimum 0.5s, maximum 30s
        return max(0.5, min(30.0, duration))

    def sync_speak_with_emotion(
        self,
        text: str,
        emotion: str = "neutral",
        intensity: float = 0.6,
        speak_function: Callable[[str, Any | None], None] | None = None,
    ) -> bool:
        """Parle avec synchronisation fine des mouvements émotionnels.

        Les mouvements se produisent PENDANT la parole, pas avant/après.

        Args:
            text: Texte à prononcer
            emotion: Émotion à exprimer
            intensity: Intensité de l'émotion (0.0 à 1.0)
            speak_function: Fonction pour parler (dire_texte par défaut)

        Returns:
            True si succès
        """
        if not self.robot_api:
            logger.warning("Robot API non disponible pour synchronisation")
            return False

        try:
            # Estimer durée de la parole
            speech_duration = self.estimate_speech_duration(text)

            # Changer état
            self.current_state = ConversationState.SPEAKING
            self.is_speaking = True
            self.stop_sync.clear()

            # Démarrer synchronisation dans thread séparé
            self.sync_thread = threading.Thread(
                target=self._sync_emotion_during_speech,
                args=(emotion, intensity, speech_duration),
                daemon=True,
            )
            self.sync_thread.start()

            # Parler (bloquant)
            if speak_function:
                speak_function(text, self.robot_api)
            else:
                # Import conditionnel pour éviter dépendance circulaire
                try:
                    from bbia_sim.bbia_voice import dire_texte

                    dire_texte(text, robot_api=self.robot_api)
                except ImportError:
                    logger.warning("dire_texte non disponible")
                    return False

            # Arrêter synchronisation
            self.stop_sync.set()
            if self.sync_thread:
                self.sync_thread.join(timeout=1.0)

            self.is_speaking = False
            self.current_state = ConversationState.IDLE

            return True

        except (ValueError, AttributeError, RuntimeError) as e:
            logger.exception("Erreur synchronisation parole/émotion: %s", e)
            self.is_speaking = False
            self.current_state = ConversationState.IDLE
            return False

    def _sync_emotion_during_speech(
        self,
        emotion: str,
        intensity: float,
        duration: float,
    ) -> None:
        """Synchronise les mouvements émotionnels pendant la parole.

        Args:
            emotion: Émotion à exprimer
            intensity: Intensité de l'émotion
            duration: Durée estimée de la parole
        """
        if not self.robot_api:
            return

        try:
            # Appliquer émotion initiale
            if hasattr(self.robot_api, "set_emotion"):
                self.robot_api.set_emotion(emotion, intensity)

            # Micro-mouvements pendant la parole
            start_time = time.time()
            movement_count = 0
            movement_interval = max(0.3, duration / 10.0)  # 10 mouvements max

            while not self.stop_sync.is_set() and (time.time() - start_time) < duration:
                # Micro-mouvement de tête toutes les X secondes
                if movement_count % 2 == 0:
                    self._micro_head_movement(0.05, 0.2)
                else:
                    self._micro_antenna_movement(0.03, 0.2)

                movement_count += 1
                time.sleep(movement_interval)

        except (ValueError, AttributeError, RuntimeError) as e:
            logger.warning("Erreur synchronisation mouvements: %s", e)

    def _micro_head_movement(self, amplitude: float, duration: float) -> None:
        """Micro-mouvement de tête subtil.

        Args:
            amplitude: Amplitude du mouvement (radians)
            duration: Durée du mouvement (secondes)
        """
        if not self.robot_api or not hasattr(self.robot_api, "goto_target"):
            return

        try:
            # Petit mouvement de tête
            from reachy_mini.utils import (
                create_head_pose,  # type: ignore[import-untyped]
            )

            pose = create_head_pose(yaw=amplitude, pitch=0.0, degrees=False)
            self.robot_api.goto_target(head=pose, duration=duration, method="minjerk")
        except (ImportError, AttributeError, RuntimeError):
            # Fallback si create_head_pose non disponible
            if hasattr(self.robot_api, "goto_target"):
                self.robot_api.goto_target(
                    body_yaw=amplitude, duration=duration, method="minjerk"
                )

    def _micro_antenna_movement(self, amplitude: float, duration: float) -> None:
        """Micro-mouvement d'antenne subtil.

        Args:
            amplitude: Amplitude du mouvement (radians)
            duration: Durée du mouvement (secondes)
        """
        if not self.robot_api or not hasattr(self.robot_api, "goto_target"):
            return

        try:
            # Petit mouvement d'antenne
            antennas = [amplitude, amplitude * 0.8]
            self.robot_api.goto_target(
                antennas=antennas, duration=duration, method="minjerk"
            )
        except (AttributeError, RuntimeError, ValueError) as e:
            logger.debug("Micro-mouvement antenne non disponible: %s", e)

    def start_listening_movements(self) -> None:
        """Démarre les micro-mouvements pendant l'écoute."""
        if not self.robot_api:
            return

        self.current_state = ConversationState.LISTENING
        self.stop_sync.clear()

        # Thread pour micro-mouvements pendant écoute
        self.sync_thread = threading.Thread(
            target=self._listening_micro_movements,
            daemon=True,
        )
        self.sync_thread.start()

    def stop_listening_movements(self) -> None:
        """Arrête les micro-mouvements pendant l'écoute."""
        self.stop_sync.set()
        if self.sync_thread:
            self.sync_thread.join(timeout=1.0)
        self.current_state = ConversationState.IDLE

    def _listening_micro_movements(self) -> None:
        """Micro-mouvements subtils pendant l'écoute."""
        if not self.robot_api:
            return

        movement_count = 0

        while not self.stop_sync.is_set():
            try:
                # Micro-mouvements variés pendant écoute
                if movement_count % 4 == 0:
                    # Petit mouvement de tête
                    self._micro_head_movement(0.03, 0.3)
                elif movement_count % 4 == 2:
                    # Petit mouvement d'antenne
                    self._micro_antenna_movement(0.02, 0.3)

                movement_count += 1
                time.sleep(1.5)  # Intervalle entre mouvements

            except (ValueError, AttributeError, RuntimeError) as e:
                logger.debug("Erreur micro-mouvement écoute: %s", e)
                break

    def transition_to_thinking(self) -> None:
        """Transition vers état de réflexion."""
        if not self.robot_api:
            return

        self.current_state = ConversationState.THINKING

        try:
            # Légère inclinaison de tête (réflexion)
            from reachy_mini.utils import (
                create_head_pose,  # type: ignore[import-untyped]
            )

            pose = create_head_pose(yaw=0.1, pitch=-0.1, degrees=False)
            if hasattr(self.robot_api, "goto_target"):
                self.robot_api.goto_target(head=pose, duration=0.5, method="minjerk")
        except (ImportError, AttributeError, RuntimeError):
            # Fallback
            if hasattr(self.robot_api, "goto_target"):
                self.robot_api.goto_target(body_yaw=0.1, duration=0.5, method="minjerk")

    def transition_to_reacting(self, emotion: str, intensity: float = 0.7) -> None:
        """Transition vers état de réaction émotionnelle.

        Args:
            emotion: Émotion à exprimer
            intensity: Intensité de l'émotion
        """
        if not self.robot_api:
            return

        self.current_state = ConversationState.REACTING

        try:
            # Appliquer émotion avec transition fluide
            if hasattr(self.robot_api, "set_emotion"):
                self.robot_api.set_emotion(emotion, intensity)

            # Petit mouvement expressif
            self._micro_head_movement(0.08, 0.4)
        except (ValueError, AttributeError, RuntimeError) as e:
            logger.warning("Erreur transition réaction: %s", e)

    def get_current_state(self) -> ConversationState:
        """Retourne l'état conversationnel actuel.

        Returns:
            État conversationnel actuel
        """
        return self.current_state
