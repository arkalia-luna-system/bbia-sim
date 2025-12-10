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
import re
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
        # Timing adaptatif : historique des durées réelles
        self.speech_history: list[float] = []
        self.max_history = 10

    def analyze_speech_rhythm(self, text: str) -> dict[str, Any]:
        """Analyse le rythme de la parole pour timing adaptatif.

        Détecte pauses (ponctuation), accélérations (mots courts),
        et ajuste le timing dynamiquement.

        Args:
            text: Texte à analyser

        Returns:
            Dictionnaire avec analyse du rythme
        """
        words = text.split()
        word_count = len(words)

        # Détecter pauses (ponctuation)
        punctuation_count = len(re.findall(r"[.!?;:,]", text))
        pause_ratio = punctuation_count / max(word_count, 1)

        # Détecter mots courts (accélération potentielle)
        short_words = sum(1 for w in words if len(w) <= 3)
        short_word_ratio = short_words / max(word_count, 1)

        # Estimer vitesse adaptative
        # Plus de pauses = plus lent, plus de mots courts = plus rapide
        base_wpm = 150
        if pause_ratio > 0.15:  # Beaucoup de pauses
            adjusted_wpm = base_wpm * 0.9  # Plus lent
        elif pause_ratio < 0.05:  # Peu de pauses
            adjusted_wpm = base_wpm * 1.1  # Plus rapide
        else:
            adjusted_wpm = base_wpm

        # Ajuster selon mots courts
        if short_word_ratio > 0.4:  # Beaucoup de mots courts
            adjusted_wpm *= 1.05  # Légèrement plus rapide
        elif short_word_ratio < 0.2:  # Peu de mots courts
            adjusted_wpm *= 0.95  # Légèrement plus lent

        return {
            "word_count": word_count,
            "pause_ratio": pause_ratio,
            "short_word_ratio": short_word_ratio,
            "adjusted_wpm": int(adjusted_wpm),
            "base_wpm": base_wpm,
        }

    def estimate_speech_duration(
        self, text: str, words_per_minute: int | None = None
    ) -> float:
        """Estime la durée de la parole en secondes avec timing adaptatif.

        Args:
            text: Texte à prononcer
            words_per_minute: Vitesse de parole (mots/minute).
                Si None, utilise timing adaptatif

        Returns:
            Durée estimée en secondes
        """
        if words_per_minute is None:
            # Timing adaptatif : analyser rythme
            rhythm = self.analyze_speech_rhythm(text)
            words_per_minute = rhythm["adjusted_wpm"]
            logger.debug(
                "Timing adaptatif: %d mots/min (base: %d, pauses: %.2f, courts: %.2f)",
                words_per_minute,
                rhythm["base_wpm"],
                rhythm["pause_ratio"],
                rhythm["short_word_ratio"],
            )

        word_count = len(text.split())
        duration = (word_count / words_per_minute) * 60.0

        # Ajuster selon historique si disponible
        if self.speech_history:
            avg_duration = sum(self.speech_history) / len(self.speech_history)
            # Moyenne pondérée : 70% estimation, 30% historique
            duration = duration * 0.7 + avg_duration * 0.3

        # Minimum 0.5s, maximum 30s
        estimated = max(0.5, min(30.0, duration))

        # Mettre à jour historique
        self.speech_history.append(estimated)
        if len(self.speech_history) > self.max_history:
            self.speech_history.pop(0)

        return estimated

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
        """Micro-mouvement de tête subtil (amélioré pour plus de subtilité).

        Args:
            amplitude: Amplitude du mouvement (radians)
            duration: Durée du mouvement (secondes)
        """
        if not self.robot_api or not hasattr(self.robot_api, "goto_target"):
            return

        try:
            # Micro-mouvements plus subtils (0.01-0.02 rad comme recommandé)
            import random  # nosec B311

            # Amplitude réduite pour micro-mouvements subtils
            micro_amplitude = amplitude * random.uniform(0.3, 0.6)  # nosec B311
            micro_pitch = random.uniform(-0.01, 0.01)  # nosec B311

            from reachy_mini.utils import (
                create_head_pose,  # type: ignore[import-untyped]
            )

            pose = create_head_pose(
                yaw=micro_amplitude, pitch=micro_pitch, degrees=False
            )
            self.robot_api.goto_target(head=pose, duration=duration, method="minjerk")
        except (ImportError, AttributeError, RuntimeError):
            # Fallback si create_head_pose non disponible
            if hasattr(self.robot_api, "goto_target"):
                import random  # nosec B311

                micro_amplitude = amplitude * random.uniform(0.3, 0.6)  # nosec B311
                self.robot_api.goto_target(
                    body_yaw=micro_amplitude, duration=duration, method="minjerk"
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
        """Micro-mouvements subtils pendant l'écoute (amélioré avec respiration).

        Ajoute des micro-expressions et effet "respiration" pour robot plus vivant.
        """
        if not self.robot_api:
            return

        movement_count = 0
        import random  # nosec B311

        while not self.stop_sync.is_set():
            try:
                # Micro-mouvements variés avec effet "respiration"
                if movement_count % 6 == 0:
                    # Micro-mouvement de tête très subtil (0.01-0.02 rad)
                    self._micro_head_movement(0.015, 0.4)
                elif movement_count % 6 == 2:
                    # Micro-mouvement d'antenne très subtil
                    self._micro_antenna_movement(0.01, 0.3)
                elif movement_count % 6 == 4:
                    # Effet "respiration" : micro-mouvement très petit et lent
                    if hasattr(self.robot_api, "goto_target"):
                        # Micro-mouvement de corps (respiration)
                        self.robot_api.goto_target(
                            body_yaw=random.uniform(-0.005, 0.005),  # nosec B311
                            duration=2.0,  # Lent comme respiration
                            method="minjerk",
                        )

                movement_count += 1
                # Intervalle variable pour plus de naturel
                interval = random.uniform(1.2, 2.0)  # nosec B311
                time.sleep(interval)

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
