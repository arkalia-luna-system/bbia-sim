#!/usr/bin/env python3
"""
BBIA Idle Animations - Animations d'inactivité pour robot Reachy Mini
Respiration automatique, poses de passage subtiles, tremblement vocal.

Inspiré de l'app conversationnelle officielle Reachy Mini.
"""

import logging
import math
import threading
import time
from typing import TYPE_CHECKING

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from .robot_api import RobotAPI


class BBIABreathingAnimation:
    """Animation de respiration subtile pour le robot en mode idle."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise l'animation de respiration.

        Args:
            robot_api: Instance RobotAPI pour contrôle robot
        """
        self.robot_api = robot_api
        self.is_active = False
        self.breathing_thread: threading.Thread | None = None
        self.stop_event = threading.Event()

        # Configuration respiration
        self.breathing_rate = 0.15  # Hz (cycles par seconde)
        self.breathing_amplitude = 0.02  # rad (amplitude max très subtile)
        self.breathing_duration = 4.0  # secondes par cycle complet

        logger.info("💨 BBIABreathingAnimation initialisé")

    def start(self) -> bool:
        """Démarre l'animation de respiration."""
        if self.is_active:
            logger.warning("Respiration déjà active")
            return False

        if not self.robot_api or not self.robot_api.is_connected:
            logger.warning("Robot non connecté - respiration désactivée")
            return False

        self.is_active = True
        self.stop_event.clear()

        # Créer thread pour animation background
        self.breathing_thread = threading.Thread(
            target=self._breathing_loop, daemon=True
        )
        self.breathing_thread.start()

        logger.info("✅ Animation respiration démarrée")
        return True

    def stop(self) -> None:
        """Arrête l'animation de respiration."""
        if not self.is_active:
            return

        self.is_active = False
        self.stop_event.set()

        if self.breathing_thread:
            self.breathing_thread.join(timeout=2.0)
            if self.breathing_thread.is_alive():
                logger.warning("Thread respiration n'a pas pu être arrêté dans les 2 secondes")

        logger.info("⏹️ Animation respiration arrêtée")

    def _breathing_loop(self) -> None:
        """Boucle principale de respiration."""
        try:
            # Obtenir position de base (neutre)
            base_pitch = 0.0

            start_time = time.time()

            while not self.stop_event.is_set():
                if not self.robot_api or not self.robot_api.is_connected:
                    break

                # Calculer phase respiration (sinusoïde lente)
                elapsed = time.time() - start_time
                phase = 2.0 * math.pi * self.breathing_rate * elapsed

                # Amplitude respiration (sinusoïde douce)
                breathing_offset = self.breathing_amplitude * math.sin(phase)

                # Appliquer mouvement subtil à la tête (pitch uniquement)
                # Conforme SDK: utiliser goto_target avec create_head_pose
                try:
                    from reachy_mini.utils import create_head_pose

                    # Position respiration: pitch légèrement relevé avec oscillation
                    pose = create_head_pose(pitch=base_pitch + breathing_offset, yaw=0.0)

                    # Appliquer mouvement fluide (duration courte pour continuité)
                    if hasattr(self.robot_api, "goto_target"):
                        self.robot_api.goto_target(
                            head=pose, duration=0.5, method="minjerk"
                        )
                    elif hasattr(self.robot_api, "set_target_head_pose"):
                        self.robot_api.set_target_head_pose(pose)

                except ImportError:
                    # Fallback: utiliser set_joint_pos directement si SDK non disponible
                    # Utiliser stewart_1 pour pitch (simulation MuJoCo)
                    if hasattr(self.robot_api, "set_joint_pos"):
                        pitch_joint = "stewart_1"
                        position = base_pitch + breathing_offset
                        self.robot_api.set_joint_pos(pitch_joint, position)

                # Attendre avant prochaine frame (50ms = 20 FPS pour animation fluide)
                time.sleep(0.05)

        except Exception as e:
            logger.error(f"Erreur boucle respiration: {e}")
        finally:
            self.is_active = False


class BBIAPoseTransitionManager:
    """Gestionnaire de poses de passage subtiles (idle poses)."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le gestionnaire de poses de passage.

        Args:
            robot_api: Instance RobotAPI pour contrôle robot
        """
        self.robot_api = robot_api
        self.is_active = False
        self.pose_thread: threading.Thread | None = None
        self.stop_event = threading.Event()

        # Poses de passage (subtiles, naturelles)
        self.idle_poses = [
            {"name": "look_slightly_left", "yaw": -0.05, "pitch": 0.02},
            {"name": "look_slightly_right", "yaw": 0.05, "pitch": 0.02},
            {"name": "look_up_slightly", "yaw": 0.0, "pitch": 0.03},
            {"name": "neutral", "yaw": 0.0, "pitch": 0.0},
        ]

        self.current_pose_index = 0
        self.pose_transition_interval = 15.0  # secondes entre poses

        logger.info("🎭 BBIAPoseTransitionManager initialisé")

    def start(self) -> bool:
        """Démarre les transitions de poses."""
        if self.is_active:
            logger.warning("Transitions de poses déjà actives")
            return False

        if not self.robot_api or not self.robot_api.is_connected:
            logger.warning("Robot non connecté - transitions désactivées")
            return False

        self.is_active = True
        self.stop_event.clear()

        self.pose_thread = threading.Thread(
            target=self._pose_transition_loop, daemon=True
        )
        self.pose_thread.start()

        logger.info("✅ Transitions de poses démarrées")
        return True

    def stop(self) -> None:
        """Arrête les transitions de poses."""
        if not self.is_active:
            return

        self.is_active = False
        self.stop_event.set()

        if self.pose_thread:
            self.pose_thread.join(timeout=2.0)
            if self.pose_thread.is_alive():
                logger.warning(
                    "Thread transitions poses n'a pas pu être arrêté dans les 2 secondes"
                )

        logger.info("⏹️ Transitions de poses arrêtées")

    def _pose_transition_loop(self) -> None:
        """Boucle principale de transitions de poses."""
        try:
            last_pose_time = time.time()

            while not self.stop_event.is_set():
                if not self.robot_api or not self.robot_api.is_connected:
                    break

                # Vérifier si on doit changer de pose
                elapsed = time.time() - last_pose_time
                if elapsed >= self.pose_transition_interval:
                    # Sélectionner pose suivante
                    self.current_pose_index = (
                        self.current_pose_index + 1
                    ) % len(self.idle_poses)
                    pose = self.idle_poses[self.current_pose_index]

                    # Appliquer pose avec transition fluide
                    try:
                        from reachy_mini.utils import create_head_pose

                        head_pose = create_head_pose(
                            yaw=pose["yaw"], pitch=pose["pitch"]
                        )

                        if hasattr(self.robot_api, "goto_target"):
                            self.robot_api.goto_target(
                                head=head_pose, duration=2.0, method="minjerk"
                            )
                        elif hasattr(self.robot_api, "set_target_head_pose"):
                            self.robot_api.set_target_head_pose(head_pose)

                        logger.debug(f"Transition vers pose: {pose['name']}")
                        last_pose_time = time.time()

                    except ImportError:
                        # Fallback: mouvement direct si SDK non disponible
                        pass

                # Attendre avant prochaine vérification (1 seconde)
                time.sleep(1.0)

        except Exception as e:
            logger.error(f"Erreur boucle transitions poses: {e}")
        finally:
            self.is_active = False


class BBIAVocalTremor:
    """Tremblement vocal réactif (réaction subtile à la voix)."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le tremblement vocal.

        Args:
            robot_api: Instance RobotAPI pour contrôle robot
        """
        self.robot_api = robot_api
        self.is_active = False
        self.last_audio_level = 0.0
        self.tremor_amplitude = 0.015  # rad (très subtil)

        logger.info("🎤 BBIAVocalTremor initialisé")

    def update_audio_level(self, audio_level: float) -> None:
        """Met à jour le niveau audio et applique tremblement si nécessaire.

        Args:
            audio_level: Niveau audio (0.0 à 1.0)
        """
        if not self.is_active or not self.robot_api:
            return

        # Tremblement proportionnel au niveau audio
        if audio_level > 0.3:  # Seuil minimum pour activation
            tremor_offset = self.tremor_amplitude * audio_level

            try:
                from reachy_mini.utils import create_head_pose

                # Tremblement subtil sur pitch
                pose = create_head_pose(pitch=tremor_offset, yaw=0.0)

                if hasattr(self.robot_api, "set_target_head_pose"):
                    self.robot_api.set_target_head_pose(pose)

            except ImportError:
                pass

        self.last_audio_level = audio_level

    def start(self) -> None:
        """Active le tremblement vocal."""
        self.is_active = True
        logger.info("✅ Tremblement vocal activé")

    def stop(self) -> None:
        """Désactive le tremblement vocal."""
        self.is_active = False
        logger.info("⏹️ Tremblement vocal désactivé")


class BBIIdleAnimationManager:
    """Gestionnaire centralisé des animations idle.

    Coordonne respiration, poses de passage et tremblement vocal.
    Inspiré de l'app conversationnelle officielle Reachy Mini.
    """

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le gestionnaire d'animations idle.

        Args:
            robot_api: Instance RobotAPI pour contrôle robot
        """
        self.robot_api = robot_api

        # Composants animations
        self.breathing = BBIABreathingAnimation(robot_api)
        self.pose_transitions = BBIAPoseTransitionManager(robot_api)
        self.vocal_tremor = BBIAVocalTremor(robot_api)

        # État global
        self.is_active = False

        logger.info("🎬 BBIIdleAnimationManager initialisé")

    def start(self) -> bool:
        """Démarre toutes les animations idle."""
        if self.is_active:
            logger.warning("Animations idle déjà actives")
            return False

        if not self.robot_api or not self.robot_api.is_connected:
            logger.warning("Robot non connecté - animations idle désactivées")
            return False

        # Démarrer composants
        breathing_ok = self.breathing.start()
        poses_ok = self.pose_transitions.start()
        self.vocal_tremor.start()

        self.is_active = breathing_ok or poses_ok

        if self.is_active:
            logger.info("✅ Animations idle démarrées (respiration + poses)")
        else:
            logger.warning("⚠️ Aucune animation idle n'a pu démarrer")

        return self.is_active

    def stop(self) -> None:
        """Arrête toutes les animations idle."""
        if not self.is_active:
            return

        self.breathing.stop()
        self.pose_transitions.stop()
        self.vocal_tremor.stop()

        self.is_active = False
        logger.info("⏹️ Animations idle arrêtées")

    def update_vocal_tremor(self, audio_level: float) -> None:
        """Met à jour le tremblement vocal avec niveau audio.

        Args:
            audio_level: Niveau audio (0.0 à 1.0)
        """
        if self.is_active:
            self.vocal_tremor.update_audio_level(audio_level)

    def is_running(self) -> bool:
        """Vérifie si les animations idle sont actives."""
        return self.is_active
