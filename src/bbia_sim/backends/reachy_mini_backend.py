#!/usr/bin/env python3
"""
ReachyMiniBackend - Implémentation SDK officiel Reachy-Mini
Backend utilisant le SDK officiel reachy_mini
"""

import logging
import time
from typing import Any, Optional

try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose

    REACHY_MINI_AVAILABLE = True
except ImportError:
    REACHY_MINI_AVAILABLE = False
    ReachyMini = None
    create_head_pose = None

from ..robot_api import RobotAPI

logger = logging.getLogger(__name__)


class ReachyMiniBackend(RobotAPI):
    """Backend Reachy-Mini officiel pour RobotAPI."""

    def __init__(self, robot_ip: str = "localhost", robot_port: int = 8080):
        super().__init__()
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot: Optional[ReachyMini] = None
        self.step_count = 0
        self.start_time: float = 0.0

        # Mapping joints officiel Reachy-Mini (noms réels du modèle MuJoCo)
        # 6 joints tête + 2 antennes + 1 corps = 9 joints total
        self.joint_mapping = {
            # Tête (6 joints Stewart platform - noms réels)
            "stewart_1": 0,  # Premier joint tête
            "stewart_2": 1,  # Deuxième joint tête
            "stewart_3": 2,  # Troisième joint tête
            "stewart_4": 3,  # Quatrième joint tête
            "stewart_5": 4,  # Cinquième joint tête
            "stewart_6": 5,  # Sixième joint tête
            # Antennes (2 joints)
            "left_antenna": 0,  # Antenne gauche
            "right_antenna": 1,  # Antenne droite
            # Corps (1 joint - nom réel)
            "yaw_body": 0,  # Rotation corps
        }

        # Limites officielles Reachy-Mini (en radians) - noms réels
        self.joint_limits = {
            # Tête (plateforme Stewart - noms réels)
            "stewart_1": (-0.5, 0.5),
            "stewart_2": (-0.5, 0.5),
            "stewart_3": (-0.5, 0.5),
            "stewart_4": (-0.5, 0.5),
            "stewart_5": (-0.5, 0.5),
            "stewart_6": (-0.5, 0.5),
            # Antennes
            "left_antenna": (-1.0, 1.0),
            "right_antenna": (-1.0, 1.0),
            # Corps (nom réel)
            "yaw_body": (-3.14, 3.14),  # Rotation complète
        }

        # Joints interdits (sécurité)
        self.forbidden_joints = {
            "left_antenna",  # Antennes trop fragiles
            "right_antenna",
        }

    def connect(self) -> bool:
        """Connecte au robot Reachy-Mini officiel."""
        if not REACHY_MINI_AVAILABLE:
            logger.error("SDK reachy_mini non disponible")
            return False

        try:
            # Connexion au robot Reachy-Mini avec timeout court pour simulation
            self.robot = ReachyMini(timeout=1.0)  # Timeout court pour éviter blocage
            self.is_connected = True
            self.start_time = time.time()
            logger.info("Connecté au robot Reachy-Mini officiel")
            return True
        except TimeoutError:
            # Pas de robot physique - mode simulation
            logger.info("Pas de robot physique détecté - mode simulation activé")
            self.robot = None
            self.is_connected = True  # Mode simulation
            self.start_time = time.time()
            return True
        except Exception as e:
            logger.error(f"Erreur connexion Reachy-Mini: {e}")
            self.is_connected = False
            return False

    def disconnect(self) -> bool:
        """Déconnecte du robot Reachy-Mini."""
        try:
            if self.robot:
                # Le SDK gère automatiquement la déconnexion
                self.robot = None
            self.is_connected = False
            logger.info("Déconnecté du robot Reachy-Mini")
            return True
        except Exception as e:
            logger.error(f"Erreur déconnexion: {e}")
            return False

    def get_available_joints(self) -> list[str]:
        """Retourne la liste des joints disponibles."""
        return list(self.joint_mapping.keys())

    def get_joint_pos(self, joint_name: str) -> Optional[float]:
        """Récupère la position actuelle d'un joint."""
        if not self.is_connected or not self.robot:
            return 0.0

        try:
            # Le SDK officiel retourne tuple[list[float], list[float]] (head_positions, antenna_positions)
            head_positions, antenna_positions = self.robot.get_current_joint_positions()

            # Mapping des noms vers indices (basé sur le modèle MuJoCo officiel)
            if joint_name == "yaw_body":
                # yaw_body est le premier élément des positions tête
                return float(head_positions[0]) if len(head_positions) > 0 else 0.0
            elif joint_name in ["left_antenna", "right_antenna"]:
                # Antennes dans antenna_positions
                antenna_idx = 0 if joint_name == "left_antenna" else 1
                return (
                    float(antenna_positions[antenna_idx])
                    if len(antenna_positions) > antenna_idx
                    else 0.0
                )
            elif joint_name.startswith("stewart_"):
                # Stewart joints dans head_positions (indices 1,3,5,7,9,11)
                stewart_idx = int(joint_name.split("_")[1])  # 1,2,3,4,5,6
                head_idx = stewart_idx * 2 - 1  # 1,3,5,7,9,11
                return (
                    float(head_positions[head_idx])
                    if len(head_positions) > head_idx
                    else 0.0
                )

            logger.warning(f"Joint {joint_name} non trouvé")
            return None

        except Exception as e:
            logger.error(f"Erreur lecture joint {joint_name}: {e}")
            return None

    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        """Définit la position d'un joint."""
        # Validation sécurité (toujours vérifier les joints interdits)
        if joint_name in self.forbidden_joints:
            logger.warning(f"Joint {joint_name} interdit pour sécurité")
            return False

        # Mode simulation si pas de robot physique OU pas connecté
        if not self.robot or not self.is_connected:
            logger.info(f"Mode simulation: joint {joint_name} = {position}")
            return True

        # Clamp amplitude
        position = max(
            -self.safe_amplitude_limit, min(self.safe_amplitude_limit, position)
        )

        try:
            if joint_name == "yaw_body":
                # Contrôler la rotation du corps via l'API officielle
                self.robot.set_target_body_yaw(position)
            elif joint_name in ["left_antenna", "right_antenna"]:
                # Contrôler les antennes via l'API officielle
                current_antennas = self.robot.get_present_antenna_joint_positions()
                antenna_idx = self.joint_mapping[joint_name]
                if len(current_antennas) > antenna_idx:
                    current_antennas[antenna_idx] = position
                    self.robot.set_target_antenna_joint_positions(current_antennas)
            else:
                # Contrôler la tête via pose (stewart joints) - API officielle
                # Créer une pose basée sur la position du joint
                pose = create_head_pose(
                    roll=position if "stewart_1" in joint_name else 0,
                    pitch=position if "stewart_2" in joint_name else 0,
                    yaw=position if "stewart_3" in joint_name else 0,
                    degrees=False,
                )
                self.robot.set_target_head_pose(pose)

            return True
        except Exception as e:
            logger.error(f"Erreur contrôle joint {joint_name}: {e}")
            return False

    def set_emotion(self, emotion: str, intensity: float = 0.5) -> bool:
        """Définit une émotion sur le robot."""
        # Mapping émotions vers poses tête
        emotion_poses = {
            "happy": create_head_pose(pitch=0.1, yaw=0.0, degrees=False),
            "sad": create_head_pose(pitch=-0.1, yaw=0.0, degrees=False),
            "neutral": create_head_pose(pitch=0.0, yaw=0.0, degrees=False),
            "excited": create_head_pose(pitch=0.2, yaw=0.1, degrees=False),
            "curious": create_head_pose(pitch=0.05, yaw=0.2, degrees=False),
            "calm": create_head_pose(pitch=-0.05, yaw=0.0, degrees=False),
        }

        # Vérifier si l'émotion est valide
        if emotion not in emotion_poses:
            logger.warning(f"Émotion {emotion} non reconnue")
            return False

        if not self.is_connected or not self.robot:
            # Mode simulation : toujours OK pour émotions valides
            self.current_emotion = emotion
            self.emotion_intensity = intensity
            logger.info(f"Émotion simulée: {emotion} (intensité: {intensity})")
            return True

        try:
            pose = emotion_poses[emotion]
            # Appliquer l'intensité
            pose[:3, 3] *= intensity
            self.robot.set_target_head_pose(pose)
            self.current_emotion = emotion
            self.emotion_intensity = intensity
            return True

        except Exception as e:
            logger.error(f"Erreur émotion {emotion}: {e}")
            return False

    def look_at(self, target_x: float, target_y: float, target_z: float = 0.0) -> bool:
        """Fait regarder le robot vers un point."""
        if not self.is_connected or not self.robot:
            # Mode simulation : toujours OK
            logger.info(f"Look_at simulé: ({target_x}, {target_y}, {target_z})")
            return True

        try:
            # Utiliser la méthode officielle look_at_world
            self.robot.look_at_world(target_x, target_y, target_z)
            return True
        except Exception as e:
            logger.error(f"Erreur look_at: {e}")
            return False

    def run_behavior(self, behavior_name: str, duration: float = 5.0, **kwargs) -> bool:
        """Exécute un comportement."""
        # Vérifier si le comportement est valide
        valid_behaviors = ["wake_up", "goto_sleep", "nod"]
        if behavior_name not in valid_behaviors:
            logger.warning(f"Comportement {behavior_name} non implémenté")
            return False

        if not self.is_connected or not self.robot:
            # Mode simulation : toujours OK pour comportements valides
            logger.info(f"Comportement simulé: {behavior_name} ({duration}s)")
            return True

        try:
            # Mapping comportements vers méthodes officielles
            if behavior_name == "wake_up":
                self.robot.wake_up()
            elif behavior_name == "goto_sleep":
                self.robot.goto_sleep()
            elif behavior_name == "nod":
                # Mouvement de hochement simple
                pose1 = create_head_pose(pitch=0.1, degrees=False)
                pose2 = create_head_pose(pitch=-0.1, degrees=False)
                self.robot.set_target_head_pose(pose1)
                time.sleep(0.5)
                self.robot.set_target_head_pose(pose2)
                time.sleep(0.5)
                self.robot.set_target_head_pose(create_head_pose(degrees=False))

            return True
        except Exception as e:
            logger.error(f"Erreur comportement {behavior_name}: {e}")
            return False

    def step(self) -> bool:
        """Effectue un pas de simulation."""
        self.step_count += 1
        return True

    def get_telemetry(self) -> dict[str, Any]:
        """Récupère les données de télémétrie."""
        try:
            current_time = time.time()
            elapsed_time = current_time - self.start_time if self.start_time > 0 else 0

            return {
                "step_count": self.step_count,
                "elapsed_time": elapsed_time,
                "steps_per_second": (
                    self.step_count / elapsed_time if elapsed_time > 0 else 0
                ),
                "current_emotion": self.current_emotion,
                "emotion_intensity": self.emotion_intensity,
                "is_connected": self.is_connected,
            }
        except Exception as e:
            logger.error(f"Erreur télémétrie: {e}")
            return {}

    # ===== MÉTHODES SDK OFFICIEL SUPPLÉMENTAIRES =====

    def get_current_head_pose(self) -> Optional[Any]:
        """Récupère la pose actuelle de la tête."""
        if not self.is_connected or not self.robot:
            return None

        try:
            return self.robot.get_current_head_pose()
        except Exception as e:
            logger.error(f"Erreur get_current_head_pose: {e}")
            return None

    def get_present_antenna_joint_positions(self) -> Optional[list[float]]:
        """Récupère les positions actuelles des antennes."""
        if not self.is_connected or not self.robot:
            return [0.0, 0.0]  # Mode simulation

        try:
            return self.robot.get_present_antenna_joint_positions()
        except Exception as e:
            logger.error(f"Erreur get_present_antenna_joint_positions: {e}")
            return None

    def set_target_body_yaw(self, body_yaw: float) -> bool:
        """Définit la rotation cible du corps."""
        if not self.is_connected or not self.robot:
            logger.info(f"Mode simulation: body_yaw = {body_yaw}")
            return True

        try:
            self.robot.set_target_body_yaw(body_yaw)
            return True
        except Exception as e:
            logger.error(f"Erreur set_target_body_yaw: {e}")
            return False

    def set_target_antenna_joint_positions(self, antennas: list[float]) -> bool:
        """Définit les positions cibles des antennes."""
        if not self.is_connected or not self.robot:
            logger.info(f"Mode simulation: antennas = {antennas}")
            return True

        try:
            self.robot.set_target_antenna_joint_positions(antennas)
            return True
        except Exception as e:
            logger.error(f"Erreur set_target_antenna_joint_positions: {e}")
            return False

    def look_at_image(
        self, u: int, v: int, duration: float = 1.0, perform_movement: bool = True
    ) -> bool:
        """Fait regarder le robot vers un point dans l'image."""
        if not self.is_connected or not self.robot:
            logger.info(f"Mode simulation: look_at_image({u}, {v})")
            return True

        try:
            self.robot.look_at_image(u, v, duration, perform_movement)
            return True
        except Exception as e:
            logger.error(f"Erreur look_at_image: {e}")
            return False

    def goto_target(
        self,
        head: Optional[Any] = None,
        antennas: Optional[list[float]] = None,
        duration: float = 0.5,
        body_yaw: Optional[float] = None,
    ) -> bool:
        """Va vers une cible spécifique."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: goto_target")
            return True

        try:
            self.robot.goto_target(
                head=head, antennas=antennas, duration=duration, body_yaw=body_yaw
            )
            return True
        except Exception as e:
            logger.error(f"Erreur goto_target: {e}")
            return False

    def enable_motors(self) -> bool:
        """Active les moteurs."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: enable_motors")
            return True

        try:
            self.robot.enable_motors()
            return True
        except Exception as e:
            logger.error(f"Erreur enable_motors: {e}")
            return False

    def disable_motors(self) -> bool:
        """Désactive les moteurs."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: disable_motors")
            return True

        try:
            self.robot.disable_motors()
            return True
        except Exception as e:
            logger.error(f"Erreur disable_motors: {e}")
            return False

    def enable_gravity_compensation(self) -> bool:
        """Active la compensation de gravité."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: enable_gravity_compensation")
            return True

        try:
            self.robot.enable_gravity_compensation()
            return True
        except Exception as e:
            logger.error(f"Erreur enable_gravity_compensation: {e}")
            return False

    def disable_gravity_compensation(self) -> bool:
        """Désactive la compensation de gravité."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: disable_gravity_compensation")
            return True

        try:
            self.robot.disable_gravity_compensation()
            return True
        except Exception as e:
            logger.error(f"Erreur disable_gravity_compensation: {e}")
            return False
