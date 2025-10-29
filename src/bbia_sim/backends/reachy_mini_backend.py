#!/usr/bin/env python3
"""
ReachyMiniBackend - Implémentation SDK officiel Reachy-Mini
Backend utilisant le SDK officiel reachy_mini
"""

import logging
import time
from typing import TYPE_CHECKING, Any, Optional, Union

import numpy as np

if TYPE_CHECKING:
    from reachy_mini.utils import HeadPose

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

    def __init__(
        self,
        localhost_only: bool = True,
        spawn_daemon: bool = False,
        use_sim: bool = False,
        timeout: float = 5.0,
        automatic_body_yaw: bool = False,
        log_level: str = "INFO",
        media_backend: str = "default",
    ) -> None:
        """
        Initialise le backend Reachy Mini officiel.

        Args:
            localhost_only: Si True, se connecte uniquement au localhost
            spawn_daemon: Si True, lance le daemon automatiquement
            use_sim: Si True, utilise le mode simulation du SDK
            timeout: Timeout de connexion en secondes
            automatic_body_yaw: Active la rotation automatique du corps
            log_level: Niveau de log ('DEBUG', 'INFO', 'WARNING', 'ERROR')
            media_backend: Backend média à utiliser
        """
        super().__init__()
        self.localhost_only = localhost_only
        self.spawn_daemon = spawn_daemon
        self.use_sim = use_sim
        self.timeout = timeout
        self.automatic_body_yaw = automatic_body_yaw
        self.log_level = log_level
        self.media_backend = media_backend
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

        # Limites officielles Reachy-Mini (en radians) - LIMITES EXACTES
        # Source: reachy_mini_REAL_OFFICIAL.xml - valeurs exactes du modèle physique
        self.joint_limits = {
            # Tête (plateforme Stewart - limites exactes du modèle officiel XML)
            "stewart_1": (-0.8377580409572196, 1.3962634015955222),  # Exact du XML
            "stewart_2": (-1.396263401595614, 1.2217304763958803),  # Exact du XML
            "stewart_3": (-0.8377580409572173, 1.3962634015955244),  # Exact du XML
            "stewart_4": (-1.3962634015953894, 0.8377580409573525),  # Exact du XML
            "stewart_5": (-1.2217304763962082, 1.396263401595286),  # Exact du XML
            "stewart_6": (-1.3962634015954123, 0.8377580409573296),  # Exact du XML
            # Antennes: pas de range dans XML - limites conservatrices sécurité hardware
            # Note: Antennes fragiles, limites réduites même si SDK permet plus
            "left_antenna": (-1.0, 1.0),  # Limite de sécurité (hardware fragile)
            "right_antenna": (-1.0, 1.0),  # Limite de sécurité (hardware fragile)
            # Corps (limite exacte du modèle officiel XML)
            "yaw_body": (
                -2.792526803190975,
                2.792526803190879,
            ),  # Exact du XML - rotation complète
        }

        # Joints interdits (sécurité)
        self.forbidden_joints = {
            "left_antenna",  # Antennes trop fragiles
            "right_antenna",
        }

    def connect(self) -> bool:
        """Connecte au robot Reachy-Mini officiel."""
        if not REACHY_MINI_AVAILABLE:
            logger.info("SDK reachy_mini non disponible - mode simulation activé")
            self.robot = None
            self.is_connected = True  # Mode simulation
            self.start_time = time.time()
            return True

        # Si use_sim=True, utiliser directement le mode simulation
        if self.use_sim:
            logger.info("Mode simulation activé (use_sim=True)")
            self.robot = None
            self.is_connected = True
            self.start_time = time.time()
            return True

        try:
            # Connexion au robot Reachy-Mini avec paramètres officiels
            # Timeout réduit pour éviter d'attendre trop longtemps
            self.robot = ReachyMini(
                localhost_only=self.localhost_only,
                spawn_daemon=self.spawn_daemon,
                use_sim=False,  # Essayer la connexion réelle
                timeout=min(
                    self.timeout, 3.0
                ),  # Max 3 secondes pour éviter timeout long
                automatic_body_yaw=self.automatic_body_yaw,
                log_level=self.log_level,
                media_backend=self.media_backend,
            )
            self.is_connected = True
            self.start_time = time.time()
            logger.info("✅ Connecté au robot Reachy-Mini officiel")
            return True
        except (TimeoutError, ConnectionError, OSError) as e:
            # Pas de robot physique - mode simulation automatique
            logger.info(
                f"⏱️  Pas de robot physique détecté (timeout/connexion) - "
                f"mode simulation activé: {e}"
            )
            self.robot = None
            self.is_connected = True  # Mode simulation
            self.start_time = time.time()
            return True
        except Exception as e:
            # Autres erreurs - activer mode simulation pour éviter crash
            error_msg = str(e)
            if "timeout" in error_msg.lower() or "connection" in error_msg.lower():
                logger.info(
                    f"⏱️  Erreur connexion (timeout probable) - "
                    f"mode simulation activé: {error_msg}"
                )
                self.robot = None
                self.is_connected = True  # Mode simulation
                self.start_time = time.time()
                return True
            else:
                logger.warning(
                    f"⚠️  Erreur connexion Reachy-Mini "
                    f"(mode simulation activé): {error_msg}"
                )
                self.robot = None
                self.is_connected = True  # Mode simulation par sécurité
                self.start_time = time.time()
                return True

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

    def get_joint_pos(self, joint_name: str) -> float:
        """Récupère la position actuelle d'un joint."""
        if not self.is_connected or not self.robot:
            return 0.0

        try:
            # SDK retourne tuple[list[float], list[float]] (head_pos, antenna_pos)
            head_positions, antenna_positions = self.robot.get_current_joint_positions()

            # Mapping des noms vers indices (basé sur le modèle MuJoCo officiel)
            if joint_name == "yaw_body":
                # SDK retourne yaw_body dans une structure séparée
                # NOTE: get_current_joint_positions() retourne
                # (head_positions, antenna_positions)
                # Le yaw_body n'est pas inclus, utiliser une méthode séparée
                try:
                    # Méthode 1: Essayer get_current_body_yaw si disponible dans le SDK
                    if hasattr(self.robot, "get_current_body_yaw"):
                        body_yaw = self.robot.get_current_body_yaw()
                        if body_yaw is not None:
                            return float(body_yaw)
                except (AttributeError, Exception) as e:
                    logger.debug(f"get_current_body_yaw non disponible: {e}")

                # Méthode 2: Essayer de lire via l'état interne du robot si disponible
                try:
                    if hasattr(self.robot, "state") and hasattr(
                        self.robot.state, "body_yaw"
                    ):
                        return float(self.robot.state.body_yaw)
                except (AttributeError, Exception):
                    pass

                # Méthode 3: Fallback sûr - retourner 0.0 (position neutre)
                # En production, cette valeur devrait être lue via une API SDK dédiée
                logger.debug(
                    "yaw_body: utilisation fallback (0.0) - "
                    "vérifier SDK pour méthode de lecture dédiée"
                )
                return 0.0
            elif joint_name in ["left_antenna", "right_antenna"]:
                # Antennes dans antenna_positions
                antenna_idx = 0 if joint_name == "left_antenna" else 1
                return (
                    float(antenna_positions[antenna_idx])
                    if len(antenna_positions) > antenna_idx
                    else 0.0
                )
            elif joint_name.startswith("stewart_"):
                # Stewart joints dans head_positions
                # IMPORTANT: SDK retourne positions des 6 stewart joints directement
                # Structure: head_positions contient 6 éléments (indices 0-5)
                stewart_idx = (
                    int(joint_name.split("_")[1]) - 1
                )  # Convertir 1-6 vers 0-5

                # Vérifier que l'index est valide et que le tableau a >= 6 éléments
                if stewart_idx < 0 or stewart_idx > 5:
                    logger.warning(
                        f"Index stewart invalide: {stewart_idx} pour {joint_name}"
                    )
                    return 0.0

                if len(head_positions) == 6:
                    # Structure standard: indices 0-5 correspondent à stewart_1-6
                    value = float(head_positions[stewart_idx])
                    # Vérification sécurité: NaN ou inf
                    if not (float("-inf") < value < float("inf")):
                        logger.warning(
                            f"Valeur invalide (NaN/inf) pour {joint_name}: {value}"
                        )
                        return 0.0
                    return value
                elif len(head_positions) == 12:
                    # Structure legacy: stewart joints aux indices impairs
                    # (1,3,5,7,9,11) - Existe dans certaines versions anciennes
                    head_idx = stewart_idx * 2 + 1  # 1,3,5,7,9,11 pour stewart_1-6
                    if 0 <= head_idx < len(head_positions):
                        value = float(head_positions[head_idx])
                        # Vérification sécurité: NaN ou inf
                        if not (float("-inf") < value < float("inf")):
                            logger.warning(
                                f"Valeur invalide (NaN/inf) pour {joint_name}: {value}"
                            )
                            return 0.0
                        return value
                    else:
                        logger.warning(
                            f"Index head_positions invalide: {head_idx} "
                            f"pour {joint_name}"
                        )
                        return 0.0
                else:
                    # Structure inattendue - retourner 0.0 en sécurité
                    logger.warning(
                        f"Structure head_positions inattendue "
                        f"(len={len(head_positions)}, attendu 6 ou 12) "
                        f"pour {joint_name}"
                    )
                    return 0.0

            logger.warning(f"Joint {joint_name} non trouvé")
            return 0.0

        except Exception as e:
            logger.error(f"Erreur lecture joint {joint_name}: {e}")
            return 0.0

    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        """Définit la position d'un joint."""
        # Validation sécurité (toujours vérifier les joints interdits)
        if joint_name in self.forbidden_joints:
            logger.warning(f"Joint {joint_name} interdit pour sécurité")
            return False

        # Mode simulation : retourner True SAUF pour les joints interdits
        # (stewart_4/5/6) qui doivent toujours retourner False pour conformité SDK
        if joint_name in ["stewart_4", "stewart_5", "stewart_6"]:
            logger.warning(
                f"⚠️  Tentative de contrôle individuel du joint {joint_name} - "
                f"Ce joint ne peut PAS être contrôlé individuellement.\n"
                f"   → Utilisez goto_target() ou set_target_head_pose() "
                f"avec create_head_pose() pour contrôler la tête via la "
                f"cinématique inverse."
            )
            return False  # Toujours False, même en simulation (conformité SDK)

        if not self.robot or not self.is_connected:
            logger.info(f"Mode simulation: joint {joint_name} = {position}")
            return True

        # Clamping sécurisé multi-niveaux (expert robotique)
        # Niveau 1: Limites physiques du joint (hardware)
        if joint_name in self.joint_limits:
            min_limit, max_limit = self.joint_limits[joint_name]
            # Vérifier si la position demandée est dans les limites hardware
            if position < min_limit or position > max_limit:
                logger.warning(
                    f"Position {position:.4f} rad hors limites hardware "
                    f"[{min_limit:.4f}, {max_limit:.4f}] pour joint "
                    f"{joint_name} - clampage appliqué"
                )
                # Clamp dans les limites hardware
                position = max(min_limit, min(max_limit, position))

            # Niveau 2: Limite de sécurité logicielle (plus restrictive)
            # Appliquer la limite seulement si elle est plus restrictive
            # que les limites hardware
            safe_min = max(-self.safe_amplitude_limit, min_limit)
            safe_max = min(self.safe_amplitude_limit, max_limit)

            # Ne clamp que si la limite de sécurité est réellement plus restrictive
            if safe_min > min_limit or safe_max < max_limit:
                old_position = position
                position = max(safe_min, min(safe_max, position))
                if position != old_position:
                    logger.debug(
                        f"Position {old_position:.4f} rad clampée à {position:.4f} rad "
                        f"par limite de sécurité pour joint {joint_name}"
                    )
        else:
            # Si pas de limite spécifique, utiliser seulement la limite de sécurité
            position = max(
                -self.safe_amplitude_limit, min(self.safe_amplitude_limit, position)
            )

        try:
            if joint_name == "yaw_body":
                # Contrôler la rotation du corps via l'API officielle
                self.robot.set_target_body_yaw(position)
            elif joint_name in ["left_antenna", "right_antenna"]:
                # Contrôler les antennes via l'API officielle
                if self.robot:
                    current_antennas = self.robot.get_present_antenna_joint_positions()
                    antenna_idx = self.joint_mapping[joint_name]
                    if len(current_antennas) > antenna_idx:
                        current_antennas[antenna_idx] = position
                        self.robot.set_target_antenna_joint_positions(current_antennas)
                else:
                    # Mode simulation
                    logger.info(f"Mode simulation: antenne {joint_name} = {position}")
            else:
                # Contrôler la tête via pose (stewart joints) - API officielle
                # NOTE: Les joints Stewart sont contrôlés via la pose de la tête
                # (matrice 4x4). Le SDK ne permet pas de contrôler individuellement
                # stewart_4, 5, 6. Pour stewart_1, 2, 3, on peut utiliser
                # roll, pitch, yaw approximativement.
                if joint_name in ["stewart_4", "stewart_5", "stewart_6"]:
                    logger.warning(
                        f"Les joints {joint_name} ne peuvent pas être contrôlés "
                        f"individuellement via l'API SDK car la plateforme Stewart "
                        f"utilise la cinématique inverse. Utilisez goto_target() ou "
                        f"set_target_head_pose() avec une pose complète."
                    )
                    # Retourner False car contrôle individuel impossible
                    return False
                else:
                    # ATTENTION CRITIQUE (Expert Robotique):
                    # Les joints stewart NE PEUVENT PAS être contrôlés individuellement
                    # car la plateforme Stewart utilise la cinématique inverse (IK).
                    # Chaque joint stewart influence plusieurs degrés de liberté
                    # simultanément (roll, pitch, yaw, position X/Y/Z).
                    #
                    # L'approximation roll/pitch/yaw est INCORRECTE car:
                    # - stewart_1 ≠ roll, stewart_2 ≠ pitch, stewart_3 ≠ yaw
                    # - Les 6 joints agissent ensemble via la cinématique inverse
                    #
                    # MÉTHODES CORRECTES pour contrôler la tête:
                    # 1. goto_target(head=pose_4x4, ...) - méthode recommandée
                    # 2. set_target_head_pose(pose_4x4) - contrôle direct
                    # 3. look_at_world(x, y, z) - regarder vers un point 3D
                    #
                    # Cette méthode retourne False pour forcer l'utilisation
                    # des méthodes correctes.
                    logger.warning(
                        f"⚠️ Contrôle individuel du joint {joint_name} IMPOSSIBLE "
                        f"(cinématique inverse requise). Utilisez goto_target() ou "
                        f"look_at_world() pour un contrôle correct."
                    )
                    return False

            return True
        except Exception as e:
            logger.error(f"Erreur contrôle joint {joint_name}: {e}")
            return False

    def set_emotion(self, emotion: str, intensity: float = 0.5) -> bool:
        """Définit une émotion sur le robot."""
        # Liste des émotions valides
        valid_emotions = {"happy", "sad", "neutral", "excited", "curious", "calm"}

        # Vérifier si l'émotion est valide
        if emotion not in valid_emotions:
            logger.warning(f"Émotion {emotion} non reconnue")
            return False

        # Vérifier si le SDK est disponible
        if not REACHY_MINI_AVAILABLE or create_head_pose is None:
            # Mode simulation sans SDK
            self.current_emotion = emotion
            self.emotion_intensity = intensity
            logger.info(
                f"Émotion simulée (sans SDK): {emotion} (intensité: {intensity})"
            )
            return True

        # Mapping émotions vers poses tête - valeurs de base (intensité 1.0)
        emotion_base_poses = {
            "happy": {"pitch": 0.1, "yaw": 0.0},
            "sad": {"pitch": -0.1, "yaw": 0.0},
            "neutral": {"pitch": 0.0, "yaw": 0.0},
            "excited": {"pitch": 0.2, "yaw": 0.1},
            "curious": {"pitch": 0.05, "yaw": 0.2},
            "calm": {"pitch": -0.05, "yaw": 0.0},
        }

        if not self.is_connected or not self.robot:
            # Mode simulation : toujours OK pour émotions valides
            self.current_emotion = emotion
            self.emotion_intensity = intensity
            logger.info(f"Émotion simulée: {emotion} (intensité: {intensity})")
            return True

        try:
            # Appliquer l'intensité aux angles AVANT de créer la pose
            base_angles = emotion_base_poses[emotion]
            pose = create_head_pose(
                pitch=base_angles["pitch"] * intensity,
                yaw=base_angles["yaw"] * intensity,
                degrees=False,
            )
            self.robot.set_target_head_pose(pose)
            self.current_emotion = emotion
            self.emotion_intensity = intensity
            return True

        except Exception as e:
            logger.error(f"Erreur émotion {emotion}: {e}")
            return False

    def look_at(
        self,
        target_x: float,
        target_y: float,
        target_z: float = 0.0,
        duration: float = 1.0,
        perform_movement: bool = True,
    ) -> bool:
        """Fait regarder le robot vers un point."""
        if not self.is_connected or not self.robot:
            # Mode simulation : toujours OK
            logger.info(
                f"Look_at simulé: ({target_x}, {target_y}, {target_z}, "
                f"duration={duration})"
            )
            return True

        try:
            # Utiliser la méthode officielle look_at_world avec tous les paramètres SDK
            self.robot.look_at_world(
                target_x, target_y, target_z, duration, perform_movement
            )
            return True
        except Exception as e:
            logger.error(f"Erreur look_at: {e}")
            return False

    def run_behavior(
        self, behavior_name: str, duration: float = 5.0, **kwargs: dict[str, Any]  # noqa: ARG002
    ) -> bool:
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

    def get_current_head_pose(self) -> np.ndarray:
        """Récupère la pose actuelle de la tête."""
        if not self.is_connected or not self.robot:
            return np.eye(4, dtype=np.float64)  # Mode simulation

        try:
            result = self.robot.get_current_head_pose()
            return (
                np.array(result, dtype=np.float64)
                if result is not None
                else np.eye(4, dtype=np.float64)
            )
        except Exception as e:
            logger.error(f"Erreur get_current_head_pose: {e}")
            return np.eye(4, dtype=np.float64)

    def get_current_body_yaw(self) -> float:
        """Récupère la rotation actuelle du corps (body yaw).

        PERFORMANCE EXPERT: Cette méthode permet d'obtenir l'état actuel sans
        recharger toutes les positions. Utilisé pour des mouvements synchronisés.
        """
        if not self.is_connected or not self.robot:
            return 0.0  # Mode simulation

        try:
            # Méthode directe du SDK si disponible
            if hasattr(self.robot, "get_current_body_yaw"):
                return float(self.robot.get_current_body_yaw())
            # Fallback: Extraire depuis get_current_joint_positions si nécessaire
            # Note: yaw_body n'est pas dans head_positions ou antenna_positions
            # donc on retourne 0.0 comme valeur par défaut
            logger.debug(
                "get_current_body_yaw non disponible dans SDK, "
                "utilisation valeur par défaut"
            )
            return 0.0
        except Exception as e:
            logger.error(f"Erreur get_current_body_yaw: {e}")
            return 0.0

    def get_present_antenna_joint_positions(self) -> list[float]:
        """Récupère les positions actuelles des antennes."""
        if not self.is_connected or not self.robot:
            return [0.0, 0.0]  # Mode simulation

        try:
            result = self.robot.get_present_antenna_joint_positions()
            return list(result) if result is not None else [0.0, 0.0]
        except Exception as e:
            logger.error(f"Erreur get_present_antenna_joint_positions: {e}")
            return [0.0, 0.0]

    def set_target_body_yaw(self, body_yaw: float) -> None:
        """Définit la rotation cible du corps."""
        if not self.is_connected or not self.robot:
            logger.info(f"Mode simulation: body_yaw = {body_yaw}")
            return

        try:
            self.robot.set_target_body_yaw(body_yaw)
        except Exception as e:
            logger.error(f"Erreur set_target_body_yaw: {e}")

    def set_target_antenna_joint_positions(self, antennas: list[float]) -> None:
        """Définit les positions cibles des antennes."""
        if not self.is_connected or not self.robot:
            logger.info(f"Mode simulation: antennas = {antennas}")
            return

        try:
            self.robot.set_target_antenna_joint_positions(antennas)
        except Exception as e:
            logger.error(f"Erreur set_target_antenna_joint_positions: {e}")

    def look_at_image(
        self, u: int, v: int, duration: float = 1.0, perform_movement: bool = True
    ) -> np.ndarray:
        """Fait regarder le robot vers un point dans l'image."""
        if not self.is_connected or not self.robot:
            logger.info(f"Mode simulation: look_at_image({u}, {v})")
            return np.eye(4, dtype=np.float64)

        try:
            return self.robot.look_at_image(u, v, duration, perform_movement)
        except Exception as e:
            logger.error(f"Erreur look_at_image: {e}")
            return np.eye(4, dtype=np.float64)

    def goto_target(
        self,
        head: Optional["HeadPose"] = None,
        antennas: Optional[Union[np.ndarray, list[float]]] = None,
        duration: float = 0.5,
        method: str = "minjerk",
        body_yaw: float = 0.0,
    ) -> None:
        """Va vers une cible spécifique avec technique d'interpolation."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: goto_target")
            return

        try:
            # Convertir string vers InterpolationTechnique si nécessaire
            # PERFORMANCE EXPERT: Support complet des 4 techniques d'interpolation
            # MIN_JERK (défaut): Mouvements fluides et naturels (physiquement réalistes)
            # LINEAR: Mouvements linéaires (plus rapides, moins fluides)
            # EASE_IN_OUT: Accélération/décélération progressive
            # (expressifs pour émotions)
            # CARTOON: Mouvements exagérés style cartoon
            # (parfait pour animations expressives)
            if isinstance(method, str):
                try:
                    from reachy_mini.utils.interpolation import InterpolationTechnique

                    # Conversion sécurisée avec validation et mapping flexible
                    method_normalized = method.strip().upper().replace("_", "_")

                    # Mapping flexible pour accepter différentes écritures
                    method_mapping = {
                        "MIN_JERK": InterpolationTechnique.MIN_JERK,
                        "MINJERK": InterpolationTechnique.MIN_JERK,
                        "MIN-JERK": InterpolationTechnique.MIN_JERK,
                        "LINEAR": InterpolationTechnique.LINEAR,
                        "EASE_IN_OUT": InterpolationTechnique.EASE_IN_OUT,
                        "EASEINOUT": InterpolationTechnique.EASE_IN_OUT,
                        "EASE-IN-OUT": InterpolationTechnique.EASE_IN_OUT,
                        "CARTOON": InterpolationTechnique.CARTOON,
                    }

                    if method_normalized in method_mapping:
                        method_enum = method_mapping[method_normalized]
                    elif hasattr(InterpolationTechnique, method_normalized):
                        method_enum = getattr(InterpolationTechnique, method_normalized)
                    else:
                        # Essayer de convertir directement
                        method_enum = InterpolationTechnique(method)
                except (ValueError, AttributeError) as conversion_error:
                    logger.warning(
                        f"Technique d'interpolation '{method}' non reconnue "
                        f"({conversion_error}), utilisation de MIN_JERK par défaut "
                        f"(fluide et naturel)"
                    )
                    from reachy_mini.utils.interpolation import InterpolationTechnique

                    method_enum = InterpolationTechnique.MIN_JERK
            else:
                method_enum = method

            # Validation des paramètres avant appel SDK
            # Convertir antennas si c'est un numpy array pour compatibilité
            if antennas is not None and isinstance(antennas, np.ndarray):
                antennas = antennas.tolist()

            # Appel SDK avec validation
            self.robot.goto_target(
                head=head,
                antennas=antennas,
                duration=max(0.0, float(duration)),  # Duration doit être positive
                method=method_enum,
                body_yaw=float(body_yaw) if body_yaw is not None else 0.0,
            )
        except Exception as e:
            logger.error(f"Erreur goto_target: {e}", exc_info=True)

    def enable_motors(self) -> None:
        """Active les moteurs."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: enable_motors")
            return

        try:
            self.robot.enable_motors()
        except Exception as e:
            logger.error(f"Erreur enable_motors: {e}")

    def disable_motors(self) -> None:
        """Désactive les moteurs."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: disable_motors")
            return

        try:
            self.robot.disable_motors()
        except Exception as e:
            logger.error(f"Erreur disable_motors: {e}")

    def enable_gravity_compensation(self) -> None:
        """Active la compensation de gravité."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: enable_gravity_compensation")
            return

        try:
            self.robot.enable_gravity_compensation()
        except Exception as e:
            logger.error(f"Erreur enable_gravity_compensation: {e}")

    def disable_gravity_compensation(self) -> None:
        """Désactive la compensation de gravité."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: disable_gravity_compensation")
            return

        try:
            self.robot.disable_gravity_compensation()
        except Exception as e:
            logger.error(f"Erreur disable_gravity_compensation: {e}")

    # ===== MÉTHODES SDK OFFICIEL SUPPLÉMENTAIRES AVANCÉES =====

    def set_automatic_body_yaw(self, body_yaw: float) -> None:
        """Définit la rotation automatique du corps."""
        if not self.is_connected or not self.robot:
            logger.info(f"Mode simulation: set_automatic_body_yaw = {body_yaw}")
            return

        try:
            self.robot.set_automatic_body_yaw(body_yaw)
        except Exception as e:
            logger.error(f"Erreur set_automatic_body_yaw: {e}")

    def set_target(
        self,
        head: Optional["HeadPose"] = None,
        antennas: Optional[Union[np.ndarray, list[float]]] = None,
        body_yaw: Optional[float] = None,
    ) -> None:
        """Définit une cible complète (tête + antennes + corps)."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: set_target")
            return

        try:
            self.robot.set_target(head=head, antennas=antennas, body_yaw=body_yaw)
        except Exception as e:
            logger.error(f"Erreur set_target: {e}")

    def start_recording(self) -> None:
        """Commence l'enregistrement des mouvements."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: start_recording")
            return

        try:
            self.robot.start_recording()
        except Exception as e:
            logger.error(f"Erreur start_recording: {e}")

    def stop_recording(self) -> Optional[list[dict[str, Any]]]:
        """Arrête l'enregistrement et retourne les données."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: stop_recording")
            return []  # Mode simulation

        try:
            result = self.robot.stop_recording()
            return list(result) if result is not None else []
        except Exception as e:
            logger.error(f"Erreur stop_recording: {e}")
            return None

    def play_move(
        self,
        move: object,
        play_frequency: float = 100.0,
        initial_goto_duration: float = 0.0,
    ) -> None:
        """Joue un mouvement enregistré."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: play_move")
            return

        try:
            self.robot.play_move(move, play_frequency, initial_goto_duration)
        except Exception as e:
            logger.error(f"Erreur play_move: {e}")

    def async_play_move(
        self,
        move: object,
        play_frequency: float = 100.0,
        initial_goto_duration: float = 0.0,
    ) -> None:
        """Joue un mouvement enregistré de manière asynchrone."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: async_play_move")
            return

        try:
            self.robot.async_play_move(move, play_frequency, initial_goto_duration)
        except Exception as e:
            logger.error(f"Erreur async_play_move: {e}")

    # ===== SUPPORT MODULES IO ET MEDIA =====

    @property
    def io(self) -> Optional[object]:
        """Accès au module IO du robot."""
        if not self.is_connected or not self.robot:
            logger.warning("Mode simulation: io non disponible")
            return None
        return getattr(self.robot, "io", None)

    @property
    def media(self) -> Optional[object]:
        """Accès au module Media du robot."""
        if not self.is_connected or not self.robot:
            logger.warning("Mode simulation: media non disponible")
            return None
        return getattr(self.robot, "media", None)

    # ===== MÉTHODES UTILITAIRES POUR MOVE =====

    def create_move_from_positions(
        self, positions: list[dict], duration: float = 1.0
    ) -> Optional[object]:
        """Crée un objet Move à partir de positions."""
        try:
            from reachy_mini.motion.move import Move

            # Créer une classe Move simple pour notre usage
            class SimpleMove(Move):
                def __init__(self, positions: list[dict[str, Any]], duration: float) -> None:
                    self._positions = positions
                    self._duration = duration

                def duration(self) -> float:
                    return float(self._duration)

                def evaluate(self, t: float) -> dict[str, Any]:
                    # Interpolation simple entre les positions
                    if not self._positions or t <= 0:
                        return self._positions[0] if self._positions else {}
                    if t >= 1:
                        return self._positions[-1] if self._positions else {}

                    # Interpolation linéaire
                    idx = int(t * (len(self._positions) - 1))
                    if idx >= len(self._positions) - 1:
                        return dict(self._positions[-1]) if self._positions else {}

                    pos1: dict[str, Any] = self._positions[idx]
                    pos2: dict[str, Any] = self._positions[idx + 1]

                    # Interpolation simple
                    result: dict[str, Any] = {}
                    for key in pos1:
                        if key in pos2:
                            result[key] = pos1[key] + (pos2[key] - pos1[key]) * (
                                t * (len(self._positions) - 1) - idx
                            )
                        else:
                            result[key] = pos1[key]
                    return result

            return SimpleMove(positions, duration)

        except Exception as e:
            logger.error(f"Erreur création Move: {e}")
            return None

    def record_movement(self, duration: float = 5.0) -> Optional[list[dict]]:
        """Enregistre un mouvement pendant une durée donnée."""
        self.start_recording()

        import time

        time.sleep(duration)

        return self.stop_recording()

    # ===== ALIAS POUR CONFORMITÉ PARFAITE SDK OFFICIEL =====

    def get_current_joint_positions(self) -> tuple[list[float], list[float]]:
        """Alias SDK officiel pour get_joint_pos."""
        if not self.is_connected or not self.robot:
            return ([0.0] * 12, [0.0, 0.0])  # Mode simulation

        try:
            result = self.robot.get_current_joint_positions()
            if result is not None and len(result) == 2:
                return (list(result[0]), list(result[1]))
            else:
                return ([0.0] * 12, [0.0, 0.0])
        except Exception as e:
            logger.error(f"Erreur get_current_joint_positions: {e}")
            return ([0.0] * 12, [0.0, 0.0])

    def set_target_head_pose(self, pose: "HeadPose") -> None:
        """Alias SDK officiel pour set_joint_pos avec pose."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: set_target_head_pose")
            return

        try:
            self.robot.set_target_head_pose(pose)
        except Exception as e:
            logger.error(f"Erreur set_target_head_pose: {e}")

    def look_at_world(
        self,
        x: float,
        y: float,
        z: float,
        duration: float = 1.0,
        perform_movement: bool = True,
    ) -> None:
        """Alias SDK officiel pour look_at."""
        if not self.is_connected or not self.robot:
            logger.info(f"Mode simulation: look_at_world({x}, {y}, {z})")
            return

        try:
            self.robot.look_at_world(x, y, z, duration, perform_movement)
        except Exception as e:
            logger.error(f"Erreur look_at_world: {e}")

    def wake_up(self) -> None:
        """Alias SDK officiel pour run_behavior('wake_up')."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: wake_up")
            return

        try:
            self.robot.wake_up()
        except Exception as e:
            logger.error(f"Erreur wake_up: {e}")

    def goto_sleep(self) -> None:
        """Alias SDK officiel pour run_behavior('goto_sleep')."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: goto_sleep")
            return

        try:
            self.robot.goto_sleep()
        except Exception as e:
            logger.error(f"Erreur goto_sleep: {e}")
