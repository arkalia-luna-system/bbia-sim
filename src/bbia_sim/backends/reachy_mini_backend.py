#!/usr/bin/env python3
"""ReachyMiniBackend - Implémentation SDK officiel Reachy-Mini
Backend utilisant le SDK officiel reachy_mini
"""

import logging
import threading
import time
from functools import lru_cache
from typing import TYPE_CHECKING, Any, Optional

import numpy as np
import numpy.typing as npt

from ..utils.types import JointPositions, TelemetryData

try:
    from reachy_mini import ReachyMini  # type: ignore[import-untyped]
    from reachy_mini.utils import create_head_pose  # type: ignore[import-untyped]

    REACHY_MINI_AVAILABLE = True
except ImportError:
    REACHY_MINI_AVAILABLE = False
    ReachyMini = None
    create_head_pose = None

if TYPE_CHECKING:
    from reachy_mini.utils import HeadPose  # type: ignore[import-untyped]

from ..robot_api import RobotAPI

# Constantes Stewart platform (alignées SDK officiel)
STEWART_JOINTS_COUNT = 6  # 6 joints stewart (stewart_1 à stewart_6)
STEWART_MAX_INDEX = 5  # Index max (0-5 pour stewart_1-6)
STEWART_LEGACY_COUNT = 12  # Structure legacy (anciennes versions SDK)
JOINT_POSITIONS_TUPLE_SIZE = 2  # Tuple (head_positions, antenna_positions)

# Constantes poses SDK officiel (conformes reachy_mini.reachy_mini)
INIT_HEAD_POSE = np.eye(4, dtype=np.float64)

SLEEP_HEAD_JOINT_POSITIONS = [
    0,
    -0.9848156658225817,
    1.2624661884298831,
    -0.24390294527381684,
    0.20555342557667577,
    -1.2363885150358267,
    1.0032234352772091,
]

SLEEP_ANTENNAS_JOINT_POSITIONS = np.array([-3.05, 3.05], dtype=np.float64)

SLEEP_HEAD_POSE = np.array(
    [
        [0.911, 0.004, 0.413, -0.021],
        [-0.004, 1.0, -0.001, 0.001],
        [-0.413, -0.001, 0.911, -0.044],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

logger = logging.getLogger(__name__)


# OPTIMISATION PERFORMANCE: Cache LRU pour poses fréquentes
# (évite recréation de poses identiques)
@lru_cache(maxsize=50)
def _create_cached_head_pose(
    pitch: float,
    yaw: float,
    roll: float = 0.0,
    degrees: bool = False,
) -> npt.NDArray[np.float64]:
    """Crée une pose de tête avec cache LRU pour optimiser les poses fréquentes.

    Args:
        pitch: Angle pitch en radians (ou degrés si degrees=True)
        yaw: Angle yaw en radians (ou degrés si degrees=True)
        roll: Angle roll en radians (ou degrés si degrees=True), défaut: 0.0
        degrees: Si True, angles en degrés, sinon en radians

    Returns:
        Matrice de transformation 4x4 (numpy array)

    """
    if not REACHY_MINI_AVAILABLE or create_head_pose is None:
        # Fallback: créer matrice identité si SDK non disponible
        return np.eye(4, dtype=np.float64)

    try:
        # Appeler la fonction originale du SDK (pas récursif)
        pose = create_head_pose(
            pitch=pitch,
            yaw=yaw,
            roll=roll,
            degrees=degrees,
        )
        # Retourner une copie pour éviter modification du cache
        return pose.copy() if hasattr(pose, "copy") else pose
    except (ValueError, RuntimeError, AttributeError) as e:
        logger.debug("Erreur création pose cache: %s, fallback identité", e)
        return np.eye(4, dtype=np.float64)
    except Exception as e:
        logger.debug("Erreur inattendue création pose cache: %s, fallback identité", e)
        return np.eye(4, dtype=np.float64)


# OPTIMISATION RAM: Constantes module-level partagées
# (évite recréation à chaque instance)
JOINT_MAPPING_STATIC = {
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

JOINT_LIMITS_STATIC = {
    # Tête (plateforme Stewart - limites exactes du modèle officiel XML)
    "stewart_1": (-0.8377580409572196, 1.3962634015955222),  # Exact du XML
    "stewart_2": (-1.396263401595614, 1.2217304763958803),  # Exact du XML
    "stewart_3": (-0.8377580409572173, 1.3962634015955244),  # Exact du XML
    "stewart_4": (-1.3962634015953894, 0.8377580409573525),  # Exact du XML
    "stewart_5": (-1.2217304763962082, 1.396263401595286),  # Exact du XML
    "stewart_6": (-1.3962634015954123, 0.8377580409573296),  # Exact du XML
    # Antennes: maintenant avec limites définies dans XML (-0.3 à 0.3 rad)
    # Limites conservatrices pour protection hardware (antennes fragiles)
    "left_antenna": (-0.3, 0.3),  # Limite de sécurité alignée avec XML
    "right_antenna": (-0.3, 0.3),  # Limite de sécurité alignée avec XML
    # Corps (limite exacte du modèle officiel XML)
    "yaw_body": (
        -2.792526803190975,
        2.792526803190879,
    ),  # Exact du XML - rotation complète
}


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
        """Initialise le backend Reachy Mini officiel.

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
        self.robot: ReachyMini | None = None
        self.step_count = 0
        self.start_time: float = 0.0

        # Watchdog monitoring temps réel (SDK officiel utilise threads avec Event)
        self._watchdog_thread: threading.Thread | None = None
        self._should_stop_watchdog = threading.Event()
        self._watchdog_interval = 0.1  # 100ms entre vérifications
        self._last_heartbeat: float = 0.0
        # Verrou pour garantir l'idempotence stricte de _start_watchdog
        self._watchdog_lock = threading.Lock()

        # Cache pour get_available_joints (OPTIMISATION: résultat constant)
        self._cached_available_joints: list[str] | None = None

        # Constantes joints Stewart (évite magic numbers)
        self.STEWART_JOINTS_COUNT = 6  # 6 joints tête plateforme Stewart
        self.STEWART_MAX_INDEX = 5  # Indices 0-5 pour stewart_1-6
        self.HEAD_POSITIONS_LEGACY_COUNT = 12  # Structure legacy (indices impairs)

        # OPTIMISATION RAM: Référencer constantes module-level partagées
        # (évite recréation)
        # Mapping joints officiel Reachy-Mini (noms réels du modèle MuJoCo)
        # 6 joints tête + 2 antennes + 1 corps = 9 joints total
        self.joint_mapping = JOINT_MAPPING_STATIC

        # Limites officielles Reachy-Mini (en radians) - LIMITES EXACTES
        # Source: reachy_mini_REAL_OFFICIAL.xml - valeurs exactes du modèle physique
        self.joint_limits = JOINT_LIMITS_STATIC

        # Joints interdits (sécurité)
        # Note: Antennes maintenant animables avec limites sûres (-0.3 à 0.3 rad)
        # Garder dans forbidden_joints si on veut les bloquer par défaut (optionnel)
        # Initialiser depuis GlobalConfig pour cohérence
        from ..global_config import GlobalConfig

        self.forbidden_joints: set[str] = set(GlobalConfig.FORBIDDEN_JOINTS)
        # Optionnel: ajouter "left_antenna" ou "right_antenna" pour bloquer

    def __enter__(self) -> "ReachyMiniBackend":
        """Context manager entry point (conforme SDK officiel)."""
        self.connect()
        return self

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> None:
        """Context manager exit point (conforme SDK officiel)."""
        self.disconnect()

    def _activate_simulation_mode(self, reason: str = "") -> None:
        """Active le mode simulation avec initialisation des timers.

        Args:
            reason: Raison de l'activation du mode simulation (pour logging)
        """
        self.robot = None
        self.is_connected = True  # Mode simulation
        self.start_time = time.time()
        self._last_heartbeat = time.time()
        if reason:
            logger.info("Mode simulation activé: %s", reason)

    def _try_connect_robot(self) -> bool:
        """Tente de se connecter au robot physique.

        Returns:
            True si connexion réussie, False sinon
        """
        try:
            self.robot = ReachyMini(
                localhost_only=self.localhost_only,
                spawn_daemon=self.spawn_daemon,
                use_sim=False,  # Essayer la connexion réelle
                timeout=min(self.timeout, 3.0),  # Max 3 secondes
                automatic_body_yaw=self.automatic_body_yaw,
                log_level=self.log_level,
                media_backend=self.media_backend,
            )
            self.is_connected = True
            self.start_time = time.time()
            self._last_heartbeat = time.time()
            if not self.use_sim:
                self._start_watchdog()
            logger.info("✅ Connecté au robot Reachy-Mini officiel")
            return True
        except (OSError, TimeoutError, ConnectionError) as e:
            logger.info(
                "⏱️  Pas de robot physique détecté (timeout/connexion) - "
                "mode simulation activé: %s",
                e,
            )
            self._activate_simulation_mode()
            if not self.use_sim:
                self._start_watchdog()
            return False
        except (ValueError, AttributeError, RuntimeError, ImportError) as e:
            error_msg = str(e)
            logger.warning(
                "⚠️  Erreur connexion Reachy-Mini (mode simulation activé): %s",
                error_msg,
            )
            self._activate_simulation_mode()
            self._start_watchdog()
            return False
        except Exception as e:
            error_msg = str(e)
            logger.warning(
                "⚠️  Erreur inattendue connexion Reachy-Mini (mode simulation activé): %s",
                error_msg,
            )
            self._activate_simulation_mode()
            self._start_watchdog()
            return False

    def connect(self) -> bool:
        """Connecte au robot Reachy-Mini officiel."""
        if not REACHY_MINI_AVAILABLE:
            self._activate_simulation_mode("SDK reachy_mini non disponible")
            if not self.use_sim:
                self._start_watchdog()
            return True

        # Si use_sim=True, utiliser directement le mode simulation
        if self.use_sim:
            self._activate_simulation_mode("use_sim=True")
            self._start_watchdog()
            return True

        # Tenter connexion réelle
        if self._try_connect_robot():
            return True

        # Fallback: mode simulation activé
        return True

    def disconnect(self) -> bool:
        """Déconnecte du robot Reachy-Mini."""
        # Best-effort: toujours laisser l'instance dans un état sûr
        try:
            self._stop_watchdog()
        except (AttributeError, RuntimeError) as e:
            logger.debug("Stop watchdog lors déconnexion: %s", e)
        except Exception as e:
            logger.debug("Erreur inattendue stop watchdog: %s", e)
        try:
            if self.robot:
                self.robot = None
        except (AttributeError, RuntimeError) as e:
            logger.debug("Nettoyage robot lors déconnexion: %s", e)
        except Exception as e:
            logger.debug("Erreur inattendue nettoyage robot: %s", e)
        self.is_connected = False
        logger.info("Déconnecté du robot Reachy-Mini")
        return True

    def _start_watchdog(self) -> None:
        """Démarre le thread watchdog pour monitoring temps réel."""
        with self._watchdog_lock:
            # Ne jamais réutiliser un thread externe: watchdog strictement par instance
            if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
                logger.debug("Watchdog déjà actif pour cette instance")
                return
            # IMPORTANT: Ne jamais réutiliser un thread global nommé
            # "ReachyWatchdog" appartenant à une autre instance.
            # Chaque instance possède son propre Event `_should_stop_watchdog`.
            # Réutiliser un thread externe empêcherait l'arrêt propre dans les tests
            # et introduirait des conditions de course.
            # Nettoyer si un ancien thread est terminé
            self._watchdog_thread = None

            self._should_stop_watchdog.clear()
            # Utiliser un nom unique par instance pour éviter les collisions inter-tests
            self._watchdog_thread = threading.Thread(
                target=self._watchdog_monitor,
                daemon=True,
                name=f"ReachyWatchdog-{id(self)}",
            )
            self._watchdog_thread.start()
            logger.debug("Watchdog démarré")

    def _stop_watchdog(self) -> None:
        """Arrête le thread watchdog."""
        if self._watchdog_thread is None:
            return

        self._should_stop_watchdog.set()
        # Capturer la ref locale pour éviter les races si le monitor la remet à None
        thread = self._watchdog_thread
        if thread is not None and thread.is_alive():
            current = threading.current_thread()
            if current is not thread:
                deadline = time.time() + 1.5
                while thread.is_alive() and time.time() < deadline:
                    thread.join(timeout=0.1)
        # Dans tous les cas, ne pas conserver de référence (facilite l'assert test)
        self._watchdog_thread = None
        logger.debug("Watchdog arrêté")

    def _watchdog_monitor(self) -> None:
        """Thread watchdog pour monitoring temps réel.

        Surveille l'état du robot et appelle emergency_stop() en cas d'anomalie.
        Conforme au SDK officiel qui utilise threads avec Event.
        """
        logger.debug("Watchdog monitoring démarré")
        max_heartbeat_timeout = 2.0  # 2 secondes max sans heartbeat

        while not self._should_stop_watchdog.is_set():
            try:
                current_time = time.time()

                # Mettre à jour heartbeat si robot connecté et actif
                if self.is_connected:
                    if self.robot:
                        # Robot physique: vérifier état via SDK si possible
                        try:
                            # Vérification légère: essayer
                            # get_current_joint_positions
                            # Si exception, robot peut être déconnecté
                            self.robot.get_current_joint_positions()
                            self._last_heartbeat = current_time
                        except (AttributeError, RuntimeError, OSError) as e:
                            logger.warning(
                                "Watchdog: robot semble déconnecté: %s. "
                                "Activation emergency_stop...",
                                e,
                            )
                            # Demander l'arrêt du watchdog avant emergency_stop
                            self._should_stop_watchdog.set()
                            self.emergency_stop()
                            break
                    else:
                        # Mode simulation: heartbeat automatique
                        self._last_heartbeat = current_time
                else:
                    # Robot déconnecté: pas besoin de monitoring
                    break

                # Vérifier timeout heartbeat (sécurité)
                if current_time - self._last_heartbeat > max_heartbeat_timeout:
                    logger.warning(
                        "Watchdog: heartbeat timeout (%ss). "
                        "Activation emergency_stop...",
                        max_heartbeat_timeout,
                    )
                    self._should_stop_watchdog.set()
                    self.emergency_stop()
                    break

            except (AttributeError, RuntimeError, ConnectionError) as e:
                logger.exception("Erreur watchdog: %s", e)
                # En cas d'erreur, attendre un peu avant retry
                time.sleep(self._watchdog_interval)
            except Exception as e:
                logger.exception("Erreur inattendue watchdog: %s", e)
                # En cas d'erreur, attendre un peu avant retry
                time.sleep(self._watchdog_interval)

            # Attente entre vérifications
            self._should_stop_watchdog.wait(self._watchdog_interval)

        logger.debug("Watchdog monitoring terminé")
        # Assurer la visibilité immédiate de l'arrêt côté tests
        self._watchdog_thread = None

    def __del__(self) -> None:  # pragma: no cover - best-effort cleanup
        try:
            self._stop_watchdog()
        except Exception as e:
            # Logging silencieux en __del__ pour éviter erreurs lors du
            # garbage collection
            logger.debug("Erreur lors de l'arrêt du watchdog dans __del__: %s", e)

    def get_available_joints(self) -> list[str]:
        """Retourne la liste des joints disponibles.

        Note: OPTIMISATION - Utilise un cache car le résultat ne change pas après l'initialisation.
        """
        if self._cached_available_joints is None:
            self._cached_available_joints = list(self.joint_mapping.keys())
        return self._cached_available_joints

    def _get_yaw_body_position(self) -> float:
        """Récupère la position yaw_body via différentes méthodes SDK."""
        # Méthode 1: Essayer get_current_body_yaw si disponible
        try:
            if self.robot is not None and hasattr(self.robot, "get_current_body_yaw"):
                body_yaw = self.robot.get_current_body_yaw()
                if body_yaw is not None:
                    return float(body_yaw)
        except (AttributeError, Exception) as e:
            logger.debug("get_current_body_yaw non disponible: %s", e)

        # Méthode 2: Essayer de lire via l'état interne du robot
        try:
            if (
                self.robot is not None
                and hasattr(self.robot, "state")
                and self.robot.state is not None
                and hasattr(self.robot.state, "body_yaw")
            ):
                return float(self.robot.state.body_yaw)
        except (AttributeError, Exception):
            pass

        # Méthode 3: Fallback sûr
        logger.debug(
            "yaw_body: utilisation fallback (0.0) - "
            "vérifier SDK pour méthode de lecture dédiée",
        )
        return 0.0

    def _get_antenna_position(
        self, joint_name: str, antenna_positions: list[float]
    ) -> float:
        """Récupère la position d'une antenne."""
        antenna_idx = 0 if joint_name == "left_antenna" else 1
        return (
            float(antenna_positions[antenna_idx])
            if len(antenna_positions) > antenna_idx
            else 0.0
        )

    def _get_stewart_joint_position(
        self, joint_name: str, head_positions: list[float]
    ) -> float:
        """Récupère la position d'un joint Stewart."""
        stewart_idx = int(joint_name.split("_")[1]) - 1  # Convertir 1-6 vers 0-5

        if stewart_idx < 0 or stewart_idx > self.STEWART_MAX_INDEX:
            logger.warning(
                "Index stewart invalide: %s pour %s", stewart_idx, joint_name
            )
            return 0.0

        # Structure standard: 6 éléments (indices 0-5)
        if len(head_positions) == self.STEWART_JOINTS_COUNT:
            value = float(head_positions[stewart_idx])
            if not (float("-inf") < value < float("inf")):
                logger.warning(
                    "Valeur invalide (NaN/inf) pour %s: %s", joint_name, value
                )
                return 0.0
            return value

        # Structure legacy: indices impairs (1,3,5,7,9,11)
        if len(head_positions) == self.HEAD_POSITIONS_LEGACY_COUNT:
            head_idx = stewart_idx * 2 + 1
            if 0 <= head_idx < len(head_positions):
                value = float(head_positions[head_idx])
                if not (float("-inf") < value < float("inf")):
                    logger.warning(
                        "Valeur invalide (NaN/inf) pour %s: %s", joint_name, value
                    )
                    return 0.0
                return value
            logger.warning(
                "Index head_positions invalide: %s pour %s", head_idx, joint_name
            )
            return 0.0

        # Structure inattendue
        logger.warning(
            "Structure head_positions inattendue " "(len=%d, attendu %d ou %d) pour %s",
            len(head_positions),
            self.STEWART_JOINTS_COUNT,
            self.HEAD_POSITIONS_LEGACY_COUNT,
            joint_name,
        )
        return 0.0

    def get_joint_pos(self, joint_name: str) -> float:
        """Récupère la position actuelle d'un joint."""
        if not self.is_connected or not self.robot:
            return 0.0

        try:
            # SDK retourne tuple[list[float], list[float]] (head_pos, antenna_pos)
            head_positions, antenna_positions = self.robot.get_current_joint_positions()

            # Routing selon le type de joint
            if joint_name == "yaw_body":
                return self._get_yaw_body_position()
            if joint_name in ["left_antenna", "right_antenna"]:
                return self._get_antenna_position(joint_name, antenna_positions)
            if joint_name.startswith("stewart_"):
                return self._get_stewart_joint_position(joint_name, head_positions)
            logger.warning("Joint %s non trouvé", joint_name)
            return 0.0

        except (AttributeError, RuntimeError, IndexError, ValueError) as e:
            logger.exception("Erreur lecture joint %s: %s", joint_name, e)
            return 0.0
        except Exception as e:
            logger.exception("Erreur inattendue lecture joint %s: %s", joint_name, e)
            return 0.0

    def _validate_joint_name(self, joint_name: str) -> bool:
        """Valide le nom du joint (sécurité et joints interdits)."""
        if joint_name in self.forbidden_joints:
            logger.warning("Joint %s interdit pour sécurité", joint_name)
            return False
        return True

    def _validate_stewart_joint(self, joint_name: str) -> bool:
        """Valide les joints Stewart (ne peuvent pas être contrôlés individuellement)."""
        if joint_name.startswith("stewart_"):
            logger.warning(
                "⚠️  Tentative de contrôle individuel du joint %s - "
                "Ce joint ne peut PAS être contrôlé individuellement "
                "(plateforme Stewart utilise IK).\n"
                "   → Utilisez goto_target() ou set_target_head_pose() "
                "avec create_head_pose() pour contrôler la tête via la "
                "cinématique inverse, ou utilisez look_at_world() pour "
                "regarder vers un point.",
                joint_name,
            )
            return False
        return True

    def _clamp_joint_position(self, joint_name: str, position: float) -> float:
        """Applique le clamping multi-niveaux (hardware + sécurité)."""
        # Niveau 1: Limites physiques du joint (hardware)
        if joint_name in self.joint_limits:
            min_limit, max_limit = self.joint_limits[joint_name]
            # Vérifier si la position demandée est dans les limites hardware
            if position < min_limit or position > max_limit:
                logger.warning(
                    "Position %.4f rad hors limites hardware "
                    "[%.4f, %.4f] pour joint %s - clampage appliqué",
                    position,
                    min_limit,
                    max_limit,
                    joint_name,
                )
                # Clamp dans les limites hardware
                position = max(min_limit, min(max_limit, position))

            # Niveau 2: Limite de sécurité logicielle (plus restrictive)
            safe_min = max(-self.safe_amplitude_limit, min_limit)
            safe_max = min(self.safe_amplitude_limit, max_limit)

            # Ne clamp que si la limite de sécurité est réellement plus restrictive
            if safe_min > min_limit or safe_max < max_limit:
                old_position = position
                position = max(safe_min, min(safe_max, position))
                if position != old_position:
                    logger.debug(
                        "Position %.4f rad clampée à %.4f rad par limite de sécurité "
                        "pour joint %s",
                        old_position,
                        position,
                        joint_name,
                    )
        else:
            # Si pas de limite spécifique, utiliser seulement la limite de sécurité
            position = max(
                -self.safe_amplitude_limit,
                min(self.safe_amplitude_limit, position),
            )
        return position

    def _set_yaw_body(self, position: float) -> bool:
        """Contrôle la rotation du corps via l'API officielle."""
        if self.robot:
            self.robot.set_target_body_yaw(position)
            return True
        return False

    def _set_antenna_joint(self, joint_name: str, position: float) -> bool:
        """Contrôle les antennes via l'API officielle."""
        if self.robot:
            current_antennas = self.robot.get_present_antenna_joint_positions()
            antenna_idx = self.joint_mapping[joint_name]
            if len(current_antennas) > antenna_idx:
                current_antennas[antenna_idx] = position
                self.robot.set_target_antenna_joint_positions(current_antennas)
                return True
        else:
            # Mode simulation
            logger.info("Mode simulation: antenne %s = %s", joint_name, position)
            return True
        return False

    def _set_stewart_joint(self, joint_name: str) -> bool:
        """Gestion joints Stewart (retourne False - IK requise)."""
        if joint_name in ["stewart_4", "stewart_5", "stewart_6"]:
            logger.warning(
                "Les joints %s ne peuvent pas être contrôlés "
                "individuellement via l'API SDK car la plateforme Stewart "
                "utilise la cinématique inverse. Utilisez goto_target() ou "
                "set_target_head_pose() avec une pose complète.",
                joint_name,
            )
            return False

        # Autres joints stewart (stewart_1, 2, 3)
        logger.warning(
            f"⚠️ Contrôle individuel du joint {joint_name} IMPOSSIBLE "
            f"(cinématique inverse requise). Utilisez goto_target() ou "
            f"look_at_world() pour un contrôle correct.",
        )
        return False

    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        """Définit la position d'un joint."""
        # Validation sécurité
        if not self._validate_joint_name(joint_name):
            return False

        # Validation joints Stewart
        if not self._validate_stewart_joint(joint_name):
            return False

        # Mode simulation si robot non connecté
        if not self.robot or not self.is_connected:
            logger.info("Mode simulation: joint %s = %s", joint_name, position)
            return True

        # Clamping sécurisé multi-niveaux
        position = self._clamp_joint_position(joint_name, position)

        # Application selon le type de joint
        try:
            if joint_name == "yaw_body":
                return self._set_yaw_body(position)
            if joint_name in ["left_antenna", "right_antenna"]:
                return self._set_antenna_joint(joint_name, position)
            if joint_name.startswith("stewart_") or joint_name in [
                "stewart_4",
                "stewart_5",
                "stewart_6",
            ]:
                return self._set_stewart_joint(joint_name)
            # Joint inconnu
            logger.warning(
                f"⚠️ Contrôle individuel du joint {joint_name} IMPOSSIBLE "
                f"(cinématique inverse requise). Utilisez goto_target() ou "
                f"look_at_world() pour un contrôle correct.",
            )
            return False
        except (AttributeError, RuntimeError, ValueError, IndexError) as e:
            logger.exception("Erreur contrôle joint %s: %s", joint_name, e)
            return False
        except Exception as e:
            logger.exception("Erreur inattendue contrôle joint %s: %s", joint_name, e)
            return False

    def set_emotion(self, emotion: str, intensity: float = 0.5) -> bool:
        """Définit une émotion sur le robot."""
        # Liste des émotions valides (SDK officiel)
        valid_emotions = {"happy", "sad", "neutral", "excited", "curious", "calm"}

        # Vérifier si l'émotion est valide
        if emotion not in valid_emotions:
            logger.warning("Émotion %s non reconnue (SDK: %s)", emotion, valid_emotions)
            return False

        # Validation et clamp de l'intensité [0.0, 1.0] - conforme SDK
        if not 0.0 <= intensity <= 1.0:
            logger.warning(
                f"Intensité {intensity} hors limites [0.0, 1.0], clamp appliqué",
            )
            intensity = max(0.0, min(1.0, intensity))

        # Vérifier si le SDK est disponible
        if not REACHY_MINI_AVAILABLE or create_head_pose is None:
            # Mode simulation sans SDK
            self.current_emotion = emotion
            self.emotion_intensity = intensity
            logger.info(
                f"Émotion simulée (sans SDK): {emotion} (intensité: {intensity})",
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
            logger.info("Émotion simulée: %s (intensité: %s)", emotion, intensity)
            return True

        try:
            # Appliquer l'intensité aux angles AVANT de créer la pose
            base_angles = emotion_base_poses[emotion]
            # OPTIMISATION PERFORMANCE: Utiliser cache LRU pour poses fréquentes
            pose = _create_cached_head_pose(
                pitch=base_angles["pitch"] * intensity,
                yaw=base_angles["yaw"] * intensity,
                degrees=False,
            )
            self.robot.set_target_head_pose(pose)
            self.current_emotion = emotion
            self.emotion_intensity = intensity
            return True

        except (KeyError, AttributeError, RuntimeError, ValueError) as e:
            logger.exception("Erreur émotion %s: %s", emotion, e)
            return False
        except Exception as e:
            logger.exception("Erreur inattendue émotion %s: %s", emotion, e)
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
                f"duration={duration})",
            )
            return True

        try:
            # Utiliser la méthode officielle look_at_world
            # avec tous les paramètres SDK
            self.robot.look_at_world(
                target_x,
                target_y,
                target_z,
                duration,
                perform_movement,
            )
            return True
        except Exception as e:
            logger.exception("Erreur look_at: %s", e)
            return False

    def run_behavior(
        self,
        behavior_name: str,
        duration: float = 5.0,
        **kwargs: dict[str, Any],  # noqa: ARG002
    ) -> bool:
        """Exécute un comportement.

        Args:
            behavior_name: Nom du comportement à exécuter ("wake_up", "goto_sleep", "nod")
            duration: Durée du comportement en secondes
            **kwargs: Arguments supplémentaires pour le comportement (non utilisés actuellement)

        Returns:
            True si le comportement a été exécuté avec succès, False sinon
        """
        # Vérifier si le comportement est valide
        valid_behaviors = ["wake_up", "goto_sleep", "nod"]
        if behavior_name not in valid_behaviors:
            logger.warning("Comportement %s non implémenté", behavior_name)
            return False

        if not self.is_connected or not self.robot:
            # Mode simulation : toujours OK pour comportements valides
            logger.info("Comportement simulé: %s (%ss)", behavior_name, duration)
            return True

        try:
            # Mapping comportements vers méthodes officielles
            if behavior_name == "wake_up":
                self.robot.wake_up()
            elif behavior_name == "goto_sleep":
                self.robot.goto_sleep()
            elif behavior_name == "nod":
                # Mouvement de hochement simple
                # PERFORMANCE: Utiliser goto_target avec interpolation
                # au lieu de sleep
                # pour éviter blocage et réduire latence
                # OPTIMISATION PERFORMANCE: Utiliser cache LRU pour poses fréquentes
                pose1 = _create_cached_head_pose(
                    pitch=0.1, yaw=0.0, roll=0.0, degrees=False
                )
                pose2 = _create_cached_head_pose(
                    pitch=-0.1, yaw=0.0, roll=0.0, degrees=False
                )

                # Option optimale: goto_target avec interpolation
                if hasattr(self.robot, "goto_target"):
                    # Utiliser interpolation minjerk pour fluidité
                    self.robot.goto_target(head=pose1, duration=0.5, method="minjerk")
                    self.robot.goto_target(head=pose2, duration=0.5, method="minjerk")
                    self.robot.goto_target(
                        # OPTIMISATION PERFORMANCE: Utiliser cache LRU pour pose identité
                        head=_create_cached_head_pose(
                            pitch=0.0, yaw=0.0, roll=0.0, degrees=False
                        ),
                        duration=0.5,
                        method="minjerk",
                    )
                else:
                    # Fallback: set_target_head_pose avec sleep (compatible)
                    self.robot.set_target_head_pose(pose1)
                    time.sleep(0.5)
                    self.robot.set_target_head_pose(pose2)
                    time.sleep(0.5)
                    # OPTIMISATION PERFORMANCE: Utiliser cache LRU pour pose identité
                    self.robot.set_target_head_pose(
                        _create_cached_head_pose(
                            pitch=0.0, yaw=0.0, roll=0.0, degrees=False
                        )
                    )

            return True
        except Exception as e:
            logger.exception("Erreur comportement %s: %s", behavior_name, e)
            return False

    def step(self) -> bool:
        """Effectue un pas de simulation."""
        self.step_count += 1
        return True

    def get_telemetry(self) -> TelemetryData:
        """Récupère la télémétrie complète du robot (positions, état, capteurs, IMU).

        Inclut les données IMU (accéléromètre, gyroscope, magnétomètre) si disponibles
        via le SDK officiel Reachy Mini.
        """
        try:
            current_time = time.time()
            elapsed_time = current_time - self.start_time if self.start_time > 0 else 0

            # PERFORMANCE: Calculer latence moyenne si disponible
            # (pour monitoring performance temps réel)
            latency_info = {}
            if hasattr(self, "_operation_latencies") and self._operation_latencies:
                import statistics

                avg_latency = statistics.mean(
                    self._operation_latencies[-100:],
                )  # Derniers 100
                latency_info = {
                    "avg_latency_ms": avg_latency * 1000,
                    "operations_sampled": len(self._operation_latencies),
                }

            # Intégration IMU si disponible via SDK
            imu_data = None
            if self.is_connected and self.robot:
                try:
                    # Essayer d'accéder à robot.io.get_imu() si disponible
                    if (
                        hasattr(self.robot, "io")
                        and self.robot.io
                        and hasattr(self.robot.io, "get_imu")
                    ):
                        imu_raw = self.robot.io.get_imu()
                        # Normaliser format IMU (dict avec acceleration,
                        # gyroscope, magnetometer)
                        if isinstance(imu_raw, dict):
                            imu_data = imu_raw
                        elif imu_raw is not None:
                            # Si format différent, essayer de le normaliser
                            logger.debug(
                                f"Format IMU non standard: {type(imu_raw)}",
                            )
                except (AttributeError, RuntimeError) as imu_err:
                    logger.debug("IMU non disponible: %s", imu_err)
                except Exception as imu_err:
                    logger.debug("Erreur inattendue IMU: %s", imu_err)

            telemetry = {
                "step_count": self.step_count,
                "elapsed_time": elapsed_time,
                "steps_per_second": (
                    self.step_count / elapsed_time if elapsed_time > 0 else 0
                ),
                "current_emotion": self.current_emotion,
                "emotion_intensity": self.emotion_intensity,
                "is_connected": self.is_connected,
                **latency_info,  # Ajouter info latence si disponible
            }

            # Ajouter IMU si disponible
            if imu_data:
                telemetry["imu"] = imu_data

            return telemetry  # type: ignore[return-value]
        except (AttributeError, RuntimeError, KeyError, TypeError) as e:
            logger.exception("Erreur télémétrie: %s", e)
            return {}
        except Exception as e:
            logger.exception("Erreur inattendue télémétrie: %s", e)
            return {}

    # ===== MÉTHODES SDK OFFICIEL SUPPLÉMENTAIRES =====

    def get_current_head_pose(self) -> npt.NDArray[np.float64]:
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
            logger.exception("Erreur get_current_head_pose: %s", e)
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
                "utilisation valeur par défaut",
            )
            return 0.0
        except Exception as e:
            logger.exception("Erreur get_current_body_yaw: %s", e)
            return 0.0

    def get_present_antenna_joint_positions(self) -> list[float]:
        """Récupère les positions actuelles des antennes."""
        if not self.is_connected or not self.robot:
            return [0.0, 0.0]  # Mode simulation

        try:
            result = self.robot.get_present_antenna_joint_positions()
            return list(result) if result is not None else [0.0, 0.0]
        except Exception as e:
            logger.exception("Erreur get_present_antenna_joint_positions: %s", e)
            return [0.0, 0.0]

    def set_target_body_yaw(self, body_yaw: float) -> None:
        """Définit la rotation cible du corps."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: body_yaw = %s", body_yaw)
            return

        try:
            self.robot.set_target_body_yaw(body_yaw)
        except Exception as e:
            logger.exception("Erreur set_target_body_yaw: %s", e)

    def set_target_antenna_joint_positions(self, antennas: list[float]) -> None:
        """Définit les positions cibles des antennes."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: antennas = %s", antennas)
            return

        try:
            self.robot.set_target_antenna_joint_positions(antennas)
        except Exception as e:
            logger.exception("Erreur set_target_antenna_joint_positions: %s", e)

    def look_at_image(
        self,
        u: int,
        v: int,
        duration: float = 1.0,
        perform_movement: bool = True,
    ) -> npt.NDArray[np.float64]:
        """Fait regarder le robot vers un point dans l'image."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: look_at_image(%s, %s)", u, v)
            return np.eye(4, dtype=np.float64)

        try:
            result = self.robot.look_at_image(u, v, duration, perform_movement)
            # Cast pour mypy - result peut être Any depuis le SDK
            if isinstance(result, np.ndarray):
                return result
            # Si result n'est pas un ndarray, essayer de le convertir
            if result is not None:
                try:
                    # Conversion sûre vers ndarray
                    result_arr = np.array(result, dtype=np.float64)
                    if result_arr.shape == (4, 4):
                        return result_arr
                except (ValueError, TypeError):
                    pass
            # Valeur par défaut si conversion impossible
            return np.eye(4, dtype=np.float64)
        except Exception as e:
            logger.exception("Erreur look_at_image: %s", e)
            return np.eye(4, dtype=np.float64)

    def goto_target(
        self,
        head: Optional["HeadPose"] = None,
        antennas: npt.NDArray[np.float64] | list[float] | None = None,
        duration: float = 0.5,
        method: str = "minjerk",
        body_yaw: float | None = 0.0,
    ) -> None:
        """Va vers une cible spécifique avec technique d'interpolation
        (conforme SDK officiel).

        Args:
            head: Matrice 4x4 représentant la pose de la tête (ou None)
            antennas: Angles des antennes en radians [right, left] (ou None)
            duration: Durée du mouvement en secondes (doit être > 0)
            method: Technique d'interpolation
                ("minjerk", "linear", "ease_in_out", "cartoon")
            body_yaw: Angle yaw du corps en radians
                (None = garder position actuelle, conforme SDK)

        """
        # Validation stricte: duration doit être positive et non-nulle (conforme SDK)
        duration_float = float(duration)
        if duration_float <= 0.0:
            raise ValueError(
                (
                    "Duration must be positive and non-zero. "
                    "Use set_target() for immediate position setting."
                ),
            )

        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: goto_target")
            return

        try:
            # Convertir string vers InterpolationTechnique si nécessaire
            # PERFORMANCE EXPERT: Support complet des 4 techniques
            # d'interpolation
            # MIN_JERK (défaut): Mouvements fluides et naturels
            # (physiquement réalistes)
            # LINEAR: Mouvements linéaires (plus rapides, moins fluides)
            # EASE_IN_OUT: Accélération/décélération progressive
            # (expressifs pour émotions)
            # CARTOON: Mouvements exagérés style cartoon
            # (parfait pour animations expressives)
            if isinstance(method, str):
                try:
                    from reachy_mini.utils.interpolation import (  # type: ignore[import-untyped]
                        InterpolationTechnique,
                    )

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
                        "Technique d'interpolation '%s' non reconnue "
                        "(%s), utilisation de MIN_JERK "
                        "par défaut (fluide et naturel)",
                        method,
                        conversion_error,
                    )
                    from reachy_mini.utils.interpolation import (  # type: ignore[import-untyped]
                        InterpolationTechnique,
                    )

                    method_enum = InterpolationTechnique.MIN_JERK
            else:
                method_enum = method

            # Validation des paramètres avant appel SDK
            # OPTIMISATION PERFORMANCE: Convertir antennas seulement si nécessaire
            # (le SDK accepte numpy array, mais liste est plus rapide)
            if antennas is not None and isinstance(antennas, np.ndarray):
                # Conversion nécessaire pour compatibilité SDK (liste plus rapide)
                antennas = antennas.tolist()

            # Appel SDK avec validation (conforme SDK officiel)
            # body_yaw=None signifie "garder position actuelle" dans le SDK
            self.robot.goto_target(
                head=head,
                antennas=antennas,
                duration=duration_float,
                method=method_enum,
                body_yaw=body_yaw,  # Passer None directement si None (SDK gère)
            )
        except ValueError:
            # Propager les ValueError de validation
            raise
        except Exception as e:
            logger.exception("Erreur goto_target: %s", e)

    def enable_motors(self) -> None:
        """Active les moteurs."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: enable_motors")
            return

        try:
            self.robot.enable_motors()
        except Exception as e:
            logger.exception("Erreur enable_motors: %s", e)

    def disable_motors(self) -> None:
        """Désactive les moteurs."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: disable_motors")
            return

        try:
            self.robot.disable_motors()
        except Exception as e:
            logger.exception("Erreur disable_motors: %s", e)

    def emergency_stop(self) -> bool:
        """Arrêt d'urgence via SDK officiel.

        Désactive immédiatement tous les moteurs et met le robot en mode sécurité.
        Conforme au SDK Reachy Mini officiel.
        """
        if not self.is_connected:
            logger.warning("Robot non connecté - emergency_stop ignoré")
            return False

        try:
            # Arrêter watchdog immédiatement
            self._stop_watchdog()

            # En simulation sans robot physique, on déconnecte mais retourne False
            if not self.robot:
                self.is_connected = False
                logger.info("Emergency stop (simulation): robot déconnecté")
                return False

            # Robot physique: arrêt réel
            self.robot.disable_motors()
            self.is_connected = False
            logger.critical("🔴 ARRÊT D'URGENCE ACTIVÉ")
            return True
        except Exception as e:
            logger.exception("Erreur emergency_stop: %s", e)
            # S'assurer d'un état sûr même en cas d'erreur
            self.is_connected = False
            return False

    def enable_gravity_compensation(self) -> None:
        """Active la compensation de gravité."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: enable_gravity_compensation")
            return

        try:
            self.robot.enable_gravity_compensation()
        except Exception as e:
            logger.exception("Erreur enable_gravity_compensation: %s", e)

    def disable_gravity_compensation(self) -> None:
        """Désactive la compensation de gravité."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: disable_gravity_compensation")
            return

        try:
            self.robot.disable_gravity_compensation()
        except Exception as e:
            logger.exception("Erreur disable_gravity_compensation: %s", e)

    # ===== MÉTHODES SDK OFFICIEL SUPPLÉMENTAIRES AVANCÉES =====

    def set_automatic_body_yaw(self, body_yaw: float) -> None:
        """Définit la rotation automatique du corps."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: set_automatic_body_yaw = %s", body_yaw)
            return

        try:
            self.robot.set_automatic_body_yaw(body_yaw)
        except Exception as e:
            logger.exception("Erreur set_automatic_body_yaw: %s", e)

    def set_target(
        self,
        head: Optional["HeadPose"] = None,
        antennas: npt.NDArray[np.float64] | list[float] | None = None,
        body_yaw: float | None = None,
    ) -> None:
        """Définit une cible complète (tête + antennes + corps)."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: set_target")
            return

        try:
            self.robot.set_target(head=head, antennas=antennas, body_yaw=body_yaw)
        except Exception as e:
            logger.exception("Erreur set_target: %s", e)

    def start_recording(self) -> None:
        """Commence l'enregistrement des mouvements."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: start_recording")
            return

        try:
            self.robot.start_recording()
        except Exception as e:
            logger.exception("Erreur start_recording: %s", e)

    def stop_recording(self) -> list[JointPositions] | None:
        """Arrête l'enregistrement et retourne les données."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: stop_recording")
            return []  # Mode simulation

        try:
            result = self.robot.stop_recording()
            return list(result) if result is not None else []
        except Exception as e:
            logger.exception("Erreur stop_recording: %s", e)
            return None

    def play_move(
        self,
        move: object,
        play_frequency: float = 100.0,
        initial_goto_duration: float = 0.0,
    ) -> None:
        """Joue un mouvement enregistré (conforme SDK officiel).

        Note: Dans le SDK officiel, play_move est un alias synchrone de async_play_move.
        Utiliser async_play_move pour de meilleures performances en mode async.

        Args:
            move: Objet Move du SDK reachy_mini.motion.move
            play_frequency: Fréquence de lecture (Hz, défaut 100.0)
            initial_goto_duration: Durée goto initial vers position de départ
                (s, défaut 0.0)

        """
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: play_move")
            return

        try:
            # SDK officiel : play_move = async_to_sync(async_play_move)
            # On utilise directement play_move du SDK
            self.robot.play_move(move, play_frequency, initial_goto_duration)
        except Exception as e:
            logger.exception("Erreur play_move: %s", e)

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
            logger.exception("Erreur async_play_move: %s", e)

    # ===== SUPPORT MODULES IO ET MEDIA =====

    @property
    def io(self) -> object:
        """Accès au module IO du robot.

        Retourne toujours un objet IO valide :
        - robot.io du SDK si robot physique disponible
        - SimulationIOModule si en mode simulation

        Returns:
            Module IO (SDK ou simulation)

        """
        if self.robot and hasattr(self.robot, "io") and self.robot.io:
            return self.robot.io

        # Mode simulation : retourner shim de simulation
        from .simulation_shims import SimulationIOModule

        if not hasattr(self, "_sim_io_module"):
            self._sim_io_module = SimulationIOModule()
        return self._sim_io_module

    @property
    def media(self) -> object:
        """Accès au module Media du robot.

        Retourne toujours un objet Media valide :
        - robot.media du SDK si robot physique disponible
        - SimulationMediaModule si en mode simulation

        Returns:
            Module Media (SDK ou simulation)

        """
        if self.robot and hasattr(self.robot, "media") and self.robot.media:
            return self.robot.media

        # Mode simulation : retourner shim de simulation
        from .simulation_shims import SimulationMediaModule

        if not hasattr(self, "_sim_media_module"):
            self._sim_media_module = SimulationMediaModule()
        return self._sim_media_module

    # ===== MÉTHODES UTILITAIRES POUR MOVE =====

    def create_move_from_positions(
        self,
        positions: list[JointPositions],
        duration: float = 1.0,
    ) -> object | None:
        """Crée un objet Move à partir de positions."""
        try:
            from reachy_mini.motion.move import Move  # type: ignore[import-untyped]

            # Créer une classe Move simple pour notre usage
            class SimpleMove(Move):
                def __init__(
                    self,
                    positions: list[JointPositions],
                    duration: float,
                ) -> None:
                    self._positions = positions
                    self._duration = duration

                def duration(self) -> float:
                    return float(self._duration)

                def evaluate(self, t: float) -> JointPositions:
                    # Interpolation simple entre les positions
                    if not self._positions or t <= 0:
                        return self._positions[0] if self._positions else {}
                    if t >= 1:
                        return self._positions[-1] if self._positions else {}

                    # Interpolation linéaire
                    idx = int(t * (len(self._positions) - 1))
                    if idx >= len(self._positions) - 1:
                        return dict(self._positions[-1]) if self._positions else {}

                    pos1: JointPositions = self._positions[idx]
                    pos2: JointPositions = self._positions[idx + 1]

                    # Interpolation simple
                    result: JointPositions = {}
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
            logger.exception("Erreur création Move: %s", e)
            return None

    def record_movement(self, duration: float = 5.0) -> list[JointPositions] | None:
        """Enregistre un mouvement pendant une durée donnée."""
        # PERFORMANCE: Utiliser time.sleep avec vérification pour éviter blocage
        # Critique si appelé dans boucle temps réel
        self.start_recording()

        import time

        # Clamp durée pour éviter sleep trop long (sécurité)
        safe_duration = max(0.0, min(duration, 60.0))  # Max 60s
        time.sleep(safe_duration)

        return self.stop_recording()

    # ===== ALIAS POUR CONFORMITÉ PARFAITE SDK OFFICIEL =====

    def get_current_joint_positions(self) -> tuple[list[float], list[float]]:
        """Alias SDK officiel pour get_joint_pos."""
        if not self.is_connected or not self.robot:
            return (
                [0.0] * STEWART_LEGACY_COUNT,
                [0.0, 0.0],
            )  # Mode simulation

        try:
            result = self.robot.get_current_joint_positions()
            if result is not None and len(result) == JOINT_POSITIONS_TUPLE_SIZE:
                return (list(result[0]), list(result[1]))
            return ([0.0] * 12, [0.0, 0.0])
        except Exception as e:
            logger.exception("Erreur get_current_joint_positions: %s", e)
            return ([0.0] * 12, [0.0, 0.0])

    def set_target_head_pose(self, pose: "HeadPose") -> None:
        """Alias SDK officiel pour set_joint_pos avec pose."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: set_target_head_pose")
            return

        try:
            self.robot.set_target_head_pose(pose)
        except Exception as e:
            logger.exception("Erreur set_target_head_pose: %s", e)

    def look_at_world(
        self,
        x: float,
        y: float,
        z: float,
        duration: float = 1.0,
        perform_movement: bool = True,
    ) -> npt.NDArray[np.float64] | None:
        """Alias SDK officiel pour look_at.

        Returns:
            Matrice 4x4 représentant la pose de la tête si disponible, None sinon.

        """
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: look_at_world(%s, %s, %s)", x, y, z)
            # Calculer une pose 4x4 approximative depuis les coordonnées
            import numpy as np

            # Calculer les angles pitch et yaw depuis les coordonnées x, y, z
            distance = np.sqrt(x**2 + y**2 + z**2)
            if distance > 0:
                pitch = -np.arcsin(z / distance)  # Angle vers le haut/bas
                yaw = np.arctan2(y, x)  # Angle horizontal
            else:
                pitch = 0.0
                yaw = 0.0

            # Créer matrice de rotation 4x4 (pose de la tête)
            pose = np.eye(4, dtype=np.float64)
            # Rotation autour de Y (pitch) puis Z (yaw)
            cos_pitch = np.cos(pitch)
            sin_pitch = np.sin(pitch)
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)

            # Rotation combinée pitch (Y) puis yaw (Z)
            pose[0, 0] = cos_yaw * cos_pitch
            pose[0, 1] = -sin_yaw
            pose[0, 2] = cos_yaw * sin_pitch
            pose[1, 0] = sin_yaw * cos_pitch
            pose[1, 1] = cos_yaw
            pose[1, 2] = sin_yaw * sin_pitch
            pose[2, 0] = -sin_pitch
            pose[2, 2] = cos_pitch

            # Position (optionnel, pour look_at la position reste au centre)
            pose[0, 3] = 0.0
            pose[1, 3] = 0.0
            pose[2, 3] = 0.0
            pose[3, 3] = 1.0

            return pose

        try:
            result = self.robot.look_at_world(x, y, z, duration, perform_movement)
            # Si le SDK retourne une pose, la retourner,
            # sinon calculer une approximation
            if result is not None:
                if isinstance(result, np.ndarray) and result.shape == (4, 4):
                    return result
                try:
                    result_arr = np.array(result, dtype=np.float64)
                    if isinstance(result_arr, np.ndarray) and result_arr.shape == (
                        4,
                        4,
                    ):
                        return result_arr
                except (ValueError, TypeError):
                    pass
            else:
                # Calculer une approximation si le SDK ne retourne pas de pose
                import numpy as np

                distance = np.sqrt(x**2 + y**2 + z**2)
                if distance > 0:
                    pitch = -np.arcsin(z / distance)
                    yaw = np.arctan2(y, x)
                else:
                    pitch = 0.0
                    yaw = 0.0
                pose = np.eye(4, dtype=np.float64)
                cos_pitch = np.cos(pitch)
                sin_pitch = np.sin(pitch)
                cos_yaw = np.cos(yaw)
                sin_yaw = np.sin(yaw)
                pose[0, 0] = cos_yaw * cos_pitch
                pose[0, 1] = -sin_yaw
                pose[0, 2] = cos_yaw * sin_pitch
                pose[1, 0] = sin_yaw * cos_pitch
                pose[1, 1] = cos_yaw
                pose[1, 2] = sin_yaw * sin_pitch
                pose[2, 0] = -sin_pitch
                pose[2, 2] = cos_pitch
                return pose
        except Exception as e:
            logger.exception("Erreur look_at_world: %s", e)
            return None
        # Si aucune condition précédente n'a retourné, retourner None par sécurité
        return None

    def wake_up(self) -> None:
        """Alias SDK officiel pour run_behavior('wake_up')."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: wake_up")
            return

        try:
            self.robot.wake_up()
        except Exception as e:
            logger.exception("Erreur wake_up: %s", e)

    def goto_sleep(self) -> None:
        """Alias SDK officiel pour run_behavior('goto_sleep')."""
        if not self.is_connected or not self.robot:
            logger.info("Mode simulation: goto_sleep")
            return

        try:
            self.robot.goto_sleep()
        except Exception as e:
            logger.exception("Erreur goto_sleep: %s", e)
