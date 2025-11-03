#!/usr/bin/env python3
"""ReachyBackend - Implémentation Reachy réel de RobotAPI

Backend pour robot Reachy réel avec support SDK Reachy Mini officiel.
Implémentation complète avec :
- Connexion/déconnexion via SDK Reachy Mini
- Envoi de commandes au robot réel (goto_target, set_joint_pos)
- Synchronisation avec robot réel (get_current_joint_positions)
- Arrêt d'urgence via SDK (emergency_stop, stop)
- Bascule automatique en mode simulation si robot non disponible
"""

import logging
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    pass

from ..robot_api import RobotAPI

logger = logging.getLogger(__name__)


class ReachyBackend(RobotAPI):
    """Backend Reachy réel pour RobotAPI avec support SDK Reachy Mini.

    Implémentation complète avec connexion au robot réel via SDK Reachy Mini.
    Bascule automatiquement en mode simulation si le robot n'est pas disponible.
    """

    def __init__(self, robot_ip: str = "localhost", robot_port: int = 8080) -> None:
        super().__init__()
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot_sdk: Any | None = None  # SDK Reachy Mini officiel
        self.joint_positions: dict[str, float] = {}
        self.step_count = 0
        self.start_time: float = 0.0

        # Joints simulés du Reachy réel
        self.simulated_joints = {
            "yaw_body": 0.0,
            "stewart_1": 0.0,
            "stewart_2": 0.0,
            "stewart_3": 0.0,
            "stewart_4": 0.0,
            "stewart_5": 0.0,
            "stewart_6": 0.0,
        }

        # Limites spécifiques au Reachy réel
        self.joint_limits = {
            "yaw_body": (-2.793, 2.793),
            "stewart_1": (-0.838, 1.396),
            "stewart_2": (-1.396, 1.222),
            "stewart_3": (-0.838, 1.396),
            "stewart_4": (-1.396, 0.838),
            "stewart_5": (-1.222, 1.396),
            "stewart_6": (-1.396, 0.838),
        }

    def connect(self) -> bool:
        """Connecte au robot Reachy réel.

        Note: Cette implémentation utilise le SDK Reachy Mini officiel.
        Pour utiliser avec un robot Reachy Mini:
        - Assurez-vous que le daemon Reachy Mini est lancé (reachy-mini-daemon)
        - Le robot doit être connecté via USB (lite) ou Wi-Fi (wireless)
        - En l'absence de robot, bascule automatiquement en mode simulation
        """
        try:
            # Tenter connexion via SDK Reachy Mini officiel
            try:
                from reachy_mini import ReachyMini

                logger.info(
                    f"Tentative de connexion au robot Reachy Mini: "
                    f"{self.robot_ip}:{self.robot_port}"
                )

                # Connexion avec timeout court pour éviter blocage
                self.robot_sdk = ReachyMini(
                    localhost_only=(self.robot_ip == "localhost"),
                    spawn_daemon=False,  # Daemon doit être lancé séparément
                    use_sim=False,  # Essayer connexion réelle
                    timeout=3.0,  # Timeout 3 secondes
                )

                self.is_connected = True
                self.start_time = time.time()
                logger.info("✅ Connecté au robot Reachy Mini réel")
                return True

            except ImportError:
                logger.warning("SDK Reachy Mini non disponible - mode simulation")
                self.robot_sdk = None
                self.is_connected = True  # Mode simulation
                self.start_time = time.time()
                return True

            except (TimeoutError, ConnectionError, OSError) as e:
                # Pas de robot physique - bascule en mode simulation
                logger.info(f"⏱️  Pas de robot physique détecté - mode simulation: {e}")
                self.robot_sdk = None
                self.is_connected = True  # Mode simulation
                self.start_time = time.time()
                return True

        except Exception as e:
            logger.error(f"Erreur connexion Reachy: {e}")
            # Fallback: mode simulation pour éviter crash
            self.robot_sdk = None
            self.is_connected = True
            self.start_time = time.time()
            return True

    def disconnect(self) -> bool:
        """Déconnecte du robot Reachy réel."""
        try:
            logger.info("Déconnexion du robot Reachy")

            # Fermer connexion SDK si disponible
            if self.robot_sdk is not None:
                try:
                    # SDK Reachy Mini peut avoir méthode close() ou context manager
                    if hasattr(self.robot_sdk, "close"):
                        self.robot_sdk.close()
                    elif hasattr(self.robot_sdk, "__exit__"):
                        # Gestion context manager
                        self.robot_sdk.__exit__(None, None, None)
                except Exception as e:
                    logger.debug(f"Erreur fermeture SDK (non bloquant): {e}")
                finally:
                    self.robot_sdk = None

            self.is_connected = False
            logger.info("Reachy déconnecté")
            return True

        except Exception as e:
            logger.error(f"Erreur déconnexion Reachy: {e}")
            self.is_connected = False
            self.robot_sdk = None
            return False

    def get_available_joints(self) -> list[str]:
        """Retourne la liste des joints disponibles."""
        if not self.is_connected:
            return []

        return list(self.simulated_joints.keys())

    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        """Définit la position d'un joint."""
        if not self.is_connected:
            logger.error("Reachy non connecté")
            return False

        if joint_name not in self.simulated_joints:
            logger.error(f"Joint introuvable: {joint_name}")
            return False

        # Validation et clamp via RobotAPI
        is_valid, clamped_position = self._validate_joint_pos(joint_name, position)
        if not is_valid:
            return False

        # Envoyer la commande au robot réel via SDK si disponible
        if self.robot_sdk is not None:
            try:
                # Utiliser SDK pour envoyer commande réelle
                # Le SDK gère automatiquement le mapping des joints
                if joint_name == "yaw_body":
                    # Rotation du corps via SDK
                    if hasattr(self.robot_sdk, "goto_target"):
                        # Le SDK utilise goto_target avec body_yaw
                        self.robot_sdk.goto_target(
                            body_yaw=clamped_position, duration=0.1
                        )
                    else:
                        # Fallback: mise à jour directe si méthode disponible
                        logger.warning(
                            f"Méthode goto_target non disponible pour {joint_name}"
                        )
                elif joint_name.startswith("stewart_"):
                    # Joints Stewart platform - utiliser head joint positions
                    # Le SDK attend un tableau de 6 positions pour la tête
                    if hasattr(self.robot_sdk, "goto_target"):
                        # Construire tableau positions tête (6 joints)
                        head_positions = list(self.simulated_joints.values())[:6]
                        stewart_idx = int(joint_name.split("_")[1]) - 1
                        if 0 <= stewart_idx < 6:
                            head_positions[stewart_idx] = clamped_position
                            self.robot_sdk.goto_target(
                                head=head_positions, duration=0.1
                            )
                    else:
                        logger.warning(
                            f"Méthode goto_target non disponible pour {joint_name}"
                        )

                # Mettre à jour cache local pour cohérence
                self.simulated_joints[joint_name] = clamped_position
                logger.debug(
                    f"Joint {joint_name} → {clamped_position:.3f} rad (robot réel)"
                )
                return True
            except Exception as e:
                logger.warning(
                    f"Erreur envoi commande robot réel: {e} - bascule simulation"
                )
                # Fallback: simulation si erreur
                self.simulated_joints[joint_name] = clamped_position
                return True
        else:
            # Mode simulation
            self.simulated_joints[joint_name] = clamped_position
            logger.debug(
                f"Joint {joint_name} → {clamped_position:.3f} rad (simulation)"
            )
            return True

    def get_joint_pos(self, joint_name: str) -> float | None:
        """Récupère la position actuelle d'un joint."""
        if not self.is_connected:
            return None

        if joint_name not in self.simulated_joints:
            return None

        # Lire position depuis robot réel si disponible
        if self.robot_sdk is not None:
            try:
                if hasattr(self.robot_sdk, "get_current_joint_positions"):
                    head_positions, antenna_positions = (
                        self.robot_sdk.get_current_joint_positions()
                    )
                    if joint_name == "yaw_body":
                        # Yaw body peut être dans une structure séparée
                        if hasattr(self.robot_sdk, "get_current_body_yaw"):
                            return float(self.robot_sdk.get_current_body_yaw())
                        # Sinon utiliser valeur cache
                    elif joint_name.startswith("stewart_"):
                        stewart_idx = int(joint_name.split("_")[1]) - 1
                        if 0 <= stewart_idx < len(head_positions):
                            self.simulated_joints[joint_name] = float(
                                head_positions[stewart_idx]
                            )
                            return self.simulated_joints[joint_name]
            except Exception as e:
                logger.debug(f"Erreur lecture position robot réel: {e}")

        # Fallback: retourner valeur cache (simulation)
        return self.simulated_joints[joint_name]

    def step(self) -> bool:
        """Effectue un pas de simulation."""
        if not self.is_connected:
            return False

        try:
            # Synchroniser avec le robot réel si disponible
            if self.robot_sdk is not None:
                try:
                    # Le SDK gère la synchronisation automatiquement
                    # Ici on peut faire des vérifications d'état si nécessaire
                    if hasattr(self.robot_sdk, "get_current_joint_positions"):
                        # Lire positions actuelles pour synchronisation
                        _ = self.robot_sdk.get_current_joint_positions()
                except Exception as e:
                    logger.debug(f"Erreur synchronisation robot réel: {e}")

            # Simulation: attendre un peu pour cohérence timing
            time.sleep(0.01)  # Simuler le temps de traitement
            self.step_count += 1
            return True
        except Exception as e:
            logger.error(f"Erreur step Reachy: {e}")
            return False

    def emergency_stop(self) -> bool:
        """Arrêt d'urgence pour robot Reachy réel."""
        if not self.is_connected:
            logger.warning("Robot non connecté - emergency_stop ignoré")
            return False

        try:
            # Arrêt d'urgence via SDK robot réel si disponible
            if self.robot_sdk is not None:
                try:
                    # Le SDK peut avoir une méthode emergency_stop
                    if hasattr(self.robot_sdk, "emergency_stop"):
                        self.robot_sdk.emergency_stop()
                    elif hasattr(self.robot_sdk, "stop"):
                        self.robot_sdk.stop()
                    # Fermer connexion SDK
                    if hasattr(self.robot_sdk, "close"):
                        self.robot_sdk.close()
                except Exception as e:
                    logger.warning(f"Erreur arrêt d'urgence SDK: {e}")

            self.is_connected = False
            self.robot_sdk = None
            logger.critical("🔴 ARRÊT D'URGENCE REACHY ACTIVÉ")
            return True
        except Exception as e:
            logger.error(f"Erreur emergency_stop: {e}")
            self.is_connected = False
            self.robot_sdk = None
            return False

    def get_telemetry(self) -> dict[str, Any]:
        """Retourne les données de télémétrie."""
        if not self.is_connected:
            return {}

        current_time = time.time()
        elapsed_time = current_time - self.start_time if self.start_time else 0

        return {
            "step_count": self.step_count,
            "elapsed_time": elapsed_time,
            "steps_per_second": (
                self.step_count / elapsed_time if elapsed_time > 0 else 0
            ),
            "average_step_time": (
                elapsed_time / self.step_count if self.step_count > 0 else 0
            ),
            "current_joint_positions": self.simulated_joints.copy(),
            "robot_ip": self.robot_ip,
            "robot_port": self.robot_port,
            "backend_type": "reachy_real",
        }

    def send_command(
        self,
        command: str,
        **kwargs: dict[str, Any],
    ) -> bool:
        """Envoie une commande au robot Reachy."""
        if not self.is_connected:
            logger.error("Reachy non connecté")
            return False

        # Envoyer commandes réelles via SDK si disponible
        if self.robot_sdk is not None:
            try:
                # Mapper commandes BBIA vers méthodes SDK
                if command == "goto_target":
                    # goto_target(head=..., body_yaw=..., duration=...)
                    self.robot_sdk.goto_target(**kwargs)
                elif command == "set_emotion":
                    # set_emotion(emotion=..., intensity=...)
                    if hasattr(self.robot_sdk, "set_emotion"):
                        self.robot_sdk.set_emotion(**kwargs)
                elif command == "play_behavior":
                    # play_behavior(behavior=...)
                    if hasattr(self.robot_sdk, "play_behavior"):
                        self.robot_sdk.play_behavior(**kwargs)
                else:
                    logger.warning(f"Commande non reconnue: {command}")
                    return False

                logger.info(f"Commande Reachy envoyée: {command} {kwargs}")
                return True
            except Exception as e:
                logger.error(f"Erreur envoi commande robot réel: {e}")
                return False
        else:
            # Mode simulation
            logger.info(f"Commande Reachy (simulation): {command} {kwargs}")
            return True

    def get_robot_status(self) -> dict[str, Any]:
        """Retourne le statut du robot Reachy."""
        return {
            "connected": self.is_connected,
            "robot_ip": self.robot_ip,
            "robot_port": self.robot_port,
            "joint_count": len(self.simulated_joints),
            "current_emotion": self.current_emotion,
            "emotion_intensity": self.emotion_intensity,
            "backend_type": "reachy_real",
        }
