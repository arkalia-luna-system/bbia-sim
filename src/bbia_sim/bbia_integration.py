#!/usr/bin/env python3
"""BBIA Integration - Module d'intégration BBIA ↔ Robot Reachy Mini
Connecte tous les modules BBIA au simulateur MuJoCo pour créer une simulation complète.
"""

import asyncio
import logging
from typing import TYPE_CHECKING, Optional

from .bbia_audio import detecter_son, enregistrer_audio, lire_audio

if TYPE_CHECKING:
    pass
from .bbia_behavior import BBIABehaviorManager
from .bbia_emotions import BBIAEmotions
from .bbia_vision import BBIAVision
from .bbia_voice import dire_texte, reconnaitre_parole
from .daemon.simulation_service import SimulationService

# Import conditionnel SDK Reachy-mini pour optimisations
try:
    from reachy_mini.utils import create_head_pose

    REACHY_MINI_UTILS_AVAILABLE = True
except ImportError:
    REACHY_MINI_UTILS_AVAILABLE = False
    create_head_pose = None

# Configuration du logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class BBIAIntegration:
    """Module d'intégration principal qui connecte tous les modules BBIA au robot Reachy Mini.

    Fonctionnalités :
    - Mapping émotions → articulations du robot
    - Réactions visuelles → mouvements
    - Synchronisation audio ↔ mouvements
    - Gestion des comportements complexes
    """

    def __init__(self, simulation_service: Optional[SimulationService] = None) -> None:
        """Initialise l'intégration BBIA avec le service de simulation."""
        # Service de simulation
        self.simulation_service = simulation_service or SimulationService()

        # Modules BBIA
        # OPTIMISATION SDK: Passer robot_api aux modules pour utiliser robot.media
        robot_api = None
        if (
            hasattr(self.simulation_service, "robot_api")
            and self.simulation_service.robot_api
        ):
            robot_api = self.simulation_service.robot_api

        self.emotions = BBIAEmotions()
        self.vision = BBIAVision(
            robot_api=robot_api
        )  # Passer robot_api pour camera SDK

        # OPTIMISATION EXPERT: Passer robot_api au BBIABehaviorManager si disponible
        # pour que les comportements puissent contrôler directement le robot
        self.behavior = BBIABehaviorManager(robot_api=robot_api)

        # Fonctions audio et voice (pas de classes)
        self.audio_functions = {
            "enregistrer": enregistrer_audio,
            "lire": lire_audio,
            "detecter": detecter_son,
        }
        self.voice_functions = {"dire": dire_texte, "reconnaitre": reconnaitre_parole}

        # État de l'intégration
        self.is_active = False
        self.current_emotion = "neutral"
        self.emotion_intensity = 0.5

        # Mapping émotions → articulations
        self.emotion_mappings = self._create_emotion_mappings()

        # Configuration des réactions
        self.reaction_config = {
            "face_detection": {"head_turn_speed": 0.5, "tracking_active": True},
            "object_detection": {"point_speed": 0.3, "focus_duration": 2.0},
            "sound_detection": {"turn_speed": 0.4, "reaction_delay": 0.2},
        }

        logger.info("🎭 BBIA Integration initialisée")
        logger.info(f"   • Émotions disponibles : {len(self.emotions.emotions)}")
        logger.info("   • Articulations contrôlables : 16")
        logger.info(
            f"   • Service simulation : {'✅' if self.simulation_service else '❌'}"
        )

    def _create_emotion_mappings(self) -> dict[str, dict[str, float]]:
        """Crée le mapping des émotions vers les positions d'articulations.

        NOTE: Ce mapping est compatible avec le SDK officiel Reachy Mini.
        Les valeurs utilisent create_head_pose (pitch, yaw) et yaw_body uniquement.
        Les joints interdits (antennes, stewart_4,5,6) ne sont PAS utilisés.
        """
        # Mapping conforme SDK officiel : utilise pitch/yaw pour la tête via create_head_pose
        # et yaw_body pour le corps. Les valeurs respectent les limites réelles du modèle.
        return {
            "neutral": {
                "yaw_body": 0.0,
                "head_pitch": 0.0,  # Pour create_head_pose
                "head_yaw": 0.0,  # Pour create_head_pose
            },
            "happy": {
                "yaw_body": 0.05,  # Légère rotation corps (limitée à 0.3 rad max)
                "head_pitch": 0.1,  # Conforme SDK officiel
                "head_yaw": 0.0,  # Conforme SDK officiel
            },
            "sad": {
                "yaw_body": -0.05,
                "head_pitch": -0.1,  # Conforme SDK officiel
                "head_yaw": 0.0,
            },
            "angry": {
                "yaw_body": 0.0,
                "head_pitch": 0.15,  # Légèrement relevée
                "head_yaw": 0.0,
            },
            "surprised": {
                "yaw_body": 0.08,
                "head_pitch": 0.15,  # Tête relevée
                "head_yaw": 0.0,
            },
            "curious": {
                "yaw_body": 0.12,  # Rotation corps pour regarder
                "head_pitch": 0.05,  # Conforme SDK officiel
                "head_yaw": 0.2,  # Conforme SDK officiel
            },
            "excited": {
                "yaw_body": 0.15,
                "head_pitch": 0.2,  # Conforme SDK officiel
                "head_yaw": 0.1,  # Conforme SDK officiel
            },
            "fearful": {
                "yaw_body": -0.12,
                "head_pitch": -0.15,  # Tête baissée
                "head_yaw": 0.0,
            },
            # Émotions supplémentaires BBIA (mappées vers les 6 émotions SDK officiel)
            "confused": {
                "yaw_body": 0.1,
                "head_pitch": 0.05,
                "head_yaw": 0.15,  # Regard oblique (curieux)
            },
            "determined": {
                "yaw_body": 0.0,
                "head_pitch": 0.0,
                "head_yaw": 0.0,
            },
            "nostalgic": {
                "yaw_body": -0.08,
                "head_pitch": -0.05,  # Calme
                "head_yaw": 0.0,
            },
            "proud": {
                "yaw_body": 0.1,
                "head_pitch": 0.15,  # Tête relevée (excited)
                "head_yaw": 0.05,
            },
            "calm": {
                "yaw_body": 0.0,
                "head_pitch": -0.05,  # Conforme SDK officiel
                "head_yaw": 0.0,
            },
        }

    async def start_integration(self) -> bool:
        """Démarre l'intégration BBIA avec le simulateur."""
        try:
            logger.info("🚀 Démarrage de l'intégration BBIA...")

            # Démarrer la simulation si nécessaire
            if not self.simulation_service.is_running:
                logger.info("📦 Démarrage du service de simulation...")
                await self.simulation_service.start_simulation(headless=False)

                # Attendre que la simulation soit prête
                await asyncio.sleep(2)

            # Vérifier que la simulation est prête
            if not self.simulation_service.is_simulation_ready():
                logger.error("❌ Service de simulation non prêt")
                return False

            # Initialiser l'état BBIA
            self.is_active = True
            self.current_emotion = "neutral"

            # Appliquer l'émotion neutre initiale
            await self.apply_emotion_to_robot("neutral", 0.5)

            logger.info("✅ Intégration BBIA démarrée avec succès")
            logger.info(f"   • Émotion initiale : {self.current_emotion}")
            logger.info(f"   • Intensité : {self.emotion_intensity}")

            return True

        except Exception as e:
            logger.error(f"❌ Erreur démarrage intégration : {e}")
            return False

    async def stop_integration(self) -> None:
        """Arrête l'intégration BBIA."""
        logger.info("🛑 Arrêt de l'intégration BBIA...")

        self.is_active = False

        # Remettre le robot en position neutre
        await self.apply_emotion_to_robot("neutral", 0.5)

        logger.info("✅ Intégration BBIA arrêtée")

    async def apply_emotion_to_robot(
        self, emotion: str, intensity: float = 0.5
    ) -> bool:
        """Applique une émotion au robot via les articulations.

        Args:
            emotion: Nom de l'émotion à appliquer
            intensity: Intensité de l'émotion (0.0 à 1.0)

        Returns:
            True si l'émotion a été appliquée avec succès

        """
        if not self.is_active:
            logger.warning("⚠️ Intégration BBIA non active")
            return False

        if emotion not in self.emotion_mappings:
            logger.error(f"❌ Émotion inconnue : {emotion}")
            return False

        try:
            logger.info(f"🎭 Application émotion '{emotion}' (intensité: {intensity})")

            # Mettre à jour l'état BBIA
            self.emotions.set_emotion(emotion, intensity)
            self.current_emotion = emotion
            self.emotion_intensity = intensity

            # Récupérer le mapping de l'émotion
            emotion_mapping = self.emotion_mappings[emotion]

            # Appliquer les positions conformément au SDK officiel Reachy Mini
            # Si le service utilise RobotAPI avec ReachyMiniBackend, utiliser set_emotion
            # Sinon, appliquer manuellement via create_head_pose

            # Vérifier si on peut utiliser l'API directe
            if hasattr(self.simulation_service, "robot_api") and hasattr(
                self.simulation_service.robot_api, "set_emotion"
            ):
                # Utiliser l'API du backend directement (plus propre)
                # Mapper les émotions BBIA vers les 6 émotions SDK officiel
                sdk_emotion_map = {
                    "happy": "happy",
                    "sad": "sad",
                    "neutral": "neutral",
                    "excited": "excited",
                    "curious": "curious",
                    "calm": "calm",
                    # Mapper les autres vers l'émotion SDK la plus proche
                    "angry": "excited",  # Excitation négative
                    "surprised": "curious",  # Curiosité extrême
                    "fearful": "sad",  # Tristesse avec peur
                    "confused": "curious",  # Curiosité
                    "determined": "neutral",  # Neutre avec détermination
                    "nostalgic": "sad",  # Tristesse douce
                    "proud": "happy",  # Joie avec fierté
                }
                sdk_emotion = sdk_emotion_map.get(emotion, "neutral")
                robot_api = self.simulation_service.robot_api

                # OPTIMISATION EXPERT: Calculer les angles AVANT d'appliquer pour pouvoir utiliser goto_target
                # avec interpolation fluide (plus naturel et expressif que set_emotion directe)
                head_pitch = emotion_mapping.get("head_pitch", 0.0) * intensity
                head_yaw = emotion_mapping.get("head_yaw", 0.0) * intensity
                adjusted_yaw = emotion_mapping.get("yaw_body", 0.0) * intensity

                # OPTIMISATION INTELLIGENCE: Sélection intelligente de la technique d'interpolation selon l'émotion
                # CARTOON pour émotions expressives, EASE_IN_OUT pour douces, MIN_JERK pour naturelles
                emotion_interpolation_map = {
                    "happy": "cartoon",  # Expressif et animé
                    "excited": "cartoon",  # Très expressif
                    "surprised": "cartoon",  # Sautillant
                    "calm": "ease_in_out",  # Doux et fluide
                    "sad": "ease_in_out",  # Lent et mélancolique
                    "nostalgic": "ease_in_out",  # Doux
                    "neutral": "minjerk",  # Naturel
                    "curious": "minjerk",  # Naturel
                    "angry": "cartoon",  # Expressif
                    "fearful": "ease_in_out",  # Doux mais inquiet
                    "determined": "minjerk",  # Naturel mais ferme
                    "proud": "cartoon",  # Expressif
                }
                interpolation_method = emotion_interpolation_map.get(emotion, "minjerk")

                # Méthode 1 (préférée): goto_target avec interpolation optimisée selon l'émotion pour transitions expressives
                # et mouvement combiné tête+corps synchronisé (meilleure expressivité émotionnelle)
                if hasattr(robot_api, "goto_target") and hasattr(
                    robot_api, "get_current_head_pose"
                ):
                    try:
                        from reachy_mini.utils import create_head_pose

                        # Créer pose tête avec angles de l'émotion
                        pose = create_head_pose(
                            pitch=head_pitch, yaw=head_yaw, degrees=False
                        )

                        # Duration adaptative selon l'intensité (plus lente = plus expressive)
                        transition_duration = 0.5 + (
                            intensity * 0.5
                        )  # 0.5 à 1.0 secondes

                        # Mouvement combiné tête+corps avec interpolation optimisée selon l'émotion
                        robot_api.goto_target(
                            head=pose,
                            body_yaw=adjusted_yaw,
                            duration=transition_duration,
                            method=interpolation_method,  # Interpolation intelligente selon émotion
                        )

                        logger.info(
                            f"✅ Émotion '{emotion}' appliquée via goto_target expressif "
                            f"(pitch={head_pitch:.3f}, yaw={head_yaw:.3f}, body={adjusted_yaw:.3f}, "
                            f"duration={transition_duration:.2f}s, method={interpolation_method})"
                        )
                        return True
                    except (ImportError, AttributeError, Exception) as e:
                        logger.debug(
                            f"goto_target non disponible, fallback set_emotion: {e}"
                        )

                # Méthode 2 (fallback): set_emotion directe (moins fluide mais fonctionne)
                success = robot_api.set_emotion(sdk_emotion, intensity)

                # Appliquer yaw_body séparément si goto_target n'a pas été utilisé
                if "yaw_body" in emotion_mapping and success:
                    # Utiliser goto_target si disponible pour mouvement fluide combiné
                    if (
                        hasattr(robot_api, "goto_target")
                        and REACHY_MINI_UTILS_AVAILABLE
                    ):
                        try:
                            # Utiliser la pose actuelle de la tête (préservée par set_emotion)
                            # et ajouter seulement le mouvement corps
                            robot_api.goto_target(
                                body_yaw=adjusted_yaw,
                                duration=0.6,  # Durée fluide
                                method="minjerk",
                            )
                            logger.debug(
                                f"Body yaw ajusté via goto_target (optimisé): {adjusted_yaw:.3f}"
                            )
                        except (ImportError, AttributeError, Exception) as e:
                            logger.debug(f"goto_target non disponible (fallback): {e}")
                            # Fallback: application séparée
                            if hasattr(robot_api, "set_joint_pos"):
                                robot_api.set_joint_pos("yaw_body", adjusted_yaw)
                            else:
                                self.simulation_service.set_joint_position(
                                    "yaw_body", adjusted_yaw
                                )
                    else:
                        # Fallback: application séparée directe
                        if hasattr(robot_api, "set_joint_pos"):
                            robot_api.set_joint_pos("yaw_body", adjusted_yaw)
                        else:
                            self.simulation_service.set_joint_position(
                                "yaw_body", adjusted_yaw
                            )

                if success:
                    logger.info(f"✅ Émotion '{emotion}' appliquée via SDK officiel")
                    return True

            # Fallback: Application manuelle (mode simulation sans SDK)
            # OPTIMISATION EXPERT: Utiliser goto_target pour mouvement combiné fluide
            # au lieu de set_target_head_pose + set_joint_pos séparés
            head_pitch = emotion_mapping.get("head_pitch", 0.0) * intensity
            head_yaw = emotion_mapping.get("head_yaw", 0.0) * intensity
            adjusted_yaw = (
                emotion_mapping["yaw_body"] * intensity
                if "yaw_body" in emotion_mapping
                else 0.0
            )

            # IMPORTANT (Expert Robotique): Les joints stewart NE PEUVENT PAS être contrôlés individuellement
            # car la plateforme Stewart utilise la cinématique inverse (IK).
            # Utiliser goto_target (méthode recommandée SDK) pour mouvement combiné fluide
            if (
                hasattr(self.simulation_service, "robot_api")
                and self.simulation_service.robot_api
            ):
                try:
                    from reachy_mini.utils import create_head_pose

                    pose = create_head_pose(
                        pitch=head_pitch, yaw=head_yaw, degrees=False
                    )

                    # Méthode 1 (préférée): goto_target avec interpolation fluide
                    if hasattr(self.simulation_service.robot_api, "goto_target"):
                        self.simulation_service.robot_api.goto_target(
                            head=pose,
                            body_yaw=adjusted_yaw,
                            duration=0.8,
                            method="minjerk",  # Interpolation recommandée SDK
                        )
                        logger.info(
                            f"✅ Émotion '{emotion}' appliquée via goto_target SDK "
                            f"(pitch={head_pitch:.3f}, yaw={head_yaw:.3f}, body={adjusted_yaw:.3f})"
                        )
                    # Méthode 2 (fallback): set_target_head_pose + set_joint_pos séparés
                    elif hasattr(
                        self.simulation_service.robot_api, "set_target_head_pose"
                    ):
                        self.simulation_service.robot_api.set_target_head_pose(pose)
                        if adjusted_yaw != 0.0:
                            if hasattr(
                                self.simulation_service.robot_api, "set_joint_pos"
                            ):
                                self.simulation_service.robot_api.set_joint_pos(
                                    "yaw_body", adjusted_yaw
                                )
                            else:
                                self.simulation_service.set_joint_position(
                                    "yaw_body", adjusted_yaw
                                )
                        logger.info(
                            f"✅ Émotion '{emotion}' appliquée via pose tête SDK "
                            f"(pitch={head_pitch:.3f}, yaw={head_yaw:.3f})"
                        )
                except (ImportError, AttributeError, Exception) as e:
                    logger.warning(
                        f"Impossible d'utiliser create_head_pose/goto_target: {e}. "
                        f"Utilisation fallback joints directs."
                    )
                    # Fallback final: utiliser seulement yaw_body
                    if adjusted_yaw != 0.0:
                        self.simulation_service.set_joint_position(
                            "yaw_body", adjusted_yaw
                        )

            logger.info(
                f"Émotion '{emotion}' : pitch={head_pitch:.3f}, "
                f"yaw={head_yaw:.3f}, body={emotion_mapping.get('yaw_body', 0.0)*intensity:.3f}"
            )

            logger.info(f"✅ Émotion '{emotion}' appliquée au robot")
            return True

        except Exception as e:
            logger.error(f"❌ Erreur application émotion : {e}")
            return False

    async def react_to_vision_detection(self, detection_data: dict) -> bool:
        """Réagit aux détections visuelles en contrôlant le robot.

        Args:
            detection_data: Données de détection (objets, visages, etc.)

        Returns:
            True si la réaction a été appliquée

        """
        if not self.is_active:
            return False

        try:
            # Réaction à la détection de visage
            if "faces" in detection_data and detection_data["faces"]:
                logger.info("👤 Visage détecté - Réaction de curiosité")
                await self.apply_emotion_to_robot("curious", 0.7)

                # OPTIMISATION EXPERT: Utiliser look_at_world/look_at_image du SDK si disponible
                # pour un suivi de visage fluide et précis (au lieu de set_joint_position direct)
                face_data = detection_data["faces"][0]
                if (
                    hasattr(self.simulation_service, "robot_api")
                    and self.simulation_service.robot_api
                ):
                    robot_api = self.simulation_service.robot_api
                    try:
                        # Méthode 1 (préférée): look_at_world si position 3D disponible
                        pos_3d = face_data.get("position_3d", {})
                        if pos_3d and hasattr(robot_api, "look_at_world"):
                            x = float(pos_3d.get("x", 0.3))
                            y = float(pos_3d.get("y", 0.0))
                            z = float(pos_3d.get("z", 0.2))
                            if (
                                -2.0 <= x <= 2.0
                                and -2.0 <= y <= 2.0
                                and -1.0 <= z <= 1.0
                            ):
                                robot_api.look_at_world(
                                    x, y, z, duration=1.0, perform_movement=True
                                )
                                logger.info(
                                    f"Look_at_world vers visage: ({x:.2f}, {y:.2f}, {z:.2f})"
                                )
                            else:
                                raise ValueError("Position 3D hors limites")
                        # Méthode 2: look_at_image si coordonnées image disponibles
                        elif hasattr(robot_api, "look_at_image"):
                            bbox = face_data.get("bbox", {})
                            if bbox:
                                u = int(bbox.get("center_x", 320))
                                v = int(bbox.get("center_y", 240))
                                if 0 <= u <= 640 and 0 <= v <= 480:
                                    robot_api.look_at_image(u, v, duration=1.0)
                                    logger.info(
                                        f"Look_at_image vers visage: ({u}, {v})"
                                    )
                        # Méthode 3 (fallback): rotation corps
                        elif hasattr(robot_api, "set_joint_pos"):
                            face_position = face_data.get("position", (0, 0))
                            head_turn = (
                                float(face_position[0]) * 0.3
                            )  # Ajuster selon la position
                            robot_api.set_joint_pos("yaw_body", head_turn)
                    except Exception as e:
                        logger.warning(f"Erreur suivi visage SDK (fallback): {e}")
                        # Fallback final: méthode originale
                        face_position = face_data.get("position", (0, 0))
                        head_turn = float(face_position[0]) * 0.3
                        self.simulation_service.set_joint_position(
                            "yaw_body", head_turn
                        )
                else:
                    # Fallback: méthode originale sans SDK
                    face_position = face_data.get("position", (0, 0))
                    head_turn = float(face_position[0]) * 0.3
                    self.simulation_service.set_joint_position("yaw_body", head_turn)

                return True

            # Réaction à la détection d'objet intéressant
            if "objects" in detection_data and detection_data["objects"]:
                logger.info("📦 Objet détecté - Réaction de surprise")
                await self.apply_emotion_to_robot("surprised", 0.6)

                return True

            return False

        except Exception as e:
            logger.error(f"❌ Erreur réaction visuelle : {e}")
            return False

    async def sync_voice_with_movements(
        self, text: str, emotion: str = "neutral"
    ) -> bool:
        """Synchronise la voix avec les mouvements du robot.

        Args:
            text: Texte à prononcer
            emotion: Émotion à exprimer pendant la parole

        Returns:
            True si la synchronisation a été appliquée

        """
        if not self.is_active:
            return False

        try:
            logger.info(f"🗣️ Synchronisation voix + mouvements : '{text[:30]}...'")

            # Appliquer l'émotion pendant la parole
            await self.apply_emotion_to_robot(emotion, 0.6)

            # OPTIMISATION EXPERT: Utiliser goto_target pour mouvements fluides synchronisés
            # au lieu de set_joint_position répétés (meilleure fluidité et performance)
            words = text.split()
            robot_api = None
            if (
                hasattr(self.simulation_service, "robot_api")
                and self.simulation_service.robot_api
            ):
                robot_api = self.simulation_service.robot_api

            for i, _word in enumerate(words):
                # Petit mouvement de tête pour chaque mot important
                if i % 3 == 0:  # Tous les 3 mots
                    head_movement = 0.1 if i % 6 == 0 else -0.1

                    # Utiliser goto_target si disponible (meilleure fluidité)
                    if robot_api and hasattr(robot_api, "goto_target"):
                        try:
                            robot_api.goto_target(
                                body_yaw=head_movement,
                                duration=0.15,  # Durée courte pour mouvement subtil
                                method="minjerk",
                            )
                        except Exception as e:
                            logger.debug(f"Erreur goto_target voix (fallback): {e}")
                            # Fallback vers set_joint_pos
                            if hasattr(robot_api, "set_joint_pos"):
                                robot_api.set_joint_pos("yaw_body", head_movement)
                            else:
                                self.simulation_service.set_joint_position(
                                    "yaw_body", head_movement
                                )
                    else:
                        # Fallback: méthode originale
                        self.simulation_service.set_joint_position(
                            "yaw_body", head_movement
                        )

                # Petite pause entre les mots
                await asyncio.sleep(0.1)

            # Retour à l'émotion normale après la parole
            await self.apply_emotion_to_robot(
                self.current_emotion, self.emotion_intensity
            )

            return True

        except Exception as e:
            logger.error(f"❌ Erreur synchronisation voix : {e}")
            return False

    async def execute_behavior_sequence(self, behavior_name: str) -> bool:
        """Exécute une séquence de comportement complète.

        Args:
            behavior_name: Nom du comportement à exécuter

        Returns:
            True si la séquence a été exécutée

        """
        if not self.is_active:
            return False

        try:
            logger.info(f"🎬 Exécution séquence comportement : {behavior_name}")

            # Exécuter le comportement BBIA
            self.behavior.add_to_queue(behavior_name)

            # Appliquer les émotions correspondantes au robot
            if behavior_name == "greeting":
                await self.apply_emotion_to_robot("happy", 0.8)
                await asyncio.sleep(1.0)
                await self.apply_emotion_to_robot("neutral", 0.5)

            elif behavior_name == "hide":
                await self.apply_emotion_to_robot("fearful", 0.7)
                await asyncio.sleep(2.0)
                await self.apply_emotion_to_robot("neutral", 0.5)

            elif behavior_name == "antenna_animation":
                await self.apply_emotion_to_robot("excited", 0.9)
                await asyncio.sleep(1.5)
                await self.apply_emotion_to_robot("neutral", 0.5)

            logger.info(f"✅ Séquence '{behavior_name}' exécutée")
            return True

        except Exception as e:
            logger.error(f"❌ Erreur exécution comportement : {e}")
            return False

    def get_integration_status(self) -> dict:
        """Retourne le statut de l'intégration BBIA."""
        return {
            "is_active": self.is_active,
            "current_emotion": self.current_emotion,
            "emotion_intensity": self.emotion_intensity,
            "simulation_ready": self.simulation_service.is_simulation_ready(),
            "available_emotions": list(self.emotion_mappings.keys()),
            "reaction_config": self.reaction_config,
        }


# Fonction utilitaire pour créer une instance d'intégration
async def create_bbia_integration(model_path: Optional[str] = None) -> BBIAIntegration:
    """Crée et initialise une instance d'intégration BBIA.

    Args:
        model_path: Chemin vers le modèle MJCF (optionnel)

    Returns:
        Instance d'intégration BBIA prête à utiliser

    """
    # Créer le service de simulation
    simulation_service = SimulationService(model_path)

    # Créer l'intégration BBIA
    integration = BBIAIntegration(simulation_service)

    # Démarrer l'intégration
    success = await integration.start_integration()

    if not success:
        raise RuntimeError("Impossible de démarrer l'intégration BBIA")

    return integration
