#!/usr/bin/env python3
"""
BBIA Integration - Module d'intégration BBIA ↔ Robot Reachy Mini
Connecte tous les modules BBIA au simulateur MuJoCo pour créer une simulation complète
"""

import asyncio
import logging
from typing import Optional

from .bbia_audio import detecter_son, enregistrer_audio, lire_audio
from .bbia_behavior import BBIABehaviorManager
from .bbia_emotions import BBIAEmotions
from .bbia_vision import BBIAVision
from .bbia_voice import dire_texte, reconnaitre_parole
from .daemon.simulation_service import SimulationService

# Configuration du logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class BBIAIntegration:
    """
    Module d'intégration principal qui connecte tous les modules BBIA au robot Reachy Mini.

    Fonctionnalités :
    - Mapping émotions → articulations du robot
    - Réactions visuelles → mouvements
    - Synchronisation audio ↔ mouvements
    - Gestion des comportements complexes
    """

    def __init__(self, simulation_service: Optional[SimulationService] = None):
        """Initialise l'intégration BBIA avec le service de simulation."""

        # Service de simulation
        self.simulation_service = simulation_service or SimulationService()

        # Modules BBIA
        self.emotions = BBIAEmotions()
        self.vision = BBIAVision()
        self.behavior = BBIABehaviorManager()

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
        """Crée le mapping des émotions vers les positions d'articulations."""

        return {
            "neutral": {
                "yaw_body": 0.0,
                "right_antenna": 0.0,
                "left_antenna": 0.0,
                "stewart_1": 0.0,
                "stewart_2": 0.0,
                "stewart_3": 0.0,
                "stewart_4": 0.0,
                "stewart_5": 0.0,
                "stewart_6": 0.0,
            },
            "happy": {
                "yaw_body": 0.1,
                "right_antenna": 0.3,
                "left_antenna": 0.3,
                "stewart_1": 0.2,
                "stewart_2": 0.1,
                "stewart_3": 0.1,
                "stewart_4": 0.1,
                "stewart_5": 0.2,
                "stewart_6": 0.1,
            },
            "sad": {
                "yaw_body": -0.1,
                "right_antenna": -0.2,
                "left_antenna": -0.2,
                "stewart_1": -0.1,
                "stewart_2": -0.2,
                "stewart_3": -0.2,
                "stewart_4": -0.2,
                "stewart_5": -0.1,
                "stewart_6": -0.2,
            },
            "angry": {
                "yaw_body": 0.0,
                "right_antenna": 0.5,
                "left_antenna": 0.5,
                "stewart_1": 0.3,
                "stewart_2": 0.3,
                "stewart_3": 0.3,
                "stewart_4": 0.3,
                "stewart_5": 0.3,
                "stewart_6": 0.3,
            },
            "surprised": {
                "yaw_body": 0.2,
                "right_antenna": 0.4,
                "left_antenna": 0.4,
                "stewart_1": 0.4,
                "stewart_2": 0.4,
                "stewart_3": 0.4,
                "stewart_4": 0.4,
                "stewart_5": 0.4,
                "stewart_6": 0.4,
            },
            "curious": {
                "yaw_body": 0.15,
                "right_antenna": 0.2,
                "left_antenna": 0.2,
                "stewart_1": 0.25,
                "stewart_2": 0.15,
                "stewart_3": 0.15,
                "stewart_4": 0.15,
                "stewart_5": 0.25,
                "stewart_6": 0.15,
            },
            "excited": {
                "yaw_body": 0.3,
                "right_antenna": 0.6,
                "left_antenna": 0.6,
                "stewart_1": 0.5,
                "stewart_2": 0.4,
                "stewart_3": 0.4,
                "stewart_4": 0.4,
                "stewart_5": 0.5,
                "stewart_6": 0.4,
            },
            "fearful": {
                "yaw_body": -0.2,
                "right_antenna": -0.3,
                "left_antenna": -0.3,
                "stewart_1": -0.2,
                "stewart_2": -0.3,
                "stewart_3": -0.3,
                "stewart_4": -0.3,
                "stewart_5": -0.2,
                "stewart_6": -0.3,
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

    async def stop_integration(self):
        """Arrête l'intégration BBIA."""

        logger.info("🛑 Arrêt de l'intégration BBIA...")

        self.is_active = False

        # Remettre le robot en position neutre
        await self.apply_emotion_to_robot("neutral", 0.5)

        logger.info("✅ Intégration BBIA arrêtée")

    async def apply_emotion_to_robot(
        self, emotion: str, intensity: float = 0.5
    ) -> bool:
        """
        Applique une émotion au robot via les articulations.

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

            # Appliquer les positions d'articulations avec l'intensité
            for joint_name, base_position in emotion_mapping.items():
                # Ajuster la position selon l'intensité
                adjusted_position = base_position * intensity

                # Appliquer la position au robot
                self.simulation_service.set_joint_position(
                    joint_name, adjusted_position
                )

            logger.info(f"✅ Émotion '{emotion}' appliquée au robot")
            return True

        except Exception as e:
            logger.error(f"❌ Erreur application émotion : {e}")
            return False

    async def react_to_vision_detection(self, detection_data: dict) -> bool:
        """
        Réagit aux détections visuelles en contrôlant le robot.

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

                # Tourner la tête vers le visage (simulation)
                face_position = detection_data["faces"][0].get("position", (0, 0))
                head_turn = face_position[0] * 0.3  # Ajuster selon la position
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
        """
        Synchronise la voix avec les mouvements du robot.

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

            # Simuler des mouvements subtils pendant la parole
            words = text.split()
            for i, _word in enumerate(words):
                # Petit mouvement de tête pour chaque mot important
                if i % 3 == 0:  # Tous les 3 mots
                    head_movement = 0.1 if i % 6 == 0 else -0.1
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
        """
        Exécute une séquence de comportement complète.

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
    """
    Crée et initialise une instance d'intégration BBIA.

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
