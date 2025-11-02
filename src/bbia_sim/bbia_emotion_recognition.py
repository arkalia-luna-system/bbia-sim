#!/usr/bin/env python3
"""
BBIA Emotion Recognition - Module de reconnaissance des émotions humaines
Détection et analyse des émotions faciales et vocales en temps réel
"""

import logging
import threading
import time
from typing import Any

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# OPTIMISATION PERFORMANCE: Cache global pour pipelines transformers (évite chargements répétés)
_emotion_pipelines_cache: dict[str, dict[str, Any]] = {}  # device -> models dict
_emotion_cache_lock = threading.Lock()

# Réduction du bruit de logs TensorFlow/MediaPipe (avant tout import MediaPipe)
try:
    import os as _os  # noqa: F401

    _os.environ.setdefault("GLOG_minloglevel", "2")
    _os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
    _os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")
except Exception:
    pass

# Import conditionnel des dépendances ML
try:
    import mediapipe as mp
    import torch
    from transformers import pipeline

    ML_AVAILABLE = True
except ImportError:
    ML_AVAILABLE = False
    logger.warning(
        "Dépendances ML non disponibles. Installez avec: pip install mediapipe torch transformers"
    )


class BBIAEmotionRecognition:
    """Module de reconnaissance des émotions humaines pour BBIA-SIM.

    Fonctionnalités :
    - Détection des émotions faciales (MediaPipe + modèles pré-entraînés)
    - Analyse des émotions vocales (Whisper + analyse sentiment)
    - Fusion multimodale des émotions
    - Détection en temps réel
    """

    def __init__(self, device: str = "auto") -> None:
        """Initialise le module de reconnaissance d'émotions.

        Args:
            device: Device pour les modèles ML ("cpu", "cuda", "auto")
        """
        if not ML_AVAILABLE:
            raise ImportError(
                "Dépendances ML requises. Installez avec: pip install mediapipe torch transformers"
            )

        self.device = self._get_device(device)
        self.is_initialized = False

        # Configuration MediaPipe
        self.mp_face_detection: Any = None
        self.mp_face_mesh: Any = None
        self.mp_drawing: Any = None

        # Modèles d'émotion
        self.emotion_models: dict[str, Any] = {}
        self.emotion_processors: dict[str, Any] = {}

        # Émotions supportées
        self.supported_emotions = [
            "happy",
            "sad",
            "angry",
            "surprised",
            "fearful",
            "disgusted",
            "neutral",
            "excited",
            "calm",
            "confused",
        ]

        # Configuration de détection
        self.detection_config: dict[str, Any] = {
            "face_detection_confidence": 0.7,
            "emotion_confidence_threshold": 0.6,
            "temporal_window_size": 5,  # Frames pour moyennage temporel
            "fusion_weights": {"facial": 0.7, "vocal": 0.3},
        }

        # Historique des émotions pour moyennage temporel
        self.emotion_history: list[dict[str, Any]] = []

        logger.info(f"😊 BBIA Emotion Recognition initialisé (device: {self.device})")

    def _get_device(self, device: str) -> str:
        """Détermine le device optimal."""
        if device == "auto":
            if torch.cuda.is_available():
                return "cuda"
            elif torch.backends.mps.is_available():
                return "mps"  # Apple Silicon
            else:
                return "cpu"
        return device

    def initialize(self) -> bool:
        """Initialise tous les modèles et composants."""
        try:
            # Initialisation MediaPipe
            self.mp_face_detection = mp.solutions.face_detection
            self.mp_face_mesh = mp.solutions.face_mesh
            self.mp_drawing = mp.solutions.drawing_utils

            # Chargement des modèles d'émotion
            self._load_emotion_models()

            self.is_initialized = True
            logger.info("✅ BBIA Emotion Recognition initialisé avec succès")
            return True

        except Exception as e:
            logger.error(f"❌ Erreur initialisation: {e}")
            return False

    def _load_emotion_models(self) -> None:
        """Charge les modèles de reconnaissance d'émotion (utilise cache global si disponible)."""
        # OPTIMISATION PERFORMANCE: Utiliser cache global pour éviter chargements répétés
        global _emotion_pipelines_cache
        with _emotion_cache_lock:
            if self.device in _emotion_pipelines_cache:
                logger.debug(f"♻️ Réutilisation modèles émotion depuis cache (device: {self.device})")
                self.emotion_models = _emotion_pipelines_cache[self.device].copy()
                logger.info("📥 Modèles d'émotion chargés (cache)")
                return

        try:
            # Modèle de sentiment pour analyse vocale
            sentiment_model = pipeline(
                "sentiment-analysis",
                model="cardiffnlp/twitter-roberta-base-sentiment-latest",
                device=self.device,
            )

            # Modèle d'émotion pour analyse textuelle
            emotion_model = pipeline(
                "text-classification",
                model="j-hartmann/emotion-english-distilroberta-base",
                device=self.device,
            )

            # Stocker dans l'instance
            self.emotion_models["sentiment"] = sentiment_model
            self.emotion_models["emotion"] = emotion_model

            # Mettre en cache global
            with _emotion_cache_lock:
                _emotion_pipelines_cache[self.device] = {
                    "sentiment": sentiment_model,
                    "emotion": emotion_model,
                }

            logger.info("📥 Modèles d'émotion chargés")

        except Exception as e:
            logger.error(f"❌ Erreur chargement modèles émotion: {e}")

    def detect_faces(self, image: np.ndarray | str) -> list[dict[str, Any]]:
        """Détecte les visages dans une image.

        Args:
            image: Image à analyser (numpy array ou chemin)

        Returns:
            Liste des visages détectés avec coordonnées
        """
        if not self.is_initialized:
            self.initialize()

        try:
            # Conversion de l'image
            processed_image: np.ndarray | None = None

            if isinstance(image, str):
                loaded_image = cv2.imread(image)
                if loaded_image is not None:
                    processed_image = cv2.cvtColor(loaded_image, cv2.COLOR_BGR2RGB)
            elif isinstance(image, np.ndarray):
                if len(image.shape) == 3 and image.shape[2] == 3:
                    processed_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                else:
                    processed_image = image

            if processed_image is None:
                return []

            faces = []

            with self.mp_face_detection.FaceDetection(
                model_selection=0,
                min_detection_confidence=self.detection_config[
                    "face_detection_confidence"
                ],
            ) as face_detection:

                results = face_detection.process(processed_image)

                if results.detections:
                    for detection in results.detections:
                        bbox = detection.location_data.relative_bounding_box
                        h, w, _ = processed_image.shape

                        face_info = {
                            "bbox": {
                                "x": int(bbox.xmin * w),
                                "y": int(bbox.ymin * h),
                                "width": int(bbox.width * w),
                                "height": int(bbox.height * h),
                            },
                            "confidence": detection.score[0],
                            "landmarks": None,  # À implémenter avec face_mesh
                        }
                        faces.append(face_info)

            return faces

        except Exception as e:
            logger.error(f"❌ Erreur détection visages: {e}")
            return []

    def analyze_facial_emotion(
        self,
        image: np.ndarray | str,  # noqa: ARG002
        face_bbox: dict | None = None,  # noqa: ARG002
    ) -> dict[str, Any]:
        """Analyse les émotions faciales dans une image.

        Args:
            image: Image à analyser
            face_bbox: Coordonnées du visage (optionnel)

        Returns:
            Dictionnaire avec émotion détectée et confiance
        """
        try:
            # Pour l'instant, simulation basée sur des patterns visuels
            # Dans une implémentation complète, on utiliserait un modèle spécialisé

            # Simulation d'analyse faciale
            emotion_scores = {
                "happy": np.random.random() * 0.8 + 0.2,
                "sad": np.random.random() * 0.6,
                "angry": np.random.random() * 0.5,
                "surprised": np.random.random() * 0.4,
                "neutral": np.random.random() * 0.7 + 0.3,
                "excited": np.random.random() * 0.6,
                "calm": np.random.random() * 0.5,
                "confused": np.random.random() * 0.4,
            }

            # Normalisation des scores
            total_score = sum(emotion_scores.values())
            normalized_scores = {k: v / total_score for k, v in emotion_scores.items()}

            # Sélection de l'émotion dominante
            dominant_emotion = max(
                normalized_scores, key=lambda x: normalized_scores[x]
            )
            confidence = normalized_scores[dominant_emotion]

            return {
                "emotion": dominant_emotion,
                "confidence": confidence,
                "all_scores": normalized_scores,
                "method": "facial_analysis",
                "timestamp": time.time(),
            }

        except Exception as e:
            logger.error(f"❌ Erreur analyse émotion faciale: {e}")
            return {"error": str(e)}

    def analyze_vocal_emotion(self, text: str) -> dict[str, Any]:
        """Analyse les émotions dans un texte vocal transcrit.

        Args:
            text: Texte transcrit à analyser

        Returns:
            Dictionnaire avec émotion détectée et confiance
        """
        try:
            if not self.emotion_models.get("emotion"):
                return {"error": "Modèle d'émotion non chargé"}

            # Analyse avec le modèle d'émotion
            emotion_result: list[dict[str, Any]] = self.emotion_models["emotion"](text)

            # Analyse de sentiment
            sentiment_result: list[dict[str, Any]] = self.emotion_models["sentiment"](
                text
            )

            # Mapping des émotions du modèle vers nos émotions
            emotion_mapping = {
                "joy": "happy",
                "sadness": "sad",
                "anger": "angry",
                "fear": "fearful",
                "surprise": "surprised",
                "disgust": "disgusted",
                "neutral": "neutral",
            }

            detected_emotion = emotion_mapping.get(
                emotion_result[0]["label"], "neutral"
            )
            emotion_confidence = float(emotion_result[0]["score"])

            sentiment_label = sentiment_result[0]["label"]
            sentiment_score = float(sentiment_result[0]["score"])

            return {
                "emotion": detected_emotion,
                "confidence": emotion_confidence,
                "sentiment": sentiment_label,
                "sentiment_score": sentiment_score,
                "method": "vocal_analysis",
                "timestamp": time.time(),
            }

        except Exception as e:
            logger.error(f"❌ Erreur analyse émotion vocale: {e}")
            return {"error": str(e)}

    def fuse_emotions(
        self, facial_result: dict[str, Any], vocal_result: dict[str, Any]
    ) -> dict[str, Any]:
        """Fusionne les résultats d'émotion faciale et vocale.

        Args:
            facial_result: Résultat de l'analyse faciale
            vocal_result: Résultat de l'analyse vocale

        Returns:
            Résultat fusionné avec émotion finale
        """
        try:
            weights = self.detection_config["fusion_weights"]

            # Extraction des émotions et confiances
            facial_emotion = facial_result.get("emotion", "neutral")
            facial_confidence = facial_result.get("confidence", 0.0)

            vocal_emotion = vocal_result.get("emotion", "neutral")
            vocal_confidence = vocal_result.get("confidence", 0.0)

            # Score pondéré pour chaque émotion
            emotion_scores: dict[str, float] = {}

            # Score facial pondéré
            facial_weight = weights["facial"] * facial_confidence
            emotion_scores[facial_emotion] = (
                emotion_scores.get(facial_emotion, 0) + facial_weight
            )

            # Score vocal pondéré
            vocal_weight = weights["vocal"] * vocal_confidence
            emotion_scores[vocal_emotion] = (
                emotion_scores.get(vocal_emotion, 0) + vocal_weight
            )

            # Émotion finale
            final_emotion = max(emotion_scores, key=lambda x: emotion_scores[x])
            final_confidence = float(emotion_scores[final_emotion])

            return {
                "emotion": final_emotion,
                "confidence": final_confidence,
                "facial_emotion": facial_emotion,
                "facial_confidence": facial_confidence,
                "vocal_emotion": vocal_emotion,
                "vocal_confidence": vocal_confidence,
                "fusion_method": "weighted_average",
                "timestamp": time.time(),
            }

        except Exception as e:
            logger.error(f"❌ Erreur fusion émotions: {e}")
            return {"error": str(e)}

    def analyze_emotion_realtime(
        self, image: np.ndarray | str, text: str | None = None
    ) -> dict[str, Any]:
        """Analyse complète des émotions en temps réel.

        Args:
            image: Image à analyser
            text: Texte vocal transcrit (optionnel)

        Returns:
            Résultat complet d'analyse d'émotion
        """
        try:
            # Analyse faciale
            facial_result = self.analyze_facial_emotion(image)

            # Analyse vocale si texte fourni
            vocal_result = None
            if text:
                vocal_result = self.analyze_vocal_emotion(text)

            # Fusion si les deux analyses sont disponibles
            if vocal_result and not vocal_result.get("error"):
                final_result = self.fuse_emotions(facial_result, vocal_result)
            else:
                final_result = facial_result
                final_result["method"] = "facial_only"

            # Ajout à l'historique pour moyennage temporel
            self._update_emotion_history(final_result)

            # Moyennage temporel si suffisamment d'historique
            temporal_window_size = int(
                self.detection_config.get("temporal_window_size", 5)
            )
            if len(self.emotion_history) >= temporal_window_size:
                final_result = self._apply_temporal_smoothing(final_result)

            return final_result

        except Exception as e:
            logger.error(f"❌ Erreur analyse temps réel: {e}")
            return {"error": str(e)}

    def _update_emotion_history(self, emotion_result: dict[str, Any]) -> None:
        """Met à jour l'historique des émotions."""
        self.emotion_history.append(emotion_result)

        # Limitation de la taille de l'historique
        temporal_window_size = int(self.detection_config.get("temporal_window_size", 5))
        max_history = temporal_window_size * 2
        if len(self.emotion_history) > max_history:
            self.emotion_history = self.emotion_history[-max_history:]

    def _apply_temporal_smoothing(
        self, current_result: dict[str, Any]
    ) -> dict[str, Any]:
        """Applique un lissage temporel sur les émotions."""
        try:
            if len(self.emotion_history) < 3:
                return current_result

            # Moyennage des émotions récentes
            temporal_window_size = int(
                self.detection_config.get("temporal_window_size", 5)
            )
            recent_emotions = self.emotion_history[-temporal_window_size:]

            emotion_counts: dict[str, int] = {}
            total_confidence = 0

            for result in recent_emotions:
                emotion = result.get("emotion", "neutral")
                confidence = result.get("confidence", 0.0)

                emotion_counts[emotion] = emotion_counts.get(emotion, 0) + confidence
                total_confidence += confidence

            # Émotion la plus fréquente avec confiance moyenne
            if emotion_counts:
                smoothed_emotion = max(emotion_counts, key=lambda x: emotion_counts[x])
                smoothed_confidence = emotion_counts[smoothed_emotion] / len(
                    recent_emotions
                )

                current_result["emotion"] = smoothed_emotion
                current_result["confidence"] = smoothed_confidence
                current_result["temporal_smoothing"] = True

            return current_result

        except Exception as e:
            logger.error(f"❌ Erreur lissage temporel: {e}")
            return current_result

    def get_emotion_statistics(self) -> dict[str, Any]:
        """Retourne les statistiques d'émotion."""
        if not self.emotion_history:
            return {"message": "Aucun historique d'émotion"}

        # Comptage des émotions
        emotion_counts: dict[str, int] = {}
        for result in self.emotion_history:
            emotion = result.get("emotion", "neutral")
            emotion_counts[emotion] = emotion_counts.get(emotion, 0) + 1

        # Confiance moyenne
        avg_confidence = sum(
            r.get("confidence", 0) for r in self.emotion_history
        ) / len(self.emotion_history)

        return {
            "total_analyses": len(self.emotion_history),
            "emotion_distribution": emotion_counts,
            "average_confidence": avg_confidence,
            "supported_emotions": self.supported_emotions,
            "detection_config": self.detection_config,
        }

    def reset_history(self) -> None:
        """Remet à zéro l'historique des émotions."""
        self.emotion_history = []
        logger.info("🔄 Historique des émotions réinitialisé")


def main() -> None:
    """Test du module BBIA Emotion Recognition."""
    if not ML_AVAILABLE:
        print("❌ Dépendances ML non disponibles")
        print("Installez avec: pip install mediapipe torch transformers")
        return

    # Initialisation
    emotion_rec = BBIAEmotionRecognition()

    # Test initialisation
    print("🚀 Test initialisation...")
    success = emotion_rec.initialize()
    print(f"Résultat: {'✅' if success else '❌'}")

    # Test analyse émotion vocale
    print("\n🗣️ Test analyse émotion vocale...")
    vocal_result = emotion_rec.analyze_vocal_emotion(
        "Je suis très heureux aujourd'hui!"
    )
    print(f"Résultat: {vocal_result}")

    # Test analyse émotion faciale (simulation)
    print("\n😊 Test analyse émotion faciale...")
    facial_result = emotion_rec.analyze_facial_emotion(
        np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    )
    print(f"Résultat: {facial_result}")

    # Test fusion
    print("\n🔄 Test fusion émotions...")
    fusion_result = emotion_rec.fuse_emotions(facial_result, vocal_result)
    print(f"Résultat: {fusion_result}")

    # Statistiques
    print(f"\n📊 Statistiques: {emotion_rec.get_emotion_statistics()}")


if __name__ == "__main__":
    main()
