#!/usr/bin/env python3
"""Module de reconnaissance faciale personnalisée avec DeepFace.

Fonctionnalités :
- Reconnaissance de personnes spécifiques (famille, amis)
- Détection d'émotions sur visages
- Compatible SDK Reachy Mini (pas de conflit)
- Backend ONNX recommandé pour Raspberry Pi 5
"""

import logging
import os
import tempfile
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

logger = logging.getLogger(__name__)

# Import conditionnel DeepFace
DEEPFACE_AVAILABLE = False
try:
    from deepface import DeepFace

    DEEPFACE_AVAILABLE = True
except ImportError:
    DeepFace = None
    logger.debug("DeepFace non disponible. Installer avec: pip install deepface")


class BBIAPersonRecognition:
    """Module de reconnaissance faciale personnalisée avec DeepFace.

    Permet à BBIA de reconnaître des personnes spécifiques (famille, amis)
    et de détecter leurs émotions.
    """

    def __init__(self, db_path: str = "faces_db", model_name: str = "VGG-Face") -> None:
        """Initialise le module de reconnaissance faciale.

        Args:
            db_path: Chemin vers la base de données de visages (dossier avec photos)
            model_name: Modèle DeepFace à utiliser (VGG-Face, Facenet, etc.)

        """
        self.db_path = Path(db_path)
        self.model_name = model_name
        self.is_initialized = False

        if not DEEPFACE_AVAILABLE:
            logger.debug(
                "⚠️ DeepFace non disponible. Installer avec: pip install deepface",
            )
            return

        # Créer le dossier de base de données s'il n'existe pas
        self.db_path.mkdir(parents=True, exist_ok=True)

        logger.info(
            f"👤 BBIAPersonRecognition initialisé (db: {db_path}, modèle: {model_name})",
        )
        self.is_initialized = True

    def register_person(self, image_path: str, person_name: str) -> bool:
        """Enregistre une personne dans la base de données.

        Args:
            image_path: Chemin vers la photo de la personne
            person_name: Nom de la personne (ex: "Alice", "Maman")

        Returns:
            True si enregistré avec succès

        """
        if not self.is_initialized:
            logger.warning("⚠️ DeepFace non disponible, enregistrement impossible")
            return False

        try:
            # Créer un dossier pour cette personne
            person_dir = self.db_path / person_name
            person_dir.mkdir(parents=True, exist_ok=True)

            # Copier l'image dans le dossier
            import shutil

            target_path = person_dir / Path(image_path).name
            shutil.copy2(image_path, target_path)

            logger.info("✅ Personne '%s' enregistrée: %s", person_name, target_path)
            return True

        except Exception:
            logger.exception("❌ Erreur enregistrement personne '%s':", person_name)
            return False

    def recognize_person(
        self,
        image_path: str | npt.NDArray[np.uint8],
        enforce_detection: bool = False,
    ) -> dict[str, Any] | None:
        """Reconnaît une personne dans une image.

        Args:
            image_path: Chemin vers l'image ou numpy array (BGR)
            enforce_detection: Si True, échoue si aucun visage détecté

        Returns:
            Dict avec 'name' (nom personne), 'confidence' (0-1), 'distance' (similarité)
            ou None si personne non reconnue

        """
        if not self.is_initialized:
            return None

        try:
            # Si numpy array, sauvegarder temporairement
            temp_path = None
            if isinstance(image_path, np.ndarray):
                # cv2 importé conditionnellement si nécessaire
                try:
                    import cv2  # noqa: PLC0415
                except ImportError:
                    cv2 = None  # type: ignore[assignment]
                
                if cv2 is None:
                    raise ImportError("cv2 (OpenCV) requis pour traiter les images numpy")

                # Utiliser tempfile pour créer un fichier temporaire sécurisé
                fd, temp_path = tempfile.mkstemp(
                    suffix=".jpg",
                    prefix="bbia_face_temp_",
                )
                os.close(fd)  # Fermer le descripteur de fichier
                cv2.imwrite(temp_path, image_path)
                image_path = temp_path

            # Rechercher la personne dans la base de données
            df = DeepFace.find(
                img_path=str(image_path),
                db_path=str(self.db_path),
                model_name=self.model_name,
                enforce_detection=enforce_detection,
                detector_backend="opencv",  # Plus rapide que "retinaface"
                silent=True,  # Réduire logs
            )

            # Nettoyer fichier temporaire
            if temp_path and Path(temp_path).exists():
                Path(temp_path).unlink()

            # Traiter résultats
            if df is not None and len(df) > 0:
                best_match = df.iloc[0]
                identity_path = Path(best_match["identity"])

                # Extraire le nom de la personne depuis le chemin
                person_name = identity_path.parent.name

                # Calculer confidence (distance 0 = parfait match, distance 1 = très différent)
                distance = float(best_match["distance"])
                confidence = max(0.0, 1.0 - distance)  # Convertir distance → confidence

                # Seuil de confiance (ajuster si nécessaire)
                if confidence >= 0.6:  # Seuil minimal 60%
                    return {
                        "name": person_name,
                        "confidence": confidence,
                        "distance": distance,
                        "model": self.model_name,
                    }

            return None

        except ValueError as e:
            # Aucun visage détecté (normal si enforce_detection=False)
            if "No face detected" in str(e) and not enforce_detection:
                logger.debug("Aucun visage détecté dans l'image")
                return None
            logger.warning("Erreur reconnaissance visage: %s", e)
            return None
        except Exception:
            logger.exception("❌ Erreur reconnaissance personne")
            return None
        finally:
            # Nettoyer fichier temporaire si nécessaire
            if temp_path and Path(temp_path).exists():
                try:
                    Path(temp_path).unlink()
                except Exception as cleanup_error:
                    logger.debug(
                        "Erreur lors du nettoyage du fichier temporaire %s: %s",
                        temp_path,
                        cleanup_error,
                    )

    def detect_emotion(
        self,
        image_path: str | npt.NDArray[np.uint8],
        enforce_detection: bool = False,
    ) -> dict[str, Any] | None:
        """Détecte l'émotion dominante sur un visage.

        Args:
            image_path: Chemin vers l'image ou numpy array (BGR)
            enforce_detection: Si True, échoue si aucun visage détecté

        Returns:
            Dict avec 'emotion' (nom), 'confidence' (0-1), et détails de toutes les émotions
            ou None si aucun visage

        """
        if not self.is_initialized:
            return None

        try:
            # Si numpy array, sauvegarder temporairement
            temp_path = None
            if isinstance(image_path, np.ndarray):
                # cv2 importé conditionnellement si nécessaire
                try:
                    import cv2  # noqa: PLC0415
                except ImportError:
                    cv2 = None  # type: ignore[assignment]
                
                if cv2 is None:
                    raise ImportError("cv2 (OpenCV) requis pour traiter les images numpy")

                # Utiliser tempfile pour créer un fichier temporaire sécurisé
                fd, temp_path = tempfile.mkstemp(
                    suffix=".jpg",
                    prefix="bbia_emotion_temp_",
                )
                os.close(fd)  # Fermer le descripteur de fichier
                cv2.imwrite(temp_path, image_path)
                image_path = temp_path

            # Analyser l'émotion
            result = DeepFace.analyze(
                img_path=str(image_path),
                actions=["emotion"],
                enforce_detection=enforce_detection,
                detector_backend="opencv",
                silent=True,
            )

            # Nettoyer fichier temporaire
            if temp_path and Path(temp_path).exists():
                Path(temp_path).unlink()

            # Traiter résultats (peut être liste ou dict selon version DeepFace)
            if isinstance(result, list):
                result = result[0]

            if result and "dominant_emotion" in result:
                emotion = result["dominant_emotion"]
                emotion_scores = result.get("emotion", {})

                # Trouver la confiance de l'émotion dominante
                confidence = (
                    emotion_scores.get(emotion, 0.0) / 100.0
                )  # Convertir % → 0-1

                return {
                    "emotion": emotion,
                    "confidence": confidence,
                    "scores": emotion_scores,
                }

            return None

        except ValueError as e:
            if "No face detected" in str(e) and not enforce_detection:
                logger.debug("Aucun visage détecté pour analyse émotion")
                return None
            logger.warning("Erreur détection émotion: %s", e)
            return None
        except Exception:
            logger.exception("❌ Erreur détection émotion")
            return None
        finally:
            if temp_path and Path(temp_path).exists():
                try:
                    Path(temp_path).unlink()
                except Exception as cleanup_error:
                    logger.debug(
                        "Erreur lors du nettoyage du fichier temporaire %s: %s",
                        temp_path,
                        cleanup_error,
                    )

    def recognize_with_emotion(
        self,
        image_path: str | npt.NDArray[np.uint8],
    ) -> dict[str, Any] | None:
        """Reconnaît une personne ET détecte son émotion en une seule fois.

        Args:
            image_path: Chemin vers l'image ou numpy array

        Returns:
            Dict avec 'name', 'confidence', 'emotion', 'emotion_confidence'
            ou None si personne non reconnue

        """
        if not self.is_initialized:
            return None

        # Reconnaître la personne
        person_result = self.recognize_person(image_path, enforce_detection=False)

        if not person_result:
            return None

        # Détecter l'émotion
        emotion_result = self.detect_emotion(image_path, enforce_detection=False)

        # Combiner les résultats
        result = person_result.copy()
        if emotion_result:
            result["emotion"] = emotion_result["emotion"]
            result["emotion_confidence"] = emotion_result["confidence"]

        return result

    def get_registered_persons(self) -> list[str]:
        """Retourne la liste des personnes enregistrées.

        Returns:
            Liste des noms de personnes

        """
        if not self.db_path.exists():
            return []

        persons = []
        for item in self.db_path.iterdir():
            if item.is_dir():
                persons.append(item.name)

        return sorted(persons)


def create_face_recognition(
    db_path: str = "faces_db",
    model_name: str = "VGG-Face",
) -> BBIAPersonRecognition | None:
    """Factory function pour créer une instance BBIAPersonRecognition.

    Args:
        db_path: Chemin vers la base de données de visages
        model_name: Modèle DeepFace à utiliser

    Returns:
        Instance BBIAPersonRecognition ou None si DeepFace non disponible

    """
    if not DEEPFACE_AVAILABLE:
        logger.debug("⚠️ DeepFace non disponible")
        return None

    return BBIAPersonRecognition(db_path=db_path, model_name=model_name)


# Test rapide
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    logging.info("🧪 Test module Face Recognition BBIA")
    logging.info("=" * 40)

    face_rec = create_face_recognition()
    if face_rec:
        logging.info("✅ Module Face Recognition créé")
        logging.info(f"   • Base de données: {face_rec.db_path}")
        logging.info(f"   • Modèle: {face_rec.model_name}")
        logging.info(
            f"   • Personnes enregistrées: {len(face_rec.get_registered_persons())}",
        )
    else:
        logging.error("❌ Impossible de créer le module (DeepFace non disponible)")
        logging.info("   Installer avec: pip install deepface")
