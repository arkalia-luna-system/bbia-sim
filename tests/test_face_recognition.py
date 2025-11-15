"""Tests pour le module face_recognition.py."""

import os
import tempfile
from pathlib import Path
from unittest.mock import MagicMock, Mock, patch

import numpy as np
import pytest

from bbia_sim.face_recognition import (
    BBIAPersonRecognition,
    create_face_recognition,
)


class TestBBIAPersonRecognition:
    """Tests pour BBIAPersonRecognition."""

    def test_init_without_deepface(self):
        """Test initialisation sans DeepFace."""
        with patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", False):
            rec = BBIAPersonRecognition()
            assert rec.is_initialized is False
            assert rec.db_path == Path("faces_db")
            assert rec.model_name == "VGG-Face"

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    @patch("bbia_sim.face_recognition.DeepFace")
    def test_init_with_deepface(self, mock_deepface):
        """Test initialisation avec DeepFace."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir, model_name="Facenet")
            assert rec.is_initialized is True
            assert rec.db_path == Path(tmpdir)
            assert rec.model_name == "Facenet"
            assert Path(tmpdir).exists()

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", False)
    def test_register_person_without_deepface(self):
        """Test enregistrement personne sans DeepFace."""
        rec = BBIAPersonRecognition()
        result = rec.register_person("test.jpg", "Alice")
        assert result is False

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    def test_register_person_with_deepface(self):
        """Test enregistrement personne avec DeepFace."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Créer une image temporaire
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as f:
                f.write(b"fake image data")
                image_path = f.name

            try:
                result = rec.register_person(image_path, "Alice")
                assert result is True
                person_dir = Path(tmpdir) / "Alice"
                assert person_dir.exists()
                assert len(list(person_dir.iterdir())) > 0
            finally:
                os.unlink(image_path)

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    def test_register_person_error(self):
        """Test enregistrement personne avec erreur."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Image inexistante
            result = rec.register_person("nonexistent.jpg", "Bob")
            assert result is False

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", False)
    def test_recognize_person_without_deepface(self):
        """Test reconnaissance personne sans DeepFace."""
        rec = BBIAPersonRecognition()
        result = rec.recognize_person("test.jpg")
        assert result is None

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    @patch("bbia_sim.face_recognition.DeepFace")
    def test_recognize_person_with_deepface(self, mock_deepface):
        """Test reconnaissance personne avec DeepFace."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Mock DeepFace.find avec DataFrame-like object
            mock_row = MagicMock()
            mock_row.__getitem__ = Mock(
                side_effect=lambda key: {
                    "identity": str(Path(tmpdir) / "Alice" / "photo.jpg"),
                    "distance": 0.3,
                }[key]
            )
            mock_iloc = MagicMock()
            mock_iloc.__getitem__ = Mock(return_value=mock_row)
            mock_df = Mock()
            mock_df.iloc = mock_iloc
            mock_df.__len__ = Mock(return_value=1)
            mock_deepface.find.return_value = mock_df

            result = rec.recognize_person("test.jpg")
            assert result is not None
            assert result["name"] == "Alice"
            assert result["confidence"] >= 0.6
            assert result["distance"] == 0.3

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    @patch("bbia_sim.face_recognition.DeepFace")
    def test_recognize_person_numpy_array(self, mock_deepface):
        """Test reconnaissance personne avec numpy array."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Mock DeepFace.find avec DataFrame-like object
            mock_row = MagicMock()
            mock_row.__getitem__ = Mock(
                side_effect=lambda key: {
                    "identity": str(Path(tmpdir) / "Bob" / "photo.jpg"),
                    "distance": 0.2,
                }[key]
            )
            mock_iloc = MagicMock()
            mock_iloc.__getitem__ = Mock(return_value=mock_row)
            mock_df = Mock()
            mock_df.iloc = mock_iloc
            mock_df.__len__ = Mock(return_value=1)
            mock_deepface.find.return_value = mock_df

            # Créer un numpy array
            img_array = np.zeros((100, 100, 3), dtype=np.uint8)

            with patch("cv2.imwrite") as mock_imwrite:
                result = rec.recognize_person(img_array)
                assert mock_imwrite.called
                assert result is not None
                assert result["name"] == "Bob"

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    @patch("bbia_sim.face_recognition.DeepFace")
    def test_recognize_person_no_match(self, mock_deepface):
        """Test reconnaissance personne sans correspondance."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Mock DeepFace.find retourne None
            mock_deepface.find.return_value = None

            result = rec.recognize_person("test.jpg")
            assert result is None

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    @patch("bbia_sim.face_recognition.DeepFace")
    def test_recognize_person_low_confidence(self, mock_deepface):
        """Test reconnaissance personne avec faible confiance."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Mock DeepFace.find avec distance élevée (faible confiance)
            mock_row = MagicMock()
            mock_row.__getitem__ = Mock(
                side_effect=lambda key: {
                    "identity": str(Path(tmpdir) / "Charlie" / "photo.jpg"),
                    "distance": 0.8,  # Confiance = 0.2 (< 0.6)
                }[key]
            )
            mock_iloc = MagicMock()
            mock_iloc.__getitem__ = Mock(return_value=mock_row)
            mock_df = Mock()
            mock_df.iloc = mock_iloc
            mock_df.__len__ = Mock(return_value=1)
            mock_deepface.find.return_value = mock_df

            result = rec.recognize_person("test.jpg")
            assert result is None  # Seuil de confiance non atteint

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    @patch("bbia_sim.face_recognition.DeepFace")
    def test_recognize_person_value_error(self, mock_deepface):
        """Test reconnaissance personne avec ValueError (pas de visage)."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Mock DeepFace.find lève ValueError
            mock_deepface.find.side_effect = ValueError("No face detected")

            result = rec.recognize_person("test.jpg", enforce_detection=False)
            assert result is None

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", False)
    def test_detect_emotion_without_deepface(self):
        """Test détection émotion sans DeepFace."""
        rec = BBIAPersonRecognition()
        result = rec.detect_emotion("test.jpg")
        assert result is None

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    @patch("bbia_sim.face_recognition.DeepFace")
    def test_detect_emotion_with_deepface(self, mock_deepface):
        """Test détection émotion avec DeepFace."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Mock DeepFace.analyze
            mock_deepface.analyze.return_value = {
                "dominant_emotion": "happy",
                "emotion": {"happy": 85.0, "sad": 10.0, "neutral": 5.0},
            }

            result = rec.detect_emotion("test.jpg")
            assert result is not None
            assert result["emotion"] == "happy"
            assert result["confidence"] == 0.85
            assert "scores" in result

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    @patch("bbia_sim.face_recognition.DeepFace")
    def test_detect_emotion_list_result(self, mock_deepface):
        """Test détection émotion avec résultat en liste."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Mock DeepFace.analyze retourne une liste
            mock_deepface.analyze.return_value = [
                {
                    "dominant_emotion": "sad",
                    "emotion": {"happy": 5.0, "sad": 90.0, "neutral": 5.0},
                }
            ]

            result = rec.detect_emotion("test.jpg")
            assert result is not None
            assert result["emotion"] == "sad"

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    @patch("bbia_sim.face_recognition.DeepFace")
    def test_detect_emotion_numpy_array(self, mock_deepface):
        """Test détection émotion avec numpy array."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Mock DeepFace.analyze
            mock_deepface.analyze.return_value = {
                "dominant_emotion": "neutral",
                "emotion": {"happy": 30.0, "sad": 20.0, "neutral": 50.0},
            }

            # Créer un numpy array
            img_array = np.zeros((100, 100, 3), dtype=np.uint8)

            with patch("cv2.imwrite") as mock_imwrite:
                result = rec.detect_emotion(img_array)
                assert mock_imwrite.called
                assert result is not None
                assert result["emotion"] == "neutral"

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    def test_recognize_with_emotion(self):
        """Test reconnaissance avec émotion."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Mock les méthodes
            rec.recognize_person = Mock(
                return_value={"name": "Alice", "confidence": 0.8, "distance": 0.2}
            )
            rec.detect_emotion = Mock(
                return_value={"emotion": "happy", "confidence": 0.9}
            )

            result = rec.recognize_with_emotion("test.jpg")
            assert result is not None
            assert result["name"] == "Alice"
            assert result["emotion"] == "happy"
            assert result["emotion_confidence"] == 0.9

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    def test_recognize_with_emotion_no_person(self):
        """Test reconnaissance avec émotion sans personne reconnue."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Mock recognize_person retourne None
            rec.recognize_person = Mock(return_value=None)

            result = rec.recognize_with_emotion("test.jpg")
            assert result is None

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    def test_get_registered_persons(self):
        """Test récupération personnes enregistrées."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True

            # Créer des dossiers de personnes
            (Path(tmpdir) / "Alice").mkdir()
            (Path(tmpdir) / "Bob").mkdir()
            (Path(tmpdir) / "Charlie").mkdir()
            (Path(tmpdir) / "file.txt").touch()  # Fichier (pas un dossier)

            persons = rec.get_registered_persons()
            assert len(persons) == 3
            assert "Alice" in persons
            assert "Bob" in persons
            assert "Charlie" in persons
            assert sorted(persons) == persons  # Vérifier tri

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    def test_get_registered_persons_no_db(self):
        """Test récupération personnes sans base de données."""
        with tempfile.TemporaryDirectory() as tmpdir:
            rec = BBIAPersonRecognition(db_path=tmpdir)
            rec.is_initialized = True
            rec.db_path = Path(tmpdir) / "nonexistent"

            persons = rec.get_registered_persons()
            assert persons == []


class TestCreateFaceRecognition:
    """Tests pour la fonction create_face_recognition."""

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", False)
    def test_create_face_recognition_without_deepface(self):
        """Test création sans DeepFace."""
        result = create_face_recognition()
        assert result is None

    @patch("bbia_sim.face_recognition.DEEPFACE_AVAILABLE", True)
    def test_create_face_recognition_with_deepface(self):
        """Test création avec DeepFace."""
        with tempfile.TemporaryDirectory() as tmpdir:
            result = create_face_recognition(db_path=tmpdir, model_name="Facenet")
            assert result is not None
            assert isinstance(result, BBIAPersonRecognition)
            assert result.db_path == Path(tmpdir)
            assert result.model_name == "Facenet"
