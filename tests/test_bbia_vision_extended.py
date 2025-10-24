#!/usr/bin/env python3

"""
Tests étendus pour BBIA Vision
Tests ciblés pour améliorer la couverture de code
"""

from datetime import datetime
from unittest.mock import patch

from src.bbia_sim.bbia_vision import BBIAVision


class TestBBIAVisionExtended:
    """Tests étendus pour BBIAVision"""

    def setup_method(self):
        """Configuration avant chaque test"""
        self.vision = BBIAVision()

    def test_init_defaults(self):
        """Test initialisation avec valeurs par défaut"""
        assert self.vision.camera_active is True
        assert self.vision.vision_quality == "HD"
        assert self.vision.detection_range == 3.0
        assert self.vision.objects_detected == []
        assert self.vision.faces_detected == []
        assert self.vision.tracking_active is False
        assert self.vision.current_focus is None

    def test_init_specs(self):
        """Test spécifications hardware"""
        specs = self.vision.specs
        assert specs["camera"] == "Grand angle"
        assert specs["resolution"] == "1080p"
        assert specs["fov"] == "120°"
        assert specs["focus"] == "Auto"
        assert specs["night_vision"] is False

    @patch("builtins.print")
    def test_scan_environment_success(self, mock_print):
        """Test scan environnement réussi"""
        result = self.vision.scan_environment()

        assert "objects" in result
        assert "faces" in result
        assert "timestamp" in result
        assert len(result["objects"]) == 5
        assert len(result["faces"]) == 2
        assert len(self.vision.objects_detected) == 5
        assert len(self.vision.faces_detected) == 2

    @patch("builtins.print")
    def test_recognize_object_found(self, mock_print):
        """Test reconnaissance d'objet trouvé"""
        self.vision.scan_environment()
        result = self.vision.recognize_object("chaise")

        assert result is not None
        assert result["name"] == "chaise"
        assert result["distance"] == 1.2
        assert result["confidence"] == 0.95

    @patch("builtins.print")
    def test_recognize_object_not_found(self, mock_print):
        """Test reconnaissance d'objet non trouvé"""
        self.vision.scan_environment()
        result = self.vision.recognize_object("objet_inexistant")

        assert result is None

    @patch("builtins.print")
    def test_recognize_object_empty_list(self, mock_print):
        """Test reconnaissance avec liste vide"""
        result = self.vision.recognize_object("chaise")

        assert result is None

    @patch("builtins.print")
    def test_detect_faces_with_data(self, mock_print):
        """Test détection de visages avec données existantes"""
        self.vision.scan_environment()
        faces = self.vision.detect_faces()

        assert len(faces) == 2
        assert faces[0]["name"] == "humain"
        assert faces[1]["emotion"] == "happy"

    @patch("builtins.print")
    def test_detect_faces_empty(self, mock_print):
        """Test détection de visages sans données"""
        faces = self.vision.detect_faces()

        assert len(faces) == 2  # scan_environment est appelé automatiquement

    @patch("builtins.print")
    def test_track_object_success(self, mock_print):
        """Test suivi d'objet réussi"""
        self.vision.scan_environment()
        result = self.vision.track_object("livre")

        assert result is True
        assert self.vision.tracking_active is True
        assert self.vision.current_focus is not None
        assert self.vision.current_focus["name"] == "livre"

    @patch("builtins.print")
    def test_track_object_failure(self, mock_print):
        """Test suivi d'objet échoué"""
        result = self.vision.track_object("objet_inexistant")

        assert result is False
        assert self.vision.tracking_active is False
        assert self.vision.current_focus is None

    @patch("builtins.print")
    def test_stop_tracking_active(self, mock_print):
        """Test arrêt de suivi actif"""
        self.vision.scan_environment()
        self.vision.track_object("livre")
        self.vision.stop_tracking()

        assert self.vision.tracking_active is False
        assert self.vision.current_focus is None

    @patch("builtins.print")
    def test_stop_tracking_inactive(self, mock_print):
        """Test arrêt de suivi inactif"""
        self.vision.stop_tracking()

        assert self.vision.tracking_active is False
        assert self.vision.current_focus is None

    def test_get_focus_status_tracking_active(self):
        """Test statut focus avec suivi actif"""
        self.vision.scan_environment()
        self.vision.track_object("livre")
        status = self.vision.get_focus_status()

        assert status["tracking_active"] is True
        assert status["current_focus"] is not None
        assert status["objects_count"] == 5
        assert status["faces_count"] == 2

    def test_get_focus_status_no_tracking(self):
        """Test statut focus sans suivi"""
        status = self.vision.get_focus_status()

        assert status["tracking_active"] is False
        assert status["current_focus"] is None
        assert status["objects_count"] == 0
        assert status["faces_count"] == 0

    @patch("builtins.print")
    def test_analyze_emotion_valid(self, mock_print):
        """Test analyse d'émotion valide"""
        face_data = {"emotion": "happy"}
        emotion = self.vision.analyze_emotion(face_data)

        assert emotion == "happy"

    @patch("builtins.print")
    def test_analyze_emotion_default(self, mock_print):
        """Test analyse d'émotion par défaut"""
        face_data = {}
        emotion = self.vision.analyze_emotion(face_data)

        assert emotion == "neutral"

    def test_calculate_distance_origin(self):
        """Test calcul distance origine"""
        distance = self.vision.calculate_distance((0.0, 0.0))
        assert distance == 0.0

    def test_calculate_distance_positive(self):
        """Test calcul distance positive"""
        distance = self.vision.calculate_distance((3.0, 4.0))
        assert distance == 5.0

    def test_calculate_distance_negative(self):
        """Test calcul distance négative"""
        distance = self.vision.calculate_distance((-3.0, -4.0))
        assert distance == 5.0

    def test_get_vision_stats_default(self):
        """Test statistiques vision par défaut"""
        stats = self.vision.get_vision_stats()

        assert stats["camera_active"] is True
        assert stats["vision_quality"] == "HD"
        assert stats["detection_range"] == 3.0
        assert stats["objects_detected"] == 0
        assert stats["faces_detected"] == 0
        assert stats["tracking_active"] is False
        assert "specs" in stats

    def test_get_vision_stats_with_data(self):
        """Test statistiques vision avec données"""
        self.vision.scan_environment()
        self.vision.track_object("livre")
        stats = self.vision.get_vision_stats()

        assert stats["objects_detected"] == 5
        assert stats["faces_detected"] == 2
        assert stats["tracking_active"] is True

    def test_scan_environment_objects_structure(self):
        """Test structure des objets détectés"""
        result = self.vision.scan_environment()

        for obj in result["objects"]:
            assert "name" in obj
            assert "distance" in obj
            assert "confidence" in obj
            assert "position" in obj
            assert isinstance(obj["position"], tuple)
            assert len(obj["position"]) == 2

    def test_scan_environment_faces_structure(self):
        """Test structure des visages détectés"""
        result = self.vision.scan_environment()

        for face in result["faces"]:
            assert "name" in face
            assert "distance" in face
            assert "confidence" in face
            assert "emotion" in face
            assert "position" in face
            assert isinstance(face["position"], tuple)
            assert len(face["position"]) == 2

    def test_scan_environment_timestamp(self):
        """Test timestamp du scan"""
        result = self.vision.scan_environment()

        timestamp = result["timestamp"]
        assert isinstance(timestamp, str)
        # Vérifier que c'est un format ISO valide
        datetime.fromisoformat(timestamp)

    @patch("builtins.print")
    def test_track_object_case_sensitive(self, mock_print):
        """Test suivi d'objet sensible à la casse"""
        self.vision.scan_environment()
        result = self.vision.track_object("CHAISE")  # Majuscules

        assert result is False  # "chaise" en minuscules dans les données

    def test_current_focus_after_tracking(self):
        """Test focus actuel après suivi"""
        self.vision.scan_environment()
        self.vision.track_object("table")

        focus = self.vision.current_focus
        assert focus is not None
        assert focus["name"] == "table"
        assert focus["distance"] == 2.1
        assert focus["confidence"] == 0.92

    def test_objects_detected_persistence(self):
        """Test persistance des objets détectés"""
        self.vision.scan_environment()
        initial_count = len(self.vision.objects_detected)

        # Nouveau scan
        self.vision.scan_environment()
        new_count = len(self.vision.objects_detected)

        assert new_count == initial_count  # Même nombre d'objets

    def test_faces_detected_persistence(self):
        """Test persistance des visages détectés"""
        self.vision.scan_environment()
        initial_count = len(self.vision.faces_detected)

        # Nouveau scan
        self.vision.scan_environment()
        new_count = len(self.vision.faces_detected)

        assert new_count == initial_count  # Même nombre de visages
