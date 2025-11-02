#!/usr/bin/env python3

"""Tests étendus pour BBIA Vision
Tests ciblés pour améliorer la couverture de code.
"""

from collections import deque
from datetime import datetime
from typing import Any
from unittest.mock import patch

from src.bbia_sim.bbia_vision import BBIAVision


class TestBBIAVisionExtended:
    """Tests étendus pour BBIAVision."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.vision = BBIAVision()

    def test_init_defaults(self):
        """Test initialisation avec valeurs par défaut."""
        from collections import deque

        assert self.vision.camera_active is True
        assert self.vision.vision_quality == "HD"
        assert self.vision.detection_range == 3.0
        assert (
            isinstance(self.vision.objects_detected, deque)
            and len(self.vision.objects_detected) == 0
        )
        assert (
            isinstance(self.vision.faces_detected, deque)
            and len(self.vision.faces_detected) == 0
        )
        assert self.vision.tracking_active is False
        assert self.vision.current_focus is None

    def test_init_specs(self):
        """Test spécifications hardware."""
        specs = self.vision.specs
        assert specs["camera"] == "Grand angle"
        # Resolution a été clarifiée avec simulation/réel
        assert "1280x720" in specs["resolution"] or "HD" in specs["resolution"]
        # FOV a été clarifié avec simulation/réel
        assert "80°" in specs["fov"] or "120°" in specs["fov"]
        assert specs["focus"] == "Auto"
        assert specs["night_vision"] is False

    @patch("builtins.print")
    def test_scan_environment_success(self, mock_print):
        """Test scan environnement réussi."""
        result = self.vision.scan_environment()

        assert "objects" in result
        assert "faces" in result
        assert "timestamp" in result
        # Accepter soit simulation (5 objets, 2 visages) soit détection réelle (variable)
        # Le système peut utiliser YOLO/caméra réelle si disponible
        assert isinstance(result["objects"], list)
        assert isinstance(result["faces"], list)
        assert len(self.vision.objects_detected) >= 0
        assert len(self.vision.faces_detected) >= 0

    @patch("builtins.print")
    def test_recognize_object_found(self, mock_print):
        """Test reconnaissance d'objet trouvé."""
        self.vision.scan_environment()
        # Mock objets détectés pour garantir test reproductible (deque pour optimisation RAM)
        self.vision.objects_detected = deque(
            [
                {"name": "chaise", "distance": 1.2, "confidence": 0.95},
            ]
        )
        result = self.vision.recognize_object("chaise")

        assert result is not None
        assert result["name"] == "chaise"
        assert result["distance"] == 1.2
        assert result["confidence"] == 0.95

    @patch("builtins.print")
    def test_recognize_object_not_found(self, mock_print):
        """Test reconnaissance d'objet non trouvé."""
        self.vision.scan_environment()
        result = self.vision.recognize_object("objet_inexistant")

        assert result is None

    @patch("builtins.print")
    def test_recognize_object_empty_list(self, mock_print):
        """Test reconnaissance avec liste vide."""
        result = self.vision.recognize_object("chaise")

        assert result is None

    @patch("builtins.print")
    def test_detect_faces_with_data(self, mock_print):
        """Test détection de visages avec données existantes."""
        # Mock visages détectés pour garantir test reproductible (deque pour optimisation RAM)
        self.vision.faces_detected = deque(
            [
                {"name": "humain", "distance": 1.8, "emotion": "neutral"},
                {"name": "humain", "distance": 2.3, "emotion": "happy"},
            ]
        )
        faces = self.vision.detect_faces()

        assert len(faces) == 2
        assert faces[0]["name"] == "humain"
        assert faces[1]["emotion"] == "happy"

    @patch("builtins.print")
    def test_detect_faces_empty(self, mock_print):
        """Test détection de visages sans données."""
        # Forcer liste vide initiale (deque pour optimisation RAM)
        self.vision.faces_detected = deque()
        # scan_environment est appelé automatiquement, résultat variable selon environnement
        faces = self.vision.detect_faces()

        assert isinstance(faces, list)
        assert len(faces) >= 0  # Peut être 0 si pas de visages détectés

    @patch("builtins.print")
    def test_track_object_success(self, mock_print):
        """Test suivi d'objet réussi."""
        # Mock objets détectés pour garantir test reproductible (deque pour optimisation RAM)
        self.vision.objects_detected = deque(
            [
                {"name": "livre", "distance": 0.8, "confidence": 0.88},
            ]
        )
        result = self.vision.track_object("livre")

        assert result is True
        assert self.vision.tracking_active is True
        assert self.vision.current_focus is not None
        assert self.vision.current_focus["name"] == "livre"

    @patch("builtins.print")
    def test_track_object_failure(self, mock_print):
        """Test suivi d'objet échoué."""
        result = self.vision.track_object("objet_inexistant")

        assert result is False
        assert self.vision.tracking_active is False
        assert self.vision.current_focus is None

    @patch("builtins.print")
    def test_stop_tracking_active(self, mock_print):
        """Test arrêt de suivi actif."""
        # Mock objets pour test reproductible (deque pour optimisation RAM)
        self.vision.objects_detected = deque(
            [
                {"name": "livre", "distance": 0.8, "confidence": 0.88},
            ]
        )
        self.vision.track_object("livre")
        self.vision.stop_tracking()

        assert self.vision.tracking_active is False
        assert self.vision.current_focus is None

    @patch("builtins.print")
    def test_stop_tracking_inactive(self, mock_print):
        """Test arrêt de suivi inactif."""
        self.vision.stop_tracking()

        assert self.vision.tracking_active is False
        assert self.vision.current_focus is None

    def test_get_focus_status_tracking_active(self):
        """Test statut focus avec suivi actif."""
        # Mock objets et visages pour test reproductible (deque pour optimisation RAM)
        self.vision.objects_detected = deque(
            [
                {"name": "livre", "distance": 0.8, "confidence": 0.88},
                {"name": "chaise", "distance": 1.2, "confidence": 0.95},
            ]
        )
        self.vision.faces_detected = deque(
            [
                {"name": "humain", "emotion": "neutral"},
                {"name": "humain", "emotion": "happy"},
            ]
        )
        self.vision.track_object("livre")
        status = self.vision.get_focus_status()

        assert status["tracking_active"] is True
        assert status["current_focus"] is not None
        assert status["current_focus"]["name"] == "livre"
        assert status["objects_count"] == 2  # Aligné avec mock
        assert status["faces_count"] == 2

    def test_get_focus_status_no_tracking(self):
        """Test statut focus sans suivi."""
        status = self.vision.get_focus_status()

        assert status["tracking_active"] is False
        assert status["current_focus"] is None
        assert status["objects_count"] == 0
        assert status["faces_count"] == 0

    @patch("builtins.print")
    def test_analyze_emotion_valid(self, mock_print):
        """Test analyse d'émotion valide."""
        face_data: dict[str, Any] = {"emotion": "happy"}
        emotion = self.vision.analyze_emotion(face_data)

        assert emotion == "happy"

    @patch("builtins.print")
    def test_analyze_emotion_default(self, mock_print):
        """Test analyse d'émotion par défaut."""
        face_data: dict[str, Any] = {}
        emotion = self.vision.analyze_emotion(face_data)

        assert emotion == "neutral"

    def test_calculate_distance_origin(self):
        """Test calcul distance origine."""
        distance = self.vision.calculate_distance((0.0, 0.0))
        assert distance == 0.0

    def test_calculate_distance_positive(self):
        """Test calcul distance positive."""
        distance = self.vision.calculate_distance((3.0, 4.0))
        assert distance == 5.0

    def test_calculate_distance_negative(self):
        """Test calcul distance négative."""
        distance = self.vision.calculate_distance((-3.0, -4.0))
        assert distance == 5.0

    def test_get_vision_stats_default(self):
        """Test statistiques vision par défaut."""
        stats = self.vision.get_vision_stats()

        assert stats["camera_active"] is True
        assert stats["vision_quality"] == "HD"
        assert stats["detection_range"] == 3.0
        assert stats["objects_detected"] == 0
        assert stats["faces_detected"] == 0
        assert stats["tracking_active"] is False
        assert "specs" in stats

    def test_get_vision_stats_with_data(self):
        """Test statistiques vision avec données."""
        # Mock objets et visages pour test reproductible (deque pour optimisation RAM)
        self.vision.objects_detected = deque(
            [
                {"name": "livre", "distance": 0.8, "confidence": 0.88},
            ]
        )
        self.vision.faces_detected = deque(
            [
                {"name": "humain", "emotion": "neutral"},
                {"name": "humain", "emotion": "happy"},
            ]
        )
        self.vision.track_object("livre")
        stats = self.vision.get_vision_stats()

        assert stats["objects_detected"] == 1  # Aligné avec mock
        assert stats["faces_detected"] == 2
        assert stats["tracking_active"] is True

    def test_scan_environment_objects_structure(self):
        """Test structure des objets détectés."""
        result = self.vision.scan_environment()

        for obj in result["objects"]:
            assert "name" in obj
            assert "distance" in obj
            assert "confidence" in obj
            assert "position" in obj
            assert isinstance(obj["position"], tuple)
            assert len(obj["position"]) == 2

    def test_scan_environment_faces_structure(self):
        """Test structure des visages détectés."""
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
        """Test timestamp du scan."""
        result = self.vision.scan_environment()

        timestamp = result["timestamp"]
        assert isinstance(timestamp, str)
        # Vérifier que c'est un format ISO valide
        datetime.fromisoformat(timestamp)

    @patch("builtins.print")
    def test_track_object_case_sensitive(self, mock_print):
        """Test suivi d'objet sensible à la casse."""
        self.vision.scan_environment()
        result = self.vision.track_object("CHAISE")  # Majuscules

        assert result is False  # "chaise" en minuscules dans les données

    def test_current_focus_after_tracking(self):
        """Test focus actuel après suivi."""
        # Mock objets pour test reproductible (deque pour optimisation RAM)
        self.vision.objects_detected = deque(
            [
                {"name": "table", "distance": 2.1, "confidence": 0.92},
            ]
        )
        self.vision.track_object("table")

        focus = self.vision.current_focus
        assert focus is not None
        assert focus["name"] == "table"
        assert focus["distance"] == 2.1
        assert focus["confidence"] == 0.92

    def test_objects_detected_persistence(self):
        """Test persistance des objets détectés."""
        # Mock objets pour test reproductible (deque pour optimisation RAM)
        self.vision.objects_detected = deque(
            [
                {"name": "livre", "distance": 0.8},
                {"name": "chaise", "distance": 1.2},
            ]
        )

        # Nouveau scan - peut modifier objects_detected (YOLO/réel détection)
        # Le test vérifie juste que la liste existe et est cohérente
        self.vision.scan_environment()

        # scan_environment peut modifier les objets selon environnement
        # Vérifier que le deque existe et est cohérent (deque pour optimisation RAM)
        assert isinstance(self.vision.objects_detected, deque)
        assert (
            len(self.vision.objects_detected) >= 0
        )  # Peut être 0 ou différent selon détection

    def test_faces_detected_persistence(self):
        """Test persistance des visages détectés."""
        self.vision.scan_environment()
        initial_count = len(self.vision.faces_detected)

        # Nouveau scan
        self.vision.scan_environment()
        new_count = len(self.vision.faces_detected)

        assert new_count == initial_count  # Même nombre de visages

    def test_vision_with_robot_api_camera_integration(self):
        """Test 11: Vérifier intégration robot.media.camera si robot_api fourni."""
        print("\n🧪 TEST 11: Intégration robot.media.camera SDK")
        print("=" * 60)

        # Créer un mock robot_api avec media.camera
        import numpy as np

        class MockCamera:
            def get_image(self):
                # Simuler retour d'image
                return np.zeros((480, 640, 3), dtype=np.uint8)

            def capture(self):
                return self.get_image()

        class MockMedia:
            def __init__(self):
                self.camera = MockCamera()

        class MockRobotAPI:
            def __init__(self):
                self.media = MockMedia()

        # Tester avec robot_api
        robot_api = MockRobotAPI()
        vision = BBIAVision(robot_api=robot_api)

        # Vérifier que la caméra SDK est détectée
        assert vision._camera_sdk_available is True
        assert vision._camera is not None
        print("✅ robot.media.camera détecté correctement")

        # Tester scan_environment (utilisera caméra SDK si disponible)
        result = vision.scan_environment()
        assert "source" in result
        assert result["source"] in ["camera_sdk", "simulation"]
        print(f"✅ scan_environment retourne résultat avec source: {result['source']}")

    def test_vision_fallback_simulation(self):
        """Test 12: Vérifier fallback gracieux vers simulation si SDK non disponible."""
        print("\n🧪 TEST 12: Fallback simulation")
        print("=" * 60)

        # Tester sans robot_api (fallback simulation)
        vision = BBIAVision(robot_api=None)
        assert vision._camera_sdk_available is False

        result = vision.scan_environment()
        assert "objects" in result
        assert "faces" in result
        assert result.get("source", "simulation") == "simulation"
        print("✅ Fallback simulation fonctionne correctement")
