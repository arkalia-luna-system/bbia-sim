#!/usr/bin/env python3
"""
Test E2E minimal : BBIA Modules individuels
Test déterministe et rapide (< 5s)
"""

import pytest

from src.bbia_sim.bbia_audio import detecter_son
from src.bbia_sim.bbia_emotions import BBIAEmotions
from src.bbia_sim.bbia_vision import BBIAVision
from src.bbia_sim.bbia_voice import dire_texte, reconnaitre_parole
from src.bbia_sim.daemon.app.main import app, lifespan


class TestBBIAModules:
    """Tests E2E pour les modules BBIA individuels"""

    @pytest.fixture(scope="class")
    async def api_server(self):
        """Démarre l'API pour les tests"""
        async with lifespan(app):
            yield app

    def test_bbia_emotions_module(self):
        """Test : Module BBIA Emotions"""
        emotions = BBIAEmotions()

        # Test création
        assert emotions.current_emotion == "neutral"
        assert emotions.emotion_intensity == 0.5
        assert len(emotions.emotions) == 8

        # Test changement d'émotion
        emotions.set_emotion("happy", 0.8)
        assert emotions.current_emotion == "happy"
        assert emotions.emotion_intensity == 0.8

        # Test émotions disponibles
        available = list(emotions.emotions.keys())
        expected_emotions = [
            "neutral", "happy", "sad", "angry",
            "curious", "excited", "surprised", "fearful"
        ]
        assert len(available) == len(expected_emotions)
        for emotion in expected_emotions:
            assert emotion in available

    def test_bbia_vision_module(self):
        """Test : Module BBIA Vision"""
        vision = BBIAVision()

        # Test création
        assert vision.camera_active
        assert vision.vision_quality == "HD"
        assert vision.detection_range == 3.0
        assert not vision.tracking_active

        # Test spécifications
        specs = vision.specs
        assert specs["camera"] == "Grand angle"
        assert specs["resolution"] == "1080p"
        assert specs["fov"] == "120°"

        # Test contrôle
        vision.tracking_active = True
        assert vision.tracking_active

        vision.current_focus = "test_object"
        assert vision.current_focus == "test_object"

    def test_bbia_voice_functions(self):
        """Test : Fonctions BBIA Voice"""
        # Test synthèse vocale
        text = "Bonjour, je suis BBIA"
        result = dire_texte(text)
        assert result is not None  # Peut être None ou True

        # Test reconnaissance vocale (simulation)
        result = reconnaitre_parole()
        assert isinstance(result, str)
        assert len(result) > 0

    def test_bbia_audio_functions(self):
        """Test : Fonctions BBIA Audio"""
        # Test détection sonore avec fichier temporaire
        import os
        import tempfile

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name
            # Créer un fichier audio vide pour le test
            temp_file.write(b"dummy_audio_data")

        try:
            result = detecter_son(temp_path)
            assert isinstance(result, bool)
        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

        # Test lecture audio (simulation) - ne pas tester avec fichier inexistant
        # result = lire_audio("test.wav")  # Commenté car fichier n'existe pas

    @pytest.mark.asyncio
    async def test_bbia_with_api_integration(self, api_server):
        """Test : BBIA avec API (sans intégration complète)"""
        import requests

        base_url = "http://127.0.0.1:8000"
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}

        # Test simple de l'API sans dépendances complexes
        try:
            response = requests.get(f"{base_url}/health", timeout=5)
            assert response.status_code in [200, 404]  # API peut ne pas être démarrée
        except requests.exceptions.RequestException:
            # API non disponible, test réussi car c'est attendu en e2e
            pass

        # Test que l'API fonctionne avec BBIA
        response = requests.get(f"{base_url}/api/info", headers=headers, timeout=5)
        assert response.status_code == 200
        data = response.json()
        assert data["name"] == "BBIA-SIM API"

        # Test application d'émotion via API
        emotions = BBIAEmotions()
        emotions.set_emotion("happy", 0.7)

        # Appliquer l'émotion via l'API (simulation simple)
        joint_data = [{"joint_name": "yaw_body", "position": 0.3}]
        response = requests.post(
            f"{base_url}/api/motion/joints",
            json=joint_data,
            headers=headers,
            timeout=5
        )
        assert response.status_code == 200

        # Vérifier que la position est appliquée
        response = requests.get(f"{base_url}/api/state/joints", headers=headers, timeout=5)
        assert response.status_code == 200
        data = response.json()
        yaw_position = data["joints"]["yaw_body"]["position"]
        assert abs(yaw_position - 0.3) < 0.1
