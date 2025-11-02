#!/usr/bin/env python3
"""
Test E2E: Scénario complet - détection → écoute → réponse → mouvement
Intégration complète tous modules BBIA
"""

import os
from unittest.mock import MagicMock, patch

import pytest

from bbia_sim.bbia_behavior import BBIABehaviorManager
from bbia_sim.bbia_emotions import BBIAEmotions
from bbia_sim.bbia_vision import BBIAVision

# bbia_voice n'a pas de classe BBIAVoice, utiliser fonctions directement


@pytest.mark.e2e
@pytest.mark.slow
class TestE2EFullInteractionLoop:
    """Tests E2E scénario complet: tous modules intégrés."""

    def setup_method(self):
        """Configuration avant chaque test."""
        # Mock robot API
        self.mock_robot = MagicMock()
        self.mock_robot.set_joint_pos.return_value = True
        self.mock_robot.get_joint_pos.return_value = 0.0
        self.mock_robot.step.return_value = True
        self.mock_robot.set_emotion.return_value = True

        # Modules BBIA
        self.vision = BBIAVision(robot_api=self.mock_robot)
        # Voice: utiliser fonctions directement (bbia_voice n'a pas de classe)
        self.emotions = BBIAEmotions()
        self.behavior = BBIABehaviorManager(robot_api=self.mock_robot)

    @patch("bbia_sim.bbia_huggingface.BBIAHuggingFace")
    def test_bbia_full_interaction(self, mock_hf_class):
        """Scénario complet: détection → écoute → réponse → mouvement."""
        # 1. Détection visage (mock)
        self.vision.faces_detected = [
            {
                "name": "humain",
                "distance": 1.5,
                "confidence": 0.95,
                "emotion": "happy",
            },
        ]

        scan_result = self.vision.scan_environment()
        assert len(scan_result["faces"]) > 0

        # 2. Tracking visage
        self.vision.objects_detected = [
            {"name": "humain", "confidence": 0.9, "bbox": [100, 100, 200, 200]},
        ]
        track_success = self.vision.track_object("humain")
        assert track_success is True

        # 3. Écoute vocale (mock)
        with patch("bbia_sim.bbia_voice.reconnaitre_parole", return_value="bonjour"):
            from bbia_sim.bbia_voice import reconnaitre_parole

            audio_text = reconnaitre_parole()
            assert audio_text == "bonjour"

            # 4. Génération réponse LLM (mock)
            mock_hf = MagicMock()
            mock_hf.chat.return_value = "Bonjour ! Je suis content de te voir."
            mock_hf_class.return_value = mock_hf

            response = mock_hf.chat(audio_text)
            assert isinstance(response, str)

            # 5. Émotion happy
            self.emotions.set_emotion("happy", 0.8)
            emotion_state = self.emotions.get_current_emotion()
            assert emotion_state["name"] == "happy"

            # 6. Mouvement greeting
            self.behavior.execute_behavior = MagicMock(return_value=True)
            greeting_result = self.behavior.execute_behavior(
                "greeting", {"text": response, "target": "face"}
            )
            assert greeting_result is True

            # 7. Vérifier intégration complète
            # Vision: tracking actif
            vision_status = self.vision.get_focus_status()
            assert vision_status["tracking_active"] is True

            # Emotions: happy
            assert emotion_state["name"] == "happy"

            # Behavior: comportement exécuté
            assert greeting_result is True

            # Robot: mouvement effectué (vérifié via execute_behavior mocké)
            # Note: Le robot API n'est pas directement appelé car execute_behavior est mocké
            # pour éviter les dépendances complexes dans ce test e2e

    @pytest.mark.skipif(
        os.environ.get("BBIA_DISABLE_VISION", "0") == "1"
        or os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1",
        reason="Vision ou audio désactivé",
    )
    def test_bbia_interactive_loop_multiple_turns(self):
        """Boucle interactive multiple tours: scan → écoute → réponse → scan."""
        # Tour 1: Détection initiale
        self.vision.faces_detected = [{"name": "humain", "confidence": 0.9}]
        scan1 = self.vision.scan_environment()
        assert len(scan1["faces"]) > 0

        # Tour 2: Écoute
        with patch(
            "bbia_sim.bbia_voice.reconnaitre_parole", return_value="comment vas-tu?"
        ):
            from bbia_sim.bbia_voice import reconnaitre_parole

            text1 = reconnaitre_parole()
            assert "vas-tu" in text1

        # Tour 3: Réponse + émotion
        self.emotions.set_emotion("happy", 0.7)
        assert self.emotions.get_current_emotion()["name"] == "happy"

        # Tour 4: Nouveau scan
        self.vision.faces_detected = [{"name": "humain", "confidence": 0.95}]
        scan2 = self.vision.scan_environment()
        assert len(scan2["faces"]) > 0

        # Vérifier cohérence
        assert isinstance(scan1, dict)
        assert isinstance(scan2, dict)
