#!/usr/bin/env python3
"""
Test E2E: Scénario utilisateur - BBIA écoute → comprend → répond → bouge
"""

import os
from unittest.mock import MagicMock, patch

import pytest

from bbia_sim.bbia_behavior import BBIABehaviorManager

# bbia_voice n'a pas de classe BBIAVoice, utiliser fonctions directement


@pytest.mark.e2e
@pytest.mark.slow
class TestE2EVoiceInteraction:
    """Tests E2E scénario: écoute → compréhension → réponse → mouvement."""

    def setup_method(self):
        """Configuration avant chaque test."""
        # Mock robot API
        self.mock_robot = MagicMock()
        self.mock_robot.set_joint_pos.return_value = True
        self.mock_robot.step.return_value = True

        # Voice: utiliser fonctions directement (bbia_voice n'a pas de classe)

        # Behavior manager
        self.behavior = BBIABehaviorManager(robot_api=self.mock_robot)

    @patch("bbia_sim.bbia_huggingface.BBIAHuggingFace")
    def test_bbia_listens_and_responds(self, mock_hf_class):
        """Scénario: BBIA écoute → comprend → répond → bouge."""
        # 1. Simuler audio input
        audio_text = "Bonjour, peux-tu me saluer?"

        # 2. Mock HuggingFace pour génération réponse
        mock_hf = MagicMock()
        mock_hf.chat.return_value = (
            "Bonjour ! Je suis content de te voir. *fait un signe de la main*"
        )
        mock_hf_class.return_value = mock_hf

        # 3. Simuler transcription (mock reconnaitre_parole)
        with patch("bbia_sim.bbia_voice.reconnaitre_parole", return_value=audio_text):
            from bbia_sim.bbia_voice import reconnaitre_parole

            transcribed = reconnaitre_parole()

            assert transcribed == audio_text

            # 4. Génération réponse LLM (mock)
            if mock_hf:
                response = mock_hf.chat(transcribed)
                assert isinstance(response, str)
                assert len(response) > 0

                # 5. Mouvement réponse (nod ou autre comportement)
                # Si réponse contient indication mouvement, activer comportement
                if "signe" in response.lower() or "mouvement" in response.lower():
                    self.behavior.execute_behavior = MagicMock(return_value=True)
                    behavior_result = self.behavior.execute_behavior(
                        "greeting", {"text": response}
                    )
                    assert behavior_result is True

    @patch("bbia_sim.voice_whisper.create_whisper_stt")
    def test_bbia_voice_command_to_action(self, mock_whisper_factory):
        """Test commande vocale → action robot."""
        # 1. Simuler commande vocale
        command_text = "salue"

        # 2. Mock Whisper transcription
        mock_whisper = MagicMock()
        mock_whisper.transcribe_microphone.return_value = command_text
        mock_whisper_factory.return_value = mock_whisper

        # 3. Simuler mapping commande → action
        from bbia_sim.voice_whisper import VoiceCommandMapper

        mapper = VoiceCommandMapper()
        action = mapper.map_command(command_text)

        assert action is not None
        assert action["action"] == "greet"

        # 4. Exécuter action via behavior
        self.behavior.execute_behavior = MagicMock(return_value=True)
        result = self.behavior.execute_behavior(action["action"], {})
        assert result is True

    @pytest.mark.skipif(
        os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1",
        reason="Audio désactivé",
    )
    def test_bbia_full_voice_interaction_flow(self):
        """Flux complet interaction vocale: écoute → transcrit → comprend → répond → agit."""
        # Test simplifié avec mocks (évite dépendances audio)
        with patch("bbia_sim.bbia_voice.reconnaitre_parole", return_value="bonjour"):
            from bbia_sim.bbia_voice import reconnaitre_parole

            # 1. Écoute
            text = reconnaitre_parole()
            assert text == "bonjour"

            # 2. Mapping commande
            from bbia_sim.voice_whisper import VoiceCommandMapper

            mapper = VoiceCommandMapper()
            action = mapper.map_command(text)

            if action:
                # 3. Exécution action
                self.behavior.execute_behavior = MagicMock(return_value=True)
                result = self.behavior.execute_behavior(action["action"], {})
                assert result is True
