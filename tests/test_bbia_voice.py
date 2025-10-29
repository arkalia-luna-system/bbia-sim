import unittest
from unittest.mock import MagicMock, patch

import pytest

from src.bbia_sim import bbia_voice


class TestBBIAVoice(unittest.TestCase):
    @pytest.mark.audio
    @patch("src.bbia_sim.bbia_voice._pyttsx3_engine_cache", None)
    @patch("src.bbia_sim.bbia_voice.get_bbia_voice")
    @patch("src.bbia_sim.bbia_voice._get_pyttsx3_engine")
    def test_dire_texte(self, mock_get_engine, mock_get_voice):
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine
        mock_get_voice.return_value = "test_voice_id"

        bbia_voice.dire_texte("Bonjour")

        mock_get_engine.assert_called()
        mock_get_voice.assert_called()
        mock_engine.setProperty.assert_called()
        mock_engine.say.assert_called_with("Bonjour")
        mock_engine.runAndWait.assert_called()

    @patch("src.bbia_sim.bbia_voice.sr.Recognizer")
    @patch("src.bbia_sim.bbia_voice.sr.Microphone")
    def test_reconnaitre_parole(self, mock_microphone, mock_recognizer):
        mock_rec = MagicMock()
        mock_recognizer.return_value = mock_rec
        mock_audio = MagicMock()
        mock_rec.listen.return_value = mock_audio
        mock_rec.recognize_google.return_value = "test"
        mock_microphone.return_value.__enter__.return_value = MagicMock()
        result = bbia_voice.reconnaitre_parole(duree=1, frequence=16000)
        self.assertEqual(result, "test")
        # Cas : aucune parole reconnue
        mock_rec.recognize_google.side_effect = Exception("Erreur")
        result = bbia_voice.reconnaitre_parole(duree=1, frequence=16000)
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
