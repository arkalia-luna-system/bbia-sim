import unittest
from unittest.mock import patch, MagicMock
import src.bbia_sim.bbia_voice as bbia_voice

class TestBBIAVoice(unittest.TestCase):
    @patch('src.bbia_sim.bbia_voice.pyttsx3.init')
    def test_dire_texte(self, mock_init):
        mock_engine = MagicMock()
        mock_init.return_value = mock_engine
        bbia_voice.dire_texte('Bonjour')
        mock_engine.say.assert_called_with('Bonjour')
        mock_engine.runAndWait.assert_called()

    @patch('src.bbia_sim.bbia_voice.sr.Recognizer')
    @patch('src.bbia_sim.bbia_voice.sr.Microphone')
    def test_reconnaitre_parole(self, mock_microphone, mock_recognizer):
        mock_rec = MagicMock()
        mock_recognizer.return_value = mock_rec
        mock_audio = MagicMock()
        mock_rec.listen.return_value = mock_audio
        mock_rec.recognize_google.return_value = 'test'
        mock_microphone.return_value.__enter__.return_value = MagicMock()
        result = bbia_voice.reconnaitre_parole(duree=1, frequence=16000)
        self.assertEqual(result, 'test')
        # Cas : aucune parole reconnue
        mock_rec.recognize_google.side_effect = Exception('Erreur')
        result = bbia_voice.reconnaitre_parole(duree=1, frequence=16000)
        self.assertIsNone(result)

if __name__ == '__main__':
    unittest.main() 