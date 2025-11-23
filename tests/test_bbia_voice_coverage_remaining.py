#!/usr/bin/env python3
"""Tests pour améliorer la couverture de bbia_voice.py - Lignes manquantes.

Objectif : Couvrir les lignes non testées pour atteindre 70%+ de couverture.
Lignes manquantes identifiées :
- 162-163, 167-168, 172-173, 187-188 : Branches de sélection de voix (Aurelie/Amelie)
- 251-269 : Gestion erreurs SDK dans dire_texte (play_audio, speaker.play, etc.)
- 281-286, 288, 299 : Fallbacks sounddevice dans dire_texte
- 312-384 : Gestion SDK media dans dire_texte (play_audio, speaker.play_file, etc.)
- 425-476 : Gestion SDK microphone dans reconnaitre_parole
- 493-494, 498-504 : Gestion erreurs speech_recognition
- 521, 532-533 : Cas limites dans lister_voix_disponibles
- 551-575 : Thread worker transcription asynchrone
- 587-600, 607-624 : Start/stop transcription asynchrone
- 648-682, 692-759 : Transcription asynchrone et synchrone
- 782-849 : Fonction transcribe_audio (Whisper)
"""

import os
import queue
import tempfile
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

# Désactiver audio pour CI
os.environ["BBIA_DISABLE_AUDIO"] = "1"

import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim import bbia_voice


class TestBBIAVoiceCoverageRemaining(unittest.TestCase):
    """Tests pour couvrir les lignes manquantes de bbia_voice.py."""

    def setUp(self) -> None:
        """Setup test."""
        # Réinitialiser les caches
        bbia_voice._pyttsx3_engine_cache = None
        bbia_voice._bbia_voice_id_cache = None
        # Réinitialiser transcription asynchrone
        bbia_voice._transcribe_active = False
        bbia_voice._transcribe_thread = None
        bbia_voice._transcribe_queue = queue.Queue(maxsize=5)
        bbia_voice._last_transcribe_result = None

    def tearDown(self) -> None:
        """Cleanup."""
        # Arrêter transcription asynchrone si active
        if bbia_voice._transcribe_active:
            bbia_voice.stop_async_transcription()
        # Nettoyer les caches
        bbia_voice._pyttsx3_engine_cache = None
        bbia_voice._bbia_voice_id_cache = None

    # ========== Tests pour get_bbia_voice - Branches non couvertes ==========

    def test_get_bbia_voice_aurelie_enhanced_fr(self) -> None:
        """Test sélection Aurelie Enhanced fr."""
        voice = MagicMock()
        voice.name = "Aurélie"
        voice.id = "com.apple.speech.voice.Aurelie.enhanced.fr-FR"

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = [voice]

        result = bbia_voice.get_bbia_voice(mock_engine)
        self.assertEqual(result, "com.apple.speech.voice.Aurelie.enhanced.fr-FR")

    def test_get_bbia_voice_amelie_enhanced_fr(self) -> None:
        """Test sélection Amelie Enhanced fr."""
        voice = MagicMock()
        voice.name = "Amélie"
        voice.id = "com.apple.speech.voice.Amelie.enhanced.fr-FR"

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = [voice]

        result = bbia_voice.get_bbia_voice(mock_engine)
        self.assertEqual(result, "com.apple.speech.voice.Amelie.enhanced.fr-FR")

    def test_get_bbia_voice_aurelie_fr_CA(self) -> None:
        """Test sélection Aurelie fr-CA."""
        voice1 = MagicMock()
        voice1.name = "Aurélie"
        voice1.id = "com.apple.speech.voice.Aurelie.fr-CA"

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = [voice1]

        result = bbia_voice.get_bbia_voice(mock_engine)
        self.assertEqual(result, "com.apple.speech.voice.Aurelie.fr-CA")

    def test_get_bbia_voice_amelie_fr_CA(self) -> None:
        """Test sélection Amelie fr-CA."""
        voice1 = MagicMock()
        voice1.name = "Amélie"
        voice1.id = "com.apple.speech.voice.Amelie.fr-CA"

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = [voice1]

        result = bbia_voice.get_bbia_voice(mock_engine)
        self.assertEqual(result, "com.apple.speech.voice.Amelie.fr-CA")

    def test_get_bbia_voice_aurelie_any(self) -> None:
        """Test sélection toute Aurelie."""
        voice = MagicMock()
        voice.name = "Aurélie"
        voice.id = "com.apple.speech.voice.Aurelie"

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = [voice]

        result = bbia_voice.get_bbia_voice(mock_engine)
        self.assertEqual(result, "com.apple.speech.voice.Aurelie")

    def test_get_bbia_voice_amelie_any(self) -> None:
        """Test sélection toute Amelie."""
        voice = MagicMock()
        voice.name = "Amélie"
        voice.id = "com.apple.speech.voice.Amelie"

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = [voice]

        result = bbia_voice.get_bbia_voice(mock_engine)
        self.assertEqual(result, "com.apple.speech.voice.Amelie")

    # ========== Tests pour dire_texte - Gestion erreurs SDK ==========

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice._get_cached_voice_id")
    def test_dire_texte_sdk_play_audio_typeerror(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ) -> None:
        """Test dire_texte avec TypeError sur play_audio."""
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine
        mock_get_voice.return_value = "test_voice"

        mock_media = MagicMock()
        mock_media.play_audio.side_effect = [TypeError("test"), None]
        mock_robot_api = MagicMock()
        mock_robot_api.media = mock_media

        bbia_voice.dire_texte("Test", robot_api=mock_robot_api)

        # Vérifier que play_audio a été appelé deux fois (avec et sans volume)
        self.assertEqual(mock_media.play_audio.call_count, 2)

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice._get_cached_voice_id")
    def test_dire_texte_sdk_play_audio_exception(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ) -> None:
        """Test dire_texte avec Exception sur play_audio."""
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine
        mock_get_voice.return_value = "test_voice"

        mock_media = MagicMock()
        mock_media.play_audio.side_effect = Exception("test error")
        mock_speaker = MagicMock()
        mock_speaker.play.return_value = None
        mock_media.speaker = mock_speaker
        mock_robot_api = MagicMock()
        mock_robot_api.media = mock_media

        bbia_voice.dire_texte("Test", robot_api=mock_robot_api)

        # Vérifier que speaker.play a été appelé en fallback
        mock_speaker.play.assert_called_once()

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice._get_cached_voice_id")
    def test_dire_texte_sdk_speaker_play_error(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ) -> None:
        """Test dire_texte avec erreur sur speaker.play."""
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine
        mock_get_voice.return_value = "test_voice"

        mock_media = MagicMock()
        mock_media.play_audio.side_effect = Exception("test error")
        mock_speaker = MagicMock()
        mock_speaker.play.side_effect = Exception("speaker error")
        mock_speaker.play_file.return_value = None
        mock_media.speaker = mock_speaker
        mock_robot_api = MagicMock()
        mock_robot_api.media = mock_media

        bbia_voice.dire_texte("Test", robot_api=mock_robot_api)

        # Vérifier que play_file a été appelé en fallback
        mock_speaker.play_file.assert_called_once()

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice._get_cached_voice_id")
    def test_dire_texte_sdk_wav_creation_error(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ) -> None:
        """Test dire_texte avec erreur création WAV."""
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine
        mock_get_voice.return_value = "test_voice"

        mock_media = MagicMock()
        mock_media.play_audio.side_effect = Exception("test error")
        mock_speaker = MagicMock()
        mock_speaker.play.side_effect = Exception("speaker error")
        mock_speaker.play_file.side_effect = Exception("play_file error")
        mock_media.speaker = mock_speaker
        mock_robot_api = MagicMock()
        mock_robot_api.media = mock_media

        # Mock wave.open pour lever une exception
        with patch("wave.open", side_effect=ValueError("WAV error")):
            bbia_voice.dire_texte("Test", robot_api=mock_robot_api)

        # Devrait fallback vers pyttsx3
        mock_engine.say.assert_called_once()

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice._get_cached_voice_id")
    def test_dire_texte_sdk_cleanup_error(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ) -> None:
        """Test dire_texte avec erreur nettoyage fichier temporaire."""
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine
        mock_get_voice.return_value = "test_voice"

        mock_media = MagicMock()
        mock_media.play_audio.side_effect = Exception("test error")
        mock_speaker = MagicMock()
        mock_speaker.play.side_effect = Exception("speaker error")
        mock_speaker.play_file.return_value = None
        mock_media.speaker = mock_speaker
        mock_robot_api = MagicMock()
        mock_robot_api.media = mock_media

        # Mock Path.unlink pour lever une exception
        with patch("pathlib.Path.unlink", side_effect=OSError("cleanup error")):
            bbia_voice.dire_texte("Test", robot_api=mock_robot_api)

        # Devrait quand même fonctionner
        mock_speaker.play_file.assert_called_once()

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice._get_cached_voice_id")
    def test_dire_texte_sdk_media_exception(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ) -> None:
        """Test dire_texte avec Exception générale media."""
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine
        mock_get_voice.return_value = "test_voice"

        mock_robot_api = MagicMock()
        mock_media = MagicMock()
        mock_media.play_audio.side_effect = Exception("play_audio error")
        mock_speaker = MagicMock()
        mock_speaker.play.side_effect = Exception("play error")
        mock_speaker.play_file.side_effect = Exception("play_file error")
        mock_media.speaker = mock_speaker
        mock_robot_api.media = mock_media
        # Simuler une exception lors de la création du WAV (wave.open)
        with patch("wave.open", side_effect=Exception("wave error")):
            bbia_voice.dire_texte("Test", robot_api=mock_robot_api)

        # Devrait fallback vers pyttsx3
        mock_engine.say.assert_called_once()

    # ========== Tests pour dire_texte - TTS Backend ==========

    @patch("os.environ.get")
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice._get_cached_voice_id")
    def test_dire_texte_tts_backend_success(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ) -> None:
        """Test dire_texte avec TTS backend qui réussit."""

        def env_get_side_effect(key, default=None):
            if key == "BBIA_TTS_BACKEND":
                return "kittentts"
            elif key == "BBIA_DISABLE_AUDIO":
                return "0"
            return default

        mock_env_get.side_effect = env_get_side_effect

        mock_backend = MagicMock()
        mock_backend.synthesize_to_wav.return_value = True

        mock_media = MagicMock()
        mock_media.play_audio.return_value = None
        mock_robot_api = MagicMock()
        mock_robot_api.media = mock_media

        # Mock get_tts_backend dans le module
        with patch.object(bbia_voice, "get_tts_backend", return_value=mock_backend):
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                wav_path = tmp.name
                tmp.write(b"fake wav data")
            try:
                bbia_voice.dire_texte("Test", robot_api=mock_robot_api)
                mock_media.play_audio.assert_called_once()
            finally:
                if os.path.exists(wav_path):
                    os.unlink(wav_path)

    @patch("os.environ.get")
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice._get_cached_voice_id")
    def test_dire_texte_tts_backend_fallback_sounddevice(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ) -> None:
        """Test dire_texte avec fallback sounddevice."""

        def env_get_side_effect(key, default=None):
            if key == "BBIA_TTS_BACKEND":
                return "kittentts"
            elif key == "BBIA_DISABLE_AUDIO":
                return "0"
            return default

        mock_env_get.side_effect = env_get_side_effect

        mock_backend = MagicMock()
        mock_backend.synthesize_to_wav.return_value = True

        mock_robot_api = MagicMock()
        mock_robot_api.media = None

        with patch.object(bbia_voice, "get_tts_backend", return_value=mock_backend):
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                wav_path = tmp.name
                # Créer un vrai fichier WAV minimal
                import wave

                with wave.open(wav_path, "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(16000)
                    wf.writeframes(b"\x00\x00" * 100)
            try:
                with patch("sounddevice.play") as mock_play:
                    with patch("sounddevice.wait"):
                        bbia_voice.dire_texte("Test", robot_api=mock_robot_api)
                        # Vérifier que sounddevice.play a été appelé OU que pyttsx3 a été utilisé en fallback
                        if not mock_play.called:
                            # Si sounddevice n'a pas été appelé, pyttsx3 devrait l'être
                            mock_get_engine.assert_called()
            finally:
                if os.path.exists(wav_path):
                    os.unlink(wav_path)

    @patch("os.environ.get")
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice._get_cached_voice_id")
    def test_dire_texte_tts_backend_fallback_errors(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ) -> None:
        """Test dire_texte avec erreurs fallback."""

        def env_get_side_effect(key, default=None):
            if key == "BBIA_TTS_BACKEND":
                return "kittentts"
            elif key == "BBIA_DISABLE_AUDIO":
                return "0"
            return default

        mock_env_get.side_effect = env_get_side_effect

        mock_backend = MagicMock()
        mock_backend.synthesize_to_wav.return_value = True

        mock_robot_api = MagicMock()
        mock_robot_api.media = None

        with patch.object(bbia_voice, "get_tts_backend", return_value=mock_backend):
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                wav_path = tmp.name
                import wave

                with wave.open(wav_path, "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(16000)
                    wf.writeframes(b"\x00\x00" * 100)
            try:
                # Simuler ImportError pour sounddevice
                with patch(
                    "sounddevice.play", side_effect=ImportError("no sounddevice")
                ):
                    bbia_voice.dire_texte("Test", robot_api=mock_robot_api)
                # Devrait fallback vers pyttsx3
                mock_get_engine.assert_called()
            finally:
                if os.path.exists(wav_path):
                    os.unlink(wav_path)

    @patch("os.environ.get")
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice._get_cached_voice_id")
    def test_dire_texte_tts_backend_import_error(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ) -> None:
        """Test dire_texte avec ImportError TTS backend."""

        def env_get_side_effect(key, default=None):
            if key == "BBIA_TTS_BACKEND":
                return "kittentts"
            elif key == "BBIA_DISABLE_AUDIO":
                return "0"
            return default

        mock_env_get.side_effect = env_get_side_effect

        with patch.object(
            bbia_voice, "get_tts_backend", side_effect=ImportError("no backend")
        ):
            bbia_voice.dire_texte("Test", robot_api=None)
            # Devrait fallback vers pyttsx3
            mock_get_engine.assert_called()

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice._get_cached_voice_id")
    def test_dire_texte_pyttsx3_exception(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ) -> None:
        """Test dire_texte avec Exception pyttsx3."""
        mock_engine = MagicMock()
        mock_engine.say.side_effect = Exception("pyttsx3 error")
        mock_get_engine.return_value = mock_engine
        mock_get_voice.return_value = "test_voice"

        with self.assertRaises(Exception):
            bbia_voice.dire_texte("Test", robot_api=None)

    # ========== Tests pour reconnaitre_parole - SDK ==========

    @patch("bbia_sim.bbia_voice.sr.Recognizer")
    @patch("bbia_sim.bbia_voice.sr.AudioFile")
    def test_reconnaitre_parole_sdk_record_audio(
        self, mock_audio_file, mock_recognizer
    ) -> None:
        """Test reconnaitre_parole avec SDK record_audio."""
        mock_robot_api = MagicMock()
        mock_media = MagicMock()
        mock_media.record_audio.return_value = b"fake audio data"
        mock_robot_api.media = mock_media

        mock_rec = MagicMock()
        mock_recognizer.return_value = mock_rec
        mock_audio = MagicMock()
        mock_rec.record.return_value = mock_audio
        mock_rec.recognize_google.return_value = "texte reconnu"

        result = bbia_voice.reconnaitre_parole(duree=1, robot_api=mock_robot_api)
        self.assertEqual(result, "texte reconnu")

    @patch("bbia_sim.bbia_voice.sr.Recognizer")
    @patch("bbia_sim.bbia_voice.sr.AudioFile")
    def test_reconnaitre_parole_sdk_audio_ndarray(
        self, mock_audio_file, mock_recognizer
    ) -> None:
        """Test reconnaitre_parole avec numpy array."""
        mock_robot_api = MagicMock()
        mock_media = MagicMock()
        audio_array = np.array([1, 2, 3, 4], dtype=np.int16)
        mock_media.record_audio.return_value = audio_array
        mock_robot_api.media = mock_media

        mock_rec = MagicMock()
        mock_recognizer.return_value = mock_rec
        mock_audio = MagicMock()
        mock_rec.record.return_value = mock_audio
        mock_rec.recognize_google.return_value = "texte reconnu"

        result = bbia_voice.reconnaitre_parole(duree=1, robot_api=mock_robot_api)
        self.assertEqual(result, "texte reconnu")

    @patch("bbia_sim.bbia_voice.sr.Recognizer")
    @patch("bbia_sim.bbia_voice.sr.AudioFile")
    def test_reconnaitre_parole_sdk_exception(
        self, mock_audio_file, mock_recognizer
    ) -> None:
        """Test reconnaitre_parole avec Exception SDK."""
        mock_robot_api = MagicMock()
        mock_media = MagicMock()
        mock_media.record_audio.side_effect = Exception("SDK error")
        mock_robot_api.media = mock_media

        mock_rec = MagicMock()
        mock_recognizer.return_value = mock_rec
        mock_mic = MagicMock()
        mock_mic.__enter__.return_value = MagicMock()
        with patch("bbia_sim.bbia_voice.sr.Microphone", return_value=mock_mic):
            mock_audio = MagicMock()
            mock_rec.listen.return_value = mock_audio
            mock_rec.recognize_google.return_value = "fallback texte"

            result = bbia_voice.reconnaitre_parole(duree=1, robot_api=mock_robot_api)
            self.assertEqual(result, "fallback texte")

    @patch("bbia_sim.bbia_voice.sr.Recognizer")
    @patch("bbia_sim.bbia_voice.sr.Microphone")
    def test_reconnaitre_parole_unknown_value_error(
        self, mock_microphone, mock_recognizer
    ) -> None:
        """Test reconnaitre_parole avec UnknownValueError."""
        mock_rec = MagicMock()
        mock_recognizer.return_value = mock_rec
        mock_mic = MagicMock()
        mock_mic.__enter__.return_value = MagicMock()
        mock_microphone.return_value = mock_mic
        mock_audio = MagicMock()
        mock_rec.listen.return_value = mock_audio

        import speech_recognition as sr

        mock_rec.recognize_google.side_effect = sr.UnknownValueError()

        result = bbia_voice.reconnaitre_parole(duree=1, robot_api=None)
        self.assertIsNone(result)

    @patch("bbia_sim.bbia_voice.sr.Recognizer")
    @patch("bbia_sim.bbia_voice.sr.Microphone")
    def test_reconnaitre_parole_microphone_exception(
        self, mock_microphone, mock_recognizer
    ) -> None:
        """Test reconnaitre_parole avec Exception microphone."""
        mock_microphone.side_effect = Exception("microphone error")

        result = bbia_voice.reconnaitre_parole(duree=1, robot_api=None)
        self.assertIsNone(result)

    # ========== Tests pour lister_voix_disponibles ==========

    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    def test_lister_voix_languages_decode_error(self, mock_get_engine) -> None:
        """Test lister_voix_disponibles avec erreur decode."""
        mock_engine = MagicMock()
        mock_voice = MagicMock()
        # Créer un mock pour languages[0] qui lève une exception lors de decode
        mock_lang = MagicMock()
        mock_lang.decode.side_effect = UnicodeDecodeError(
            "utf-8", b"invalid\xff", 0, 1, "invalid"
        )
        mock_voice.languages = [mock_lang]
        mock_engine.getProperty.return_value = [mock_voice]
        mock_get_engine.return_value = mock_engine

        result = bbia_voice.lister_voix_disponibles()
        self.assertEqual(len(result), 1)

    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    def test_lister_voix_languages_exception(self, mock_get_engine) -> None:
        """Test lister_voix_disponibles avec Exception."""
        mock_engine = MagicMock()
        mock_voice = MagicMock()
        mock_voice.languages = None
        mock_engine.getProperty.return_value = [mock_voice]
        mock_get_engine.return_value = mock_engine

        result = bbia_voice.lister_voix_disponibles()
        self.assertEqual(len(result), 1)

    # ========== Tests pour transcription asynchrone ==========

    def test_start_async_transcription(self) -> None:
        """Test start_async_transcription."""
        result = bbia_voice.start_async_transcription()
        self.assertTrue(result)
        self.assertTrue(bbia_voice._transcribe_active)

        # Déjà actif
        result2 = bbia_voice.start_async_transcription()
        self.assertTrue(result2)

        # Nettoyer
        bbia_voice.stop_async_transcription()

    def test_stop_async_transcription(self) -> None:
        """Test stop_async_transcription."""
        # Démarrer d'abord
        bbia_voice.start_async_transcription()
        time.sleep(0.1)  # Laisser le thread démarrer

        # Arrêter
        bbia_voice.stop_async_transcription()
        self.assertFalse(bbia_voice._transcribe_active)

        # Arrêter quand déjà arrêté (ne devrait rien faire)
        bbia_voice.stop_async_transcription()

    @patch("bbia_sim.bbia_voice.transcribe_audio")
    def test_transcribe_audio_async_timeout(self, mock_transcribe) -> None:
        """Test transcribe_audio_async avec timeout."""
        mock_transcribe.return_value = None  # Simule une transcription lente

        bbia_voice.start_async_transcription()
        time.sleep(0.1)

        # Tester avec timeout court
        result = bbia_voice.transcribe_audio_async(
            b"fake audio", sample_rate=16000, timeout=0.01
        )
        # Devrait retourner None si timeout
        self.assertIsNone(result)

        bbia_voice.stop_async_transcription()

    # ========== Tests pour transcribe_audio (Whisper) ==========

    @patch("os.environ.get", return_value="1")
    def test_transcribe_audio_disabled(self, mock_env_get) -> None:
        """Test transcribe_audio avec audio désactivé."""
        result = bbia_voice.transcribe_audio(b"fake audio")
        self.assertIsNone(result)

    @patch("os.environ.get", return_value="0")
    def test_transcribe_audio_whisper_not_available(self, mock_env_get) -> None:
        """Test transcribe_audio avec Whisper non disponible."""
        # Mock directement dans le module bbia_voice
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False):
            result = bbia_voice.transcribe_audio(b"fake audio")
            self.assertIsNone(result)

    @patch("os.environ.get", return_value="0")
    def test_transcribe_audio_whisper_import_error(self, mock_env_get) -> None:
        """Test transcribe_audio avec ImportError."""
        # Mock l'import pour lever ImportError lors de l'import dans transcribe_audio
        original_import = __import__

        def mock_import(name, *args, **kwargs):
            if "voice_whisper" in name:
                raise ImportError("no whisper")
            return original_import(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=mock_import):
            result = bbia_voice.transcribe_audio(b"fake audio")
            self.assertIsNone(result)

    @patch("os.environ.get", return_value="0")
    def test_transcribe_audio_whisper_load_failure(self, mock_env_get) -> None:
        """Test transcribe_audio avec échec chargement modèle."""
        mock_stt = MagicMock()
        mock_stt.is_loaded = False
        mock_stt.load_model.return_value = False

        # Mock WhisperSTT directement
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            with patch("bbia_sim.voice_whisper.WhisperSTT", return_value=mock_stt):
                with patch("soundfile.write"):
                    result = bbia_voice.transcribe_audio(b"fake audio")
                    self.assertIsNone(result)

    @patch("os.environ.get", return_value="0")
    def test_transcribe_audio_whisper_exception(self, mock_env_get) -> None:
        """Test transcribe_audio avec Exception générale."""
        mock_stt = MagicMock()
        mock_stt.is_loaded = True
        mock_stt.transcribe_audio.side_effect = Exception("whisper error")

        # Mock WhisperSTT directement
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            with patch("bbia_sim.voice_whisper.WhisperSTT", return_value=mock_stt):
                # Mock soundfile pour éviter les erreurs
                with patch("soundfile.write"):
                    result = bbia_voice.transcribe_audio(b"fake audio")
                    # Devrait gérer l'exception et retourner None
                    self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
