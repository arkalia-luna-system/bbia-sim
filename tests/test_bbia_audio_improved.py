#!/usr/bin/env python3
"""Tests améliorés pour BBIA Audio - Coverage 70%+."""

import os
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np


# Désactiver audio pour CI
os.environ["BBIA_DISABLE_AUDIO"] = "1"

import sys


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim import bbia_audio


class TestBBIAAudioImproved(unittest.TestCase):
    """Tests améliorés pour bbia_audio avec coverage élevé."""

    def setUp(self) -> None:
        """Setup test."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.temp_dir, "test.wav")

    def tearDown(self) -> None:
        """Cleanup."""
        import shutil

        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    @patch("os.environ.get", return_value="1")
    def test_enregistrer_audio_disabled(self, mock_env: MagicMock) -> None:
        """Test enregistrement avec audio désactivé."""
        result = bbia_audio.enregistrer_audio(self.test_file, duree=1)
        self.assertFalse(result)

    @patch("os.environ.get", return_value="1")
    def test_lire_audio_disabled(self, mock_env: MagicMock) -> None:
        """Test lecture avec audio désactivé."""
        # Ne devrait rien faire
        bbia_audio.lire_audio(self.test_file)

    @patch("os.environ.get", return_value="1")
    def test_detecter_son_disabled(self, mock_env: MagicMock) -> None:
        """Test détection avec audio désactivé."""
        result = bbia_audio.detecter_son(self.test_file)
        self.assertFalse(result)

    def test_is_safe_path_valid(self) -> None:
        """Test validation chemin valide."""
        self.assertTrue(bbia_audio._is_safe_path("test.wav"))
        self.assertTrue(bbia_audio._is_safe_path("./test.wav"))
        self.assertTrue(bbia_audio._is_safe_path(os.path.join(self.temp_dir, "test.wav")))

    def test_is_safe_path_invalid(self) -> None:
        """Test validation chemin invalide (path traversal)."""
        self.assertFalse(bbia_audio._is_safe_path("../etc/passwd"))
        self.assertFalse(bbia_audio._is_safe_path("../../secret.txt"))
        self.assertFalse(bbia_audio._is_safe_path("/etc/passwd"))

    def test_get_robot_media_microphone_with_api(self) -> None:
        """Test récupération microphone SDK."""
        mock_robot = MagicMock()
        mock_robot.media.microphone = MagicMock()
        result = bbia_audio._get_robot_media_microphone(mock_robot)
        self.assertIsNotNone(result)

    def test_get_robot_media_microphone_no_api(self) -> None:
        """Test récupération microphone sans API."""
        result = bbia_audio._get_robot_media_microphone(None)
        self.assertIsNone(result)

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_audio._get_sd")
    @patch("bbia_sim.bbia_audio.wave.open")
    def test_enregistrer_audio_sounddevice(
        self, mock_wave: MagicMock, mock_get_sd: MagicMock, mock_env: MagicMock
    ) -> None:
        """Test enregistrement via sounddevice."""
        mock_sd = MagicMock()
        mock_sd.rec.return_value = np.zeros((16000, 1), dtype="int16")
        mock_get_sd.return_value = mock_sd

        mock_wf = MagicMock()
        mock_wave.return_value.__enter__.return_value = mock_wf

        result = bbia_audio.enregistrer_audio(self.test_file, duree=1, frequence=16000)
        self.assertTrue(result)
        mock_sd.rec.assert_called_once()
        mock_sd.wait.assert_called_once()

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_audio._get_sd")
    def test_enregistrer_audio_sounddevice_error(
        self, mock_get_sd: MagicMock, mock_env: MagicMock
    ) -> None:
        """Test erreur enregistrement sounddevice."""
        mock_get_sd.return_value = None
        with self.assertRaises(RuntimeError):
            bbia_audio.enregistrer_audio(self.test_file, duree=1)

    @patch("os.environ.get", return_value="0")
    def test_detecter_son_with_high_amplitude(self, mock_env: MagicMock) -> None:
        """Test détection son avec amplitude élevée."""
        # Créer fichier WAV test
        import wave

        with wave.open(self.test_file, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            # Audio avec amplitude élevée
            audio_data = (np.ones(16000, dtype=np.int16) * 1000).tobytes()
            wf.writeframes(audio_data)

        result = bbia_audio.detecter_son(self.test_file, seuil=500)
        self.assertTrue(result)

    @patch("os.environ.get", return_value="0")
    def test_detecter_son_with_low_amplitude(self, mock_env: MagicMock) -> None:
        """Test détection son avec amplitude faible."""
        import wave

        with wave.open(self.test_file, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            # Audio avec amplitude faible
            audio_data = (np.zeros(16000, dtype=np.int16)).tobytes()
            wf.writeframes(audio_data)

        result = bbia_audio.detecter_son(self.test_file, seuil=500)
        self.assertFalse(result)

    @patch("os.environ.get", return_value="0")
    def test_detecter_son_file_error(self, mock_env: MagicMock) -> None:
        """Test détection son avec fichier inexistant."""
        result = bbia_audio.detecter_son("nonexistent.wav")
        self.assertFalse(result)

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_audio.soundfile")
    @patch("bbia_sim.bbia_audio._get_sd")
    @patch("bbia_sim.bbia_audio.wave.open")
    def test_lire_audio_with_soundfile_info(
        self,
        mock_wave: MagicMock,
        mock_get_sd: MagicMock,
        mock_soundfile: MagicMock,
        mock_env: MagicMock,
    ) -> None:
        """Test lecture avec soundfile info."""
        mock_info = MagicMock()
        mock_info.samplerate = 16000
        mock_soundfile.info.return_value = mock_info

        mock_wf = MagicMock()
        mock_wf.getframerate.return_value = 16000
        mock_wf.readframes.return_value = np.zeros(16000, dtype="int16").tobytes()
        mock_wf.getnframes.return_value = 16000
        mock_wave.return_value.__enter__.return_value = mock_wf

        mock_sd = MagicMock()
        mock_get_sd.return_value = mock_sd

        bbia_audio.lire_audio(self.test_file)
        mock_sd.play.assert_called_once()

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_audio.soundfile", None)
    @patch("bbia_sim.bbia_audio._get_sd")
    @patch("bbia_sim.bbia_audio.wave.open")
    def test_lire_audio_fallback_wave(
        self,
        mock_wave: MagicMock,
        mock_get_sd: MagicMock,
        mock_env: MagicMock,
    ) -> None:
        """Test lecture fallback vers wave."""
        mock_wf = MagicMock()
        mock_wf.getframerate.return_value = 16000
        mock_wf.readframes.return_value = np.zeros(16000, dtype="int16").tobytes()
        mock_wf.getnframes.return_value = 16000
        mock_wave.return_value.__enter__.return_value = mock_wf

        mock_sd = MagicMock()
        mock_get_sd.return_value = mock_sd

        bbia_audio.lire_audio(self.test_file)
        mock_sd.play.assert_called_once()

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_audio._get_sd")
    def test_lire_audio_sounddevice_error(
        self, mock_get_sd: MagicMock, mock_env: MagicMock
    ) -> None:
        """Test erreur lecture sounddevice."""
        import wave

        # Créer fichier WAV valide
        with wave.open(self.test_file, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            wf.writeframes(np.zeros(16000, dtype=np.int16).tobytes())

        mock_get_sd.return_value = None
        with self.assertRaises(RuntimeError):
            bbia_audio.lire_audio(self.test_file)


if __name__ == "__main__":
    unittest.main()
