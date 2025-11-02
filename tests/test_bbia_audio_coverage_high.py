#!/usr/bin/env python3
"""Tests coverage élevé pour bbia_audio - Objectif 70%+."""

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


class TestBBIAAudioCoverageHigh(unittest.TestCase):
    """Tests pour atteindre coverage 70%+ bbia_audio."""

    def setUp(self) -> None:
        """Setup test."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.temp_dir, "test.wav")

    def tearDown(self) -> None:
        """Cleanup."""
        import shutil

        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_audio._get_robot_media_microphone")
    @patch("bbia_sim.bbia_audio.wave.open")
    def test_enregistrer_audio_with_sdk_record_audio(
        self, mock_wave: MagicMock, mock_get_mic: MagicMock, mock_env: MagicMock
    ) -> None:
        """Test enregistrement via SDK record_audio."""
        mock_robot = MagicMock()
        mock_robot.media.record_audio = MagicMock(
            return_value=np.zeros(16000, dtype=np.int16)
        )
        mock_get_mic.return_value = mock_robot.media

        mock_wf = MagicMock()
        mock_wave.return_value.__enter__.return_value = mock_wf

        result = bbia_audio.enregistrer_audio(
            self.test_file, duree=1, frequence=16000, robot_api=mock_robot
        )
        self.assertTrue(result)

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_audio._get_robot_media_microphone")
    @patch("bbia_sim.bbia_audio.wave.open")
    def test_enregistrer_audio_with_sdk_microphone_record(
        self, mock_wave: MagicMock, mock_get_mic: MagicMock, mock_env: MagicMock
    ) -> None:
        """Test enregistrement via SDK microphone.record."""
        mock_robot = MagicMock()
        mock_robot.media = MagicMock()
        mock_robot.media.record_audio = None

        mock_mic = MagicMock()
        mock_mic.record = MagicMock(
            return_value=np.zeros(16000, dtype=np.int16).tobytes()
        )
        mock_get_mic.return_value = mock_mic

        mock_wf = MagicMock()
        mock_wave.return_value.__enter__.return_value = mock_wf

        result = bbia_audio.enregistrer_audio(
            self.test_file, duree=1, frequence=16000, robot_api=mock_robot
        )
        self.assertTrue(result)

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_audio._get_robot_media_microphone")
    def test_enregistrer_audio_sdk_error_fallback(
        self, mock_get_mic: MagicMock, mock_env: MagicMock
    ) -> None:
        """Test fallback SDK vers sounddevice en cas d'erreur."""
        mock_get_mic.return_value = MagicMock()
        mock_get_mic.return_value.record = MagicMock(side_effect=Exception("SDK error"))

        with (
            patch("bbia_sim.bbia_audio._get_sd") as mock_get_sd,
            patch("bbia_sim.bbia_audio.wave.open") as mock_wave,
        ):
            mock_sd = MagicMock()
            mock_sd.rec.return_value = np.zeros((16000, 1), dtype="int16")
            mock_get_sd.return_value = mock_sd

            mock_wf = MagicMock()
            mock_wave.return_value.__enter__.return_value = mock_wf

            result = bbia_audio.enregistrer_audio(self.test_file, duree=1)
            self.assertTrue(result)

    @patch("os.environ.get", return_value="0")
    def test_lire_audio_with_sdk_play_audio(self, mock_env: MagicMock) -> None:
        """Test lecture via SDK play_audio."""
        mock_robot = MagicMock()
        mock_robot.media.play_audio = MagicMock()

        # Créer fichier WAV valide
        import wave

        with wave.open(self.test_file, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            wf.writeframes(np.zeros(16000, dtype=np.int16).tobytes())

        bbia_audio.lire_audio(self.test_file, robot_api=mock_robot)
        mock_robot.media.play_audio.assert_called_once()

    @patch("os.environ.get", return_value="0")
    def test_lire_audio_with_sdk_speaker_play(self, mock_env: MagicMock) -> None:
        """Test lecture via SDK speaker.play."""
        mock_robot = MagicMock()
        mock_robot.media.play_audio = None
        mock_robot.media.speaker = MagicMock()
        mock_robot.media.speaker.play = MagicMock()

        import wave

        with wave.open(self.test_file, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            wf.writeframes(np.zeros(16000, dtype=np.int16).tobytes())

        bbia_audio.lire_audio(self.test_file, robot_api=mock_robot)
        mock_robot.media.speaker.play.assert_called_once()

    @patch("os.environ.get", return_value="0")
    def test_lire_audio_sdk_error_fallback(self, mock_env: MagicMock) -> None:
        """Test fallback SDK vers sounddevice en cas d'erreur."""
        mock_robot = MagicMock()
        mock_robot.media.play_audio = MagicMock(side_effect=Exception("SDK error"))
        mock_robot.media.speaker = None

        import wave

        with wave.open(self.test_file, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            wf.writeframes(np.zeros(16000, dtype=np.int16).tobytes())

        with (
            patch("bbia_sim.bbia_audio._get_sd") as mock_get_sd,
            patch("bbia_sim.bbia_audio.wave.open") as mock_wave,
        ):
            mock_sd = MagicMock()
            mock_get_sd.return_value = mock_sd

            mock_wf = MagicMock()
            mock_wf.getframerate.return_value = 16000
            mock_wf.readframes.return_value = np.zeros(16000, dtype="int16").tobytes()
            mock_wf.getnframes.return_value = 16000
            mock_wave.return_value.__enter__.return_value = mock_wf

            bbia_audio.lire_audio(self.test_file, robot_api=mock_robot)
            mock_sd.play.assert_called_once()

    def test_get_sd_with_import_error(self) -> None:
        """Test _get_sd avec ImportError."""
        # Simuler import échoué
        with patch("bbia_sim.bbia_audio.sd", bbia_audio._SoundDeviceShim()):
            result = bbia_audio._get_sd()
            self.assertIsNotNone(result)

    def test_is_safe_path_temp_dirs(self) -> None:
        """Test validation chemins répertoires temporaires."""
        import tempfile

        temp_root = tempfile.gettempdir()
        test_path = os.path.join(temp_root, "test.wav")
        self.assertTrue(bbia_audio._is_safe_path(test_path))

    def test_is_safe_path_current_dir(self) -> None:
        """Test validation chemin répertoire courant."""
        cwd = os.getcwd()
        test_path = os.path.join(cwd, "test.wav")
        self.assertTrue(bbia_audio._is_safe_path(test_path))

    @patch("os.environ.get", return_value="0")
    @patch("bbia_sim.bbia_audio.soundfile")
    def test_lire_audio_with_wrong_sample_rate(
        self, mock_soundfile: MagicMock, mock_env: MagicMock
    ) -> None:
        """Test lecture avec sample rate incorrect."""
        mock_info = MagicMock()
        mock_info.samplerate = 22050  # Différent de 16000
        mock_soundfile.info.return_value = mock_info

        import wave

        with wave.open(self.test_file, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            wf.writeframes(np.zeros(16000, dtype=np.int16).tobytes())

        # Ne devrait pas lever d'exception, juste warning
        with (
            patch("bbia_sim.bbia_audio._get_sd") as mock_get_sd,
            patch("bbia_sim.bbia_audio.wave.open") as mock_wave,
        ):
            mock_sd = MagicMock()
            mock_get_sd.return_value = mock_sd

            mock_wf = MagicMock()
            mock_wf.getframerate.return_value = 16000
            mock_wf.readframes.return_value = np.zeros(16000, dtype="int16").tobytes()
            mock_wf.getnframes.return_value = 16000
            mock_wave.return_value.__enter__.return_value = mock_wf

            bbia_audio.lire_audio(self.test_file)

    @patch("os.environ.get", return_value="0")
    def test_enregistrer_audio_bytes_conversion(self, mock_env: MagicMock) -> None:
        """Test conversion bytes dans enregistrement SDK."""
        mock_robot = MagicMock()
        mock_robot.media.record_audio = MagicMock(
            return_value=np.zeros(16000, dtype=np.float32)  # Float32 au lieu de int16
        )

        with (
            patch("bbia_sim.bbia_audio._get_robot_media_microphone") as mock_get_mic,
            patch("bbia_sim.bbia_audio.wave.open") as mock_wave,
        ):
            mock_get_mic.return_value = mock_robot.media

            mock_wf = MagicMock()
            mock_wave.return_value.__enter__.return_value = mock_wf

            result = bbia_audio.enregistrer_audio(
                self.test_file, duree=1, frequence=16000, robot_api=mock_robot
            )
            self.assertTrue(result)


if __name__ == "__main__":
    unittest.main()
