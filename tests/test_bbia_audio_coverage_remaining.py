#!/usr/bin/env python3
"""Tests pour améliorer la couverture de bbia_audio.py - Lignes manquantes.

Objectif : Couvrir les lignes non testées pour atteindre 95%+ de couverture.
Lignes manquantes identifiées :
- 30-31, 38-39, 46-47 : _SoundDeviceShim (raise RuntimeError)
- 67-69 : exception dans _get_sd
- 111-112 : exception dans _get_robot_media_microphone
- 157-158 : exception dans _is_safe_path
- 240 : conversion bytes dans enregistrer_audio
- 267-268 : exception dans enregistrer_audio SDK
- 301 : fallback error dans enregistrer_audio
- 304-308 : détection canaux dans enregistrer_audio
"""

import os
import tempfile
import unittest
from unittest.mock import MagicMock, PropertyMock, patch

import numpy as np

# Désactiver audio pour CI
os.environ["BBIA_DISABLE_AUDIO"] = "1"

import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim import bbia_audio


class TestBBIAAudioCoverageRemaining(unittest.TestCase):
    """Tests pour couvrir les lignes manquantes de bbia_audio.py."""

    def setUp(self) -> None:
        """Setup test."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.temp_dir, "test.wav")

    def tearDown(self) -> None:
        """Cleanup."""
        import shutil

        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def test_sounddevice_shim_rec_raises(self) -> None:
        """Test _SoundDeviceShim.rec() lève RuntimeError."""
        shim = bbia_audio._SoundDeviceShim()
        with self.assertRaises(
            RuntimeError, msg="sounddevice indisponible: sd.rec non opérationnel"
        ):
            shim.rec(1000, samplerate=16000)

    def test_sounddevice_shim_wait_raises(self) -> None:
        """Test _SoundDeviceShim.wait() lève RuntimeError."""
        shim = bbia_audio._SoundDeviceShim()
        with self.assertRaises(
            RuntimeError, msg="sounddevice indisponible: sd.wait non opérationnel"
        ):
            shim.wait()

    def test_sounddevice_shim_play_raises(self) -> None:
        """Test _SoundDeviceShim.play() lève RuntimeError."""
        shim = bbia_audio._SoundDeviceShim()
        with self.assertRaises(
            RuntimeError, msg="sounddevice indisponible: sd.play non opérationnel"
        ):
            shim.play(np.array([1, 2, 3], dtype=np.int16), 16000)

    def test_get_sd_with_import_error(self) -> None:
        """Test _get_sd() avec ImportError lors de l'import."""
        # Simuler un échec d'import de sounddevice
        with patch(
            "builtins.__import__",
            side_effect=ImportError("No module named 'sounddevice'"),
        ):
            # Réinitialiser sd au shim
            original_sd = bbia_audio.sd
            bbia_audio.sd = bbia_audio._SoundDeviceShim()
            try:
                result = bbia_audio._get_sd()
                # Devrait retourner le shim en cas d'erreur
                self.assertIsNotNone(result)
            finally:
                bbia_audio.sd = original_sd

    def test_get_sd_with_runtime_error(self) -> None:
        """Test _get_sd() avec RuntimeError lors de l'import."""
        with patch("builtins.__import__", side_effect=RuntimeError("PortAudio error")):
            original_sd = bbia_audio.sd
            bbia_audio.sd = bbia_audio._SoundDeviceShim()
            try:
                result = bbia_audio._get_sd()
                self.assertIsNotNone(result)
            finally:
                bbia_audio.sd = original_sd

    def test_get_sd_with_attribute_error(self) -> None:
        """Test _get_sd() avec AttributeError lors de l'import."""
        with patch(
            "builtins.__import__",
            side_effect=AttributeError("module has no attribute 'rec'"),
        ):
            original_sd = bbia_audio.sd
            bbia_audio.sd = bbia_audio._SoundDeviceShim()
            try:
                result = bbia_audio._get_sd()
                self.assertIsNotNone(result)
            finally:
                bbia_audio.sd = original_sd

    def test_get_robot_media_microphone_with_attribute_error(self) -> None:
        """Test _get_robot_media_microphone() avec AttributeError."""

        # Créer un mock qui lève AttributeError lors de l'accès à media
        class MockRobotWithError:
            def __init__(self):
                pass

            @property
            def media(self):
                raise AttributeError("No attribute 'media'")

        mock_robot = MockRobotWithError()
        result = bbia_audio._get_robot_media_microphone(mock_robot)
        self.assertIsNone(result)

    def test_get_robot_media_microphone_with_import_error(self) -> None:
        """Test _get_robot_media_microphone() avec ImportError."""
        mock_robot = MagicMock()
        # Simuler une exception ImportError
        with patch("builtins.hasattr", return_value=True):
            type(mock_robot).media = PropertyMock(
                side_effect=ImportError("Cannot import media")
            )
            result = bbia_audio._get_robot_media_microphone(mock_robot)
            self.assertIsNone(result)

    def test_get_robot_media_microphone_with_runtime_error(self) -> None:
        """Test _get_robot_media_microphone() avec RuntimeError."""
        mock_robot = MagicMock()
        # Simuler une exception RuntimeError
        with patch("builtins.hasattr", return_value=True):
            type(mock_robot).media = PropertyMock(
                side_effect=RuntimeError("Media error")
            )
            result = bbia_audio._get_robot_media_microphone(mock_robot)
            self.assertIsNone(result)

    def test_is_safe_path_with_os_error(self) -> None:
        """Test _is_safe_path() avec OSError."""
        # Simuler une OSError lors de os.path.normpath ou os.path.isabs
        with patch("os.path.normpath", side_effect=OSError("Path error")):
            result = bbia_audio._is_safe_path("test.wav")
            self.assertFalse(result)

    def test_is_safe_path_with_value_error(self) -> None:
        """Test _is_safe_path() avec ValueError."""
        # Simuler une ValueError lors de os.path.normpath
        with patch("os.path.normpath", side_effect=ValueError("Invalid path")):
            result = bbia_audio._is_safe_path("test.wav")
            self.assertFalse(result)

    def test_is_safe_path_with_type_error(self) -> None:
        """Test _is_safe_path() avec TypeError."""
        # Simuler une TypeError
        with patch("os.path.normpath", side_effect=TypeError("Type error")):
            result = bbia_audio._is_safe_path("test.wav")
            self.assertFalse(result)

    @patch("os.environ.get", return_value="0")
    def test_enregistrer_audio_bytes_conversion(self, mock_env: MagicMock) -> None:
        """Test conversion bytes dans enregistrer_audio (ligne 240)."""
        mock_robot = MagicMock()
        # Simuler record_audio qui retourne bytes directement
        mock_robot.media.record_audio = MagicMock(return_value=b"audio_bytes_data")
        mock_robot.media.__bool__ = lambda self: True

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
            # Vérifier que writeframes a été appelé avec bytes
            mock_wf.writeframes.assert_called_once()
            call_args = mock_wf.writeframes.call_args[0][0]
            self.assertIsInstance(call_args, bytes)

    @patch("os.environ.get", return_value="0")
    def test_enregistrer_audio_sdk_error_fallback(self, mock_env: MagicMock) -> None:
        """Test exception SDK dans enregistrer_audio (lignes 267-268)."""
        mock_robot = MagicMock()
        mock_robot.media.record_audio = MagicMock(side_effect=OSError("SDK error"))
        mock_robot.media.__bool__ = lambda self: True

        with (
            patch("bbia_sim.bbia_audio._get_robot_media_microphone") as mock_get_mic,
            patch("bbia_sim.bbia_audio._get_sd") as mock_get_sd,
            patch("bbia_sim.bbia_audio.wave.open") as mock_wave,
        ):
            mock_get_mic.return_value = mock_robot.media

            # Fallback vers sounddevice
            mock_sd = MagicMock()
            mock_sd.rec.return_value = np.zeros((16000, 1), dtype="int16")
            mock_get_sd.return_value = mock_sd

            mock_wf = MagicMock()
            mock_wave.return_value.__enter__.return_value = mock_wf

            result = bbia_audio.enregistrer_audio(
                self.test_file, duree=1, frequence=16000, robot_api=mock_robot
            )
            self.assertTrue(result)
            # Vérifier que sounddevice a été utilisé en fallback
            mock_sd.rec.assert_called_once()

    @patch("os.environ.get", return_value="0")
    def test_enregistrer_audio_channel_error_fallback(
        self, mock_env: MagicMock
    ) -> None:
        """Test gestion erreur canaux avec fallback (lignes 301, 304-308)."""
        mock_robot = MagicMock()
        # Pas de SDK disponible
        with (
            patch("bbia_sim.bbia_audio._get_robot_media_microphone", return_value=None),
            patch("bbia_sim.bbia_audio._get_sd") as mock_get_sd,
            patch("bbia_sim.bbia_audio.wave.open") as mock_wave,
        ):
            mock_sd = MagicMock()
            # Première tentative échoue avec erreur canaux
            mock_sd.rec.side_effect = [
                Exception("Channel error"),
                np.zeros((16000, 1), dtype="int16"),  # Deuxième tentative réussit
            ]
            # Simuler query_devices pour la détection de canaux
            mock_sd.default.device = [0, 1]
            mock_sd.query_devices.return_value = {"max_input_channels": 1}
            mock_get_sd.return_value = mock_sd

            mock_wf = MagicMock()
            mock_wave.return_value.__enter__.return_value = mock_wf

            result = bbia_audio.enregistrer_audio(
                self.test_file, duree=1, frequence=16000, robot_api=mock_robot
            )
            self.assertTrue(result)
            # Vérifier que rec a été appelé deux fois (première erreur, deuxième succès)
            self.assertEqual(mock_sd.rec.call_count, 2)
            # Vérifier que query_devices a été appelé pour détecter les canaux
            mock_sd.query_devices.assert_called_once()

    @patch("os.environ.get", return_value="0")
    def test_enregistrer_audio_channel_error_fallback_fails(
        self, mock_env: MagicMock
    ) -> None:
        """Test gestion erreur canaux avec fallback qui échoue (ligne 301)."""
        mock_robot = MagicMock()
        with (
            patch("bbia_sim.bbia_audio._get_robot_media_microphone", return_value=None),
            patch("bbia_sim.bbia_audio._get_sd") as mock_get_sd,
        ):
            mock_sd = MagicMock()
            # Première tentative échoue
            mock_sd.rec.side_effect = Exception("Channel error")
            # Fallback échoue aussi (query_devices échoue)
            mock_sd.query_devices.side_effect = Exception("Query devices error")
            mock_get_sd.return_value = mock_sd

            with self.assertRaises(RuntimeError, msg="Impossible d'enregistrer audio"):
                bbia_audio.enregistrer_audio(
                    self.test_file, duree=1, frequence=16000, robot_api=mock_robot
                )

    @patch("os.environ.get", return_value="0")
    def test_enregistrer_audio_channel_error_sd_none(self, mock_env: MagicMock) -> None:
        """Test gestion erreur canaux avec _get_sd() qui retourne None (ligne 301)."""
        mock_robot = MagicMock()
        with (
            patch("bbia_sim.bbia_audio._get_robot_media_microphone", return_value=None),
            patch("bbia_sim.bbia_audio._get_sd") as mock_get_sd,
        ):
            mock_sd = MagicMock()
            # Première tentative échoue
            mock_sd.rec.side_effect = Exception("Channel error")
            # Dans le fallback, _get_sd() retourne None
            call_count = {"count": 0}

            def get_sd_side_effect():
                call_count["count"] += 1
                if call_count["count"] == 1:
                    return mock_sd  # Première fois, retourne mock_sd
                return None  # Deuxième fois (dans le fallback), retourne None

            mock_get_sd.side_effect = get_sd_side_effect

            with self.assertRaises(RuntimeError, msg="Impossible d'enregistrer audio"):
                bbia_audio.enregistrer_audio(
                    self.test_file, duree=1, frequence=16000, robot_api=mock_robot
                )


if __name__ == "__main__":
    unittest.main()
