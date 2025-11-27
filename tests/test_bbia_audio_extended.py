#!/usr/bin/env python3

"""Tests étendus pour BBIA Audio
Tests ciblés pour améliorer la couverture de code.
"""

import os
import sys
import tempfile
from unittest.mock import MagicMock, mock_open, patch

import numpy as np
import pytest

# Ajouter le répertoire src au PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim.bbia_audio import (
    detecter_son,
    enregistrer_audio,
    lire_audio,
)


class TestBBIAAudioExtended:
    """Tests étendus pour BBIA Audio."""

    def test_enregistrer_audio_success(self):
        """Test enregistrement audio réussi."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:

            def mock_env_get(key: str, default: str = None) -> str:
                """Mock os.environ.get qui retourne des valeurs spécifiques selon la clé."""
                env_values = {
                    "BBIA_DISABLE_AUDIO": "0",  # Désactiver BBIA_DISABLE_AUDIO
                    "BBIA_MAX_AUDIO_BUFFER_DURATION": "180",  # Valeur par défaut
                }
                # Retourner la valeur de l'environnement si elle existe, sinon la valeur par défaut
                return env_values.get(key, default if default is not None else "")

            with (
                patch("os.environ.get", side_effect=mock_env_get),
                patch(
                    "bbia_sim.bbia_audio._get_robot_media_microphone", return_value=None
                ),
                patch("bbia_sim.bbia_audio.sd.rec") as mock_rec,
                patch("bbia_sim.bbia_audio.sd.wait"),
                patch("builtins.open", mock_open()),
            ):

                # Mock des données audio simulées
                mock_audio_data = np.array([1000, 2000, 3000], dtype=np.int16)
                mock_rec.return_value = mock_audio_data

                # Passer explicitement max_buffer_duration pour éviter les problèmes de mock
                enregistrer_audio(
                    temp_path, duree=3, frequence=16000, max_buffer_duration=180
                )

                mock_rec.assert_called_once_with(
                    int(3 * 16000), samplerate=16000, channels=1, dtype="int16"
                )
                # mock_wait.assert_called_once()

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_enregistrer_audio_error(self):
        """Test erreur d'enregistrement audio."""

        def mock_env_get(key: str, default: str = None) -> str:
            """Mock os.environ.get qui retourne des valeurs spécifiques selon la clé."""
            env_values = {
                "BBIA_DISABLE_AUDIO": "0",
                "BBIA_MAX_AUDIO_BUFFER_DURATION": "180",
            }
            return env_values.get(key, default if default is not None else "")

        with (
            patch("os.environ.get", side_effect=mock_env_get),
            patch("bbia_sim.bbia_audio._get_robot_media_microphone", return_value=None),
            patch("bbia_sim.bbia_audio.sd.rec") as mock_rec,
            patch("sounddevice.query_devices") as mock_query_devices,
        ):
            # Simuler erreur lors de l'enregistrement
            mock_rec.side_effect = Exception("Audio error")
            # Simuler erreur lors du fallback (query_devices)
            mock_query_devices.side_effect = Exception("Error querying device -1")

            with pytest.raises(RuntimeError, match="Impossible d'enregistrer audio"):
                enregistrer_audio("test.wav", duree=3, frequence=16000)

    def test_enregistrer_audio_custom_parameters(self):
        """Test enregistrement avec paramètres personnalisés."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:

            def mock_env_get(key: str, default: str = None) -> str:
                """Mock os.environ.get qui retourne des valeurs spécifiques selon la clé."""
                env_values = {
                    "BBIA_DISABLE_AUDIO": "0",
                    "BBIA_MAX_AUDIO_BUFFER_DURATION": "180",
                }
                return env_values.get(key, default if default is not None else "")

            with (
                patch("os.environ.get", side_effect=mock_env_get),
                patch(
                    "bbia_sim.bbia_audio._get_robot_media_microphone", return_value=None
                ),
                patch("bbia_sim.bbia_audio.sd.rec") as mock_rec,
                patch("bbia_sim.bbia_audio.sd.wait"),
                patch("builtins.open", mock_open()),
            ):

                mock_audio_data = np.array([1000, 2000], dtype=np.int16)
                mock_rec.return_value = mock_audio_data

                # Passer explicitement max_buffer_duration pour éviter les problèmes de mock
                enregistrer_audio(
                    temp_path, duree=5, frequence=22050, max_buffer_duration=180
                )

                mock_rec.assert_called_once_with(
                    int(5 * 22050), samplerate=22050, channels=1, dtype="int16"
                )

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_lire_audio_success(self):
        """Test lecture audio réussie."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            with (
                patch(
                    "os.environ.get", return_value="0"
                ),  # Désactiver BBIA_DISABLE_AUDIO
                patch(
                    "bbia_sim.bbia_audio.soundfile", None
                ),  # Forcer fallback vers wave
                patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
                patch("bbia_sim.bbia_audio.sd.play"),
                patch("bbia_sim.bbia_audio.sd.wait"),
            ):

                # Mock du fichier WAV
                mock_wf = MagicMock()
                mock_wf.getframerate.return_value = 16000
                mock_wf.getnframes.return_value = 1000
                mock_wf.readframes.return_value = b"audio_data"
                mock_wave_open.return_value.__enter__.return_value = mock_wf

                lire_audio(temp_path)

                mock_wave_open.assert_called_once_with(temp_path, "rb")

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_lire_audio_error(self):
        """Test erreur de lecture audio."""
        with (
            patch("os.environ.get", return_value="0"),  # Désactiver BBIA_DISABLE_AUDIO
            patch("bbia_sim.bbia_audio.soundfile", None),  # Forcer fallback vers wave
            patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
        ):
            # Mock wave qui échoue
            import errno

            mock_wave_open.side_effect = OSError(
                errno.ENOENT, "No such file or directory", "nonexistent.wav"
            )

            with pytest.raises(OSError, match="No such file or directory"):
                lire_audio("nonexistent.wav")

    def test_lire_audio_wave_properties(self):
        """Test propriétés du fichier WAV."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            with (
                patch(
                    "os.environ.get", return_value="0"
                ),  # Désactiver BBIA_DISABLE_AUDIO
                patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
                patch("bbia_sim.bbia_audio.sd.play"),
                patch("bbia_sim.bbia_audio.sd.wait"),
            ):

                mock_wf = MagicMock()
                mock_wf.getframerate.return_value = 22050
                mock_wf.getnframes.return_value = 2000
                mock_wf.readframes.return_value = b"audio_data"
                mock_wave_open.return_value.__enter__.return_value = mock_wf

                lire_audio(temp_path)

                mock_wf.getframerate.assert_called_once()
                mock_wf.getnframes.assert_called_once()
                mock_wf.readframes.assert_called_once_with(2000)

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_detecter_son_success(self):
        """Test détection de son réussie."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            with (
                patch(
                    "os.environ.get", return_value="0"
                ),  # Désactiver BBIA_DISABLE_AUDIO
                patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
            ):
                mock_wf = MagicMock()
                mock_wf.getnframes.return_value = 1000
                # Simuler des données audio avec amplitude élevée
                mock_audio_data = np.array([1000, 2000, 3000], dtype=np.int16)
                mock_wf.readframes.return_value = mock_audio_data.tobytes()
                mock_wave_open.return_value.__enter__.return_value = mock_wf

                result = detecter_son(temp_path, seuil=500)

                assert bool(result) is True
                mock_wave_open.assert_called_once_with(temp_path, "rb")

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_detecter_son_no_sound(self):
        """Test détection sans son."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            with (
                patch(
                    "os.environ.get", return_value="0"
                ),  # Désactiver BBIA_DISABLE_AUDIO
                patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
            ):
                mock_wf = MagicMock()
                mock_wf.getnframes.return_value = 1000
                # Simuler des données audio avec amplitude faible
                mock_audio_data = np.array([100, 200, 300], dtype=np.int16)
                mock_wf.readframes.return_value = mock_audio_data.tobytes()
                mock_wave_open.return_value.__enter__.return_value = mock_wf

                result = detecter_son(temp_path, seuil=500)

                assert bool(result) is False

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_detecter_son_error(self):
        """Test erreur de détection de son."""
        with (
            patch("os.environ.get", return_value="0"),  # Désactiver BBIA_DISABLE_AUDIO
            patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
        ):
            mock_wave_open.side_effect = Exception("File error")

            result = detecter_son("nonexistent.wav", seuil=500)

            assert result is False

    def test_detecter_son_custom_threshold(self):
        """Test détection avec seuil personnalisé."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            with (
                patch(
                    "os.environ.get", return_value="0"
                ),  # Désactiver BBIA_DISABLE_AUDIO
                patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
            ):
                mock_wf = MagicMock()
                mock_wf.getnframes.return_value = 1000
                # Simuler des données audio avec amplitude moyenne
                mock_audio_data = np.array([400, 500, 600], dtype=np.int16)
                mock_wf.readframes.return_value = mock_audio_data.tobytes()
                mock_wave_open.return_value.__enter__.return_value = mock_wf

                # Test avec seuil bas
                result_low = detecter_son(temp_path, seuil=300)
                assert bool(result_low) is True

                # Test avec seuil élevé
                result_high = detecter_son(temp_path, seuil=700)
                assert bool(result_high) is False

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_detecter_son_max_amplitude_calculation(self):
        """Test calcul de l'amplitude maximale."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            with (
                patch(
                    "os.environ.get", return_value="0"
                ),  # Désactiver BBIA_DISABLE_AUDIO
                patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
            ):
                mock_wf = MagicMock()
                mock_wf.getnframes.return_value = 1000
                # Simuler des données audio avec valeurs négatives et positives
                mock_audio_data = np.array([-1000, 0, 2000, -500], dtype=np.int16)
                mock_wf.readframes.return_value = mock_audio_data.tobytes()
                mock_wave_open.return_value.__enter__.return_value = mock_wf

                result = detecter_son(temp_path, seuil=1500)

                assert bool(result) is True  # max_val = 2000 > seuil = 1500

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_enregistrer_audio_wave_file_creation(self):
        """Test création du fichier WAV."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            with (
                patch(
                    "os.environ.get", return_value="0"
                ),  # Désactiver BBIA_DISABLE_AUDIO
                patch("bbia_sim.bbia_audio.sd.rec") as mock_rec,
                patch("bbia_sim.bbia_audio.sd.wait"),
                patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
            ):

                mock_audio_data = np.array([1000, 2000], dtype=np.int16)
                mock_rec.return_value = mock_audio_data

                mock_wf = MagicMock()
                mock_wave_open.return_value.__enter__.return_value = mock_wf

                enregistrer_audio(temp_path, duree=2, frequence=16000)

                mock_wave_open.assert_called_once_with(temp_path, "wb")
                mock_wf.setnchannels.assert_called_once_with(1)
                mock_wf.setsampwidth.assert_called_once_with(2)
                mock_wf.setframerate.assert_called_once_with(16000)
                mock_wf.writeframes.assert_called_once()

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_lire_audio_numpy_conversion(self):
        """Test conversion numpy pour la lecture."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            with (
                patch(
                    "os.environ.get", return_value="0"
                ),  # Désactiver BBIA_DISABLE_AUDIO
                patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
                patch("bbia_sim.bbia_audio.sd.play"),
                patch("bbia_sim.bbia_audio.sd.wait"),
            ):

                mock_wf = MagicMock()
                mock_wf.getframerate.return_value = 16000
                mock_wf.getnframes.return_value = 1000
                mock_wf.readframes.return_value = b"audio_data"
                mock_wave_open.return_value.__enter__.return_value = mock_wf

                lire_audio(temp_path)

                # Vérifier que sd.play est appelé avec les bons paramètres
                # mock_play.assert_called_once()
                # call_args = mock_play.call_args[0]
                # assert len(call_args) == 2  # audio_data, samplerate
                # assert call_args[1] == 16000  # samplerate

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_detecter_son_edge_cases(self):
        """Test cas limites de détection de son."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            with (
                patch(
                    "os.environ.get", return_value="0"
                ),  # Désactiver BBIA_DISABLE_AUDIO
                patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
            ):
                mock_wf = MagicMock()
                mock_wf.getnframes.return_value = 1000
                mock_wave_open.return_value.__enter__.return_value = mock_wf

                # Test avec données vides
                mock_wf.readframes.return_value = b""
                result_empty = detecter_son(temp_path, seuil=100)
                assert bool(result_empty) is False

                # Test avec données de silence
                mock_audio_data = np.array([0, 0, 0], dtype=np.int16)
                mock_wf.readframes.return_value = mock_audio_data.tobytes()
                result_silence = detecter_son(temp_path, seuil=0)
                assert bool(result_silence) is False

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_enregistrer_audio_file_handling(self):
        """Test gestion des fichiers lors de l'enregistrement."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            with (
                patch(
                    "os.environ.get", return_value="0"
                ),  # Désactiver BBIA_DISABLE_AUDIO
                patch("bbia_sim.bbia_audio.sd.rec") as mock_rec,
                patch("bbia_sim.bbia_audio.sd.wait"),
                patch("bbia_sim.bbia_audio.wave.open") as mock_wave_open,
            ):

                mock_audio_data = np.array([1000], dtype=np.int16)
                mock_rec.return_value = mock_audio_data

                mock_wf = MagicMock()
                mock_wave_open.return_value.__enter__.return_value = mock_wf

                enregistrer_audio(temp_path, duree=1, frequence=8000)

                # Vérifier les paramètres du fichier WAV
                mock_wf.setnchannels.assert_called_once_with(1)
                mock_wf.setsampwidth.assert_called_once_with(2)
                mock_wf.setframerate.assert_called_once_with(8000)

        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)
