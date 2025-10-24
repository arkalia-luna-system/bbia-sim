#!/usr/bin/env python3
"""Tests pour BBIA Audio."""

import os
import sys
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

# Ajouter le répertoire src au PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim import bbia_audio


class TestBBIAAudio(unittest.TestCase):
    @patch("bbia_sim.bbia_audio.sd")
    @patch("bbia_sim.bbia_audio.wave.open")
    def test_enregistrer_audio(self, mock_wave_open, mock_sd):
        mock_wave = MagicMock()
        mock_wave_open.return_value.__enter__.return_value = mock_wave
        mock_sd.rec.return_value = np.zeros((16000 * 1, 1), dtype="int16")
        bbia_audio.enregistrer_audio("test.wav", duree=1, frequence=16000)
        mock_sd.rec.assert_called()
        mock_wave.writeframes.assert_called()

    @patch("bbia_sim.bbia_audio.sd")
    @patch("bbia_sim.bbia_audio.wave.open")
    def test_lire_audio(self, mock_wave_open, mock_sd):
        mock_wave = MagicMock()
        mock_wave.getframerate.return_value = 16000
        mock_wave.readframes.return_value = np.zeros(16000, dtype="int16").tobytes()
        mock_wave.getnframes.return_value = 16000
        mock_wave_open.return_value.__enter__.return_value = mock_wave
        bbia_audio.lire_audio("test.wav")
        mock_sd.play.assert_called()

    @patch("bbia_sim.bbia_audio.wave.open")
    def test_detecter_son(self, mock_wave_open):
        mock_wave = MagicMock()
        # Cas : son détecté
        audio = (np.ones(16000, dtype="int16") * 1000).tobytes()
        mock_wave.readframes.return_value = audio
        mock_wave.getnframes.return_value = 16000
        mock_wave_open.return_value.__enter__.return_value = mock_wave
        self.assertTrue(bbia_audio.detecter_son("test.wav", seuil=500))
        # Cas : pas de son
        audio = (np.zeros(16000, dtype="int16")).tobytes()
        mock_wave.readframes.return_value = audio
        self.assertFalse(bbia_audio.detecter_son("test.wav", seuil=500))


if __name__ == "__main__":
    unittest.main()
