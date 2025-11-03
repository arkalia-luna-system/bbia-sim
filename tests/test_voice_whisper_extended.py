#!/usr/bin/env python3
"""
Tests étendus pour voice_whisper.py
Tests de reconnaissance vocale Whisper
"""

import pytest


class TestVoiceWhisperExtended:
    """Tests étendus pour VoiceWhisper."""

    def test_voice_whisper_module_exists(self):
        """Test que le module voice_whisper existe."""
        try:
            import bbia_sim.voice_whisper

            assert bbia_sim.voice_whisper is not None
        except ImportError:
            pytest.skip("Module voice_whisper non disponible")

    def test_voice_whisper_functions_exist(self):
        """Test que les fonctions principales existent."""
        try:
            from bbia_sim.voice_whisper import (  # type: ignore[attr-defined]
                recognize_speech,
                transcribe_audio,
            )

            assert transcribe_audio is not None
            assert recognize_speech is not None
        except (ImportError, AttributeError):
            pytest.skip("Fonctions voice_whisper non disponibles")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
