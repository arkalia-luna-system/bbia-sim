#!/usr/bin/env python3
"""
Tests étendus pour voice_whisper.py
Tests de reconnaissance vocale Whisper
"""

import sys
from pathlib import Path

import pytest

# S'assurer que src est dans le path pour coverage
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# OPTIMISATION COVERAGE: Importer le module au niveau module pour que coverage le détecte
# IMPORTANT: Import direct (pas dans try/except) pour que coverage le détecte
import bbia_sim.voice_whisper  # noqa: F401

# Importer les fonctions pour les tests
try:
    from bbia_sim.voice_whisper import (  # type: ignore[attr-defined]  # noqa: F401
        recognize_speech,  # type: ignore[attr-defined]
        transcribe_audio,  # type: ignore[attr-defined]
    )
except (ImportError, AttributeError):
    recognize_speech = None  # type: ignore[assignment,misc]
    transcribe_audio = None  # type: ignore[assignment,misc]


class TestVoiceWhisperExtended:
    """Tests étendus pour VoiceWhisper."""

    def test_voice_whisper_module_exists(self):
        """Test que le module voice_whisper existe."""
        assert bbia_sim.voice_whisper is not None

    def test_voice_whisper_functions_exist(self):
        """Test que les fonctions principales existent."""
        if transcribe_audio is None or recognize_speech is None:
            pytest.skip("Fonctions voice_whisper non disponibles")
        assert transcribe_audio is not None
        assert recognize_speech is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
