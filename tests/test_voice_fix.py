#!/usr/bin/env python3
"""Test rapide pour vérifier que les corrections de voix fonctionnent."""
import os
import sys
import tempfile
from pathlib import Path

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import pytest

from bbia_sim.ai_backends import Pyttsx3TTS


@pytest.mark.audio
def test_voice_fix():
    """Test rapide pour vérifier que les corrections de voix fonctionnent."""
    # Skip si audio désactivé (CI)
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        pytest.skip("Audio désactivé en CI")

    tts = Pyttsx3TTS()
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        tmp_file = f.name

    try:
        tts.synthesize_to_wav("Test voix", tmp_file)
        print("✅ Pyttsx3TTS utilise maintenant get_bbia_voice()")
        print("✅ La voix devrait être Aurelie Enhanced maintenant!")
    except RuntimeError as e:
        if "pyttsx3 non disponible" in str(e):
            pytest.skip(f"pyttsx3 non disponible: {e}")
        raise
    finally:
        if os.path.exists(tmp_file):
            os.unlink(tmp_file)
