#!/usr/bin/env python3
"""Test rapide pour vérifier que les corrections de voix fonctionnent."""
import sys
from pathlib import Path

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import os
import tempfile

from bbia_sim.ai_backends import Pyttsx3TTS

tts = Pyttsx3TTS()
with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
    tmp_file = f.name

try:
    tts.synthesize_to_wav("Test voix", tmp_file)
    print("✅ Pyttsx3TTS utilise maintenant get_bbia_voice()")
    print("✅ La voix devrait être Aurelie Enhanced maintenant!")
finally:
    if os.path.exists(tmp_file):
        os.unlink(tmp_file)
