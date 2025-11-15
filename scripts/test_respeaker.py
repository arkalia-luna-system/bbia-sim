#!/usr/bin/env python3
"""Test ReSpeaker sound in/out pour Reachy Mini

Usage:
    python scripts/test_respeaker.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory


def test_sound_in():
    """Test enregistrement audio (sound in)."""
    robot = RobotFactory.create_backend("reachy")

    if not robot or not robot.connect():
        print("❌ Robot non disponible")
        return False

    if not robot.media or not robot.media.microphone:
        print("❌ ReSpeaker non disponible")
        return False

    print("🎤 Test enregistrement (3 secondes)...")
    try:
        audio = robot.media.record_audio(duration=3.0, sample_rate=16000)
        if audio:
            print(f"✅ Enregistrement OK ({len(audio)} échantillons)")
            return True
        else:
            print("❌ Enregistrement vide")
            return False
    except Exception as e:
        print(f"❌ Erreur enregistrement: {e}")
        return False


def test_sound_out():
    """Test lecture audio (sound out)."""
    robot = RobotFactory.create_backend("reachy")

    if not robot or not robot.connect():
        print("❌ Robot non disponible")
        return False

    if not robot.media or not robot.media.speaker:
        print("❌ Speaker non disponible")
        return False

    print("🔊 Test lecture audio...")
    try:
        # Générer tone de test (440 Hz, 1 seconde)
        import numpy as np

        sample_rate = 16000
        duration = 1.0
        t = np.linspace(0, duration, int(sample_rate * duration))
        tone = np.sin(2 * np.pi * 440 * t).astype(np.float32)

        robot.media.speaker.play(tone, sample_rate=sample_rate)
        print("✅ Lecture OK")
        return True
    except Exception as e:
        print(f"❌ Erreur lecture: {e}")
        return False


def test_detection():
    """Test détection ReSpeaker."""
    robot = RobotFactory.create_backend("reachy")

    if not robot:
        print("❌ Robot non disponible")
        return False

    connected = robot.connect()
    if not connected:
        print("⚠️ Robot non connecté (mode simulation)")
        return False

    # Vérifier microphone
    if robot.media and robot.media.microphone:
        mic = robot.media.microphone
        print("✅ ReSpeaker détecté")
        print(f"   Canaux: {getattr(mic, 'channels', 'N/A')}")
        print(f"   Sample rate: {getattr(mic, 'sample_rate', 'N/A')} Hz")
        return True
    else:
        print("❌ ReSpeaker non détecté")
        return False


if __name__ == "__main__":
    print("🧪 Tests ReSpeaker\n")

    print("1. Test Détection")
    test_detection()

    print("\n2. Test Sound In (microphone)")
    test_sound_in()

    print("\n3. Test Sound Out (speaker)")
    test_sound_out()

    print("\n✅ Tests terminés")
