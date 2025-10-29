#!/usr/bin/env python3
"""
Test d'intégration média pour la voix avancée.
Vérifie que BBIAVoiceAdvanced utilise robot_api.media.play_audio si disponible.
"""

import sys
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


import pytest

from bbia_sim.bbia_voice_advanced import BBIAVoiceAdvanced


class FakeSpeaker:
    def __init__(self):
        self.play_file_calls = 0
        self.play_calls = 0

    def play_file(self, path: str):
        self.play_file_calls += 1

    def play(self, data: bytes):
        self.play_calls += 1


class FakeMedia:
    def __init__(self, with_play_audio: bool = True, with_speaker: bool = True):
        self.calls = 0
        self.data = None
        self.volume = None
        self.speaker = FakeSpeaker() if with_speaker else None
        self._with_play_audio = with_play_audio

    def play_audio(self, data: bytes, volume: float = 1.0):
        if not self._with_play_audio:
            raise AttributeError("play_audio not available")
        self.calls += 1
        self.data = data
        self.volume = volume


class FakeRobotAPI:
    def __init__(self, with_play_audio: bool = True, with_speaker: bool = True):
        self.media = FakeMedia(
            with_play_audio=with_play_audio, with_speaker=with_speaker
        )


@pytest.mark.parametrize(
    "with_play_audio,with_speaker,expect_play_audio,expect_speaker_play_file",
    [
        (True, True, True, False),
        (True, False, True, False),
        (False, True, False, True),
    ],
)
def test_voice_advanced_prefers_media_when_available(
    tmp_path, with_play_audio, with_speaker, expect_play_audio, expect_speaker_play_file
):
    # Créer instance fake robot_api
    robot_api = FakeRobotAPI(with_play_audio=with_play_audio, with_speaker=with_speaker)

    # Créer l'instance de voix avancée en mode "sans TTS" pour ne pas charger Coqui
    voice = BBIAVoiceAdvanced(
        use_coqui=False, temp_dir=str(tmp_path), robot_api=robot_api
    )

    # Monkeypatch: forcer playsound None pour garantir qu'on ne joue pas localement
    import bbia_sim.bbia_voice_advanced as va

    va.playsound = None

    # Monkeypatch: injecter un wav minimal vide à lire (le code lit/écrit un fichier)
    # On appelle _play_audio_file directement pour ne pas lancer TTS
    audio_file = tmp_path / "dummy.wav"
    audio_file.write_bytes(b"RIFF0000WAVEfmt ")

    voice._play_audio_file(audio_file, volume=0.5)

    assert (
        robot_api.media.calls == 1 if expect_play_audio else robot_api.media.calls == 0
    )
    if expect_speaker_play_file and with_speaker and robot_api.media.speaker is not None:
        assert robot_api.media.speaker.play_file_calls == 1
    elif robot_api.media.speaker is not None:
        assert robot_api.media.speaker.play_file_calls == 0
