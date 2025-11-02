#!/usr/bin/env python3
"""
Test headless strict garantissant le fallback speaker lorsque media.play_audio n'est pas disponible.
Ne dépend d'aucun driver audio (mocks purs) et force l'audio ON via ENV.
"""

import types

from src.bbia_sim.bbia_voice import dire_texte


class _Speaker:
    def __init__(self):
        self.play_file_calls: list[str] = []
        self.play_bytes_calls: list[bytes] = []

    def play_file(self, path: str) -> None:
        self.play_file_calls.append(path)

    def play(self, data: bytes) -> None:
        self.play_bytes_calls.append(data)


class _MediaNoPlayAudio:
    """Media sans play_audio pour forcer le fallback speaker.*"""

    def __init__(self) -> None:
        self.speaker = _Speaker()


class _RobotApiNoPlayAudio:
    def __init__(self) -> None:
        self.media = _MediaNoPlayAudio()


def test_fallback_speaker_play_bytes_when_no_play_audio(monkeypatch):
    # Activer l'audio même en CI/headless
    monkeypatch.setenv("BBIA_DISABLE_AUDIO", "0")

    robot = _RobotApiNoPlayAudio()

    # Forcer pyttsx3 à un moteur factice (évite I/O)
    import src.bbia_sim.bbia_voice as voice

    class _DummyEngine:
        def __init__(self) -> None:  # noqa: D401
            self.props: dict[str, object] = {}

        def getProperty(self, name):  # noqa: D401
            if name == "voices":
                return [types.SimpleNamespace(name="Amelie", id="fr_FR_Amelie")]
            return None

        def setProperty(self, name, value):  # noqa: D401
            self.props[name] = value

        def say(self, *args, **kwargs):  # noqa: D401
            pass

        def runAndWait(self):  # noqa: D401
            pass

    monkeypatch.setattr(voice, "pyttsx3", types.SimpleNamespace(init=lambda: _DummyEngine()))

    # Exécuter
    dire_texte("bonjour", robot_api=robot)

    # Vérifier que le fallback speaker a bien capturé un flux bytes (priorité)
    assert len(robot.media.speaker.play_bytes_calls) == 1
    # Et qu'aucun play_file n'a été nécessaire (chemin secondaire)
    assert len(robot.media.speaker.play_file_calls) == 0
