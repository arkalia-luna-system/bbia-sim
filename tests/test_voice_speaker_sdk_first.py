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


class _Media:
    def __init__(self, support_play_audio: bool = True) -> None:
        self.speaker = _Speaker()
        if support_play_audio:
            # Simule media.play_audio(bytes, volume?) du SDK
            def _play_audio(data: bytes, volume: float | None = None) -> None:  # type: ignore[override]
                self._play_audio_calls.append((data, volume))

            self.play_audio = _play_audio  # type: ignore[attr-defined]
        self._play_audio_calls: list[tuple[bytes, float | None]] = []


class _RobotApi:
    def __init__(self, support_play_audio: bool = True) -> None:
        self.media = _Media(support_play_audio=support_play_audio)


def test_dire_texte_prefers_media_play_audio(monkeypatch):
    # Assurer que l'audio n'est pas globalement désactivé en CI
    monkeypatch.setenv("BBIA_DISABLE_AUDIO", "0")
    robot = _RobotApi(support_play_audio=True)
    # Forcer pyttsx3 à utiliser un moteur factice rapide
    import src.bbia_sim.bbia_voice as voice

    class _DummyEngine:
        def __init__(self) -> None:
            self.props: dict[str, object] = {}

        def getProperty(self, name):
            if name == "voices":
                return [types.SimpleNamespace(name="Amelie", id="fr_FR_Amelie")]
            return None

        def setProperty(self, name, value):
            self.props[name] = value

        def save_to_file(self, texte, path):
            # Crée un WAV minimal valide
            import struct
            import wave

            with wave.open(path, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(struct.pack("<h", 0))

        def runAndWait(self):
            pass

        def say(self, *args, **kwargs):
            pass

    monkeypatch.setattr(voice, "pyttsx3", types.SimpleNamespace(init=lambda: _DummyEngine()))

    dire_texte("bonjour", robot_api=robot)

    # Priorité: media.play_audio doit être utilisé
    assert len(robot.media._play_audio_calls) == 1
    assert len(robot.media.speaker.play_file_calls) == 0
    assert len(robot.media.speaker.play_bytes_calls) == 0


def test_dire_texte_fallbacks_to_speaker_when_no_play_audio(monkeypatch):
    # Assurer que l'audio n'est pas globalement désactivé en CI
    monkeypatch.setenv("BBIA_DISABLE_AUDIO", "0")
    robot = _RobotApi(support_play_audio=False)
    import src.bbia_sim.bbia_voice as voice

    class _DummyEngine:
        def __init__(self) -> None:
            self.props: dict[str, object] = {}

        def getProperty(self, name):
            if name == "voices":
                return [types.SimpleNamespace(name="Amelie", id="fr_FR_Amelie")]
            return None

        def setProperty(self, name, value):
            self.props[name] = value

        def save_to_file(self, texte, path):
            import struct
            import wave

            with wave.open(path, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(struct.pack("<h", 0))

        def runAndWait(self):
            pass

        def say(self, *args, **kwargs):
            pass

    monkeypatch.setattr(voice, "pyttsx3", types.SimpleNamespace(init=lambda: _DummyEngine()))

    dire_texte("bonjour", robot_api=robot)

    # Sans play_audio, on doit utiliser speaker.play_file ou speaker.play
    assert (
        len(robot.media.speaker.play_file_calls) + len(robot.media.speaker.play_bytes_calls)
    ) == 1
