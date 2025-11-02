from io import BytesIO

from bbia_sim.bbia_voice import reconnaitre_parole


class _MediaMic:
    def __init__(self, payload: bytes) -> None:
        self._payload = payload
        self.calls: list[tuple[int, int]] = []

    def record_audio(self, duration: int, sample_rate: int):  # SDK-like
        self.calls.append((duration, sample_rate))
        return self._payload


class _RobotApi:
    def __init__(self, payload: bytes) -> None:
        self.media = _MediaMic(payload)


def _make_wav_bytes() -> bytes:
    import struct
    import wave

    bio = BytesIO()
    with wave.open(bio, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(16000)
        # 10 échantillons silencieux
        wf.writeframes(struct.pack("<10h", *([0] * 10)))
    return bio.getvalue()


def test_reconnaitre_parole_uses_sdk_record_audio(monkeypatch):
    payload = _make_wav_bytes()
    robot = _RobotApi(payload)

    # Monkeypatch de Google STT pour éviter accès réseau
    import src.bbia_sim.bbia_voice as voice

    class _DummyRecognizer:
        def record(self, source):  # type: ignore[no-untyped-def]
            return b"dummy"

        def recognize_google(self, audio, language="fr-FR"):
            return "bonjour"

    class _DummyAudioFile:
        def __init__(self, file_obj):  # type: ignore[no-untyped-def]
            self._f = file_obj

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

    monkeypatch.setattr(voice.sr, "Recognizer", lambda: _DummyRecognizer())
    monkeypatch.setattr(voice.sr, "AudioFile", _DummyAudioFile)

    texte = reconnaitre_parole(duree=1, frequence=16000, robot_api=robot)

    assert texte == "bonjour"
    assert robot.media.calls == [(1, 16000)]
