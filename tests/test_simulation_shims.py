#!/usr/bin/env python3
"""Tests pour backends/simulation_shims.py - Shims de simulation pour robot.io et robot.media."""

from __future__ import annotations

from bbia_sim.backends.simulation_shims import (
    SimulationCamera,
    SimulationIOModule,
    SimulationMediaModule,
    SimulationMicrophone,
    SimulationSpeaker,
)


class TestSimulationIOModule:
    """Tests pour SimulationIOModule."""

    def test_init(self) -> None:
        """Test initialisation."""
        io_module = SimulationIOModule()
        assert io_module is not None

    def test_get_camera_stream(self) -> None:
        """Test get_camera_stream."""
        io_module = SimulationIOModule()
        stream = io_module.get_camera_stream()
        assert stream is None

    def test_get_audio_stream(self) -> None:
        """Test get_audio_stream."""
        io_module = SimulationIOModule()
        stream = io_module.get_audio_stream()
        assert stream is None

    def test_get_imu(self) -> None:
        """Test get_imu."""
        io_module = SimulationIOModule()
        imu_data = io_module.get_imu()

        assert isinstance(imu_data, dict)
        assert "acceleration" in imu_data
        assert "gyroscope" in imu_data
        assert "magnetometer" in imu_data

        assert imu_data["acceleration"]["x"] == 0.0
        assert imu_data["acceleration"]["y"] == 0.0
        assert imu_data["acceleration"]["z"] == -9.81

        assert imu_data["gyroscope"]["x"] == 0.0
        assert imu_data["gyroscope"]["y"] == 0.0
        assert imu_data["gyroscope"]["z"] == 0.0

        assert imu_data["magnetometer"]["x"] == 0.0
        assert imu_data["magnetometer"]["y"] == 0.0
        assert imu_data["magnetometer"]["z"] == 0.0


class TestSimulationMediaModule:
    """Tests pour SimulationMediaModule."""

    def test_init(self) -> None:
        """Test initialisation."""
        media_module = SimulationMediaModule()
        assert media_module is not None
        assert media_module.camera is not None
        assert media_module.microphone is not None
        assert media_module.speaker is not None

    def test_camera_property(self) -> None:
        """Test propriété camera."""
        media_module = SimulationMediaModule()
        camera = media_module.camera
        assert isinstance(camera, SimulationCamera)

    def test_microphone_property(self) -> None:
        """Test propriété microphone."""
        media_module = SimulationMediaModule()
        microphone = media_module.microphone
        assert isinstance(microphone, SimulationMicrophone)

    def test_speaker_property(self) -> None:
        """Test propriété speaker."""
        media_module = SimulationMediaModule()
        speaker = media_module.speaker
        assert isinstance(speaker, SimulationSpeaker)

    def test_play_audio(self) -> None:
        """Test play_audio."""
        media_module = SimulationMediaModule()
        audio_bytes = b"test_audio_data"
        # Ne doit pas lever d'exception
        media_module.play_audio(audio_bytes, volume=0.5)

    def test_play_audio_full_volume(self) -> None:
        """Test play_audio avec volume maximum."""
        media_module = SimulationMediaModule()
        audio_bytes = b"test_audio_data"
        media_module.play_audio(audio_bytes, volume=1.0)

    def test_record_audio_default(self) -> None:
        """Test record_audio avec paramètres par défaut."""
        media_module = SimulationMediaModule()
        audio = media_module.record_audio()
        assert isinstance(audio, bytes)
        assert len(audio) > 0

    def test_record_audio_custom(self) -> None:
        """Test record_audio avec paramètres personnalisés."""
        media_module = SimulationMediaModule()
        audio = media_module.record_audio(duration=1.0, sample_rate=44100)
        assert isinstance(audio, bytes)
        # Vérifier taille approximative (1.0s * 44100Hz * 2 bytes/sample)
        assert len(audio) > 0


class TestSimulationCamera:
    """Tests pour SimulationCamera."""

    def test_get_image(self) -> None:
        """Test get_image."""
        camera = SimulationCamera()
        image = camera.get_image()
        assert image is None

    def test_capture(self) -> None:
        """Test capture."""
        camera = SimulationCamera()
        image = camera.capture()
        assert image is None

    def test_read(self) -> None:
        """Test read."""
        camera = SimulationCamera()
        success, frame = camera.read()
        assert success is False
        assert frame is None


class TestSimulationMicrophone:
    """Tests pour SimulationMicrophone."""

    def test_record_default(self) -> None:
        """Test record avec paramètres par défaut."""
        microphone = SimulationMicrophone()
        audio = microphone.record()
        assert isinstance(audio, bytes)
        assert len(audio) > 0

    def test_record_custom(self) -> None:
        """Test record avec paramètres personnalisés."""
        microphone = SimulationMicrophone()
        audio = microphone.record(duration=2.0, sample_rate=48000)
        assert isinstance(audio, bytes)
        # Vérifier taille approximative (2.0s * 48000Hz * 2 bytes/sample)
        assert len(audio) > 0

    def test_record_short(self) -> None:
        """Test record avec durée courte."""
        microphone = SimulationMicrophone()
        audio = microphone.record(duration=0.1, sample_rate=16000)
        assert isinstance(audio, bytes)
        assert len(audio) > 0


class TestSimulationSpeaker:
    """Tests pour SimulationSpeaker."""

    def test_play(self) -> None:
        """Test play."""
        speaker = SimulationSpeaker()
        audio_bytes = b"test_audio_data"
        # Ne doit pas lever d'exception
        speaker.play(audio_bytes)

    def test_play_empty(self) -> None:
        """Test play avec données vides."""
        speaker = SimulationSpeaker()
        # Ne doit pas lever d'exception
        speaker.play(b"")

    def test_play_file(self) -> None:
        """Test play_file."""
        speaker = SimulationSpeaker()
        file_path = "/path/to/audio.wav"
        # Ne doit pas lever d'exception
        speaker.play_file(file_path)
