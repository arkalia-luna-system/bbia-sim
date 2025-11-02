#!/usr/bin/env python3
"""
Shims de simulation pour robot.io et robot.media
Garantit que ces modules sont toujours disponibles, même en simulation
"""

import logging
from typing import Any

logger = logging.getLogger(__name__)


class SimulationIOModule:
    """Module IO de simulation pour Reachy Mini.
    
    Fournit des implémentations de simulation pour toutes les méthodes IO officielles.
    """

    def __init__(self) -> None:
        """Initialise le module IO de simulation."""
        logger.debug("📦 Module IO de simulation initialisé")

    def get_camera_stream(self) -> Any:
        """Stream vidéo de simulation.
        
        Returns:
            Objet stream simulé (None pour l'instant, à implémenter si nécessaire)
        """
        logger.debug("📹 get_camera_stream() appelé (simulation)")
        return None

    def get_audio_stream(self) -> Any:
        """Stream audio de simulation.
        
        Returns:
            Objet stream simulé (None pour l'instant, à implémenter si nécessaire)
        """
        logger.debug("🎤 get_audio_stream() appelé (simulation)")
        return None

    def get_imu(self) -> dict[str, Any]:
        """Retourne données IMU simulées.
        
        Returns:
            Dict avec accélération, gyroscope, magnétomètre (valeurs neutres)
        """
        return {
            "acceleration": {"x": 0.0, "y": 0.0, "z": -9.81},
            "gyroscope": {"x": 0.0, "y": 0.0, "z": 0.0},
            "magnetometer": {"x": 0.0, "y": 0.0, "z": 0.0},
        }


class SimulationMediaModule:
    """Module Media de simulation pour Reachy Mini.
    
    Fournit des implémentations de simulation pour toutes les méthodes media officielles.
    """

    def __init__(self) -> None:
        """Initialise le module Media de simulation."""
        logger.debug("📦 Module Media de simulation initialisé")
        self._camera = SimulationCamera()
        self._microphone = SimulationMicrophone()
        self._speaker = SimulationSpeaker()

    @property
    def camera(self) -> "SimulationCamera":
        """Accès à la caméra de simulation."""
        return self._camera

    @property
    def microphone(self) -> "SimulationMicrophone":
        """Accès au microphone de simulation."""
        return self._microphone

    @property
    def speaker(self) -> "SimulationSpeaker":
        """Accès au haut-parleur de simulation."""
        return self._speaker

    def play_audio(self, audio_bytes: bytes, volume: float = 1.0) -> None:
        """Joue de l'audio en simulation (log uniquement).
        
        Args:
            audio_bytes: Données audio en bytes
            volume: Volume (0.0-1.0)
        """
        logger.debug(f"🔊 play_audio() simulé ({len(audio_bytes)} bytes, volume={volume})")

    def record_audio(
        self, duration: float = 3.0, sample_rate: int = 16000
    ) -> bytes:
        """Enregistre de l'audio en simulation (retourne silence).
        
        Args:
            duration: Durée en secondes
            sample_rate: Fréquence d'échantillonnage
            
        Returns:
            Bytes audio (silence simulé)
        """
        logger.debug(f"🎤 record_audio() simulé ({duration}s, {sample_rate}Hz)")
        # Retourner silence simulé
        import struct

        num_samples = int(duration * sample_rate)
        return b"".join(struct.pack("<h", 0) for _ in range(num_samples))


class SimulationCamera:
    """Caméra de simulation."""

    def get_image(self) -> Any:
        """Capture une image en simulation.
        
        Returns:
            None (à implémenter si nécessaire avec numpy array simulé)
        """
        logger.debug("📷 get_image() appelé (simulation)")
        return None

    def capture(self) -> Any:
        """Capture une image (alias get_image)."""
        return self.get_image()

    def read(self) -> tuple[bool, Any]:
        """Lit une frame (compatible OpenCV VideoCapture).
        
        Returns:
            Tuple (success, frame) - (False, None) en simulation
        """
        logger.debug("📷 read() appelé (simulation)")
        return (False, None)


class SimulationMicrophone:
    """Microphone de simulation."""

    def record(
        self, duration: float = 3.0, sample_rate: int = 16000
    ) -> bytes:
        """Enregistre de l'audio en simulation.
        
        Args:
            duration: Durée en secondes
            sample_rate: Fréquence d'échantillonnage
            
        Returns:
            Bytes audio (silence simulé)
        """
        logger.debug(f"🎤 record() simulé ({duration}s, {sample_rate}Hz)")
        import struct

        num_samples = int(duration * sample_rate)
        return b"".join(struct.pack("<h", 0) for _ in range(num_samples))


class SimulationSpeaker:
    """Haut-parleur de simulation."""

    def play(self, audio_bytes: bytes) -> None:
        """Joue de l'audio en simulation.
        
        Args:
            audio_bytes: Données audio en bytes
        """
        logger.debug(f"🔊 play() simulé ({len(audio_bytes)} bytes)")

    def play_file(self, file_path: str) -> None:
        """Joue un fichier audio en simulation.
        
        Args:
            file_path: Chemin vers le fichier audio
        """
        logger.debug(f"🔊 play_file() simulé ({file_path})")

