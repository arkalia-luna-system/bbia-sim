#!/usr/bin/env python3
"""
Mock ReachyMiniBackend pour tests sans robot physique.

Permet de tester les fonctionnalités qui nécessitent normalement un robot physique
sans avoir besoin du hardware réel.
"""

import time

import numpy as np

from bbia_sim.robot_api import RobotAPI


class ReachyMiniMock(RobotAPI):
    """Mock du ReachyMiniBackend pour tests sans robot physique."""

    def __init__(self) -> None:
        """Initialise le mock."""
        super().__init__()
        self._joint_positions: dict[str, float] = {}
        self._connected = False
        self.step_count = 0

        # Joints disponibles (même liste que ReachyMiniBackend)
        self._available_joints = [
            "yaw_body",
            "stewart_1",
            "stewart_2",
            "stewart_3",
            "stewart_4",
            "stewart_5",
            "stewart_6",
            "left_antenna",
            "right_antenna",
        ]

        # Positions initiales
        for joint in self._available_joints:
            self._joint_positions[joint] = 0.0

        # Limites physiques (même que ReachyMiniBackend)
        self.joint_limits = {
            "yaw_body": (-np.pi, np.pi),
            "stewart_1": (-1.57, 1.57),
            "stewart_2": (-1.57, 1.57),
            "stewart_3": (-1.57, 1.57),
            "stewart_4": (-1.57, 1.57),
            "stewart_5": (-1.57, 1.57),
            "stewart_6": (-1.57, 1.57),
            "left_antenna": (-0.3, 0.3),  # Limites sûres
            "right_antenna": (-0.3, 0.3),
        }

        # Attributs mock
        self.io = MockIO()
        self.media = MockMedia()
        self.robot = MockRobotSDK()

    def connect(self) -> bool:
        """Simule la connexion au robot."""
        self._connected = True
        self.is_connected = True
        return True

    def disconnect(self) -> bool:
        """Simule la déconnexion du robot."""
        self._connected = False
        self.is_connected = False
        return True

    def get_available_joints(self) -> list[str]:
        """Retourne la liste des joints disponibles."""
        return self._available_joints.copy()

    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        """Simule la définition de position d'un joint."""
        if not self._connected:
            return False

        if joint_name not in self._available_joints:
            return False

        # Valider et clamp position
        valid, clamped_pos = self._validate_joint_pos(joint_name, position)
        if not valid:
            return False

        # Appliquer limites physiques
        if joint_name in self.joint_limits:
            min_pos, max_pos = self.joint_limits[joint_name]
            clamped_pos = max(min_pos, min(max_pos, clamped_pos))

        self._joint_positions[joint_name] = clamped_pos
        return True

    def get_joint_pos(self, joint_name: str) -> float | None:
        """Simule la récupération de position d'un joint."""
        if not self._connected:
            return None

        if joint_name not in self._available_joints:
            return None

        return self._joint_positions.get(joint_name, 0.0)

    def step(self) -> bool:
        """Simule un pas de simulation."""
        if not self._connected:
            return False

        self.step_count += 1
        # Petit délai pour simuler latence réelle
        time.sleep(0.001)
        return True

    def emergency_stop(self) -> bool:
        """Simule l'arrêt d'urgence."""
        if not self._connected:
            return False

        # Réinitialiser toutes les positions à zéro
        for joint in self._available_joints:
            self._joint_positions[joint] = 0.0

        return True

    def goto_target(
        self,
        head: np.ndarray | None = None,
        duration: float = 1.0,
        method: str = "minjerk",
    ) -> bool:
        """Simule goto_target (interpolation vers pose)."""
        if not self._connected:
            return False

        if head is None:
            return False

        # Simuler interpolation simple
        time.sleep(min(duration, 0.1))  # Délai simulé max 0.1s pour tests
        return True

    @property
    def is_connected(self) -> bool:
        """État de connexion."""
        return self._connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        """Setter pour is_connected."""
        self._connected = value


class MockIO:
    """Mock du module IO du robot."""

    def __init__(self) -> None:
        """Initialise le mock IO."""
        pass


class MockMedia:
    """Mock du module Media du robot."""

    def __init__(self) -> None:
        """Initialise le mock Media."""
        self.camera = MockCamera()
        self.microphone = MockMicrophone()
        self.speaker = MockSpeaker()


class MockCamera:
    """Mock de la caméra."""

    def __init__(self) -> None:
        """Initialise le mock caméra."""
        pass

    def get_frame(self) -> np.ndarray | None:
        """Simule récupération d'une frame."""
        # Retourner une image vide (640x480x3)
        return np.zeros((480, 640, 3), dtype=np.uint8)


class MockMicrophone:
    """Mock du microphone."""

    def __init__(self) -> None:
        """Initialise le mock microphone."""
        pass


class MockSpeaker:
    """Mock du haut-parleur."""

    def __init__(self) -> None:
        """Initialise le mock speaker."""
        pass


class MockRobotSDK:
    """Mock du robot SDK officiel."""

    def __init__(self) -> None:
        """Initialise le mock robot SDK."""
        self.io = MockIO()
        self.media = MockMedia()
        self.head = MockHead()
        self.joints = MockJoints()

    def get_current_joint_positions(self) -> dict[str, float]:
        """Simule récupération positions joints."""
        return {}


class MockHead:
    """Mock de la tête du robot."""

    def __init__(self) -> None:
        """Initialise le mock head."""
        pass


class MockJoints:
    """Mock des joints du robot."""

    def __init__(self) -> None:
        """Initialise le mock joints."""
        pass
