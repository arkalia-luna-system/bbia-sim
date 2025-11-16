#!/usr/bin/env python3
"""Types communs pour BBIA - TypedDict pour remplacer dict[str, Any]."""

from typing import TypedDict


class TelemetryData(TypedDict, total=False):
    """Structure de données de télémétrie du robot."""

    step_count: int
    elapsed_time: float
    steps_per_second: float
    average_step_time: float
    current_emotion: str
    emotion_intensity: float
    is_connected: bool
    avg_latency_ms: float
    operations_sampled: int
    imu: dict[str, dict[str, float]]  # IMU data: acceleration, gyroscope, magnetometer
    current_joint_positions: dict[str, float]
    robot_ip: str
    robot_port: int
    backend_type: str
    current_qpos: list[float]
    model_path: str
    latency_ms: float
    fps: float


class GotoTargetParams(TypedDict, total=False):
    """Paramètres pour la commande goto_target."""

    head: list[float] | None
    antennas: list[float] | None
    duration: float
    method: str
    body_yaw: float | None


class SetTargetParams(TypedDict, total=False):
    """Paramètres pour la commande set_target."""

    head: list[float] | None
    antennas: list[float] | None


class SetEmotionParams(TypedDict, total=False):
    """Paramètres pour la commande set_emotion."""

    emotion: str
    intensity: float


class PlayAudioParams(TypedDict, total=False):
    """Paramètres pour la commande play_audio."""

    file_path: str
    volume: float


class LookAtParams(TypedDict, total=False):
    """Paramètres pour la commande look_at."""

    x: float
    y: float
    z: float
    duration: float


# Type pour une position de joint (dict avec nom joint -> position)
JointPositions = dict[str, float]


class MovementRecording(TypedDict, total=False):
    """Enregistrement d'un mouvement.

    Les positions sont des dictionnaires {joint_name: position}
    """

    positions: list[JointPositions]
    duration: float
    start_time: float
    end_time: float


class IMUData(TypedDict):
    """Données IMU (accéléromètre, gyroscope, magnétomètre)."""

    acceleration: dict[str, float]
    gyroscope: dict[str, float]
    magnetometer: dict[str, float]


class MetricsData(TypedDict, total=False):
    """Données de métriques pour le dashboard."""

    timestamp: float
    cpu_percent: float
    memory_percent: float
    fps: float
    latency_ms: float
    step_count: int
    is_connected: bool
    current_emotion: str
    emotion_intensity: float
