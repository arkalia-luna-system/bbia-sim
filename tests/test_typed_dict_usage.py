#!/usr/bin/env python3
"""Tests pour vérifier l'utilisation des TypedDict."""

import pytest
from bbia_sim.utils.types import (
    ConversationEntry,
    DetectionResult,
    RobotStatus,
    TelemetryData,
)
from bbia_sim.robot_api import RobotAPI
from bbia_sim.backends.mujoco_backend import MuJoCoBackend


class TestTypedDictUsage:
    """Tests pour vérifier que les TypedDict sont utilisés correctement."""

    def test_robot_status_type(self):
        """Test que get_status retourne un RobotStatus."""
        backend = MuJoCoBackend()
        backend.connect()
        status = backend.get_status()

        # Vérifier que c'est un dict avec les clés attendues
        assert isinstance(status, dict)
        assert "connected" in status
        assert "current_emotion" in status
        assert "available_joints" in status

    def test_conversation_entry_structure(self):
        """Test que ConversationEntry a la structure attendue."""
        entry: ConversationEntry = {
            "user": "Bonjour",
            "bbia": "Salut !",
            "sentiment": "positive",
            "timestamp": "2025-01-01T00:00:00",
        }

        assert entry["user"] == "Bonjour"
        assert entry["bbia"] == "Salut !"
        assert entry["sentiment"] == "positive"

    def test_detection_result_structure(self):
        """Test que DetectionResult a la structure attendue."""
        detection: DetectionResult = {
            "bbox": [10, 20, 100, 200],
            "confidence": 0.95,
            "class_id": 0,
            "class_name": "person",
            "center": [55, 110],
            "area": 18000,
        }

        assert len(detection["bbox"]) == 4
        assert 0.0 <= detection["confidence"] <= 1.0
        assert detection["class_id"] >= 0
        assert isinstance(detection["class_name"], str)

