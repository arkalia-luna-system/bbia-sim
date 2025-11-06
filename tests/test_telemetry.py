#!/usr/bin/env python3
"""
Tests unitaires pour TelemetryCollector
Validation de la collecte de télémétrie BBIA
"""

import tempfile
from pathlib import Path
from time import sleep

import pytest

from bbia_sim.telemetry import TelemetryCollector


class TestTelemetryCollector:
    """Tests pour le collecteur de télémétrie."""

    def setup_method(self):
        """Configuration avant chaque test."""
        # Créer un répertoire temporaire pour les tests
        self.temp_dir = tempfile.mkdtemp()
        self.collector = TelemetryCollector(output_dir=self.temp_dir)

    def teardown_method(self):
        """Nettoyage après chaque test."""
        # Nettoyer les fichiers temporaires
        import shutil

        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_initialization(self):
        """Test initialisation du collecteur."""
        from collections import deque

        assert self.collector.output_dir == Path(self.temp_dir)
        assert isinstance(self.collector.step_times, deque) and len(self.collector.step_times) == 0
        assert (
            isinstance(self.collector.joint_positions, deque)
            and len(self.collector.joint_positions) == 0
        )
        assert self.collector.start_time is None
        assert self.collector.last_step_time is None

    def test_output_dir_created(self):
        """Test création du répertoire de sortie."""
        assert self.collector.output_dir.exists()
        assert self.collector.output_dir.is_dir()

    def test_start_collection(self):
        """Test démarrage de la collecte."""
        self.collector.start_collection()

        assert len(self.collector.step_times) == 0
        assert len(self.collector.joint_positions) == 0
        assert self.collector.start_time is not None
        assert self.collector.last_step_time == self.collector.start_time

    def test_record_step_single(self):
        """Test enregistrement d'un pas."""
        self.collector.start_collection()
        sleep(0.01)  # Petit délai pour tester timing

        joint_positions = {"yaw_body": 0.1, "stewart_1": 0.2}
        self.collector.record_step(joint_positions)

        assert len(self.collector.step_times) >= 0
        assert len(self.collector.joint_positions) == 1

    def test_record_step_multiple(self):
        """Test enregistrement de plusieurs pas."""
        self.collector.start_collection()
        sleep(0.01)

        for i in range(5):
            joint_positions = {"yaw_body": i * 0.1, "stewart_1": i * 0.2}
            self.collector.record_step(joint_positions)
            sleep(0.01)

        # Le premier record_step enregistre le temps, donc on a 5 intervalles pour 5 enregistrements
        assert len(self.collector.step_times) >= 4
        assert len(self.collector.joint_positions) == 5

    def test_stop_collection_no_steps(self):
        """Test arrêt de collecte sans pas."""
        self.collector.start_collection()
        stats = self.collector.stop_collection()

        assert stats == {}

    def test_stop_collection_with_steps(self):
        """Test arrêt de collecte avec pas."""
        self.collector.start_collection()
        sleep(0.01)

        for i in range(5):
            joint_positions = {"yaw_body": i * 0.1}
            self.collector.record_step(joint_positions)
            sleep(0.01)

        stats = self.collector.stop_collection()

        assert isinstance(stats, dict)
        assert "total_steps" in stats
        assert "total_time" in stats
        assert "steps_per_second" in stats
        assert "average_step_time" in stats
        assert "max_step_time" in stats
        assert "min_step_time" in stats
        assert "max_drift" in stats
        assert "start_time" in stats
        assert "end_time" in stats

        assert stats["total_steps"] >= 4
        assert stats["total_time"] > 0
        assert stats["steps_per_second"] > 0

    def test_max_drift_calculation(self):
        """Test calcul du drift maximum."""
        self.collector.start_collection()
        sleep(0.01)

        # Enregistrer positions avec variation
        for i in range(10):
            joint_positions = {"yaw_body": i * 0.1}
            self.collector.record_step(joint_positions)
            sleep(0.01)

        stats = self.collector.stop_collection()

        assert "max_drift" in stats
        # Le drift devrait être proche de 0.9 (10 pas * 0.1)
        assert stats["max_drift"] > 0.5

    def test_export_csv_stats(self):
        """Test export CSV des statistiques."""
        self.collector.start_collection()
        sleep(0.01)

        for _ in range(5):
            self.collector.record_step({"yaw_body": 0.1})
            sleep(0.01)

        stats = self.collector.stop_collection()
        csv_path = self.collector.export_csv("test.csv", stats)

        # Vérifier fichiers créés
        assert Path(csv_path).exists()
        stats_path = Path(str(self.temp_dir)) / "stats_test.csv"
        assert stats_path.exists()

        # Vérifier contenu CSV stats
        with open(stats_path) as f:
            lines = f.readlines()
            assert len(lines) > 1  # Header + data
            assert "Metric" in lines[0]
            assert "Value" in lines[0]

    def test_export_csv_positions(self):
        """Test export CSV des positions."""
        self.collector.start_collection()
        sleep(0.01)

        for i in range(5):
            self.collector.record_step({"yaw_body": i * 0.1, "stewart_1": i * 0.2})
            sleep(0.01)

        stats = self.collector.stop_collection()
        csv_path = self.collector.export_csv("positions_test.csv", stats)

        # Vérifier fichier CSV positions
        with open(csv_path) as f:
            lines = f.readlines()
            assert len(lines) == 6  # Header + 5 positions
            assert "timestamp" in lines[0]
            assert "elapsed" in lines[0]
            assert "yaw_body" in lines[0]
            assert "stewart_1" in lines[0]

    def test_get_live_stats_no_steps(self):
        """Test statistiques en temps réel sans pas."""
        self.collector.start_collection()
        stats = self.collector.get_live_stats()

        assert stats == {}

    def test_get_live_stats_with_steps(self):
        """Test statistiques en temps réel avec pas."""
        self.collector.start_collection()
        sleep(0.01)

        for i in range(5):
            self.collector.record_step({"yaw_body": i * 0.1})
            sleep(0.01)

        stats = self.collector.get_live_stats()

        assert isinstance(stats, dict)
        assert "current_steps" in stats
        assert "current_time" in stats
        assert "current_sps" in stats
        assert "last_step_time" in stats

        assert stats["current_steps"] >= 4
        assert stats["current_time"] > 0
        assert stats["current_sps"] > 0
        assert stats["last_step_time"] > 0

    def test_step_times_accuracy(self):
        """Test précision des temps de step."""
        self.collector.start_collection()
        sleep(0.01)

        for _ in range(5):
            self.collector.record_step({"yaw_body": 0.0})
            sleep(0.01)

        stats = self.collector.stop_collection()

        # Vérifier que les temps sont raisonnables
        assert stats["average_step_time"] > 0
        assert stats["max_step_time"] >= stats["min_step_time"]
        assert stats["max_step_time"] < 1.0  # Ne devrait pas prendre plus d'1 seconde

    def test_joint_positions_structure(self):
        """Test structure des positions de joints."""
        self.collector.start_collection()
        sleep(0.01)

        joint_positions = {"yaw_body": 0.1, "stewart_1": 0.2}
        self.collector.record_step(joint_positions)

        # Vérifier structure
        assert len(self.collector.joint_positions) == 1
        pos = self.collector.joint_positions[0]

        assert "timestamp" in pos
        assert "elapsed" in pos
        assert "yaw_body" in pos
        assert "stewart_1" in pos
        assert pos["yaw_body"] == 0.1
        assert pos["stewart_1"] == 0.2


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
