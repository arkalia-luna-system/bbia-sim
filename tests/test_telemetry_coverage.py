#!/usr/bin/env python3
"""Tests pour le module telemetry.py - Couverture complète."""

import tempfile
import time
from pathlib import Path

import pytest

from bbia_sim.telemetry import TelemetryCollector


class TestTelemetryCollector:
    """Tests pour TelemetryCollector."""

    def test_init_creates_output_dir(self) -> None:
        """Test que l'initialisation crée le répertoire de sortie."""
        with tempfile.TemporaryDirectory() as tmpdir:
            output_path = Path(tmpdir) / "test_artifacts"
            collector = TelemetryCollector(output_dir=str(output_path))
            assert output_path.exists()
            assert collector.output_dir == output_path

    def test_start_collection_initializes_state(self) -> None:
        """Test que start_collection initialise l'état correctement."""
        with tempfile.TemporaryDirectory() as tmpdir:
            collector = TelemetryCollector(output_dir=tmpdir)
            collector.start_collection()

            assert collector.start_time is not None
            assert collector.last_step_time is not None
            assert len(collector.step_times) == 0
            assert len(collector.joint_positions) == 0

    def test_record_step_adds_data(self) -> None:
        """Test que record_step ajoute des données."""
        with tempfile.TemporaryDirectory() as tmpdir:
            collector = TelemetryCollector(output_dir=tmpdir)
            collector.start_collection()

            # Enregistrer quelques steps
            for i in range(5):
                collector.record_step({"joint1": float(i), "joint2": float(i * 2)})
                time.sleep(0.01)  # Petit délai pour avoir des temps différents

            # Vérifier les données
            assert len(collector.step_times) == 4  # N-1 temps de step
            assert len(collector.joint_positions) == 5

    def test_stop_collection_returns_stats(self) -> None:
        """Test que stop_collection retourne les statistiques."""
        with tempfile.TemporaryDirectory() as tmpdir:
            collector = TelemetryCollector(output_dir=tmpdir)
            collector.start_collection()

            # Enregistrer des steps
            for i in range(10):
                collector.record_step({"joint1": float(i) * 0.1})
                time.sleep(0.005)

            stats = collector.stop_collection()

            # Vérifier les statistiques
            assert "total_steps" in stats
            assert "total_time" in stats
            assert "steps_per_second" in stats
            assert "average_step_time" in stats
            assert "max_step_time" in stats
            assert "min_step_time" in stats
            assert "max_drift" in stats
            assert stats["total_steps"] == 9  # N-1

    def test_stop_collection_empty_returns_empty_dict(self) -> None:
        """Test que stop_collection retourne dict vide si pas de données."""
        with tempfile.TemporaryDirectory() as tmpdir:
            collector = TelemetryCollector(output_dir=tmpdir)
            stats = collector.stop_collection()
            assert stats == {}

    def test_export_csv_creates_files(self) -> None:
        """Test que export_csv crée les fichiers CSV."""
        with tempfile.TemporaryDirectory() as tmpdir:
            collector = TelemetryCollector(output_dir=tmpdir)
            collector.start_collection()

            # Enregistrer des steps
            for i in range(5):
                collector.record_step({"joint1": float(i) * 0.1, "joint2": float(i) * 0.2})

            stats = collector.stop_collection()
            csv_path = collector.export_csv("test_output.csv", stats)

            # Vérifier les fichiers
            assert Path(csv_path).exists()
            stats_path = Path(tmpdir) / "stats_test_output.csv"
            assert stats_path.exists()

    def test_get_live_stats_returns_current_state(self) -> None:
        """Test que get_live_stats retourne l'état courant."""
        with tempfile.TemporaryDirectory() as tmpdir:
            collector = TelemetryCollector(output_dir=tmpdir)
            collector.start_collection()

            # Enregistrer quelques steps
            for _ in range(3):
                collector.record_step({"joint1": 0.1})
                time.sleep(0.01)

            live_stats = collector.get_live_stats()

            assert "current_steps" in live_stats
            assert "current_time" in live_stats
            assert "current_sps" in live_stats
            assert "last_step_time" in live_stats
            assert live_stats["current_steps"] == 2  # N-1

    def test_get_live_stats_empty_returns_empty_dict(self) -> None:
        """Test que get_live_stats retourne dict vide si pas de données."""
        with tempfile.TemporaryDirectory() as tmpdir:
            collector = TelemetryCollector(output_dir=tmpdir)
            live_stats = collector.get_live_stats()
            assert live_stats == {}

    def test_max_steps_history_limit(self) -> None:
        """Test que l'historique est limité par _max_steps_history."""
        with tempfile.TemporaryDirectory() as tmpdir:
            collector = TelemetryCollector(output_dir=tmpdir)
            collector._max_steps_history = 100  # Limite réduite pour le test
            collector.step_times = type(collector.step_times)(maxlen=100)
            collector.joint_positions = type(collector.joint_positions)(maxlen=100)

            collector.start_collection()

            # Enregistrer plus de steps que la limite
            for i in range(150):
                collector.record_step({"joint1": float(i)})

            # Vérifier que la limite est respectée
            assert len(collector.joint_positions) <= 100
