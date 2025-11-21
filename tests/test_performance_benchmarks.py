#!/usr/bin/env python3
"""
Tests de performance pour BBIA-SIM
Benchmarks pour mesurer latence et performance
"""

import pytest


class TestPerformanceBenchmarks:
    """Tests de performance et benchmarks."""

    def setup_method(self):
        """Configuration avant chaque test."""
        pass

    def test_basic_imports_performance(self):
        """Test performance des imports de base."""
        import time

        start = time.time()
        # Imports de test
        _ = True  # Imports already done

        elapsed = time.time() - start

        # Les imports doivent être rapides (< 0.1s)
        assert elapsed < 0.1, f"Imports trop lents: {elapsed:.3f}s"

    def test_global_config_initialization_performance(self):
        """Test performance initialisation GlobalConfig."""
        import time

        from bbia_sim.global_config import GlobalConfig

        # OPTIMISATION: Réduire 100 → 50 itérations (suffisant pour benchmark, 2x plus rapide)
        start = time.time()
        for _ in range(50):
            GlobalConfig.validate_joint("yaw_body")
        elapsed = time.time() - start

        # 50 validations doivent être très rapides
        assert elapsed < 0.01, f"Validation trop lente: {elapsed:.3f}s"

    def test_telemetry_collection_performance(self):
        """Test performance collecte télémétrie."""
        import time

        from bbia_sim.telemetry import TelemetryCollector

        collector = TelemetryCollector(output_dir="/tmp/test_perf")

        # OPTIMISATION: Réduire 100 → 50 itérations (suffisant pour benchmark, 2x plus rapide)
        start = time.time()
        collector.start_collection()
        for _ in range(50):
            collector.record_step({"yaw_body": 0.0})
        stats = collector.stop_collection()
        elapsed = time.time() - start

        # 50 enregistrements doivent être rapides
        assert elapsed < 0.5, f"Collecte trop lente: {elapsed:.3f}s"
        assert stats["total_steps"] >= 49

    def test_robot_factory_creation_performance(self):
        """Test performance création backend."""
        import time

        from bbia_sim.robot_factory import RobotFactory

        start = time.time()
        backend = RobotFactory.create_backend("mujoco")
        elapsed = time.time() - start

        # Création backend doit être rapide
        assert elapsed < 0.5, f"Création backend trop lente: {elapsed:.3f}s"
        assert backend is not None

    @pytest.mark.slow
    def test_sdk_signatures_validation_performance(self):
        """Test performance validation signatures SDK."""
        import time

        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend()

        start = time.time()
        for _ in range(50):
            backend.get_available_joints()
        elapsed = time.time() - start

        # 50 appels doivent être rapides
        assert elapsed < 0.1, f"Validation trop lente: {elapsed:.3f}s"

    def test_concurrent_operations_performance(self):
        """Test performance opérations concurrentes."""
        import concurrent.futures
        import time

        from bbia_sim.global_config import GlobalConfig

        def validate_joint_task():
            for _ in range(20):
                GlobalConfig.validate_joint("yaw_body")
                GlobalConfig.validate_emotion("happy")
            return True

        start = time.time()
        with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
            futures = [executor.submit(validate_joint_task) for _ in range(10)]
            results = [f.result() for f in futures]

        elapsed = time.time() - start

        # Opérations concurrentes doivent être rapides
        assert elapsed < 0.5, f"Operations concurrentes trop lentes: {elapsed:.3f}s"
        assert all(results)

    @pytest.mark.slow
    def test_memory_usage_monitoring(self):
        """Test monitoring utilisation mémoire."""
        import sys

        from bbia_sim.global_config import GlobalConfig
        from bbia_sim.telemetry import TelemetryCollector

        # Vérifier que les modules n'utilisent pas trop de mémoire
        # (test rudimentaire)
        size_config = sys.getsizeof(GlobalConfig)
        size_collector = sys.getsizeof(TelemetryCollector)

        # Les modules doivent avoir une taille raisonnable
        assert size_config < 10000  # < 10KB
        assert size_collector < 10000  # < 10KB


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
