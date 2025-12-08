#!/usr/bin/env python3
"""Tests pour validation des baselines de performance.

Valide que les métriques de performance respectent les baselines p50/p95/p99.
"""

import json
import tempfile
from pathlib import Path

from scripts.bbia_performance_benchmarks import BBIAPerformanceBenchmark


class TestPerformanceBaselines:
    """Tests pour validation des baselines de performance."""

    def test_extract_metrics(self) -> None:
        """Test extraction métriques depuis résultats."""
        benchmark = BBIAPerformanceBenchmark()

        # Résultats de test
        results = {
            "timestamp": "2025-12-08T10:00:00",
            "backend": "mujoco",
            "benchmarks": {
                "latency": {
                    "operations": {
                        "get_joint_pos": {
                            "mean_ms": 1.5,
                            "p50_ms": 1.2,
                            "p95_ms": 2.0,
                            "p99_ms": 2.5,
                        },
                    },
                },
            },
        }

        metrics = benchmark._extract_metrics(results)

        assert len(metrics) > 0
        assert any("latency.get_joint_pos.mean_ms" in m[0] for m in metrics)
        assert any("latency.get_joint_pos.p50_ms" in m[0] for m in metrics)
        assert any("latency.get_joint_pos.p95_ms" in m[0] for m in metrics)
        assert any("latency.get_joint_pos.p99_ms" in m[0] for m in metrics)

    def test_save_results_jsonl(self) -> None:
        """Test sauvegarde résultats en JSONL."""
        benchmark = BBIAPerformanceBenchmark()

        results = {
            "timestamp": "2025-12-08T10:00:00",
            "backend": "mujoco",
            "benchmarks": {
                "latency": {
                    "operations": {
                        "get_joint_pos": {
                            "mean_ms": 1.5,
                            "p50_ms": 1.2,
                            "p95_ms": 2.0,
                            "p99_ms": 2.5,
                        },
                    },
                },
            },
        }

        with tempfile.TemporaryDirectory() as tmpdir:
            jsonl_file = Path(tmpdir) / "test.jsonl"
            json_file = Path(tmpdir) / "test.json"
            filepath, jsonl_path = benchmark.save_results(
                results,
                filename=str(json_file),
                jsonl_file=str(jsonl_file),
            )

            assert filepath.exists()
            assert jsonl_path is not None
            assert jsonl_path.exists()

            # Vérifier contenu JSONL
            lines = jsonl_path.read_text().strip().split("\n")
            assert len(lines) > 0

            # Vérifier format JSONL
            for line in lines:
                data = json.loads(line)
                assert "metric" in data
                assert "value" in data
                assert "timestamp" in data
                assert "backend" in data

    def test_validate_baselines_valid(self) -> None:
        """Test validation baseline avec résultats valides."""
        benchmark = BBIAPerformanceBenchmark()

        results = {
            "timestamp": "2025-12-08T10:00:00",
            "backend": "mujoco",
            "benchmarks": {
                "latency": {
                    "operations": {
                        "get_joint_pos": {
                            "mean_ms": 1.5,
                            "p50_ms": 1.2,
                            "p95_ms": 2.0,
                            "p99_ms": 2.5,
                        },
                    },
                },
            },
        }

        # Créer baseline de test
        with tempfile.NamedTemporaryFile(mode="w", suffix=".jsonl", delete=False) as f:
            baseline_file = f.name
            # Baseline avec valeurs similaires (dans la tolérance)
            baseline_data = {
                "metric": "latency.get_joint_pos.p50_ms",
                "value": 1.3,  # Proche de 1.2 (dans tolérance 20%)
                "timestamp": "2025-12-07T10:00:00",
                "backend": "mujoco",
            }
            f.write(json.dumps(baseline_data) + "\n")

        try:
            is_valid, errors = benchmark.validate_baselines(
                results, baseline_file, threshold=0.2
            )

            assert is_valid
            assert len(errors) == 0

        finally:
            Path(baseline_file).unlink()

    def test_validate_baselines_invalid(self) -> None:
        """Test validation baseline avec résultats invalides (régression)."""
        benchmark = BBIAPerformanceBenchmark()

        results = {
            "timestamp": "2025-12-08T10:00:00",
            "backend": "mujoco",
            "benchmarks": {
                "latency": {
                    "operations": {
                        "get_joint_pos": {
                            "p50_ms": 5.0,  # Beaucoup plus lent que baseline
                        },
                    },
                },
            },
        }

        # Créer baseline de test
        with tempfile.NamedTemporaryFile(mode="w", suffix=".jsonl", delete=False) as f:
            baseline_file = f.name
            # Baseline avec valeur plus rapide
            baseline_data = {
                "metric": "latency.get_joint_pos.p50_ms",
                "value": 1.2,  # Beaucoup plus rapide que 5.0
                "timestamp": "2025-12-07T10:00:00",
                "backend": "mujoco",
            }
            f.write(json.dumps(baseline_data) + "\n")

        try:
            is_valid, errors = benchmark.validate_baselines(
                results, baseline_file, threshold=0.2
            )

            assert not is_valid
            assert len(errors) > 0
            assert any("p50_ms" in error for error in errors)

        finally:
            Path(baseline_file).unlink()

    def test_validate_baselines_missing_file(self) -> None:
        """Test validation baseline avec fichier manquant."""
        benchmark = BBIAPerformanceBenchmark()

        results = {
            "timestamp": "2025-12-08T10:00:00",
            "backend": "mujoco",
            "benchmarks": {},
        }

        # Fichier baseline inexistant
        is_valid, errors = benchmark.validate_baselines(
            results, "nonexistent_baseline.jsonl", threshold=0.2
        )

        # Pas de baseline = OK (pas d'erreur)
        assert is_valid
        assert len(errors) == 0

    def test_validate_baselines_p50_p95_p99(self) -> None:
        """Test validation spécifique p50/p95/p99."""
        benchmark = BBIAPerformanceBenchmark()

        results = {
            "timestamp": "2025-12-08T10:00:00",
            "backend": "mujoco",
            "benchmarks": {
                "latency": {
                    "operations": {
                        "get_joint_pos": {
                            "p50_ms": 1.2,
                            "p95_ms": 2.0,
                            "p99_ms": 2.5,
                        },
                    },
                },
            },
        }

        # Créer baseline avec p50/p95/p99
        with tempfile.NamedTemporaryFile(mode="w", suffix=".jsonl", delete=False) as f:
            baseline_file = f.name
            baselines = [
                {"metric": "latency.get_joint_pos.p50_ms", "value": 1.1},
                {"metric": "latency.get_joint_pos.p95_ms", "value": 1.9},
                {"metric": "latency.get_joint_pos.p99_ms", "value": 2.4},
            ]
            for baseline in baselines:
                f.write(json.dumps(baseline) + "\n")

        try:
            is_valid, errors = benchmark.validate_baselines(
                results, baseline_file, threshold=0.2
            )

            # Toutes les métriques sont dans la tolérance
            assert is_valid
            assert len(errors) == 0

        finally:
            Path(baseline_file).unlink()
