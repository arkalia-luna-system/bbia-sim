#!/usr/bin/env python3
"""
BBIA Performance Benchmarks - Tests de performance détaillés
Benchmarks complets pour latence, charge, mémoire et CPU
"""

import json
import logging
import statistics

# Ajouter le chemin src pour les imports
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# BBIAVoice n'existe pas encore - utiliser les fonctions directement
from bbia_sim.bbia_behavior import BBIABehaviorManager
from bbia_sim.bbia_emotions import BBIAEmotions
from bbia_sim.bbia_vision import BBIAVision
from bbia_sim.robot_factory import RobotFactory

logger = logging.getLogger(__name__)


class BBIAPerformanceBenchmark:
    """Classe pour exécuter des benchmarks de performance BBIA."""

    def __init__(self, backend: str = "mujoco"):
        """Initialise le benchmark."""
        self.backend = backend
        self.robot = None
        self.results = {}

        # Modules BBIA
        self.emotions = BBIAEmotions()
        self.vision = BBIAVision()
        # self.voice = BBIAVoice()  # Pas encore implémenté
        self.behavior_manager = BBIABehaviorManager()

        # Configuration des tests
        self.test_config = {
            "latency_tests": {"iterations": 1000, "warmup": 100},
            "load_tests": {
                "concurrent_clients": [1, 5, 10, 20, 50],
                "requests_per_client": 100,
                "timeout": 30.0,
            },
            "memory_tests": {"iterations": 100, "check_interval": 10},
            "cpu_tests": {"duration": 60.0, "sample_interval": 0.1},  # secondes
        }

    def initialize_robot(self) -> bool:
        """Initialise le robot pour les tests."""
        try:
            self.robot = RobotFactory.create_backend(self.backend)
            if self.robot:
                connected = self.robot.connect()
                logger.info(
                    f"Robot {self.backend} {'connecté' if connected else 'en simulation'}"
                )
                return True
            return False
        except Exception as e:
            logger.error(f"Erreur initialisation robot: {e}")
            return False

    def benchmark_latency(self) -> dict[str, Any]:
        """Benchmark de latence des opérations critiques."""
        logger.info("🚀 Démarrage benchmark latence...")

        if not self.robot:
            if not self.initialize_robot():
                return {"error": "Robot non disponible"}

        results = {
            "timestamp": datetime.now().isoformat(),
            "backend": self.backend,
            "operations": {},
        }

        # Test 1: Latence get_joint_pos
        latencies = []
        joints = self.robot.get_available_joints()

        # Warmup
        for _ in range(self.test_config["latency_tests"]["warmup"]):
            for joint in joints[:3]:  # Test sur 3 joints seulement
                self.robot.get_joint_pos(joint)

        # Test réel
        for _ in range(self.test_config["latency_tests"]["iterations"]):
            for joint in joints[:3]:
                start_time = time.perf_counter()
                self.robot.get_joint_pos(joint)
                end_time = time.perf_counter()
                latencies.append((end_time - start_time) * 1000)  # ms

        results["operations"]["get_joint_pos"] = {
            "mean_ms": statistics.mean(latencies),
            "median_ms": statistics.median(latencies),
            "std_ms": statistics.stdev(latencies),
            "min_ms": min(latencies),
            "max_ms": max(latencies),
            "p95_ms": np.percentile(latencies, 95),
            "p99_ms": np.percentile(latencies, 99),
            "samples": len(latencies),
        }

        # Test 2: Latence set_joint_pos
        latencies = []
        test_joint = joints[0] if joints else "yaw_body"

        # Warmup
        for _ in range(self.test_config["latency_tests"]["warmup"]):
            self.robot.set_joint_pos(test_joint, 0.1)

        # Test réel
        for _ in range(self.test_config["latency_tests"]["iterations"]):
            start_time = time.perf_counter()
            self.robot.set_joint_pos(test_joint, 0.1)
            end_time = time.perf_counter()
            latencies.append((end_time - start_time) * 1000)  # ms

        results["operations"]["set_joint_pos"] = {
            "mean_ms": statistics.mean(latencies),
            "median_ms": statistics.median(latencies),
            "std_ms": statistics.stdev(latencies),
            "min_ms": min(latencies),
            "max_ms": max(latencies),
            "p95_ms": np.percentile(latencies, 95),
            "p99_ms": np.percentile(latencies, 99),
            "samples": len(latencies),
        }

        # Test 3: Latence set_emotion
        latencies = []
        test_emotions = ["happy", "sad", "neutral", "excited"]

        # Warmup
        for _ in range(self.test_config["latency_tests"]["warmup"]):
            self.robot.set_emotion("neutral", 0.5)

        # Test réel
        for _ in range(self.test_config["latency_tests"]["iterations"]):
            emotion = test_emotions[_ % len(test_emotions)]
            start_time = time.perf_counter()
            self.robot.set_emotion(emotion, 0.5)
            end_time = time.perf_counter()
            latencies.append((end_time - start_time) * 1000)  # ms

        results["operations"]["set_emotion"] = {
            "mean_ms": statistics.mean(latencies),
            "median_ms": statistics.median(latencies),
            "std_ms": statistics.stdev(latencies),
            "min_ms": min(latencies),
            "max_ms": max(latencies),
            "p95_ms": np.percentile(latencies, 95),
            "p99_ms": np.percentile(latencies, 99),
            "samples": len(latencies),
        }

        # Test 4: Latence get_telemetry
        latencies = []

        # Warmup
        for _ in range(self.test_config["latency_tests"]["warmup"]):
            self.robot.get_telemetry()

        # Test réel
        for _ in range(self.test_config["latency_tests"]["iterations"]):
            start_time = time.perf_counter()
            self.robot.get_telemetry()
            end_time = time.perf_counter()
            latencies.append((end_time - start_time) * 1000)  # ms

        results["operations"]["get_telemetry"] = {
            "mean_ms": statistics.mean(latencies),
            "median_ms": statistics.median(latencies),
            "std_ms": statistics.stdev(latencies),
            "min_ms": min(latencies),
            "max_ms": max(latencies),
            "p95_ms": np.percentile(latencies, 95),
            "p99_ms": np.percentile(latencies, 99),
            "samples": len(latencies),
        }

        logger.info("✅ Benchmark latence terminé")
        return results

    def benchmark_load(self) -> dict[str, Any]:
        """Benchmark de charge avec clients concurrents."""
        logger.info("🚀 Démarrage benchmark charge...")

        if not self.robot:
            if not self.initialize_robot():
                return {"error": "Robot non disponible"}

        results = {
            "timestamp": datetime.now().isoformat(),
            "backend": self.backend,
            "load_tests": {},
        }

        def simulate_client(client_id: int, requests: int) -> dict[str, Any]:
            """Simule un client avec plusieurs requêtes."""
            client_results = {
                "client_id": client_id,
                "requests": requests,
                "successful_requests": 0,
                "failed_requests": 0,
                "total_time": 0.0,
                "latencies": [],
            }

            start_time = time.time()

            for i in range(requests):
                try:
                    # Simuler différentes opérations
                    operation = i % 4

                    if operation == 0:
                        # get_joint_pos
                        joint_start = time.perf_counter()
                        self.robot.get_joint_pos("yaw_body")
                        joint_end = time.perf_counter()
                        client_results["latencies"].append(
                            (joint_end - joint_start) * 1000
                        )

                    elif operation == 1:
                        # set_joint_pos
                        joint_start = time.perf_counter()
                        self.robot.set_joint_pos("yaw_body", 0.1)
                        joint_end = time.perf_counter()
                        client_results["latencies"].append(
                            (joint_end - joint_start) * 1000
                        )

                    elif operation == 2:
                        # set_emotion
                        emotion_start = time.perf_counter()
                        self.robot.set_emotion("happy", 0.5)
                        emotion_end = time.perf_counter()
                        client_results["latencies"].append(
                            (emotion_end - emotion_start) * 1000
                        )

                    elif operation == 3:
                        # get_telemetry
                        telemetry_start = time.perf_counter()
                        self.robot.get_telemetry()
                        telemetry_end = time.perf_counter()
                        client_results["latencies"].append(
                            (telemetry_end - telemetry_start) * 1000
                        )

                    client_results["successful_requests"] += 1

                except Exception as e:
                    logger.warning(f"Client {client_id} requête {i} échouée: {e}")
                    client_results["failed_requests"] += 1

            client_results["total_time"] = time.time() - start_time
            return client_results

        # Tests avec différents nombres de clients concurrents
        for num_clients in self.test_config["load_tests"]["concurrent_clients"]:
            logger.info(f"Test charge: {num_clients} clients concurrents")

            requests_per_client = self.test_config["load_tests"]["requests_per_client"]

            start_time = time.time()

            with ThreadPoolExecutor(max_workers=num_clients) as executor:
                futures = [
                    executor.submit(simulate_client, i, requests_per_client)
                    for i in range(num_clients)
                ]

                client_results = []
                for future in as_completed(
                    futures, timeout=self.test_config["load_tests"]["timeout"]
                ):
                    try:
                        result = future.result()
                        client_results.append(result)
                    except Exception as e:
                        logger.error(f"Erreur client: {e}")

            total_time = time.time() - start_time

            # Calculer les métriques agrégées
            total_requests = sum(
                r["successful_requests"] + r["failed_requests"] for r in client_results
            )
            successful_requests = sum(r["successful_requests"] for r in client_results)
            failed_requests = sum(r["failed_requests"] for r in client_results)

            all_latencies = []
            for result in client_results:
                all_latencies.extend(result["latencies"])

            results["load_tests"][f"{num_clients}_clients"] = {
                "total_requests": total_requests,
                "successful_requests": successful_requests,
                "failed_requests": failed_requests,
                "success_rate": (
                    successful_requests / total_requests if total_requests > 0 else 0
                ),
                "total_time": total_time,
                "requests_per_second": (
                    total_requests / total_time if total_time > 0 else 0
                ),
                "latency_stats": {
                    "mean_ms": statistics.mean(all_latencies) if all_latencies else 0,
                    "median_ms": (
                        statistics.median(all_latencies) if all_latencies else 0
                    ),
                    "p95_ms": np.percentile(all_latencies, 95) if all_latencies else 0,
                    "p99_ms": np.percentile(all_latencies, 99) if all_latencies else 0,
                    "max_ms": max(all_latencies) if all_latencies else 0,
                },
                "clients": client_results,
            }

        logger.info("✅ Benchmark charge terminé")
        return results

    def benchmark_memory(self) -> dict[str, Any]:
        """Benchmark de consommation mémoire."""
        logger.info("🚀 Démarrage benchmark mémoire...")

        import gc

        import psutil

        results = {
            "timestamp": datetime.now().isoformat(),
            "backend": self.backend,
            "memory_tests": {},
        }

        # Test 1: Consommation mémoire des modules BBIA
        gc.collect()  # Nettoyer avant test
        initial_memory = psutil.Process().memory_info().rss / 1024 / 1024  # MB

        # Créer plusieurs instances des modules
        modules_memory = []
        for i in range(self.test_config["memory_tests"]["iterations"]):
            if i % self.test_config["memory_tests"]["check_interval"] == 0:
                current_memory = psutil.Process().memory_info().rss / 1024 / 1024
                modules_memory.append(current_memory - initial_memory)

            # Créer des instances temporaires
            temp_emotions = BBIAEmotions()
            temp_vision = BBIAVision()
            # temp_voice = BBIAVoice()  # Pas encore implémenté
            temp_behavior = BBIABehaviorManager()

            # Simuler utilisation
            temp_emotions.set_emotion("happy", 0.8)
            temp_vision.scan_environment()
            # temp_voice.speak("Test")  # Pas encore implémenté
            temp_behavior.run_behavior("greeting")

        final_memory = psutil.Process().memory_info().rss / 1024 / 1024
        peak_memory = max(modules_memory) if modules_memory else 0

        results["memory_tests"]["bbia_modules"] = {
            "initial_memory_mb": initial_memory,
            "final_memory_mb": final_memory,
            "peak_memory_mb": peak_memory,
            "memory_growth_mb": final_memory - initial_memory,
            "samples": len(modules_memory),
        }

        # Test 2: Consommation mémoire du robot
        if self.robot:
            gc.collect()
            robot_initial_memory = psutil.Process().memory_info().rss / 1024 / 1024

            # Simuler utilisation intensive du robot
            joints = self.robot.get_available_joints()
            for _ in range(100):
                for joint in joints[:5]:  # Test sur 5 joints
                    self.robot.get_joint_pos(joint)
                    self.robot.set_joint_pos(joint, 0.1)
                self.robot.get_telemetry()

            robot_final_memory = psutil.Process().memory_info().rss / 1024 / 1024

            results["memory_tests"]["robot_operations"] = {
                "initial_memory_mb": robot_initial_memory,
                "final_memory_mb": robot_final_memory,
                "memory_growth_mb": robot_final_memory - robot_initial_memory,
            }

        logger.info("✅ Benchmark mémoire terminé")
        return results

    def benchmark_cpu(self) -> dict[str, Any]:
        """Benchmark de consommation CPU."""
        logger.info("🚀 Démarrage benchmark CPU...")

        import psutil

        results = {
            "timestamp": datetime.now().isoformat(),
            "backend": self.backend,
            "cpu_tests": {},
        }

        # Test 1: CPU pendant opérations robot
        if self.robot:
            cpu_samples = []
            start_time = time.time()

            while time.time() - start_time < self.test_config["cpu_tests"]["duration"]:
                # Mesurer CPU avant opération
                cpu_before = psutil.cpu_percent(interval=0.1)

                # Exécuter opérations robot
                joints = self.robot.get_available_joints()
                for joint in joints[:3]:
                    self.robot.get_joint_pos(joint)
                    self.robot.set_joint_pos(joint, 0.1)
                self.robot.set_emotion("happy", 0.5)
                self.robot.get_telemetry()

                # Mesurer CPU après opération
                cpu_after = psutil.cpu_percent(interval=0.1)

                cpu_samples.append(
                    {
                        "timestamp": time.time(),
                        "cpu_before": cpu_before,
                        "cpu_after": cpu_after,
                        "cpu_delta": cpu_after - cpu_before,
                    }
                )

                time.sleep(self.test_config["cpu_tests"]["sample_interval"])

            cpu_values = [sample["cpu_after"] for sample in cpu_samples]

            results["cpu_tests"]["robot_operations"] = {
                "duration": self.test_config["cpu_tests"]["duration"],
                "samples": len(cpu_samples),
                "cpu_stats": {
                    "mean": statistics.mean(cpu_values),
                    "median": statistics.median(cpu_values),
                    "max": max(cpu_values),
                    "min": min(cpu_values),
                    "std": statistics.stdev(cpu_values),
                },
                "cpu_samples": cpu_samples,
            }

        logger.info("✅ Benchmark CPU terminé")
        return results

    def run_all_benchmarks(self) -> dict[str, Any]:
        """Exécute tous les benchmarks."""
        logger.info("🚀 Démarrage suite complète de benchmarks BBIA...")

        all_results = {
            "timestamp": datetime.now().isoformat(),
            "backend": self.backend,
            "version": "1.2.0",
            "benchmarks": {},
        }

        try:
            # Benchmark latence
            all_results["benchmarks"]["latency"] = self.benchmark_latency()

            # Benchmark charge
            all_results["benchmarks"]["load"] = self.benchmark_load()

            # Benchmark mémoire
            all_results["benchmarks"]["memory"] = self.benchmark_memory()

            # Benchmark CPU
            all_results["benchmarks"]["cpu"] = self.benchmark_cpu()

            logger.info("✅ Tous les benchmarks terminés avec succès")

        except Exception as e:
            logger.error(f"❌ Erreur lors des benchmarks: {e}")
            all_results["error"] = str(e)

        return all_results

    def save_results(self, results: dict[str, Any], filename: Optional[str] = None):
        """Sauvegarde les résultats des benchmarks."""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"bbia_benchmarks_{self.backend}_{timestamp}.json"

        filepath = Path("artifacts") / filename
        filepath.parent.mkdir(exist_ok=True)

        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2, ensure_ascii=False)

        logger.info(f"📊 Résultats sauvegardés: {filepath}")
        return filepath

    def generate_report(self, results: dict[str, Any]) -> str:
        """Génère un rapport textuel des benchmarks."""
        report = []
        report.append("=" * 80)
        report.append("BBIA PERFORMANCE BENCHMARKS REPORT")
        report.append("=" * 80)
        report.append(f"Timestamp: {results['timestamp']}")
        report.append(f"Backend: {results['backend']}")
        report.append(f"Version: {results['version']}")
        report.append("")

        # Rapport latence
        if "latency" in results["benchmarks"]:
            report.append("LATENCY BENCHMARKS")
            report.append("-" * 40)
            latency_data = results["benchmarks"]["latency"]

            for operation, stats in latency_data["operations"].items():
                report.append(f"{operation}:")
                report.append(f"  Mean: {stats['mean_ms']:.3f} ms")
                report.append(f"  Median: {stats['median_ms']:.3f} ms")
                report.append(f"  P95: {stats['p95_ms']:.3f} ms")
                report.append(f"  P99: {stats['p99_ms']:.3f} ms")
                report.append(f"  Max: {stats['max_ms']:.3f} ms")
                report.append("")

        # Rapport charge
        if "load" in results["benchmarks"]:
            report.append("LOAD BENCHMARKS")
            report.append("-" * 40)
            load_data = results["benchmarks"]["load"]

            for test_name, test_data in load_data["load_tests"].items():
                report.append(f"{test_name}:")
                report.append(f"  Requests/sec: {test_data['requests_per_second']:.2f}")
                report.append(f"  Success rate: {test_data['success_rate']:.2%}")
                report.append(
                    f"  Mean latency: {test_data['latency_stats']['mean_ms']:.3f} ms"
                )
                report.append(
                    f"  P95 latency: {test_data['latency_stats']['p95_ms']:.3f} ms"
                )
                report.append("")

        # Rapport mémoire
        if "memory" in results["benchmarks"]:
            report.append("MEMORY BENCHMARKS")
            report.append("-" * 40)
            memory_data = results["benchmarks"]["memory"]

            for test_name, test_data in memory_data["memory_tests"].items():
                report.append(f"{test_name}:")
                report.append(
                    f"  Memory growth: {test_data['memory_growth_mb']:.2f} MB"
                )
                if "peak_memory_mb" in test_data:
                    report.append(
                        f"  Peak memory: {test_data['peak_memory_mb']:.2f} MB"
                    )
                report.append("")

        # Rapport CPU
        if "cpu" in results["benchmarks"]:
            report.append("CPU BENCHMARKS")
            report.append("-" * 40)
            cpu_data = results["benchmarks"]["cpu"]

            for test_name, test_data in cpu_data["cpu_tests"].items():
                report.append(f"{test_name}:")
                report.append(f"  Mean CPU: {test_data['cpu_stats']['mean']:.2f}%")
                report.append(f"  Max CPU: {test_data['cpu_stats']['max']:.2f}%")
                report.append(f"  Duration: {test_data['duration']:.1f}s")
                report.append("")

        report.append("=" * 80)
        return "\n".join(report)


def main():
    """Point d'entrée principal."""
    import argparse

    parser = argparse.ArgumentParser(description="BBIA Performance Benchmarks")
    parser.add_argument(
        "--backend",
        choices=["mujoco", "reachy", "reachy_mini"],
        default="mujoco",
        help="Backend robot à tester",
    )
    parser.add_argument(
        "--benchmark",
        choices=["latency", "load", "memory", "cpu", "all"],
        default="all",
        help="Type de benchmark à exécuter",
    )
    parser.add_argument(
        "--output",
        help="Fichier de sortie pour les résultats",
    )
    parser.add_argument(
        "--report",
        action="store_true",
        help="Générer un rapport textuel",
    )

    args = parser.parse_args()

    # Configuration du logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    # Créer le benchmark
    benchmark = BBIAPerformanceBenchmark(backend=args.backend)

    print(f"🚀 Démarrage benchmarks BBIA - Backend: {args.backend}")
    print(f"📊 Benchmark: {args.benchmark}")
    print()

    try:
        if args.benchmark == "all":
            results = benchmark.run_all_benchmarks()
        elif args.benchmark == "latency":
            results = {"benchmarks": {"latency": benchmark.benchmark_latency()}}
        elif args.benchmark == "load":
            results = {"benchmarks": {"load": benchmark.benchmark_load()}}
        elif args.benchmark == "memory":
            results = {"benchmarks": {"memory": benchmark.benchmark_memory()}}
        elif args.benchmark == "cpu":
            results = {"benchmarks": {"cpu": benchmark.benchmark_cpu()}}

        # Sauvegarder les résultats
        filepath = benchmark.save_results(results, args.output)

        # Générer le rapport
        if args.report:
            report = benchmark.generate_report(results)
            report_path = filepath.with_suffix(".txt")
            with open(report_path, "w", encoding="utf-8") as f:
                f.write(report)
            print(f"📋 Rapport généré: {report_path}")

        print("✅ Benchmarks terminés avec succès")
        print(f"📊 Résultats sauvegardés: {filepath}")

    except Exception as e:
        print(f"❌ Erreur lors des benchmarks: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
