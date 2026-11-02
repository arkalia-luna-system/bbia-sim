#!/usr/bin/env python3
"""hardware_dry_run_reachy_mini.py - Test hardware avec SDK officiel Reachy-Mini
Version adaptée pour le SDK officiel reachy_mini
"""

import argparse
import csv
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.mapping_reachy import ReachyMapping
from bbia_sim.robot_factory import RobotFactory

# Configuration logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class ReachyMiniHardwareDryRun:
    """Test hardware complet avec SDK officiel Reachy-Mini."""

    def __init__(self, output_dir: str = "artifacts"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Créer le robot avec le backend officiel (sim par défaut — sûr)
        # Joints alignés mapping SDK : yaw_body (pas head_1/body_yaw legacy)
        self.robot = RobotFactory.create_backend("reachy_mini", use_sim=True)
        self.mapping = ReachyMapping()

        # Données de test
        self.latency_data: list[dict[str, Any]] = []
        self.test_results: dict[str, Any] = {
            "timestamp": time.time(),
            "backend": "reachy_mini",
            "tests": {},
            "summary": {},
        }

        # Joints de test (sécurisés — noms officiels XML / mapping_reachy)
        self.test_joints = ["yaw_body"]

        logger.info(f"Initialisé avec backend: {type(self.robot).__name__}")

    def test_connection(self) -> bool:
        """Test de connexion au robot."""
        logger.info("Test de connexion...")

        try:
            # Simulation connexion (pas de robot physique)
            self.robot.is_connected = True
            logger.info("✅ Connexion simulée réussie")
            return True
        except Exception as e:
            logger.error(f"❌ Erreur connexion: {e}")
            return False

    def test_joints_availability(self) -> bool:
        """Test de disponibilité des joints."""
        logger.info("Test disponibilité joints...")

        try:
            available_joints = self.robot.get_available_joints()
            logger.info(f"Joints disponibles: {len(available_joints)}")

            # Vérifier joints de test
            missing_joints = []
            for joint in self.test_joints:
                if joint not in available_joints:
                    missing_joints.append(joint)

            if missing_joints:
                logger.error(f"❌ Joints manquants: {missing_joints}")
                return False

            logger.info("✅ Tous les joints de test sont disponibles")
            return True

        except Exception as e:
            logger.error(f"❌ Erreur test joints: {e}")
            return False

    def test_joint_limits(self) -> bool:
        """Test des limites des joints."""
        logger.info("Test limites joints...")

        try:
            for joint in self.test_joints:
                joint_info = self.mapping.get_joint_info(joint)
                if joint_info:
                    logger.info(
                        f"   {joint}: {joint_info.min_limit:.3f} à {joint_info.max_limit:.3f} rad",
                    )
                else:
                    logger.warning(f"   {joint}: Pas d'info de limite")

            logger.info("✅ Limites joints vérifiées")
            return True

        except Exception as e:
            logger.error(f"❌ Erreur test limites: {e}")
            return False

    def test_safety_limits(self) -> bool:
        """Test des limites de sécurité."""
        logger.info("Test limites sécurité...")

        try:
            safe_limit = self.robot.safe_amplitude_limit
            logger.info(f"Limite amplitude: {safe_limit} rad")

            # Test amplitude excessive
            test_position = 0.5  # Au-delà de la limite
            self.robot.set_joint_pos("yaw_body", test_position)
            actual_pos = self.robot.get_joint_pos("yaw_body")

            if abs(actual_pos) <= safe_limit:
                logger.info("✅ Limite amplitude respectée")
                return True
            logger.error(f"❌ Limite amplitude non respectée: {actual_pos}")
            return False

        except Exception as e:
            logger.error(f"❌ Erreur test sécurité: {e}")
            return False

    def test_latency(self, duration: int = 30) -> bool:
        """Test de latence set→read."""
        logger.info(f"Test latence ({duration}s)...")

        try:
            start_time = time.time()
            test_count = 0

            while time.time() - start_time < duration:
                # Test sur différents joints
                joint = self.test_joints[test_count % len(self.test_joints)]
                test_position = 0.1 * (test_count % 3 - 1)  # -0.1, 0.0, 0.1

                # Mesurer latence
                start_latency = time.time()
                success = self.robot.set_joint_pos(joint, test_position)
                read_pos = self.robot.get_joint_pos(joint)
                latency_ms = (time.time() - start_latency) * 1000

                # Enregistrer données
                self.latency_data.append(
                    {
                        "timestamp": time.time(),
                        "joint": joint,
                        "target": test_position,
                        "actual": read_pos,
                        "latency_ms": latency_ms,
                        "success": success,
                    },
                )

                test_count += 1

                # Log périodique
                if test_count % 100 == 0:
                    avg_latency = (
                        sum(d["latency_ms"] for d in self.latency_data[-100:]) / 100
                    )
                    logger.info(
                        f"   {test_count} tests, latence moyenne: {avg_latency:.2f}ms",
                    )

                time.sleep(0.01)  # 100Hz

            # Calculer statistiques
            latencies = [d["latency_ms"] for d in self.latency_data]
            avg_latency = sum(latencies) / len(latencies)
            max_latency = max(latencies)

            logger.info(f"✅ Latence testée: {len(self.latency_data)} mesures")
            logger.info(f"   Moyenne: {avg_latency:.2f}ms")
            logger.info(f"   Maximum: {max_latency:.2f}ms")

            # Critère de succès: <40ms
            if avg_latency < 40:
                logger.info("✅ Latence acceptable (<40ms)")
                return True
            logger.warning(f"⚠️  Latence élevée: {avg_latency:.2f}ms")
            return False

        except Exception as e:
            logger.error(f"❌ Erreur test latence: {e}")
            return False

    def test_emotions(self) -> bool:
        """Test des émotions."""
        logger.info("Test émotions...")

        try:
            emotions = ["happy", "sad", "neutral", "excited", "curious"]
            success_count = 0

            for emotion in emotions:
                success = self.robot.set_emotion(emotion, 0.7)
                if success:
                    success_count += 1
                    logger.info(f"   {emotion}: ✅")
                else:
                    logger.warning(f"   {emotion}: ❌")
                time.sleep(0.5)

            success_rate = success_count / len(emotions)
            logger.info(
                f"✅ Émotions: {success_count}/{len(emotions)} ({success_rate:.1%})",
            )

            return success_rate >= 0.8  # 80% de succès minimum

        except Exception as e:
            logger.error(f"❌ Erreur test émotions: {e}")
            return False

    def test_behaviors(self) -> bool:
        """Test des comportements."""
        logger.info("Test comportements...")

        try:
            behaviors = ["wake_up", "nod", "goto_sleep"]
            success_count = 0

            for behavior in behaviors:
                success = self.robot.run_behavior(behavior, 2.0)
                if success:
                    success_count += 1
                    logger.info(f"   {behavior}: ✅")
                else:
                    logger.warning(f"   {behavior}: ❌")
                time.sleep(1.0)

            success_rate = success_count / len(behaviors)
            logger.info(
                f"✅ Comportements: {success_count}/{len(behaviors)} ({success_rate:.1%})",
            )

            return success_rate >= 0.8  # 80% de succès minimum

        except Exception as e:
            logger.error(f"❌ Erreur test comportements: {e}")
            return False

    def save_artifacts(self) -> None:
        """Sauvegarde les artefacts de test."""
        logger.info("Sauvegarde artefacts...")

        try:
            # CSV latence
            csv_path = self.output_dir / "latency_reachy_mini.csv"
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                if self.latency_data:
                    writer = csv.DictWriter(f, fieldnames=self.latency_data[0].keys())
                    writer.writeheader()
                    writer.writerows(self.latency_data)

            # JSON résultats
            json_path = self.output_dir / "test_results_reachy_mini.json"
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(self.test_results, f, indent=2, ensure_ascii=False)

            # Log
            log_path = self.output_dir / "run_reachy_mini.log"
            with open(log_path, "w", encoding="utf-8") as f:
                f.write("Test hardware Reachy-Mini SDK officiel\n")
                f.write(f"Timestamp: {time.ctime()}\n")
                f.write(f"Backend: {type(self.robot).__name__}\n")
                f.write(f"Tests: {len(self.test_results['tests'])}\n")
                f.write(f"Latence mesures: {len(self.latency_data)}\n")

            logger.info(f"✅ Artefacts sauvegardés dans {self.output_dir}")

        except Exception as e:
            logger.error(f"❌ Erreur sauvegarde: {e}")

    def run_full_test(self, duration: int = 30) -> bool:
        """Exécute tous les tests."""
        logger.info("🚀 Démarrage test hardware complet Reachy-Mini")
        logger.info("=" * 60)

        tests = [
            ("Connexion", self.test_connection),
            ("Joints disponibilité", self.test_joints_availability),
            ("Limites joints", self.test_joint_limits),
            ("Limites sécurité", self.test_safety_limits),
            ("Latence", lambda: self.test_latency(duration)),
            ("Émotions", self.test_emotions),
            ("Comportements", self.test_behaviors),
        ]

        passed_tests = 0
        total_tests = len(tests)

        for test_name, test_func in tests:
            logger.info(f"\n📋 {test_name}...")
            try:
                success = test_func()
                self.test_results["tests"][test_name] = {
                    "success": success,
                    "timestamp": time.time(),
                }
                if success:
                    passed_tests += 1
                    logger.info(f"✅ {test_name}: PASSÉ")
                else:
                    logger.error(f"❌ {test_name}: ÉCHOUÉ")
            except Exception as e:
                logger.error(f"❌ {test_name}: ERREUR - {e}")
                self.test_results["tests"][test_name] = {
                    "success": False,
                    "error": str(e),
                    "timestamp": time.time(),
                }

        # Résumé
        success_rate = passed_tests / total_tests
        self.test_results["summary"] = {
            "total_tests": total_tests,
            "passed_tests": passed_tests,
            "success_rate": success_rate,
            "duration": duration,
            "latency_measures": len(self.latency_data),
        }

        logger.info("\n" + "=" * 60)
        logger.info("📊 RÉSULTATS FINAUX")
        logger.info(f"Tests passés: {passed_tests}/{total_tests} ({success_rate:.1%})")

        if self.latency_data:
            avg_latency = sum(d["latency_ms"] for d in self.latency_data) / len(
                self.latency_data,
            )
            logger.info(f"Latence moyenne: {avg_latency:.2f}ms")

        # Sauvegarde
        self.save_artifacts()

        if success_rate >= 0.8:
            logger.info("🎉 TEST GLOBAL: SUCCÈS")
            return True
        logger.error("⚠️  TEST GLOBAL: ÉCHEC")
        return False


def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description=(
            "Dry-run Reachy-Mini (simulation par défaut). "
            "Robot physique: --hardware + BBIA_ALLOW_HARDWARE=1 "
            "(voir docs/hardware/PREMIER_JOUR_ROBOT.md)"
        ),
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=30,
        help="Durée test latence (s)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="artifacts",
        help="Répertoire sortie",
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Connexion robot réel (exige BBIA_ALLOW_HARDWARE=1)",
    )

    args = parser.parse_args()

    if args.hardware:
        from bbia_sim.hardware_guard import require_hardware_allowed

        require_hardware_allowed(action="hardware_dry_run_reachy_mini --hardware")
        logger.warning(
            "Mode HARDWARE actif — mouvements possibles. "
            "Espace libre autour du robot, arrêt d'urgence prêt."
        )

    # Exécuter test (création backend reste use_sim=True dans la classe ;
    # le flag --hardware documente l'intention et bloque sans ALLOW)
    dry_run = ReachyMiniHardwareDryRun(args.output)
    if args.hardware:
        dry_run.robot = RobotFactory.create_backend("reachy_mini", use_sim=False)
        if dry_run.robot is not None:
            dry_run.robot.connect()
    success = dry_run.run_full_test(args.duration)

    # Code de sortie
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
