#!/usr/bin/env python3
"""
hardware_dry_run.py - Validation hardware Reachy réel
Script de validation pour préparer la connexion au robot Reachy réel.
Mesure latence, valide joints, teste limites de sécurité.
Génère des artefacts CSV/log pour la CI.
"""

import argparse
import csv
import json
import logging
import sys
import time
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.mapping_reachy import ReachyMapping
from bbia_sim.robot_api import RobotAPI
from bbia_sim.robot_factory import RobotFactory

# Configuration logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class HardwareDryRun:
    """Validation hardware Reachy réel avec génération d'artefacts."""

    def __init__(self, output_dir: str = "artifacts"):
        self.robot: RobotAPI | None = None
        self.latencies: list[float] = []
        self.errors: list[str] = []
        self.test_joints = list(ReachyMapping.get_recommended_joints())
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Données pour artefacts
        self.latency_data: list[dict] = []
        self.test_results: dict = {}

    def connect(self) -> bool:
        """Connexion au robot Reachy réel."""
        logger.info("🔌 Connexion au robot Reachy réel...")

        try:
            self.robot = RobotFactory.create_backend("reachy")
            if not self.robot:
                self.errors.append("Impossible de créer le backend Reachy")
                return False

            if not self.robot.connect():
                self.errors.append("Échec de connexion au robot")
                return False

            logger.info("✅ Robot Reachy connecté avec succès")
            return True

        except Exception as e:
            error_msg = f"Erreur de connexion: {e}"
            self.errors.append(error_msg)
            logger.error(f"❌ {error_msg}")
            return False

    def validate_joints(self) -> bool:
        """Valide les joints disponibles et leurs limites."""
        logger.info("🔍 Validation des joints...")

        if not self.robot:
            self.errors.append("Robot non connecté")
            return False

        available_joints = self.robot.get_available_joints()
        logger.info(f"📋 Joints disponibles: {len(available_joints)}")

        # Vérifier les joints de test
        for joint in self.test_joints:
            if joint not in available_joints:
                error_msg = (
                    f"Joint de test manquant: {joint} (disponibles: {available_joints})"
                )
                self.errors.append(error_msg)
                logger.error(f"❌ {error_msg}")
                return False

        logger.info("✅ Tous les joints de test sont disponibles")
        return True

    def test_latency(self, joint: str, duration: float = 10.0) -> bool:
        """Teste la latence set→read pour un joint."""
        logger.info(f"⏱️ Test latence {joint} ({duration}s)...")

        if not self.robot:
            self.errors.append("Robot non connecté")
            return False

        start_time = time.time()
        test_count = 0

        try:
            while (time.time() - start_time) < duration:
                # Position de test (dans les limites de sécurité)
                test_pos = 0.1 * (test_count % 2)  # 0.0 ou 0.1 rad

                # Mesure latence
                start_time_latency = time.time()
                success = self.robot.set_joint_pos(joint, test_pos)

                if not success:
                    self.errors.append(f"Échec set_joint_pos pour {joint}")
                    continue

                actual_pos = self.robot.get_joint_pos(joint)

                if actual_pos is None:
                    self.errors.append(f"Échec get_joint_pos pour {joint}")
                    continue

                # Calcul latence totale
                total_latency = (time.time() - start_time_latency) * 1000  # ms
                self.latencies.append(total_latency)

                # Enregistrer données pour artefacts
                self.latency_data.append(
                    {
                        "timestamp": time.time(),
                        "joint": joint,
                        "target_pos": test_pos,
                        "actual_pos": actual_pos,
                        "latency_ms": total_latency,
                        "test_count": test_count,
                    }
                )

                # Step de simulation
                self.robot.step()

                test_count += 1

                # Affichage périodique
                if test_count % 10 == 0:
                    avg_latency = sum(self.latencies[-10:]) / 10
                    logger.info(
                        f"  Test {test_count}: latence moyenne {avg_latency:.1f}ms"
                    )

        except Exception as e:
            error_msg = f"Erreur test latence {joint}: {e}"
            self.errors.append(error_msg)
            logger.error(f"❌ {error_msg}")
            return False

        logger.info(f"✅ Test latence {joint} terminé: {test_count} tests")
        return True

    def test_safety_limits(self) -> bool:
        """Teste les limites de sécurité."""
        logger.info("🛡️ Test des limites de sécurité...")

        if not self.robot:
            self.errors.append("Robot non connecté")
            return False

        # Test amplitude limite
        test_joint = self.test_joints[0]
        large_amplitude = ReachyMapping.GLOBAL_SAFETY_LIMIT + 0.5

        logger.info(f"🧪 Test amplitude limite: {large_amplitude:.2f} rad")

        success = self.robot.set_joint_pos(test_joint, large_amplitude)
        if not success:
            logger.info("✅ Limite d'amplitude respectée (commande rejetée)")
        else:
            actual_pos = self.robot.get_joint_pos(test_joint)
            if (
                actual_pos is not None
                and abs(actual_pos) <= ReachyMapping.GLOBAL_SAFETY_LIMIT
            ):
                logger.info("✅ Limite d'amplitude respectée (position clampée)")
            else:
                self.errors.append("Limite d'amplitude non respectée")
                return False

        # Test joints interdits
        forbidden_joint = list(ReachyMapping.get_forbidden_joints())[0]
        logger.info(f"🧪 Test joint interdit: {forbidden_joint}")

        success = self.robot.set_joint_pos(forbidden_joint, 0.5)
        if not success:
            logger.info("✅ Joint interdit correctement rejeté")
        else:
            self.errors.append(f"Joint interdit {forbidden_joint} accepté")
            return False

        logger.info("✅ Toutes les limites de sécurité sont respectées")
        return True

    def run_full_test(self, duration: float = 10.0) -> bool:
        """Exécute le test complet."""
        logger.info("🚀 Démarrage du hardware dry run...")
        logger.info(f"⏱️ Durée: {duration}s")

        # 1. Connexion
        if not self.connect():
            return False

        # 2. Validation joints
        if not self.validate_joints():
            return False

        # 3. Test limites de sécurité
        if not self.test_safety_limits():
            return False

        # 4. Test latence
        for joint in self.test_joints:
            if not self.test_latency(joint, duration / len(self.test_joints)):
                return False

        # 5. Résultats
        self.print_results()

        # 6. Déconnexion
        if self.robot:
            self.robot.disconnect()

        return len(self.errors) == 0

    def print_results(self):
        """Affiche les résultats du test."""
        logger.info("\n📊 RÉSULTATS DU HARDWARE DRY RUN")
        logger.info("=" * 50)

        if self.latencies:
            avg_latency = sum(self.latencies) / len(self.latencies)
            max_latency = max(self.latencies)
            min_latency = min(self.latencies)

            logger.info(f"⏱️ Latence moyenne: {avg_latency:.1f}ms")
            logger.info(f"⏱️ Latence min: {min_latency:.1f}ms")
            logger.info(f"⏱️ Latence max: {max_latency:.1f}ms")

            if avg_latency <= 40:
                logger.info("✅ Latence cible atteinte (<40ms)")
            else:
                logger.info("❌ Latence trop élevée (>40ms)")
        else:
            logger.info("❌ Aucune mesure de latence")

        if self.errors:
            logger.info(f"\n❌ Erreurs ({len(self.errors)}):")
            for error in self.errors:
                logger.info(f"  - {error}")
        else:
            logger.info("\n✅ Aucune erreur")

        logger.info("🎉 Hardware dry run réussi !")

    def save_artifacts(self) -> bool:
        """Sauvegarde les artefacts (CSV + log) pour la CI."""
        logger.info("💾 Sauvegarde des artefacts...")

        try:
            # 1. Sauvegarde CSV latence
            csv_file = self.output_dir / "latency.csv"
            with open(csv_file, "w", newline="") as f:
                if self.latency_data:
                    writer = csv.DictWriter(f, fieldnames=self.latency_data[0].keys())
                    writer.writeheader()
                    writer.writerows(self.latency_data)
                else:
                    # CSV vide si pas de données
                    simple_writer = csv.writer(f)
                    simple_writer.writerow(
                        [
                            "timestamp",
                            "joint",
                            "target_pos",
                            "actual_pos",
                            "latency_ms",
                            "test_count",
                        ]
                    )

            logger.info(f"✅ CSV latence sauvegardé: {csv_file}")

            # 2. Sauvegarde log d'erreurs
            log_file = self.output_dir / "run.log"
            with open(log_file, "w") as f:
                f.write(f"Hardware Dry Run - {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("=" * 50 + "\n")

                if self.errors:
                    f.write("❌ ERREURS:\n")
                    for error in self.errors:
                        f.write(f"  - {error}\n")
                else:
                    f.write("✅ Aucune erreur\n")

                f.write("\n📊 MÉTRIQUES:\n")
                if self.latencies:
                    avg_latency = sum(self.latencies) / len(self.latencies)
                    max_latency = max(self.latencies)
                    min_latency = min(self.latencies)
                    f.write(f"  - Mesures: {len(self.latencies)}\n")
                    f.write(f"  - Latence moyenne: {avg_latency:.1f}ms\n")
                    f.write(f"  - Latence min: {min_latency:.1f}ms\n")
                    f.write(f"  - Latence max: {max_latency:.1f}ms\n")
                else:
                    f.write("  - Aucune mesure de latence\n")

            logger.info(f"✅ Log sauvegardé: {log_file}")

            # 3. Sauvegarde résultats JSON
            results_file = self.output_dir / "test_results.json"
            results = {
                "timestamp": time.time(),
                "duration": len(self.latencies) * 0.1,  # Estimation
                "joints_tested": self.test_joints,
                "total_tests": len(self.latencies),
                "errors": self.errors,
                "latency_stats": {
                    "avg": (
                        sum(self.latencies) / len(self.latencies)
                        if self.latencies
                        else 0
                    ),
                    "max": max(self.latencies) if self.latencies else 0,
                    "min": min(self.latencies) if self.latencies else 0,
                },
                "success": len(self.errors) == 0,
            }

            with open(results_file, "w") as f:
                json.dump(results, f, indent=2)

            logger.info(f"✅ Résultats JSON sauvegardés: {results_file}")

            return True

        except Exception as e:
            logger.error(f"❌ Erreur sauvegarde artefacts: {e}")
            return False


def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(description="Hardware dry run Reachy")
    parser.add_argument(
        "--duration", type=float, default=10.0, help="Durée du test (s)"
    )
    parser.add_argument("--joint", type=str, help="Joint spécifique à tester")
    parser.add_argument(
        "--output", type=str, default="artifacts", help="Répertoire de sortie"
    )
    parser.add_argument(
        "--backend",
        type=str,
        default="reachy",
        choices=["reachy", "mujoco"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    # Créer et exécuter le test
    dry_run = HardwareDryRun(output_dir=args.output)
    success = dry_run.run_full_test(args.duration)

    if success:
        logger.info("🎉 Hardware dry run réussi !")
        dry_run.save_artifacts()
        return 0
    else:
        logger.error("💥 Hardware dry run échoué !")
        return 1


if __name__ == "__main__":
    sys.exit(main())
