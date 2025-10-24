#!/usr/bin/env python3
"""
hardware_dry_run.py - Validation hardware Reachy réel
Script de validation pour préparer la connexion au robot Reachy réel.
Mesure latence, valide joints, teste limites de sécurité.
"""

import logging
import sys
import time
from pathlib import Path
from typing import Optional

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_api import RobotAPI
from bbia_sim.robot_factory import RobotFactory

# Configuration logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class HardwareDryRun:
    """Validation hardware Reachy réel."""

    def __init__(self):
        self.robot: Optional[RobotAPI] = None
        self.latencies: list[float] = []
        self.errors: list[str] = []
        self.test_joints = ["yaw_body", "stewart_1", "stewart_2"]

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
        large_amplitude = self.robot.safe_amplitude_limit + 0.5

        logger.info(f"🧪 Test amplitude limite: {large_amplitude:.2f} rad")

        success = self.robot.set_joint_pos(test_joint, large_amplitude)
        if not success:
            logger.info("✅ Limite d'amplitude respectée (commande rejetée)")
        else:
            actual_pos = self.robot.get_joint_pos(test_joint)
            if (
                actual_pos is not None
                and abs(actual_pos) <= self.robot.safe_amplitude_limit
            ):
                logger.info("✅ Limite d'amplitude respectée (position clampée)")
            else:
                self.errors.append("Limite d'amplitude non respectée")
                return False

        # Test joints interdits
        forbidden_joint = list(self.robot.forbidden_joints)[0]
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
            logger.info(f"⏱️ Latence max: {max_latency:.1f}ms")
            logger.info(f"⏱️ Latence min: {min_latency:.1f}ms")

            # Vérification cible <40ms
            if avg_latency < 40:
                logger.info("✅ Latence cible atteinte (<40ms)")
            else:
                logger.warning(f"⚠️ Latence élevée: {avg_latency:.1f}ms (cible <40ms)")
        else:
            logger.warning("⚠️ Aucune mesure de latence")

        if self.errors:
            logger.error(f"❌ Erreurs détectées: {len(self.errors)}")
            for error in self.errors:
                logger.error(f"   • {error}")
        else:
            logger.info("✅ Aucune erreur détectée")

        logger.info("=" * 50)


def main():
    """Point d'entrée principal."""
    import argparse

    parser = argparse.ArgumentParser(description="Hardware dry run pour Reachy réel")
    parser.add_argument(
        "--duration", type=float, default=10.0, help="Durée du test en secondes"
    )
    parser.add_argument("--joint", default="yaw_body", help="Joint principal à tester")
    args = parser.parse_args()

    # Créer et exécuter le test
    dry_run = HardwareDryRun()
    success = dry_run.run_full_test(args.duration)

    if success:
        logger.info("🎉 Hardware dry run réussi !")
        return 0
    else:
        logger.error("💥 Hardware dry run échoué !")
        return 1


if __name__ == "__main__":
    exit(main())
