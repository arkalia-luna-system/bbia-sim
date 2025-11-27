#!/usr/bin/env python3
"""Script de profiling automatique pour CI avec validation baseline.

Génère des métriques de performance et les compare avec des baselines de référence.
"""

import cProfile
import json
import logging
import pstats
import sys
import time
from pathlib import Path
from typing import Any

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class PerformanceProfiler:
    """Profiler de performance avec validation baseline."""

    def __init__(self, baseline_file: str = "artifacts/performance_baseline.json"):
        """Initialise le profiler.

        Args:
            baseline_file: Chemin vers le fichier baseline de référence
        """
        self.baseline_file = Path(baseline_file)
        self.results: dict[str, Any] = {}
        self.baseline: dict[str, Any] = {}

    def load_baseline(self) -> bool:
        """Charge le baseline de référence.

        Returns:
            True si baseline chargé, False sinon
        """
        if not self.baseline_file.exists():
            logger.warning(f"Baseline non trouvé: {self.baseline_file}")
            return False

        try:
            with open(self.baseline_file) as f:
                self.baseline = json.load(f)
            logger.info(f"✅ Baseline chargé: {self.baseline_file}")
            return True
        except Exception as e:
            logger.error(f"Erreur chargement baseline: {e}")
            return False

    def profile_function(self, func, *args, **kwargs) -> dict[str, Any]:
        """Profile une fonction et retourne les métriques.

        Args:
            func: Fonction à profiler
            *args: Arguments positionnels
            **kwargs: Arguments nommés

        Returns:
            Dictionnaire avec métriques (temps, appels, etc.)
        """
        profiler = cProfile.Profile()
        profiler.enable()

        start_time = time.perf_counter()
        try:
            result = func(*args, **kwargs)
        finally:
            profiler.disable()
        elapsed_time = time.perf_counter() - start_time

        stats = pstats.Stats(profiler)
        stats.sort_stats("cumulative")

        # Extraire métriques principales
        total_calls = stats.total_calls if hasattr(stats, "total_calls") else 0
        total_time = stats.total_tt if hasattr(stats, "total_tt") else elapsed_time

        return {
            "elapsed_time": elapsed_time,
            "total_calls": total_calls,
            "total_time": total_time,
            "result": result,
        }

    def validate_against_baseline(self, metric_name: str, value: float, threshold: float = 0.2) -> bool:
        """Valide une métrique contre le baseline.

        Args:
            metric_name: Nom de la métrique
            value: Valeur actuelle
            threshold: Seuil de tolérance (20% par défaut)

        Returns:
            True si dans les limites, False sinon
        """
        if not self.baseline or metric_name not in self.baseline:
            logger.warning(f"Baseline non disponible pour: {metric_name}")
            return True  # Pas de baseline = OK

        baseline_value = self.baseline[metric_name]
        diff_percent = abs((value - baseline_value) / baseline_value) * 100

        if diff_percent > threshold * 100:
            logger.error(
                f"❌ Métrique {metric_name} hors baseline: "
                f"{value:.3f} vs {baseline_value:.3f} "
                f"(diff: {diff_percent:.1f}%)"
            )
            return False

        logger.info(
            f"✅ Métrique {metric_name} OK: "
            f"{value:.3f} vs {baseline_value:.3f} "
            f"(diff: {diff_percent:.1f}%)"
        )
        return True

    def save_results(self, output_file: str = "artifacts/performance_results.json") -> None:
        """Sauvegarde les résultats.

        Args:
            output_file: Chemin vers le fichier de sortie
        """
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, "w") as f:
            json.dump(self.results, f, indent=2)

        logger.info(f"✅ Résultats sauvegardés: {output_path}")


def main() -> int:
    """Point d'entrée principal."""
    profiler = PerformanceProfiler()

    # Charger baseline si disponible
    profiler.load_baseline()

    # Exemple de profiling (à adapter selon besoins)
    def dummy_operation() -> int:
        """Opération de test."""
        total = 0
        for i in range(1000):
            total += i
        return total

    metrics = profiler.profile_function(dummy_operation)
    profiler.results["dummy_operation"] = metrics

    # Valider contre baseline
    if profiler.baseline:
        is_valid = profiler.validate_against_baseline(
            "dummy_operation.elapsed_time",
            metrics["elapsed_time"],
        )
        if not is_valid:
            logger.error("❌ Performance dégradée par rapport au baseline")
            return 1

    # Sauvegarder résultats
    profiler.save_results()

    logger.info("✅ Profiling terminé avec succès")
    return 0


if __name__ == "__main__":
    sys.exit(main())

