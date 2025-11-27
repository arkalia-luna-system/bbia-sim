#!/usr/bin/env python3
"""Tests de répétabilité et précision des mouvements.

Issue #269: Add unit tests for move repeatability and precision.

Ces tests vérifient que les mouvements du robot sont répétables et précis,
c'est-à-dire qu'un même mouvement produit toujours le même résultat.

Usage:
    pytest tests/test_motion_repeatability.py -v
"""

import logging
import statistics
import time
from typing import TYPE_CHECKING

import pytest

if TYPE_CHECKING:
    from src.bbia_sim.backends.mujoco_backend import MuJoCoBackend
    from src.bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend
    from src.bbia_sim.robot_api import RobotAPI

logger = logging.getLogger(__name__)

# Import conditionnel des backends
try:
    from src.bbia_sim.backends.mujoco_backend import MuJoCoBackend

    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    MuJoCoBackend = None

try:
    from src.bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

    REACHY_MINI_AVAILABLE = True
except ImportError:
    REACHY_MINI_AVAILABLE = False
    ReachyMiniBackend = None


class TestMotionRepeatability:
    """Tests de répétabilité des mouvements."""

    @pytest.fixture
    def backend(self) -> "RobotAPI | None":
        """Fixture pour créer un backend de test."""
        if MuJoCoBackend and MUJOCO_AVAILABLE:
            backend = MuJoCoBackend()
            if backend.connect():
                return backend
        return None

    def test_repeatability_joint_position(self, backend: "RobotAPI | None") -> None:
        """Test répétabilité position joint.

        Vérifie qu'un même mouvement produit toujours la même position finale.
        """
        if backend is None:
            pytest.skip("Backend non disponible")
        assert backend is not None  # Pour le type checker

        target_position = 0.3  # Position cible en radians
        num_repetitions = 10
        final_positions: list[float] = []

        # Exécuter le mouvement plusieurs fois
        for _ in range(num_repetitions):
            # Retour à position initiale
            if not backend.set_joint_pos("yaw_body", 0.0):
                pytest.skip("Impossible de définir position initiale")
            # Step MuJoCo pour appliquer changement
            if hasattr(backend, "step"):
                backend.step()
            time.sleep(0.01)  # OPTIMISATION: Réduire 0.1s → 0.01s (10x plus rapide)

            # Mouvement vers position cible
            if not backend.set_joint_pos("yaw_body", target_position):
                pytest.skip("Impossible de définir position cible")
            # Step MuJoCo pour appliquer changement
            if hasattr(backend, "step"):
                backend.step()
            time.sleep(0.05)  # OPTIMISATION: Réduire 0.2s → 0.05s (4x plus rapide)

            # Enregistrer position finale
            final_pos = backend.get_joint_pos("yaw_body")
            if final_pos is None:
                pytest.skip("Impossible de lire position joint")
            final_positions.append(final_pos)

        # Calculer statistiques
        mean_pos = statistics.mean(final_positions)
        std_pos = statistics.stdev(final_positions) if len(final_positions) > 1 else 0.0
        max_error = max(abs(pos - target_position) for pos in final_positions)

        logger.info(
            f"Répétabilité joint position: "
            f"mean={mean_pos:.4f}, std={std_pos:.4f}, max_error={max_error:.4f}",
        )

        # Vérifier répétabilité (écart-type < 0.05 rad)
        assert std_pos < 0.05, f"Répétabilité insuffisante: std={std_pos:.4f}"
        # Vérifier précision (erreur max < 0.1 rad)
        assert max_error < 0.1, f"Précision insuffisante: max_error={max_error:.4f}"

    def test_repeatability_goto_target(self, backend: "RobotAPI | None") -> None:
        """Test répétabilité goto_target.

        Vérifie qu'un même mouvement goto_target produit toujours le même résultat.
        """
        if backend is None:
            pytest.skip("Backend non disponible")
        assert backend is not None  # Pour le type checker

        if not hasattr(backend, "goto_target"):
            pytest.skip("goto_target non disponible")

        # Créer pose de test
        try:
            from reachy_mini.utils import create_head_pose

            head_pose = create_head_pose(pitch=0.1, yaw=0.1, degrees=False)
        except ImportError:
            pytest.skip("SDK officiel non disponible pour create_head_pose")

        num_repetitions = 5
        final_positions: list[float] = []

        # Exécuter le mouvement plusieurs fois
        for _ in range(num_repetitions):
            # Retour à position initiale
            backend.goto_target(head=head_pose, duration=0.1, method="minjerk")
            time.sleep(0.05)  # OPTIMISATION: Réduire 0.15s → 0.05s (3x plus rapide)

            # Enregistrer position finale
            if hasattr(backend, "get_current_head_pose"):
                final_pose = backend.get_current_head_pose()
                if final_pose:
                    final_positions.append(final_pose.get("pitch", 0.0))
            else:
                # Fallback: utiliser get_joint_pos
                final_pos = backend.get_joint_pos("head_pitch")
                if final_pos is not None:
                    final_positions.append(final_pos)

        if len(final_positions) < 2:
            pytest.skip("Pas assez de positions collectées")

        # Calculer statistiques
        mean_pos = statistics.mean(final_positions)
        std_pos = statistics.stdev(final_positions)

        logger.info(
            f"Répétabilité goto_target: mean={mean_pos:.4f}, std={std_pos:.4f}",
        )

        # Vérifier répétabilité (écart-type < 0.05 rad)
        assert std_pos < 0.05, f"Répétabilité insuffisante: std={std_pos:.4f}"

    def test_precision_multiple_joints(self, backend: "RobotAPI | None") -> None:
        """Test précision mouvements multiples joints simultanés.

        Vérifie que plusieurs joints peuvent être contrôlés précisément en même temps.
        """
        if backend is None:
            pytest.skip("Backend non disponible")
        assert backend is not None  # Pour le type checker

        target_positions = {
            "yaw_body": 0.2,
            "head_yaw": 0.15,
            "head_pitch": 0.1,
        }

        num_repetitions = 5
        errors: list[dict[str, float]] = []

        # Exécuter le mouvement plusieurs fois
        for _ in range(num_repetitions):
            # Retour à position initiale
            for joint_name in target_positions:
                backend.set_joint_pos(joint_name, 0.0)
            # Step MuJoCo pour appliquer changements
            if hasattr(backend, "step"):
                backend.step()
            time.sleep(0.01)  # OPTIMISATION: Réduire 0.1s → 0.01s (10x plus rapide)

            # Mouvement vers positions cibles
            for joint_name, target_pos in target_positions.items():
                backend.set_joint_pos(joint_name, target_pos)
            # Step MuJoCo pour appliquer changements
            if hasattr(backend, "step"):
                backend.step()
            time.sleep(0.05)  # OPTIMISATION: Réduire 0.2s → 0.05s (4x plus rapide)

            # Enregistrer erreurs
            iteration_errors: dict[str, float] = {}
            for joint_name, target_pos in target_positions.items():
                actual_pos = backend.get_joint_pos(joint_name)
                if actual_pos is None:
                    continue  # Skip si position non disponible
                error = abs(actual_pos - target_pos)
                iteration_errors[joint_name] = error
            errors.append(iteration_errors)

        # Calculer erreurs moyennes par joint
        mean_errors: dict[str, float] = {}
        for joint_name in target_positions:
            joint_errors = [e[joint_name] for e in errors if joint_name in e]
            if joint_errors:
                mean_errors[joint_name] = statistics.mean(joint_errors)

        logger.info(f"Précision multiple joints: {mean_errors}")

        # Vérifier précision (erreur moyenne < 0.1 rad pour chaque joint)
        if not mean_errors:
            pytest.skip("Aucune erreur collectée pour les joints")
        for joint_name, mean_error in mean_errors.items():
            assert (
                mean_error < 0.1
            ), f"Précision insuffisante pour {joint_name}: {mean_error:.4f}"

    def test_repeatability_play_move(self, backend: "RobotAPI | None") -> None:
        """Test répétabilité play_move.

        Vérifie qu'un mouvement enregistré produit toujours le même résultat.
        """
        if backend is None:
            pytest.skip("Backend non disponible")
        assert backend is not None  # Pour le type checker

        if not hasattr(backend, "start_recording") or not hasattr(
            backend, "stop_recording"
        ):
            pytest.skip("Enregistrement/replay non disponible")

        # Enregistrer un mouvement simple
        backend.start_recording()
        backend.set_joint_pos("yaw_body", 0.2)
        if hasattr(backend, "step"):
            backend.step()
        time.sleep(0.05)  # OPTIMISATION: Réduire 0.3s → 0.05s (6x plus rapide)
        backend.set_joint_pos("yaw_body", -0.2)
        if hasattr(backend, "step"):
            backend.step()
        time.sleep(0.05)  # OPTIMISATION: Réduire 0.3s → 0.05s (6x plus rapide)
        move = backend.stop_recording()

        if move is None:
            pytest.skip("Aucun mouvement enregistré")

        num_repetitions = 5
        final_positions: list[float] = []

        # Rejouer le mouvement plusieurs fois
        for _ in range(num_repetitions):
            # Retour à position initiale
            backend.set_joint_pos("yaw_body", 0.0)
            if hasattr(backend, "step"):
                backend.step()
            time.sleep(0.01)  # OPTIMISATION: Réduire 0.1s → 0.01s (10x plus rapide)

            # Rejouer mouvement
            if hasattr(backend, "play_move"):
                backend.play_move(move, play_frequency=100.0)
            elif hasattr(backend, "async_play_move"):
                import asyncio

                asyncio.run(backend.async_play_move(move, 100.0))
            else:
                pytest.skip("play_move non disponible")

            time.sleep(0.05)  # OPTIMISATION: Réduire 0.5s → 0.05s (10x plus rapide)

            # Enregistrer position finale
            final_pos = backend.get_joint_pos("yaw_body")
            if final_pos is None:
                continue  # Skip si position non disponible
            final_positions.append(final_pos)

        if len(final_positions) < 2:
            pytest.skip("Pas assez de positions collectées")

        # Calculer statistiques
        mean_pos = statistics.mean(final_positions)
        std_pos = statistics.stdev(final_positions)

        logger.info(
            f"Répétabilité play_move: mean={mean_pos:.4f}, std={std_pos:.4f}",
        )

        # Vérifier répétabilité (écart-type < 0.1 rad pour mouvements complexes)
        assert std_pos < 0.1, f"Répétabilité insuffisante: std={std_pos:.4f}"

    def test_precision_small_movements(self, backend: "RobotAPI | None") -> None:
        """Test précision mouvements petits.

        Vérifie que les petits mouvements sont précis.
        """
        if backend is None:
            pytest.skip("Backend non disponible")
        assert backend is not None  # Pour le type checker

        small_movements = [0.01, 0.02, 0.05, 0.1]  # Petits angles en radians
        errors: list[float] = []

        for target_pos in small_movements:
            # Retour à position initiale
            backend.set_joint_pos("yaw_body", 0.0)
            if hasattr(backend, "step"):
                backend.step()
            time.sleep(0.01)  # OPTIMISATION: Réduire 0.1s → 0.01s (10x plus rapide)

            # Mouvement petit
            backend.set_joint_pos("yaw_body", target_pos)
            if hasattr(backend, "step"):
                backend.step()
            time.sleep(0.05)  # OPTIMISATION: Réduire 0.15s → 0.05s (3x plus rapide)

            # Mesurer erreur
            actual_pos = backend.get_joint_pos("yaw_body")
            if actual_pos is None:
                continue  # Skip si position non disponible
            error = abs(actual_pos - target_pos)
            errors.append(error)

        if not errors:
            pytest.skip("Aucune erreur collectée pour les petits mouvements")

        mean_error = statistics.mean(errors)
        max_error = max(errors)

        logger.info(
            f"Précision petits mouvements: "
            f"mean_error={mean_error:.4f}, max_error={max_error:.4f}",
        )

        # Vérifier précision (erreur moyenne < 0.05 rad pour petits mouvements)
        assert (
            mean_error < 0.05
        ), f"Précision petits mouvements insuffisante: {mean_error:.4f}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
