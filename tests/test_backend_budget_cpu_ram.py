#!/usr/bin/env python3
"""
Test budget CPU/RAM boucle principale backend (10-30s profiling léger).
Mesure overhead backend avec psutil.
"""

import time

import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


def get_cpu_time() -> float:
    """Obtient temps CPU écoulé en secondes."""
    try:
        import psutil

        process = psutil.Process()
        return float(process.cpu_times().user + process.cpu_times().system)
    except ImportError:
        # Fallback: utiliser process_time
        return float(time.process_time())


def get_memory_usage() -> float | None:
    """Obtient utilisation mémoire actuelle en MB."""
    try:
        import psutil

        process = psutil.Process()
        return float(process.memory_info().rss / 1024 / 1024)  # MB
    except ImportError:
        return None


@pytest.mark.unit
@pytest.mark.slow
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (boucle 5s)
def test_backend_main_loop_budget_cpu_ram() -> None:
    """Test budget CPU/RAM boucle principale backend (3s profiling optimisé)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    # OPTIMISATION RAM: Réduire 5s → 3s, 500 → 300 itérations (suffisant pour mesurer budget)
    duration_s = 3.0
    iterations = 300

    try:
        # Mesurer avant
        cpu_before = get_cpu_time()
        mem_before = get_memory_usage()

        # Boucle principale simulée
        t0 = time.perf_counter()
        for i in range(iterations):
            # Opérations typiques boucle
            backend.set_joint_pos("yaw_body", 0.1 * (i % 2))
            backend.get_joint_pos("yaw_body")
            backend.step()

            # Réguler à ~50 Hz (20ms par itération)
            if (i + 1) % 50 == 0:
                elapsed = time.perf_counter() - t0
                if elapsed >= duration_s:
                    break
                time.sleep(0.001)  # Petit délai

        # Mesurer après
        cpu_after = get_cpu_time()
        mem_after = get_memory_usage()

        cpu_time = cpu_after - cpu_before
        if mem_before and mem_after:
            mem_increase = mem_after - mem_before
        else:
            mem_increase = None

        # Budget: CPU < 1s pour 5s runtime (20% CPU max)
        assert (
            cpu_time < 1.0
        ), f"Temps CPU trop élevé: {cpu_time:.2f}s pour {duration_s}s runtime"

        # Budget: RAM < 100MB augmentation (sur 10s)
        # NOTE: Seuil flexibilisé à 120MB pour CI car:
        # - Environnements CI varient (GitHub Actions, etc.)
        # - Modèles peuvent être préchargés en cache
        # - Mesures mémoire peuvent fluctuer selon machine CI
        # En local, consommation réelle est généralement < 50MB
        # Si ce test échoue régulièrement, vérifier fuites mémoire avec tracemalloc
        if mem_increase is not None:
            assert mem_increase < 120.0, (  # Tolérance CI: 120MB (idéal local: <50MB)
                f"Augmentation RAM trop élevée: {mem_increase:.1f}MB (seuil: 120MB). "
                f"En local, devrait être <50MB. Vérifier fuites mémoire si échec régulier."
            )
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.slow
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (boucle 3s)
def test_robot_api_interface_budget_cpu_ram() -> None:
    """Test budget CPU/RAM interface RobotAPI abstraite (3s optimisé)."""
    from bbia_sim.robot_factory import RobotFactory

    robot = RobotFactory.create_backend("reachy_mini")
    if not robot or not robot.connect():
        pytest.skip("Backend non disponible")

    # OPTIMISATION RAM: Réduire 5s → 3s, 500 → 300 itérations (suffisant pour mesurer budget)
    duration_s = 3.0
    iterations = 300

    try:
        # Mesurer avant
        cpu_before = get_cpu_time()
        mem_before = get_memory_usage()

        # Boucle interface abstraite
        t0 = time.perf_counter()
        for i in range(iterations):
            robot.set_joint_pos("yaw_body", 0.1 * (i % 2))
            robot.get_joint_pos("yaw_body")
            robot.step()

            if (i + 1) % 50 == 0:
                elapsed = time.perf_counter() - t0
                if elapsed >= duration_s:
                    break
                time.sleep(0.001)

        # Mesurer après
        cpu_after = get_cpu_time()
        mem_after = get_memory_usage()

        cpu_time = cpu_after - cpu_before
        if mem_before and mem_after:
            mem_increase = mem_after - mem_before
        else:
            mem_increase = None

        # Budget: Overhead minimal interface (< 0.25s CPU pour 5s)
        assert cpu_time < 0.25, f"Overhead interface trop élevé: {cpu_time:.2f}s"

        # Budget: RAM < 90MB augmentation (interface légère, tolérance CI)
        # NOTE: Seuil flexibilisé pour CI. En local, consommation réelle < 30MB.
        # Raisons seuil CI élevé:
        # - Variabilité machines CI
        # - Cache modèles partagés
        # - Mesures mémoire fluctuantes
        if mem_increase is not None:
            assert mem_increase < 90.0, (  # Tolérance CI: 90MB (idéal local: <30MB)
                f"Augmentation RAM trop élevée: {mem_increase:.1f}MB (seuil: 90MB). "
                f"En local, devrait être <30MB. Investiguer si échec répété."
            )
    finally:
        robot.disconnect()
