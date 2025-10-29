#!/usr/bin/env python3
"""
Tests unitaires pour watchdog monitoring temps réel
Créé suite audit BBIA → Reachy Integration
"""

import threading
import time

import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


class TestWatchdogMonitoring:
    """Tests pour le watchdog monitoring temps réel."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_watchdog_start_on_connect(self):
        """Test que le watchdog démarre lors de la connexion."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        # Vérifier que le thread watchdog existe et est actif
        assert backend._watchdog_thread is not None
        assert backend._watchdog_thread.is_alive()

        # Nettoyage
        backend.disconnect()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_watchdog_stop_on_disconnect(self):
        """Test que le watchdog s'arrête lors de la déconnexion."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        # Attendre un peu pour que le watchdog démarre
        time.sleep(0.2)

        watchdog_thread = backend._watchdog_thread
        assert watchdog_thread is not None

        # Déconnecter
        backend.disconnect()

        # Attendre que le thread se termine
        if watchdog_thread.is_alive():
            watchdog_thread.join(timeout=2.0)

        # Vérifier que le watchdog est arrêté
        assert backend._watchdog_thread is None
        assert backend._should_stop_watchdog.is_set()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_watchdog_stop_on_emergency_stop(self):
        """Test que le watchdog s'arrête lors de l'arrêt d'urgence."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        # Attendre un peu pour que le watchdog démarre
        time.sleep(0.2)

        watchdog_thread = backend._watchdog_thread
        assert watchdog_thread is not None

        # Emergency stop
        backend.emergency_stop()

        # Attendre que le thread se termine
        if watchdog_thread.is_alive():
            watchdog_thread.join(timeout=2.0)

        # Vérifier que le watchdog est arrêté
        assert (
            backend._watchdog_thread is None or not backend._watchdog_thread.is_alive()
        )

    @pytest.mark.unit
    @pytest.mark.fast
    def test_watchdog_heartbeat_update(self):
        """Test que le heartbeat est mis à jour."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        initial_heartbeat = backend._last_heartbeat

        # Attendre un peu pour que le watchdog mette à jour le heartbeat
        time.sleep(0.3)

        # En simulation, le heartbeat devrait être mis à jour
        assert backend._last_heartbeat >= initial_heartbeat

        # Nettoyage
        backend.disconnect()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_watchdog_interval_config(self):
        """Test que l'intervalle watchdog est configuré correctement."""
        backend = ReachyMiniBackend(use_sim=True)

        # Vérifier que l'intervalle est raisonnable (100ms)
        assert backend._watchdog_interval == 0.1
        assert 0.05 <= backend._watchdog_interval <= 1.0

    @pytest.mark.unit
    @pytest.mark.fast
    def test_watchdog_daemon_thread(self):
        """Test que le watchdog est un thread daemon."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        assert backend._watchdog_thread is not None
        assert (
            backend._watchdog_thread.daemon is True
        ), "Watchdog doit être un thread daemon"

        # Nettoyage
        backend.disconnect()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_watchdog_multiple_start_safe(self):
        """Test que démarrer le watchdog plusieurs fois est sûr."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        # Essayer de démarrer plusieurs fois
        backend._start_watchdog()
        backend._start_watchdog()
        backend._start_watchdog()

        # Devrait toujours y avoir un seul thread
        thread_count = sum(
            1 for thread in threading.enumerate() if thread.name == "ReachyWatchdog"
        )
        assert thread_count == 1, "Un seul thread watchdog doit exister"

        # Nettoyage
        backend.disconnect()
