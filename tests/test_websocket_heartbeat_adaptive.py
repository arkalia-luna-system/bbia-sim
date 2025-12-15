#!/usr/bin/env python3
"""Tests pour heartbeat WebSocket adaptatif."""

from unittest.mock import AsyncMock

import pytest

from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager


class TestAdaptiveHeartbeat:
    """Tests pour heartbeat adaptatif selon latence."""

    def test_heartbeat_adapts_to_low_latency(self):
        """Test heartbeat plus rapide si latence faible."""
        manager = BBIAAdvancedWebSocketManager()

        # Simuler latence faible (5ms)
        for _ in range(5):
            manager._update_latency(5.0)

        heartbeat = manager._calculate_adaptive_heartbeat()

        # Latence faible → heartbeat plus rapide (proche de 10s)
        assert heartbeat < 30.0
        assert heartbeat >= 10.0

    def test_heartbeat_adapts_to_high_latency(self):
        """Test heartbeat plus lent si latence élevée."""
        manager = BBIAAdvancedWebSocketManager()

        # Simuler latence élevée (100ms)
        for _ in range(5):
            manager._update_latency(100.0)

        heartbeat = manager._calculate_adaptive_heartbeat()

        # Latence élevée (100ms) → heartbeat plus lent
        # Formule: 10 + (100/10)*2 = 10 + 20 = 30s
        assert heartbeat > 20.0
        assert heartbeat <= 60.0

    def test_heartbeat_stays_within_bounds(self):
        """Test heartbeat reste entre 10s-60s."""
        manager = BBIAAdvancedWebSocketManager()

        # Test latence très faible (1ms)
        for _ in range(5):
            manager._update_latency(1.0)
        heartbeat_low = manager._calculate_adaptive_heartbeat()
        assert heartbeat_low >= 10.0

        # Test latence très élevée (1000ms)
        for _ in range(5):
            manager._update_latency(1000.0)
        heartbeat_high = manager._calculate_adaptive_heartbeat()
        assert heartbeat_high <= 60.0

    def test_heartbeat_updates_on_latency_change(self):
        """Test heartbeat s'ajuste dynamiquement."""
        manager = BBIAAdvancedWebSocketManager()

        # Latence faible initiale (5ms)
        for _ in range(5):
            manager._update_latency(5.0)
        heartbeat1 = manager._heartbeat_interval

        # Latence élevée (100ms)
        for _ in range(5):
            manager._update_latency(100.0)
        heartbeat2 = manager._heartbeat_interval

        # Heartbeat devrait augmenter
        assert heartbeat2 > heartbeat1

    def test_heartbeat_default_when_no_history(self):
        """Test heartbeat valeur par défaut si pas d'historique."""
        manager = BBIAAdvancedWebSocketManager()

        # Pas d'historique
        heartbeat = manager._calculate_adaptive_heartbeat()

        # Devrait retourner valeur par défaut (30s)
        assert heartbeat == 30.0

    def test_latency_history_limited(self):
        """Test que l'historique de latence est limité."""
        manager = BBIAAdvancedWebSocketManager()

        # Ajouter plus que max_latency_history
        for i in range(15):
            manager._update_latency(float(i))

        # L'historique devrait être limité à max_latency_history (10)
        assert len(manager._latency_history) == manager._max_latency_history

    @pytest.mark.asyncio
    async def test_heartbeat_sends_with_adaptive_interval(self):
        """Test envoi heartbeat avec intervalle adaptatif."""
        manager = BBIAAdvancedWebSocketManager()
        manager.active_connections = [AsyncMock()]

        # Simuler latence faible
        manager._update_latency(10.0)

        # Vérifier que heartbeat utilise intervalle adaptatif
        initial_interval = manager._heartbeat_interval

        await manager._send_heartbeat()

        # Vérifier que heartbeat a été envoyé
        assert manager._last_heartbeat > 0
        assert manager._heartbeat_interval == initial_interval

    def test_update_latency_recalculates_heartbeat(self):
        """Test que _update_latency recalcule heartbeat."""
        manager = BBIAAdvancedWebSocketManager()

        # Heartbeat initial (par défaut)
        initial_heartbeat = manager._heartbeat_interval

        # Ajouter latence
        manager._update_latency(50.0)

        # Heartbeat devrait être recalculé
        assert manager._heartbeat_interval != initial_heartbeat
