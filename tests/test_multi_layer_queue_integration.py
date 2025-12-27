#!/usr/bin/env python3
"""Tests d'intégration pour file d'attente multicouche avec API."""

import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from bbia_sim.multi_layer_queue import (
    MovementPriority,
    get_multi_layer_queue,
)


class TestMultiLayerQueueIntegration:
    """Tests d'intégration pour multi-layer queue."""

    @pytest.mark.asyncio
    async def test_dance_integration(self):
        """Test intégration danses."""
        queue = get_multi_layer_queue()

        # Mock backend avec play_move
        mock_backend = MagicMock()
        mock_backend.play_move = AsyncMock()

        async def dance_func():
            # Simuler exécution danse
            await asyncio.sleep(0.1)
            if hasattr(mock_backend, "play_move"):
                await mock_backend.play_move("dance_happy")

        result = await queue.add_dance(dance_func, dance_id="test_dance")
        assert result["status"] == "queued"
        assert result["priority"] == MovementPriority.DANCE.name

        # Flush pour exécuter
        await queue.flush()

    @pytest.mark.asyncio
    async def test_emotion_integration(self):
        """Test intégration émotions."""
        queue = get_multi_layer_queue()

        # Mock BBIAEmotions
        with patch("bbia_sim.bbia_emotions.BBIAEmotions") as mock_emotions_class:
            mock_emotions = MagicMock()
            mock_emotions.set_emotion.return_value = True
            mock_emotions_class.return_value = mock_emotions

            async def emotion_func():
                from bbia_sim.bbia_emotions import BBIAEmotions

                emotions = BBIAEmotions()
                emotions.set_emotion("happy", 0.8)

            result = await queue.add_emotion(emotion_func, emotion_id="test_emotion")
            assert result["status"] == "queued"
            assert result["priority"] == MovementPriority.EMOTION.name

            # Flush pour exécuter
            await queue.flush()

            # Vérifier que set_emotion a été appelé
            mock_emotions.set_emotion.assert_called()

    @pytest.mark.asyncio
    async def test_pose_integration(self):
        """Test intégration poses."""
        queue = get_multi_layer_queue()

        # Mock backend avec goto_target
        mock_backend = MagicMock()
        mock_backend.goto_target = AsyncMock()

        async def pose_func():
            if hasattr(mock_backend, "goto_target"):
                await mock_backend.goto_target(head=None, antennas=None, duration=1.0)

        result = await queue.add_pose(pose_func, pose_id="test_pose")
        assert result["status"] == "queued"
        assert result["priority"] == MovementPriority.POSE.name

        # Flush pour exécuter
        await queue.flush()

    @pytest.mark.asyncio
    async def test_mixed_movements(self):
        """Test mouvements mixtes (danse + émotion + pose)."""
        queue = get_multi_layer_queue()

        executed = []

        async def dance_func():
            executed.append("dance")

        async def emotion_func():
            executed.append("emotion")

        async def pose_func():
            executed.append("pose")

        # Ajouter dans l'ordre inverse de priorité
        await queue.add_pose(pose_func)
        await queue.add_emotion(emotion_func)
        await queue.add_dance(dance_func)

        # Flush pour exécuter
        await queue.flush()

        # Vérifier que tous ont été exécutés
        assert len(executed) == 3
        assert "dance" in executed
        assert "emotion" in executed
        assert "pose" in executed
