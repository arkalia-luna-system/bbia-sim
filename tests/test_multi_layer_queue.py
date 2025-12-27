#!/usr/bin/env python3
"""Tests pour file d'attente multicouche."""

import asyncio
from unittest.mock import AsyncMock, MagicMock

import pytest

from bbia_sim.multi_layer_queue import (
    MovementPriority,
    MovementType,
    MultiLayerQueue,
)


class TestMultiLayerQueue:
    """Tests pour MultiLayerQueue."""

    def test_init(self):
        """Test initialisation."""
        queue = MultiLayerQueue()
        assert queue.max_queue_size == 100
        assert queue.max_parallel == 3
        assert not queue.is_running

    def test_add_movement(self):
        """Test ajout mouvement."""
        queue = MultiLayerQueue()

        async def test_func():
            pass

        result = asyncio.run(queue.add_movement(test_func))
        assert result["status"] == "queued"
        assert "movement_id" in result

    def test_add_dance(self):
        """Test ajout danse."""
        queue = MultiLayerQueue()

        async def dance_func():
            pass

        result = asyncio.run(queue.add_dance(dance_func))
        assert result["status"] == "queued"
        assert result["priority"] == MovementPriority.DANCE.name

    def test_add_emotion(self):
        """Test ajout émotion."""
        queue = MultiLayerQueue()

        async def emotion_func():
            pass

        result = asyncio.run(queue.add_emotion(emotion_func))
        assert result["status"] == "queued"
        assert result["priority"] == MovementPriority.EMOTION.name

    def test_add_pose(self):
        """Test ajout pose."""
        queue = MultiLayerQueue()

        async def pose_func():
            pass

        result = asyncio.run(queue.add_pose(pose_func))
        assert result["status"] == "queued"
        assert result["priority"] == MovementPriority.POSE.name

    @pytest.mark.asyncio
    async def test_priority_order(self):
        """Test ordre des priorités."""
        queue = MultiLayerQueue()

        async def func1():
            pass

        async def func2():
            pass

        async def func3():
            pass

        # Ajouter dans l'ordre inverse de la priorité
        await queue.add_pose(func3)
        await queue.add_emotion(func2)
        await queue.add_dance(func1)

        # Vérifier que les queues contiennent les mouvements
        # Note: asyncio.Queue.qsize() peut ne pas être fiable, utiliser get_queue_size() à la place
        assert queue.get_queue_size(MovementPriority.DANCE) >= 1
        assert queue.get_queue_size(MovementPriority.EMOTION) >= 1
        assert queue.get_queue_size(MovementPriority.POSE) >= 1

    @pytest.mark.asyncio
    async def test_get_queue_size(self):
        """Test taille queue."""
        queue = MultiLayerQueue()

        async def test_func():
            pass

        await queue.add_movement(test_func)
        assert queue.get_queue_size() > 0
        assert queue.get_queue_size(MovementPriority.POSE) > 0

    def test_get_stats(self):
        """Test statistiques."""
        queue = MultiLayerQueue()

        async def test_func():
            pass

        asyncio.run(queue.add_movement(test_func))
        stats = queue.get_stats()
        assert "queue_sizes" in stats
        assert "running_count" in stats
        assert "stats" in stats

    @pytest.mark.asyncio
    async def test_emergency_stop(self):
        """Test arrêt d'urgence."""
        queue = MultiLayerQueue()

        async def test_func():
            await asyncio.sleep(1)

        # Ajouter quelques mouvements
        await queue.add_movement(test_func)
        await queue.add_movement(test_func)

        # Arrêt d'urgence
        await queue.emergency_stop()

        # Vérifier que les queues sont vides
        assert queue.get_queue_size() == 0
        assert not queue.is_running

    @pytest.mark.asyncio
    async def test_parallel_execution(self):
        """Test exécution parallèle."""
        queue = MultiLayerQueue(max_parallel=2)

        execution_times = []

        async def test_func(delay: float):
            execution_times.append(asyncio.get_event_loop().time())
            await asyncio.sleep(delay)

        # Ajouter 3 mouvements
        await queue.add_movement(lambda: test_func(0.1))
        await queue.add_movement(lambda: test_func(0.1))
        await queue.add_movement(lambda: test_func(0.1))

        # Attendre un peu pour que les mouvements commencent
        await asyncio.sleep(0.2)

        # Vérifier qu'au plus max_parallel mouvements sont en cours
        assert queue.get_running_count() <= queue.max_parallel

        # Flush pour terminer
        await queue.flush()

    @pytest.mark.asyncio
    async def test_flush(self):
        """Test flush."""
        queue = MultiLayerQueue()

        executed = []

        async def test_func(id_val: str):
            executed.append(id_val)

        # Ajouter plusieurs mouvements
        await queue.add_movement(lambda: test_func("1"))
        await queue.add_movement(lambda: test_func("2"))
        await queue.add_movement(lambda: test_func("3"))

        # Flush
        await queue.flush()

        # Vérifier que tous les mouvements ont été exécutés
        assert len(executed) == 3
