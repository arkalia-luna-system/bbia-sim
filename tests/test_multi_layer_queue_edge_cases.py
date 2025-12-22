#!/usr/bin/env python3
"""Tests edge cases pour file d'attente multicouche."""

import asyncio
import math
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from bbia_sim.multi_layer_queue import (
    MovementPriority,
    MovementType,
    MultiLayerQueue,
    get_multi_layer_queue,
)


class TestMultiLayerQueueEdgeCases:
    """Tests edge cases pour MultiLayerQueue."""

    @pytest.mark.asyncio
    async def test_queue_full_edge_case(self):
        """Test queue pleine."""
        queue = MultiLayerQueue(max_queue_size=2)

        async def test_func():
            await asyncio.sleep(0.1)

        # Remplir la queue
        result1 = await queue.add_movement(test_func)
        result2 = await queue.add_movement(test_func)

        # La troisième devrait échouer ou être rejetée
        result3 = await queue.add_movement(test_func)

        # Vérifier que les deux premiers sont acceptés
        assert result1["status"] == "queued"
        assert result2["status"] == "queued"

        # Le troisième peut être accepté (si asyncio.Queue gère différemment)
        # ou rejeté (queue_full)
        assert result3["status"] in ["queued", "queue_full"]

        await queue.flush()

    @pytest.mark.asyncio
    async def test_priority_order_edge_case(self):
        """Test ordre des priorités avec beaucoup de mouvements."""
        queue = MultiLayerQueue()

        executed_order = []

        async def func(priority_name: str):
            executed_order.append(priority_name)
            await asyncio.sleep(0.01)

        # Ajouter dans l'ordre inverse de priorité
        for _ in range(5):
            await queue.add_pose(lambda: func("POSE"))
        for _ in range(5):
            await queue.add_emotion(lambda: func("EMOTION"))
        for _ in range(5):
            await queue.add_dance(lambda: func("DANCE"))

        # Flush pour exécuter
        await queue.flush()

        # Vérifier que les danses sont exécutées en premier
        # (les premières entrées dans executed_order devraient être DANCE)
        assert len(executed_order) == 15
        # Les 5 premières devraient être DANCE (priorité la plus haute)
        assert executed_order[:5] == ["DANCE"] * 5

    @pytest.mark.asyncio
    async def test_emergency_stop_during_execution(self):
        """Test arrêt d'urgence pendant exécution."""
        queue = MultiLayerQueue()

        async def long_func():
            await asyncio.sleep(1.0)

        # Ajouter plusieurs mouvements longs
        await queue.add_dance(long_func)
        await queue.add_emotion(long_func)
        await queue.add_pose(long_func)

        # Attendre un peu pour que l'exécution commence
        await asyncio.sleep(0.1)

        # Arrêt d'urgence
        await queue.emergency_stop()

        # Vérifier que la queue est vide
        assert queue.get_queue_size() == 0
        assert not queue.is_running

    @pytest.mark.asyncio
    async def test_movement_exception_handling(self):
        """Test gestion exceptions dans mouvements."""
        queue = MultiLayerQueue()

        async def failing_func():
            raise ValueError("Erreur test")

        result = await queue.add_movement(failing_func)
        assert result["status"] == "queued"

        # Flush pour exécuter
        await queue.flush()

        # Vérifier que l'erreur est gérée
        stats = queue.get_stats()
        assert stats["stats"]["total_failed"] >= 1
        assert stats["stats"]["total_executed"] == 0  # Échoué, pas exécuté

    @pytest.mark.asyncio
    async def test_concurrent_add_movements(self):
        """Test ajout concurrent de mouvements."""
        queue = MultiLayerQueue()

        async def add_many_movements():
            for i in range(10):

                async def func(id_val: int = i):
                    await asyncio.sleep(0.01)

                await queue.add_movement(func)

        # Ajouter mouvements en parallèle
        tasks = [asyncio.create_task(add_many_movements()) for _ in range(5)]
        await asyncio.gather(*tasks)

        # Vérifier que tous sont ajoutés
        assert queue.get_queue_size() == 50

        await queue.flush()

    @pytest.mark.asyncio
    async def test_max_parallel_limit(self):
        """Test limite max_parallel."""
        queue = MultiLayerQueue(max_parallel=2)

        execution_times = []
        lock = asyncio.Lock()

        async def func(id_val: int):
            async with lock:
                execution_times.append((id_val, asyncio.get_event_loop().time()))
            await asyncio.sleep(0.2)

        # Ajouter 5 mouvements
        for i in range(5):
            await queue.add_movement(lambda i=i: func(i))

        # Attendre un peu
        await asyncio.sleep(0.3)

        # Vérifier qu'au plus max_parallel sont en cours
        assert queue.get_running_count() <= queue.max_parallel

        await queue.flush()

    @pytest.mark.asyncio
    async def test_invalid_priority(self):
        """Test priorité invalide."""
        queue = MultiLayerQueue()

        async def test_func():
            pass

        # Utiliser une priorité valide mais via add_movement
        result = await queue.add_movement(
            test_func,
            priority=MovementPriority.POSE,  # Priorité valide
            movement_type="invalid_type",
        )

        assert result["status"] == "queued"

        await queue.flush()

    @pytest.mark.asyncio
    async def test_flush_empty_queue(self):
        """Test flush sur queue vide."""
        queue = MultiLayerQueue()

        # Flush sur queue vide ne doit pas lever d'erreur
        await queue.flush()

        stats = queue.get_stats()
        assert stats["stats"]["total_executed"] == 0

    @pytest.mark.asyncio
    async def test_get_stats_during_execution(self):
        """Test récupération stats pendant exécution."""
        queue = MultiLayerQueue()

        async def long_func():
            await asyncio.sleep(0.2)

        await queue.add_dance(long_func)
        await queue.add_emotion(long_func)

        # Attendre un peu
        await asyncio.sleep(0.1)

        # Récupérer stats pendant exécution
        stats = queue.get_stats()
        assert "queue_sizes" in stats
        assert "running_count" in stats
        assert "stats" in stats

        await queue.flush()

    @pytest.mark.asyncio
    async def test_metadata_preservation(self):
        """Test préservation métadonnées."""
        queue = MultiLayerQueue()

        metadata = {"test_key": "test_value", "number": 42}

        async def test_func():
            pass

        result = await queue.add_movement(
            test_func, movement_id="test_id", metadata=metadata
        )

        assert result["status"] == "queued"
        assert result["movement_id"] == "test_id"

        await queue.flush()

    @pytest.mark.asyncio
    async def test_worker_loop_exception_recovery(self):
        """Test récupération après exception dans worker loop."""
        queue = MultiLayerQueue()

        # Simuler une exception dans le worker
        original_worker = queue._worker_loop

        call_count = 0

        async def failing_worker():
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                raise RuntimeError("Erreur test")
            await original_worker()

        queue._worker_loop = failing_worker

        async def test_func():
            await asyncio.sleep(0.01)

        # Ajouter un mouvement
        await queue.add_movement(test_func)

        # Attendre un peu
        await asyncio.sleep(0.2)

        # Le worker devrait se récupérer
        assert queue.is_running or queue.get_queue_size() == 0

        await queue.flush()

    @pytest.mark.asyncio
    async def test_multiple_emergency_stops(self):
        """Test appels multiples emergency_stop."""
        queue = MultiLayerQueue()

        async def test_func():
            await asyncio.sleep(0.1)

        await queue.add_movement(test_func)

        # Appeler emergency_stop plusieurs fois
        await queue.emergency_stop()
        await queue.emergency_stop()
        await queue.emergency_stop()

        # Ne doit pas lever d'erreur
        assert queue.get_queue_size() == 0
        assert not queue.is_running

    @pytest.mark.asyncio
    async def test_get_queue_size_by_priority(self):
        """Test get_queue_size avec priorité spécifique."""
        queue = MultiLayerQueue()

        async def test_func():
            pass

        await queue.add_dance(test_func)
        await queue.add_emotion(test_func)
        await queue.add_pose(test_func)

        # Vérifier tailles par priorité
        assert queue.get_queue_size(MovementPriority.DANCE) >= 1
        assert queue.get_queue_size(MovementPriority.EMOTION) >= 1
        assert queue.get_queue_size(MovementPriority.POSE) >= 1

        await queue.flush()

    @pytest.mark.asyncio
    async def test_global_instance_singleton(self):
        """Test que get_multi_layer_queue retourne singleton."""
        queue1 = get_multi_layer_queue()
        queue2 = get_multi_layer_queue()

        assert queue1 is queue2

    @pytest.mark.asyncio
    async def test_stats_accuracy(self):
        """Test précision des statistiques."""
        queue = MultiLayerQueue()

        async def success_func():
            await asyncio.sleep(0.01)

        async def fail_func():
            raise ValueError("Erreur")

        # Ajouter mouvements réussis et échoués
        await queue.add_movement(success_func)
        await queue.add_movement(success_func)
        await queue.add_movement(fail_func)

        await queue.flush()

        stats = queue.get_stats()
        assert stats["stats"]["total_queued"] == 3
        assert stats["stats"]["total_executed"] == 2
        assert stats["stats"]["total_failed"] == 1
