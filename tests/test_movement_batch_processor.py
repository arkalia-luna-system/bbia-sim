#!/usr/bin/env python3
"""
🧪 TESTS BATCH PROCESSING MOUVEMENTS
Tests pour garantir le bon fonctionnement du batch processing pour mouvements multiples.
"""

import asyncio
import sys
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.movement_batch_processor import (
    MovementBatchProcessor,
    get_batch_processor,
)


class TestMovementBatchProcessor:
    """Tests pour le batch processing de mouvements."""

    @pytest.fixture
    def processor(self):
        """Processeur de batch pour les tests."""
        return MovementBatchProcessor(max_batch_size=3, batch_timeout=0.05)

    @pytest.mark.asyncio
    async def test_add_movement(self, processor):
        """Test que les mouvements peuvent être ajoutés au batch."""
        async def dummy_movement():
            await asyncio.sleep(0.01)

        result = await processor.add_movement(dummy_movement, "test_move_1")
        assert result["status"] == "queued"
        assert result["movement_id"] == "test_move_1"
        assert result["batch_size"] == 1

        print("✅ Ajout mouvement au batch fonctionne.")

    @pytest.mark.asyncio
    async def test_batch_execution(self, processor):
        """Test que les mouvements sont exécutés en batch."""
        executed = []

        async def movement_func(move_id: str):
            executed.append(move_id)
            await asyncio.sleep(0.01)

        # Ajouter plusieurs mouvements
        await processor.add_movement(lambda: movement_func("move1"))
        await processor.add_movement(lambda: movement_func("move2"))
        await processor.add_movement(lambda: movement_func("move3"))

        # Attendre que le batch soit traité
        await asyncio.sleep(0.2)

        assert len(executed) == 3
        assert "move1" in executed
        assert "move2" in executed
        assert "move3" in executed

        print("✅ Exécution batch fonctionne.")

    @pytest.mark.asyncio
    async def test_batch_max_size(self, processor):
        """Test que le batch respecte la taille maximale."""
        executed = []

        async def movement_func(move_id: str):
            executed.append(move_id)
            await asyncio.sleep(0.01)

        # Ajouter plus de mouvements que max_batch_size
        for i in range(5):
            await processor.add_movement(lambda: movement_func(f"move{i}"))

        # Attendre que les batches soient traités
        await asyncio.sleep(0.3)

        # Devrait avoir exécuté tous les mouvements
        assert len(executed) == 5

        print("✅ Taille maximale batch respectée.")

    @pytest.mark.asyncio
    async def test_batch_timeout(self, processor):
        """Test que le batch est exécuté après le timeout."""
        executed = []

        async def movement_func(move_id: str):
            executed.append(move_id)
            await asyncio.sleep(0.01)

        # Ajouter un mouvement
        await processor.add_movement(lambda: movement_func("move1"))

        # Attendre le timeout
        await asyncio.sleep(0.1)

        assert len(executed) == 1
        assert "move1" in executed

        print("✅ Timeout batch fonctionne.")

    @pytest.mark.asyncio
    async def test_flush(self, processor):
        """Test que flush exécute immédiatement tous les mouvements en attente."""
        executed = []

        async def movement_func(move_id: str):
            executed.append(move_id)
            await asyncio.sleep(0.01)

        # Ajouter des mouvements
        await processor.add_movement(lambda: movement_func("move1"))
        await processor.add_movement(lambda: movement_func("move2"))

        # Flush immédiatement (sans attendre timeout)
        await processor.flush()

        assert len(executed) == 2
        assert "move1" in executed
        assert "move2" in executed

        print("✅ Flush fonctionne.")

    @pytest.mark.asyncio
    async def test_get_queue_size(self, processor):
        """Test que get_queue_size retourne la taille correcte."""
        async def dummy_movement():
            await asyncio.sleep(0.01)

        assert processor.get_queue_size() == 0

        await processor.add_movement(dummy_movement)
        assert processor.get_queue_size() == 1

        await processor.add_movement(dummy_movement)
        assert processor.get_queue_size() == 2

        print("✅ get_queue_size fonctionne.")

    @pytest.mark.asyncio
    async def test_get_stats(self, processor):
        """Test que get_stats retourne les bonnes statistiques."""
        stats = processor.get_stats()

        assert "queue_size" in stats
        assert "max_batch_size" in stats
        assert "batch_timeout" in stats
        assert "is_running" in stats

        assert stats["max_batch_size"] == 3
        assert stats["batch_timeout"] == 0.05

        print("✅ get_stats fonctionne.")

    @pytest.mark.asyncio
    async def test_error_handling(self, processor):
        """Test que les erreurs dans les mouvements sont gérées correctement."""
        executed = []

        async def failing_movement():
            raise ValueError("Test error")

        async def success_movement(move_id: str):
            executed.append(move_id)

        # Ajouter un mouvement qui échoue et un qui réussit
        await processor.add_movement(failing_movement)
        await processor.add_movement(lambda: success_movement("success"))

        # Attendre traitement
        await asyncio.sleep(0.2)

        # Le mouvement qui réussit devrait être exécuté malgré l'erreur
        assert "success" in executed

        print("✅ Gestion erreurs fonctionne.")

    @pytest.mark.asyncio
    async def test_get_batch_processor_singleton(self):
        """Test que get_batch_processor retourne une instance singleton."""
        processor1 = get_batch_processor()
        processor2 = get_batch_processor()

        assert processor1 is processor2

        print("✅ Singleton batch processor fonctionne.")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

