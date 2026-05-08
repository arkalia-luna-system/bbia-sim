"""Batch processing pour mouvements multiples - Optimisation performance."""

import asyncio
import logging
from collections.abc import Callable, Coroutine
from typing import Any

logger = logging.getLogger(__name__)

# Configuration batch processing
BATCH_MAX_SIZE: int = 10  # Maximum 10 mouvements par batch
BATCH_TIMEOUT: float = 0.1  # Attendre 100ms pour remplir le batch


class MovementBatchProcessor:
    """Processeur de batch pour exécuter plusieurs mouvements ensemble."""

    def __init__(
        self,
        max_batch_size: int = BATCH_MAX_SIZE,
        batch_timeout: float = BATCH_TIMEOUT,
    ) -> None:
        """Initialise le processeur de batch.

        Args:
            max_batch_size: Taille maximale d'un batch
            batch_timeout: Timeout pour remplir un batch (secondes)
        """
        if max_batch_size <= 0:
            msg = "max_batch_size doit etre > 0"
            raise ValueError(msg)
        if batch_timeout < 0:
            msg = "batch_timeout doit etre >= 0"
            raise ValueError(msg)

        self.max_batch_size = max_batch_size
        self.batch_timeout = batch_timeout
        self.batch_queue: list[dict[str, Any]] = []
        self.batch_lock = asyncio.Lock()
        self.batch_task: asyncio.Task[None] | None = None
        self.is_running = False

    async def add_movement(
        self,
        movement_func: Callable[[], Coroutine[Any, Any, None]],
        movement_id: str | None = None,
    ) -> dict[str, Any]:
        """Ajoute un mouvement au batch.

        Args:
            movement_func: Fonction async qui exécute le mouvement
            movement_id: ID optionnel du mouvement

        Returns:
            Dictionnaire avec status et batch_id
        """
        loop = asyncio.get_running_loop()
        movement_data = {
            "func": movement_func,
            "id": movement_id or f"move_{loop.time()}",
            "added_at": loop.time(),
        }

        async with self.batch_lock:
            self.batch_queue.append(movement_data)

            # Démarrer le traitement si pas déjà en cours
            if not self.is_running:
                self.is_running = True
                self.batch_task = asyncio.create_task(self._process_batches())

        return {
            "status": "queued",
            "movement_id": movement_data["id"],
            "batch_size": len(self.batch_queue),
        }

    async def _process_batches(self) -> None:
        """Traite les batches de mouvements."""
        while True:
            try:
                # Attendre un peu pour remplir le batch
                await asyncio.sleep(self.batch_timeout if self.batch_timeout > 0 else 0)

                async with self.batch_lock:
                    if not self.batch_queue:
                        self.is_running = False
                        break

                    # Extraire un batch (max max_batch_size)
                    batch = self.batch_queue[: self.max_batch_size]
                    self.batch_queue = self.batch_queue[self.max_batch_size :]

                # Exécuter le batch
                if batch:
                    await self._execute_batch(batch)

            except Exception as e:
                logger.exception("Erreur traitement batch: %s", e)
                async with self.batch_lock:
                    self.is_running = False
                break

    async def _execute_batch(self, batch: list[dict[str, Any]]) -> None:
        """Exécute un batch de mouvements.

        Args:
            batch: Liste de mouvements à exécuter
        """
        logger.info("📦 Exécution batch de %d mouvements", len(batch))

        # Exécuter les mouvements en parallèle (si possible) ou séquentiellement
        # Pour l'instant, exécution séquentielle pour éviter conflits
        # TODO: Analyser si certains mouvements peuvent être parallélisés
        tasks = []
        for movement in batch:
            try:
                task = asyncio.create_task(movement["func"]())
                tasks.append((task, movement["id"]))
            except Exception as e:
                logger.error(
                    "Erreur création tâche mouvement %s: %s",
                    movement["id"],
                    e,
                )

        # Attendre que toutes les tâches se terminent
        for task, movement_id in tasks:
            try:
                await task
                logger.debug("✅ Mouvement %s terminé", movement_id)
            except Exception as e:
                logger.error("Erreur exécution mouvement %s: %s", movement_id, e)

        logger.info("✅ Batch de %d mouvements terminé", len(batch))

    async def flush(self) -> None:
        """Force l'exécution immédiate de tous les mouvements en attente."""
        batch: list[dict[str, Any]] = []
        async with self.batch_lock:
            if self.batch_queue:
                batch = self.batch_queue.copy()
                self.batch_queue.clear()

        if batch:
            await self._execute_batch(batch)

    async def wait_for_idle(self, timeout: float = 2.0) -> bool:
        """Attend que le processeur ait fini tout le travail en cours."""
        start = asyncio.get_running_loop().time()
        while True:
            async with self.batch_lock:
                queue_empty = not self.batch_queue
                running = self.is_running
                task = self.batch_task

            if queue_empty and not running:
                return True
            if task and task.done():
                return True

            if asyncio.get_running_loop().time() - start > timeout:
                return False
            await asyncio.sleep(0.01)

    def get_queue_size(self) -> int:
        """Retourne la taille actuelle de la queue."""
        return len(self.batch_queue)

    def get_stats(self) -> dict[str, Any]:
        """Retourne les statistiques du processeur de batch."""
        return {
            "queue_size": len(self.batch_queue),
            "max_batch_size": self.max_batch_size,
            "batch_timeout": self.batch_timeout,
            "is_running": self.is_running,
            "task_active": self.batch_task is not None and not self.batch_task.done(),
        }


# Instance globale du processeur de batch
_global_batch_processor: MovementBatchProcessor | None = None


def get_batch_processor() -> MovementBatchProcessor:
    """Retourne l'instance globale du processeur de batch."""
    global _global_batch_processor
    if _global_batch_processor is None:
        _global_batch_processor = MovementBatchProcessor()
    return _global_batch_processor
