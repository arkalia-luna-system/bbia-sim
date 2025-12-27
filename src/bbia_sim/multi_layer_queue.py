#!/usr/bin/env python3
"""File d'attente multicouche pour mouvements simultanés.

Support pour danses, émotions, poses simultanées avec priorités.
"""

import asyncio
import logging
from collections.abc import Callable, Coroutine
from enum import IntEnum
from typing import Any

logger = logging.getLogger(__name__)


class MovementPriority(IntEnum):
    """Priorités des mouvements."""

    EMERGENCY = 0  # Arrêt d'urgence
    DANCE = 1  # Danses (priorité haute)
    EMOTION = 2  # Émotions (priorité moyenne)
    POSE = 3  # Poses simples (priorité basse)
    BACKGROUND = 4  # Mouvements de fond (priorité très basse)


class MovementType:
    """Types de mouvements."""

    DANCE = "dance"
    EMOTION = "emotion"
    POSE = "pose"
    BACKGROUND = "background"
    EMERGENCY = "emergency"


class MultiLayerQueue:
    """File d'attente multicouche pour mouvements simultanés.

    Support pour exécuter danses, émotions, poses simultanément
    avec gestion des priorités et conflits.
    """

    def __init__(
        self,
        max_queue_size: int = 100,
        max_parallel: int = 3,
    ) -> None:
        """Initialise la file d'attente multicouche.

        Args:
            max_queue_size: Taille maximale de la queue
            max_parallel: Nombre maximum de mouvements parallèles
        """
        self.max_queue_size = max_queue_size
        self.max_parallel = max_parallel

        # Queues par priorité
        self.queues: dict[MovementPriority, asyncio.Queue[dict[str, Any]]] = {
            priority: asyncio.Queue(maxsize=max_queue_size)
            for priority in MovementPriority
        }

        # Tâches en cours d'exécution
        self.running_tasks: dict[str, asyncio.Task[None]] = {}
        self.running_lock = asyncio.Lock()

        # Statistiques
        self.stats: dict[str, Any] = {
            "total_queued": 0,
            "total_executed": 0,
            "total_failed": 0,
            "by_priority": {p.name: 0 for p in MovementPriority},
        }

        # Worker task
        self.worker_task: asyncio.Task[None] | None = None
        self.is_running = False

    async def add_movement(
        self,
        movement_func: Callable[[], Coroutine[Any, Any, None]],
        priority: MovementPriority = MovementPriority.POSE,
        movement_type: str = MovementType.POSE,
        movement_id: str | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Ajoute un mouvement à la queue.

        Args:
            movement_func: Fonction async qui exécute le mouvement
            priority: Priorité du mouvement
            movement_type: Type de mouvement (dance, emotion, pose, etc.)
            movement_id: ID optionnel du mouvement
            metadata: Métadonnées optionnelles

        Returns:
            Dictionnaire avec status et movement_id
        """
        movement_id = (
            movement_id or f"{movement_type}_{asyncio.get_event_loop().time()}"
        )

        movement_data = {
            "func": movement_func,
            "id": movement_id,
            "type": movement_type,
            "priority": priority,
            "metadata": metadata or {},
            "added_at": asyncio.get_event_loop().time(),
        }

        try:
            # Ajouter à la queue correspondante
            queue = self.queues[priority]
            await queue.put(movement_data)

            # Mettre à jour stats
            self.stats["total_queued"] += 1
            self.stats["by_priority"][priority.name] += 1

            # Démarrer le worker si pas déjà en cours
            if not self.is_running:
                self.is_running = True
                self.worker_task = asyncio.create_task(self._worker_loop())

            logger.debug(
                "Mouvement %s ajouté à la queue (priorité: %s, type: %s)",
                movement_id,
                priority.name,
                movement_type,
            )

            return {
                "status": "queued",
                "movement_id": movement_id,
                "priority": priority.name,
                "queue_size": queue.qsize(),
            }

        except asyncio.QueueFull:
            logger.warning("Queue pleine pour priorité %s", priority.name)
            return {
                "status": "queue_full",
                "movement_id": movement_id,
                "error": "Queue pleine",
            }

    async def _worker_loop(self) -> None:
        """Boucle principale du worker qui traite les mouvements."""
        while True:
            try:
                # Vérifier si on peut exécuter plus de mouvements
                async with self.running_lock:
                    if len(self.running_tasks) >= self.max_parallel:
                        # Attendre qu'un mouvement se termine
                        await asyncio.sleep(0.1)
                        continue

                # Chercher le mouvement de plus haute priorité disponible
                movement_data = None
                for priority in sorted(MovementPriority):
                    queue = self.queues[priority]
                    if not queue.empty():
                        try:
                            movement_data = queue.get_nowait()
                            break
                        except asyncio.QueueEmpty:
                            continue

                if movement_data is None:
                    # Aucun mouvement disponible, attendre un peu
                    await asyncio.sleep(0.1)
                    # Vérifier si toutes les queues sont vides et aucune tâche en cours
                    async with self.running_lock:
                        if (
                            all(q.empty() for q in self.queues.values())
                            and not self.running_tasks
                        ):
                            self.is_running = False
                            break
                    continue

                # Exécuter le mouvement
                task = asyncio.create_task(self._execute_movement(movement_data))
                async with self.running_lock:
                    self.running_tasks[movement_data["id"]] = task

            except Exception as e:
                logger.exception("Erreur dans worker loop: %s", e)
                await asyncio.sleep(0.1)

    async def _execute_movement(self, movement_data: dict[str, Any]) -> None:
        """Exécute un mouvement.

        Args:
            movement_data: Données du mouvement
        """
        movement_id = movement_data["id"]
        movement_type = movement_data["type"]

        try:
            logger.debug(
                "Exécution mouvement %s (type: %s, priorité: %s)",
                movement_id,
                movement_type,
                movement_data["priority"].name,
            )

            # Exécuter la fonction
            await movement_data["func"]()

            # Mettre à jour stats
            self.stats["total_executed"] += 1

            logger.debug("✅ Mouvement %s terminé", movement_id)

        except Exception as e:
            logger.error("Erreur exécution mouvement %s: %s", movement_id, e)
            self.stats["total_failed"] += 1

        finally:
            # Retirer de la liste des tâches en cours
            async with self.running_lock:
                self.running_tasks.pop(movement_id, None)

    async def add_dance(
        self,
        dance_func: Callable[[], Coroutine[Any, Any, None]],
        dance_id: str | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Ajoute une danse à la queue (priorité haute).

        Args:
            dance_func: Fonction async qui exécute la danse
            dance_id: ID optionnel de la danse
            metadata: Métadonnées optionnelles

        Returns:
            Dictionnaire avec status et dance_id
        """
        return await self.add_movement(
            dance_func,
            priority=MovementPriority.DANCE,
            movement_type=MovementType.DANCE,
            movement_id=dance_id,
            metadata=metadata,
        )

    async def add_emotion(
        self,
        emotion_func: Callable[[], Coroutine[Any, Any, None]],
        emotion_id: str | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Ajoute une émotion à la queue (priorité moyenne).

        Args:
            emotion_func: Fonction async qui applique l'émotion
            emotion_id: ID optionnel de l'émotion
            metadata: Métadonnées optionnelles

        Returns:
            Dictionnaire avec status et emotion_id
        """
        return await self.add_movement(
            emotion_func,
            priority=MovementPriority.EMOTION,
            movement_type=MovementType.EMOTION,
            movement_id=emotion_id,
            metadata=metadata,
        )

    async def add_pose(
        self,
        pose_func: Callable[[], Coroutine[Any, Any, None]],
        pose_id: str | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Ajoute une pose à la queue (priorité basse).

        Args:
            pose_func: Fonction async qui applique la pose
            pose_id: ID optionnel de la pose
            metadata: Métadonnées optionnelles

        Returns:
            Dictionnaire avec status et pose_id
        """
        return await self.add_movement(
            pose_func,
            priority=MovementPriority.POSE,
            movement_type=MovementType.POSE,
            movement_id=pose_id,
            metadata=metadata,
        )

    async def emergency_stop(self) -> None:
        """Arrêt d'urgence - vide toutes les queues et arrête les mouvements."""
        logger.warning("🛑 Arrêt d'urgence - vidage de toutes les queues")

        # Vider toutes les queues
        for queue in self.queues.values():
            while not queue.empty():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    break

        # Annuler toutes les tâches en cours
        async with self.running_lock:
            for task in self.running_tasks.values():
                task.cancel()
            self.running_tasks.clear()

        # Arrêter le worker
        if self.worker_task:
            self.worker_task.cancel()
            try:
                await self.worker_task
            except asyncio.CancelledError:
                pass
        self.is_running = False

    def get_queue_size(self, priority: MovementPriority | None = None) -> int:
        """Retourne la taille de la queue.

        Args:
            priority: Priorité spécifique, ou None pour total

        Returns:
            Taille de la queue
        """
        if priority is not None:
            return self.queues[priority].qsize()

        return sum(q.qsize() for q in self.queues.values())

    def get_running_count(self) -> int:
        """Retourne le nombre de mouvements en cours d'exécution."""
        return len(self.running_tasks)

    def get_stats(self) -> dict[str, Any]:
        """Retourne les statistiques de la queue."""
        return {
            "queue_sizes": {
                priority.name: queue.qsize() for priority, queue in self.queues.items()
            },
            "running_count": len(self.running_tasks),
            "max_parallel": self.max_parallel,
            "stats": self.stats.copy(),
        }

    async def flush(self) -> None:
        """Force l'exécution immédiate de tous les mouvements en attente."""
        # Attendre que toutes les tâches en cours se terminent
        async with self.running_lock:
            if self.running_tasks:
                await asyncio.gather(
                    *self.running_tasks.values(), return_exceptions=True
                )

        # Exécuter tous les mouvements restants
        while True:
            movement_data = None
            for priority in sorted(MovementPriority):
                queue = self.queues[priority]
                if not queue.empty():
                    try:
                        movement_data = queue.get_nowait()
                        break
                    except asyncio.QueueEmpty:
                        continue

            if movement_data is None:
                break

            await self._execute_movement(movement_data)


# Instance globale
_global_multi_layer_queue: MultiLayerQueue | None = None


def get_multi_layer_queue() -> MultiLayerQueue:
    """Retourne l'instance globale de la file d'attente multicouche."""
    global _global_multi_layer_queue
    if _global_multi_layer_queue is None:
        _global_multi_layer_queue = MultiLayerQueue()
    return _global_multi_layer_queue
