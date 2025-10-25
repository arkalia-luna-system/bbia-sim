"""Module WebSocket pour la télémétrie temps réel."""

import asyncio
import contextlib
import json
import logging
import secrets
from datetime import datetime
from typing import Any, Optional

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

logger = logging.getLogger(__name__)


def _secure_uniform(min_val: float, max_val: float) -> float:
    """Génère un nombre aléatoire sécurisé entre min_val et max_val."""
    range_val = max_val - min_val
    random_bytes = secrets.randbits(32)
    normalized = random_bytes / (2**32)
    return min_val + normalized * range_val


router = APIRouter()


class ConnectionManager:
    """Gestionnaire des connexions WebSocket."""

    def __init__(self) -> None:
        self.active_connections: list[WebSocket] = []
        self.is_broadcasting = False
        self.broadcast_task: Optional[asyncio.Task[None]] = None

    async def connect(self, websocket: WebSocket) -> None:
        """Accepte une nouvelle connexion WebSocket."""
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(
            f"Nouvelle connexion WebSocket. Total: {len(self.active_connections)}"
        )

    def disconnect(self, websocket: WebSocket) -> None:
        """Déconnecte un WebSocket."""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(
            f"Connexion WebSocket fermée. Total: {len(self.active_connections)}"
        )

    async def send_personal_message(self, message: str, websocket: WebSocket) -> None:
        """Envoie un message à une connexion spécifique."""
        try:
            await websocket.send_text(message)
        except Exception as e:
            logger.error(f"Erreur d'envoi de message : {e}")
            self.disconnect(websocket)

    async def broadcast(self, message: str) -> None:
        """Diffuse un message à toutes les connexions actives."""
        if not self.active_connections:
            return

        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.error(f"Erreur de diffusion : {e}")
                disconnected.append(connection)

        # Nettoyage des connexions fermées
        for connection in disconnected:
            self.disconnect(connection)

    async def start_broadcast(self) -> None:
        """Démarre la diffusion automatique de télémétrie."""
        if self.is_broadcasting:
            return

        self.is_broadcasting = True
        self.broadcast_task = asyncio.create_task(self._broadcast_loop())  # type: ignore  # type: ignore
        logger.info("Diffusion de télémétrie démarrée")

    async def stop_broadcast(self) -> None:
        """Arrête la diffusion automatique."""
        self.is_broadcasting = False
        if self.broadcast_task:
            self.broadcast_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self.broadcast_task
        logger.info("Diffusion de télémétrie arrêtée")

    async def _broadcast_loop(self) -> None:
        """Boucle de diffusion de télémétrie."""
        while self.is_broadcasting:
            try:
                # Génération des données de télémétrie simulées
                telemetry_data = self._generate_telemetry_data()

                # Diffusion à toutes les connexions
                await self.broadcast(json.dumps(telemetry_data))

                # Attente avant la prochaine diffusion (10Hz)
                await asyncio.sleep(0.1)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Erreur dans la boucle de diffusion : {e}")
                await asyncio.sleep(1)

    def _generate_telemetry_data(self) -> dict[str, Any]:
        """Génère des données de télémétrie simulées."""
        return {
            "timestamp": datetime.now().isoformat(),
            "robot": {
                "position": {
                    "x": round(_secure_uniform(-0.1, 0.1), 3),
                    "y": round(_secure_uniform(-0.1, 0.1), 3),
                    "z": round(_secure_uniform(0.0, 0.2), 3),
                },
                "orientation": {
                    "roll": round(_secure_uniform(-0.1, 0.1), 3),
                    "pitch": round(_secure_uniform(-0.1, 0.1), 3),
                    "yaw": round(_secure_uniform(-0.2, 0.2), 3),
                },
                "velocity": {
                    "linear": round(_secure_uniform(0, 0.05), 3),
                    "angular": round(_secure_uniform(0, 0.1), 3),
                },
            },
            "joints": {
                "right_shoulder_pitch": round(_secure_uniform(-0.1, 0.1), 3),
                "right_elbow_pitch": round(_secure_uniform(-0.1, 0.1), 3),
                "right_wrist_pitch": round(_secure_uniform(-0.1, 0.1), 3),
                "left_shoulder_pitch": round(_secure_uniform(-0.1, 0.1), 3),
                "left_elbow_pitch": round(_secure_uniform(-0.1, 0.1), 3),
                "left_wrist_pitch": round(_secure_uniform(-0.1, 0.1), 3),
                "head_yaw": round(_secure_uniform(-0.1, 0.1), 3),
                "head_pitch": round(_secure_uniform(-0.05, 0.05), 3),
            },
            "sensors": {
                "battery": round(_secure_uniform(80, 90), 1),
                "temperature": round(_secure_uniform(24, 27), 1),
                "cpu_usage": round(_secure_uniform(10, 30), 1),
                "memory_usage": round(_secure_uniform(40, 60), 1),
            },
            "status": {
                "mode": "autonomous",
                "errors": [],
                "warnings": [],
                "active_tasks": [],
            },
        }


# Instance globale du gestionnaire de connexions
manager = ConnectionManager()


@router.websocket("/telemetry")
async def websocket_endpoint(websocket: WebSocket) -> None:
    """Endpoint WebSocket pour la télémétrie temps réel."""
    await manager.connect(websocket)

    try:
        # Démarrage de la diffusion si c'est la première connexion
        if len(manager.active_connections) == 1:
            await manager.start_broadcast()

        # Boucle de réception des messages du client
        while True:
            try:
                # Réception d'un message du client
                data = await websocket.receive_text()
                message = json.loads(data)

                # Traitement des commandes du client
                if message.get("type") == "ping":
                    await manager.send_personal_message(
                        json.dumps(
                            {"type": "pong", "timestamp": datetime.now().isoformat()}
                        ),
                        websocket,
                    )
                elif message.get("type") == "request_status":
                    status_data = {
                        "type": "status",
                        "connections": len(manager.active_connections),
                        "broadcasting": manager.is_broadcasting,
                        "timestamp": datetime.now().isoformat(),
                    }
                    await manager.send_personal_message(
                        json.dumps(status_data), websocket
                    )

            except WebSocketDisconnect:
                break
            except json.JSONDecodeError:
                logger.warning("Message JSON invalide reçu")
            except Exception as e:
                logger.error(f"Erreur de traitement de message : {e}")
                break

    except WebSocketDisconnect:
        logger.info("Client WebSocket déconnecté")
    finally:
        manager.disconnect(websocket)

        # Arrêt de la diffusion si plus de connexions
        if len(manager.active_connections) == 0:
            await manager.stop_broadcast()


@router.get("/telemetry/info")
async def get_telemetry_info() -> dict[str, Any]:
    """Informations sur le service de télémétrie."""
    return {
        "service": "WebSocket Telemetry",
        "version": "1.0.0",
        "status": "running",
        "connections": len(manager.active_connections),
        "broadcasting": manager.is_broadcasting,
        "frequency": "10Hz",
        "endpoint": "/ws/telemetry",
        "timestamp": datetime.now().isoformat(),
    }


@router.post("/telemetry/start")
async def start_telemetry() -> dict[str, Any]:
    """Démarre la diffusion de télémétrie."""
    await manager.start_broadcast()
    return {
        "status": "started",
        "message": "Diffusion de télémétrie démarrée",
        "timestamp": datetime.now().isoformat(),
    }


@router.post("/telemetry/stop")
async def stop_telemetry() -> dict[str, Any]:
    """Arrête la diffusion de télémétrie."""
    await manager.stop_broadcast()
    return {
        "status": "stopped",
        "message": "Diffusion de télémétrie arrêtée",
        "timestamp": datetime.now().isoformat(),
    }
