"""Router WebSocket pour la télémétrie temps réel."""

import asyncio
import contextlib
import json
import logging
import time
from datetime import datetime
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from bbia_sim.daemon.simulation_service import simulation_service

logger = logging.getLogger(__name__)

router = APIRouter()


class ConnectionManager:
    """Gestionnaire des connexions WebSocket."""

    def __init__(self) -> None:
        self.active_connections: list[WebSocket] = []
        self.is_broadcasting = False
        self.broadcast_task: asyncio.Task[None] | None = None
        # OPTIMISATION RAM: Template JSON réutilisé (modification in-place au lieu de recréer)
        self._telemetry_template: dict[str, Any] | None = None
        self._max_connections = 10  # OPTIMISATION RAM: Limiter connexions simultanées

    async def connect(self, websocket: WebSocket) -> None:
        """Accepte une nouvelle connexion WebSocket."""
        # OPTIMISATION RAM: Limiter nombre de connexions simultanées
        if len(self.active_connections) >= self._max_connections:
            logger.warning(
                f"Limite de connexions atteinte ({self._max_connections}), rejet de nouvelle connexion",
            )
            await websocket.close(code=1008, reason="Too many connections")
            return

        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(
            f"Nouvelle connexion WebSocket. Total: {len(self.active_connections)}",
        )

    def disconnect(self, websocket: WebSocket) -> None:
        """Déconnecte un WebSocket."""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(
            f"Connexion WebSocket fermée. Total: {len(self.active_connections)}",
        )

    async def send_personal_message(self, message: str, websocket: WebSocket) -> None:
        """Envoie un message à une connexion spécifique."""
        try:
            await websocket.send_text(message)
        except Exception as e:
            logger.exception("Erreur d'envoi de message : %s", e)
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
                logger.exception("Erreur de diffusion ")
                disconnected.append(connection)

        # Nettoyage des connexions fermées
        for connection in disconnected:
            self.disconnect(connection)

    async def start_broadcast(self) -> None:
        """Démarre la diffusion automatique de télémétrie."""
        # OPTIMISATION RAM: Vérifier si task existe déjà et est active
        if self.is_broadcasting:
            return

        # OPTIMISATION RAM: Vérifier si task existe déjà et n'est pas terminée
        if self.broadcast_task is not None and not self.broadcast_task.done():
            logger.debug("Broadcast task déjà active")
            return

        self.is_broadcasting = True
        self.broadcast_task = asyncio.create_task(self._broadcast_loop())
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
                logger.exception("Erreur dans la boucle de diffusion ")
                await asyncio.sleep(1)

    def _generate_telemetry_data(self) -> dict[str, Any]:
        """Génère des données de télémétrie depuis la simulation (optimisé).

        OPTIMISATION RAM: Réutilise template JSON avec modification in-place au lieu de recréer.
        """
        # OPTIMISATION RAM: Initialiser template une seule fois si nécessaire
        if self._telemetry_template is None:
            self._telemetry_template = {
                "timestamp": "",
                "robot": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                    "velocity": {"linear": 0.0, "angular": 0.0},
                },
                "joints": {},
                "sensors": {
                    "battery": 85.0,
                    "temperature": 25.0,
                    "cpu_usage": 20.0,
                    "memory_usage": 50.0,
                },
                "status": {
                    "mode": "autonomous",
                    "simulation_ready": False,
                    "errors": [],
                    "warnings": [],
                    "active_tasks": [],
                },
            }

        # Récupération des données depuis la simulation
        robot_state = simulation_service.get_robot_state()
        joint_positions = robot_state.get("joint_positions", {})

        # Compatibilité: exposer alias 'neck_yaw' pour 'yaw_body' si absent
        if (
            isinstance(joint_positions, dict)
            and "neck_yaw" not in joint_positions
            and "yaw_body" in joint_positions
        ):
            # éviter de muter l'objet d'origine
            joint_positions = {
                **joint_positions,
                "neck_yaw": joint_positions["yaw_body"],
            }

        # Compat: ajouter joints bras/grippers génériques si absents
        if isinstance(joint_positions, dict):
            required_joints = [
                "right_shoulder_pitch",
                "right_elbow_pitch",
                "right_gripper_joint",
                "left_shoulder_pitch",
                "left_elbow_pitch",
                "left_gripper_joint",
            ]
            if not all(j in joint_positions for j in required_joints):
                jp_copy = dict(joint_positions)
                for jname in required_joints:
                    jp_copy.setdefault(jname, 0.0)
                joint_positions = jp_copy

        # OPTIMISATION RAM: Modifier template in-place au lieu de recréer
        import math

        current_time = time.time()
        sin_val = math.sin(current_time * 0.1) * 0.5 + 0.5  # Valeur entre 0 et 1
        cos_val = math.cos(current_time * 0.15) * 0.5 + 0.5  # Valeur entre 0 et 1

        # Modifier template in-place
        template = self._telemetry_template
        template["timestamp"] = datetime.now().isoformat()
        template["robot"]["position"]["x"] = round((sin_val - 0.5) * 0.2, 3)
        template["robot"]["position"]["y"] = round((cos_val - 0.5) * 0.2, 3)
        template["robot"]["position"]["z"] = round(0.1 + sin_val * 0.1, 3)
        template["robot"]["orientation"]["roll"] = round((sin_val - 0.5) * 0.2, 3)
        template["robot"]["orientation"]["pitch"] = round((cos_val - 0.5) * 0.2, 3)
        template["robot"]["orientation"]["yaw"] = round(sin_val * 0.4, 3)
        template["robot"]["velocity"]["linear"] = round(sin_val * 0.1, 3)
        template["robot"]["velocity"]["angular"] = round(cos_val * 0.2, 3)
        template["joints"] = joint_positions  # Données réelles de la simulation
        template["sensors"]["battery"] = round(85.0 + sin_val * 10.0, 1)
        template["sensors"]["temperature"] = round(25.0 + cos_val * 4.0, 1)
        template["sensors"]["cpu_usage"] = round(20.0 + sin_val * 20.0, 1)
        template["sensors"]["memory_usage"] = round(50.0 + cos_val * 20.0, 1)
        template["status"][
            "simulation_ready"
        ] = simulation_service.is_simulation_ready()

        # Retourner copie pour éviter mutations simultanées
        import copy

        return copy.deepcopy(template)


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
                            {"type": "pong", "timestamp": datetime.now().isoformat()},
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
                        json.dumps(status_data),
                        websocket,
                    )

            except WebSocketDisconnect:
                break
            except json.JSONDecodeError:
                logger.warning("Message JSON invalide reçu")
            except Exception as e:
                logger.exception("Erreur de traitement de message ")
                break

    except WebSocketDisconnect:
        logger.info("Client WebSocket déconnecté")
    finally:
        manager.disconnect(websocket)

        # Arrêt de la diffusion si plus de connexions
        if (
            not manager.active_connections
        ):  # OPTIMISATION: vérification de vérité plus efficace
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
