#!/usr/bin/env python3
"""
Bridge Zenoh/FastAPI pour BBIA-SIM
Permet l'intégration entre l'architecture FastAPI de BBIA-SIM et le daemon Zenoh de Reachy Mini
"""

import asyncio
import json
import logging
import time
from typing import Any, Optional

import numpy as np
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

logger = logging.getLogger(__name__)

# Import conditionnel Zenoh
try:
    import zenoh
    from zenoh import Config, Session

    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False
    zenoh = None  # type: ignore
    Session = None  # type: ignore
    Config = None  # type: ignore

# Import conditionnel SDK officiel
try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose

    REACHY_MINI_AVAILABLE = True
except ImportError:
    REACHY_MINI_AVAILABLE = False
    ReachyMini = None
    create_head_pose = None


class ZenohConfig(BaseModel):
    """Configuration Zenoh."""

    mode: str = "client"
    connect: list[str] = ["tcp://localhost:7447"]
    timeout: int = 1000
    retry_attempts: int = 3


class RobotCommand(BaseModel):
    """Commande robot via Zenoh."""

    command: str
    parameters: dict[str, Any]
    timestamp: float = 0.0

    def __init__(self, **data):
        if data.get("timestamp") is None:
            data["timestamp"] = time.time()
        super().__init__(**data)


class RobotState(BaseModel):
    """État du robot via Zenoh."""

    joints: dict[str, float]
    emotions: dict[str, Any]
    sensors: dict[str, Any]
    timestamp: float = 0.0

    def __init__(self, **data):
        if data.get("timestamp") is None:
            data["timestamp"] = time.time()
        super().__init__(**data)


class ZenohBridge:
    """Bridge entre FastAPI et Zenoh pour Reachy Mini."""

    def __init__(self, config: Optional[ZenohConfig] = None):
        """Initialise le bridge Zenoh."""
        self.config = config or ZenohConfig()
        self.session: Optional[Session] = None
        self.reachy_mini: Optional[ReachyMini] = None
        self.connected = False
        self.logger = logging.getLogger(self.__class__.__name__)

        # Topics Zenoh
        self.topics = {
            "commands": "reachy/mini/commands",
            "state": "reachy/mini/state",
            "telemetry": "reachy/mini/telemetry",
            "errors": "reachy/mini/errors",
        }

        # Subscribers et Publishers
        self.subscribers: dict[str, Any] = {}
        self.publishers: dict[str, Any] = {}

        # État du robot
        self.current_state = RobotState(joints={}, emotions={}, sensors={})

        # Queue de commandes
        self.command_queue: asyncio.Queue = asyncio.Queue()

    async def start(self) -> bool:
        """Démarre le bridge Zenoh."""
        if not ZENOH_AVAILABLE:
            self.logger.error("Zenoh non disponible")
            return False

        try:
            # Configuration Zenoh
            zenoh_config = Config()
            zenoh_config.insert_json5("mode", f'"{self.config.mode}"')
            zenoh_config.insert_json5("connect", json.dumps(self.config.connect))

            # Créer la session Zenoh
            self.session = await zenoh.open(zenoh_config)  # type: ignore
            self.logger.info("Session Zenoh créée")

            # Initialiser Reachy Mini
            if REACHY_MINI_AVAILABLE:
                self.reachy_mini = ReachyMini()
                self.logger.info("Reachy Mini initialisé")
            else:
                self.logger.warning("SDK Reachy Mini non disponible - mode simulation")

            # Créer les subscribers et publishers
            await self._setup_zenoh_topics()

            # Démarrer les tâches asynchrones
            asyncio.create_task(self._command_processor())
            asyncio.create_task(self._state_publisher())

            self.connected = True
            self.logger.info("Bridge Zenoh démarré avec succès")
            return True

        except Exception as e:
            self.logger.error(f"Erreur démarrage bridge Zenoh: {e}")
            return False

    async def stop(self) -> None:
        """Arrête le bridge Zenoh."""
        self.connected = False

        # Fermer Reachy Mini
        if self.reachy_mini:
            try:
                self.reachy_mini.close()
            except Exception as e:
                self.logger.error(f"Erreur fermeture Reachy Mini: {e}")

        # Fermer les subscribers et publishers
        for sub in self.subscribers.values():
            try:
                await sub.close()
            except Exception as e:
                self.logger.error(f"Erreur fermeture subscriber: {e}")

        for pub in self.publishers.values():
            try:
                await pub.close()
            except Exception as e:
                self.logger.error(f"Erreur fermeture publisher: {e}")

        # Fermer la session Zenoh
        if self.session:
            try:
                await self.session.close()
            except Exception as e:
                self.logger.error(f"Erreur fermeture session Zenoh: {e}")

        self.logger.info("Bridge Zenoh arrêté")

    async def _setup_zenoh_topics(self) -> None:
        """Configure les topics Zenoh."""
        if not self.session:
            return

        try:
            # Subscriber pour les commandes
            self.subscribers["commands"] = await self.session.declare_subscriber(  # type: ignore
                self.topics["commands"], self._on_command_received
            )

            # Publishers pour l'état et la télémétrie
            self.publishers["state"] = await self.session.declare_publisher(  # type: ignore
                self.topics["state"]
            )

            self.publishers["telemetry"] = await self.session.declare_publisher(  # type: ignore
                self.topics["telemetry"]
            )

            self.publishers["errors"] = await self.session.declare_publisher(  # type: ignore
                self.topics["errors"]
            )

            self.logger.info("Topics Zenoh configurés")

        except Exception as e:
            self.logger.error(f"Erreur configuration topics Zenoh: {e}")

    async def _on_command_received(self, sample: Any) -> None:
        """Traite les commandes reçues via Zenoh."""
        try:
            command_data = json.loads(sample.payload.decode())
            command = RobotCommand(**command_data)

            # Ajouter à la queue de traitement
            await self.command_queue.put(command)

        except Exception as e:
            self.logger.error(f"Erreur traitement commande Zenoh: {e}")
            await self._publish_error(f"Erreur commande: {e}")

    async def _command_processor(self) -> None:
        """Traite les commandes de la queue."""
        while self.connected:
            try:
                # Attendre une commande
                command = await asyncio.wait_for(self.command_queue.get(), timeout=1.0)

                # Exécuter la commande
                await self._execute_command(command)

            except asyncio.TimeoutError:
                continue
            except Exception as e:
                self.logger.error(f"Erreur traitement commande: {e}")
                await self._publish_error(f"Erreur traitement: {e}")

    async def _execute_command(self, command: RobotCommand) -> None:
        """Exécute une commande robot."""
        try:
            cmd = command.command
            params = command.parameters

            if cmd == "goto_target":
                await self._cmd_goto_target(params)
            elif cmd == "set_target":
                await self._cmd_set_target(params)
            elif cmd == "set_emotion":
                await self._cmd_set_emotion(params)
            elif cmd == "play_audio":
                await self._cmd_play_audio(params)
            elif cmd == "look_at":
                await self._cmd_look_at(params)
            else:
                self.logger.warning(f"Commande inconnue: {cmd}")
                await self._publish_error(f"Commande inconnue: {cmd}")

        except Exception as e:
            self.logger.error(f"Erreur exécution commande {command.command}: {e}")
            await self._publish_error(f"Erreur exécution: {e}")

    async def _cmd_goto_target(self, params: dict[str, Any]) -> None:
        """Commande goto_target."""
        if not self.reachy_mini:
            return

        head_pose = params.get("head")
        antennas = params.get("antennas")
        duration = params.get("duration", 1.0)

        if head_pose:
            # Convertir en numpy array si nécessaire
            if isinstance(head_pose, list):
                head_pose = np.array(head_pose)

            self.reachy_mini.goto_target(
                head=head_pose, antennas=antennas, duration=duration
            )

    async def _cmd_set_target(self, params: dict[str, Any]) -> None:
        """Commande set_target."""
        if not self.reachy_mini:
            return

        head_pose = params.get("head")
        antennas = params.get("antennas")

        if head_pose:
            if isinstance(head_pose, list):
                head_pose = np.array(head_pose)

            self.reachy_mini.set_target(head=head_pose, antennas=antennas)

    async def _cmd_set_emotion(self, params: dict[str, Any]) -> None:
        """Commande set_emotion."""
        emotion = params.get("emotion", "neutral")
        intensity = params.get("intensity", 0.5)

        # Mettre à jour l'état des émotions
        self.current_state.emotions[emotion] = intensity

        # Exécuter l'émotion sur le robot
        if self.reachy_mini:
            # Implémentation spécifique selon le SDK
            pass

    async def _cmd_play_audio(self, params: dict[str, Any]) -> None:
        """Commande play_audio."""
        audio_data = params.get("audio_data")

        if self.reachy_mini and audio_data:
            # Implémentation spécifique selon le SDK
            pass

    async def _cmd_look_at(self, params: dict[str, Any]) -> None:
        """Commande look_at."""
        x = params.get("x", 0.0)
        y = params.get("y", 0.0)
        z = params.get("z", 0.0)

        if self.reachy_mini and create_head_pose:
            pose = create_head_pose(x=x, y=y, z=z)
            self.reachy_mini.goto_target(head=pose, duration=1.0)

    async def _state_publisher(self) -> None:
        """Publie l'état du robot périodiquement."""
        while self.connected:
            try:
                # Mettre à jour l'état
                await self._update_robot_state()

                # Publier l'état
                await self._publish_state()

                # Attendre avant la prochaine publication
                await asyncio.sleep(0.1)  # 10Hz

            except Exception as e:
                self.logger.error(f"Erreur publication état: {e}")
                await asyncio.sleep(1.0)

    async def _update_robot_state(self) -> None:
        """Met à jour l'état du robot."""
        if not self.reachy_mini:
            return

        try:
            # Récupérer l'état des joints
            joints = self.reachy_mini.get_joint_positions()
            self.current_state.joints = joints

            # Récupérer l'état des capteurs
            sensors = self.reachy_mini.get_sensor_data()
            self.current_state.sensors = sensors

            # Mettre à jour le timestamp
            self.current_state.timestamp = time.time()

        except Exception as e:
            self.logger.error(f"Erreur mise à jour état: {e}")

    async def _publish_state(self) -> None:
        """Publie l'état du robot."""
        if not self.publishers.get("state"):
            return

        try:
            state_data = self.current_state.dict()
            await self.publishers["state"].put(json.dumps(state_data))

        except Exception as e:
            self.logger.error(f"Erreur publication état: {e}")

    async def _publish_error(self, error_message: str) -> None:
        """Publie une erreur."""
        if not self.publishers.get("errors"):
            return

        try:
            error_data = {"error": error_message, "timestamp": time.time()}
            await self.publishers["errors"].put(json.dumps(error_data))

        except Exception as e:
            self.logger.error(f"Erreur publication erreur: {e}")

    async def send_command(self, command: RobotCommand) -> bool:
        """Envoie une commande via Zenoh."""
        try:
            if not self.publishers.get("commands"):
                return False

            command_data = command.dict()
            await self.publishers["commands"].put(json.dumps(command_data))
            return True

        except Exception as e:
            self.logger.error(f"Erreur envoi commande: {e}")
            return False

    def get_current_state(self) -> RobotState:
        """Retourne l'état actuel du robot."""
        return self.current_state

    def is_connected(self) -> bool:
        """Vérifie si le bridge est connecté."""
        return self.connected


# FastAPI App pour l'intégration
app = FastAPI(title="BBIA-SIM Zenoh Bridge", version="1.3.0")

# Instance globale du bridge
bridge: Optional[ZenohBridge] = None


@app.on_event("startup")
async def startup_event():
    """Démarre le bridge au démarrage de l'app."""
    global bridge
    bridge = ZenohBridge()
    await bridge.start()


@app.on_event("shutdown")
async def shutdown_event():
    """Arrête le bridge à l'arrêt de l'app."""
    global bridge
    if bridge:
        await bridge.stop()


@app.post("/api/zenoh/command")
async def send_robot_command(command: RobotCommand):
    """Envoie une commande au robot via Zenoh."""
    if not bridge or not bridge.is_connected():
        raise HTTPException(status_code=503, detail="Bridge Zenoh non connecté")

    success = await bridge.send_command(command)
    if not success:
        raise HTTPException(status_code=500, detail="Erreur envoi commande")

    return {"status": "success", "command": command.command}


@app.get("/api/zenoh/state")
async def get_robot_state():
    """Récupère l'état actuel du robot."""
    if not bridge or not bridge.is_connected():
        raise HTTPException(status_code=503, detail="Bridge Zenoh non connecté")

    state = bridge.get_current_state()
    return state.dict()


@app.get("/api/zenoh/status")
async def get_bridge_status():
    """Récupère le statut du bridge."""
    return {
        "connected": bridge.is_connected() if bridge else False,
        "zenoh_available": ZENOH_AVAILABLE,
        "reachy_mini_available": REACHY_MINI_AVAILABLE,
    }


@app.websocket("/ws/zenoh")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket pour communication temps réel avec Zenoh."""
    await websocket.accept()

    try:
        while True:
            # Recevoir des commandes via WebSocket
            data = await websocket.receive_text()
            command_data = json.loads(data)
            command = RobotCommand(**command_data)

            # Envoyer via Zenoh
            if bridge and bridge.is_connected():
                success = await bridge.send_command(command)
                await websocket.send_text(
                    json.dumps(
                        {
                            "status": "success" if success else "error",
                            "command": command.command,
                        }
                    )
                )

            # Envoyer l'état actuel
            if bridge:
                state = bridge.get_current_state()
                await websocket.send_text(
                    json.dumps({"type": "state", "data": state.dict()})
                )

    except WebSocketDisconnect:
        pass


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="127.0.0.1", port=8001)
