#!/usr/bin/env python3
"""Bridge Zenoh/FastAPI pour BBIA-SIM
Permet l'intégration entre l'architecture FastAPI de BBIA-SIM et le daemon Zenoh de Reachy Mini
"""

import asyncio
import json
import logging
import time
from typing import Any

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
    zenoh = None  # type: ignore[assignment,no-redef]
    Session = None  # type: ignore[assignment,no-redef]
    Config = None  # type: ignore[assignment,no-redef]

# Import conditionnel SDK officiel
try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose

    REACHY_MINI_AVAILABLE = True
except ImportError:
    REACHY_MINI_AVAILABLE = False
    ReachyMini = None  # type: ignore[assignment,no-redef]
    create_head_pose = None  # type: ignore[assignment,no-redef]


class ZenohConfig(BaseModel):  # type: ignore[misc]
    """Configuration Zenoh."""

    mode: str = "client"
    connect: list[str] = ["tcp://localhost:7447"]
    timeout: int = 1000
    retry_attempts: int = 3


class RobotCommand(BaseModel):  # type: ignore[misc]
    """Commande robot via Zenoh."""

    command: str
    parameters: dict[str, Any]
    timestamp: float = 0.0

    def __init__(self, **data: Any) -> None:
        if data.get("timestamp") is None:
            data["timestamp"] = time.time()
        super().__init__(**data)


class RobotState(BaseModel):  # type: ignore[misc]
    """État du robot via Zenoh."""

    joints: dict[str, float]
    emotions: dict[str, Any]
    sensors: dict[str, Any]
    timestamp: float = 0.0

    def __init__(self, **data: Any) -> None:
        if data.get("timestamp") is None:
            data["timestamp"] = time.time()
        super().__init__(**data)


class ZenohBridge:
    """Bridge entre FastAPI et Zenoh pour Reachy Mini."""

    def __init__(self, config: ZenohConfig | None = None):
        """Initialise le bridge Zenoh."""
        self.config = config or ZenohConfig()
        self.session: Session | None = None
        self.reachy_mini: ReachyMini | None = None
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
        self.current_state = RobotState(joints={}, emotions={}, sensors={})  # type: ignore[call-arg]  # Pydantic constructeur

        # Queue de commandes
        self.command_queue: asyncio.Queue[RobotCommand] = asyncio.Queue()

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
            self.session = await zenoh.open(zenoh_config)  # type: ignore[attr-defined]  # Zenoh API dynamique
            self.logger.info("Session Zenoh créée")

            # Initialiser Reachy Mini
            if REACHY_MINI_AVAILABLE:
                if ReachyMini is None:
                    raise RuntimeError("ReachyMini non disponible")
                self.reachy_mini = ReachyMini()  # type: ignore[call-arg]
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
                self.reachy_mini.close()  # type: ignore[attr-defined]
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
                await self.session.close()  # type: ignore[attr-defined]  # Zenoh API dynamique
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
                self.topics["commands"],
                self._on_command_received,
            )

            # Publishers pour l'état et la télémétrie
            self.publishers["state"] = await self.session.declare_publisher(  # type: ignore
                self.topics["state"],
            )

            self.publishers["telemetry"] = await self.session.declare_publisher(  # type: ignore
                self.topics["telemetry"],
            )

            self.publishers["errors"] = await self.session.declare_publisher(  # type: ignore
                self.topics["errors"],
            )

            self.logger.info("Topics Zenoh configurés")

        except Exception as e:
            self.logger.error(f"Erreur configuration topics Zenoh: {e}")

    async def _on_command_received(self, sample: Any) -> None:
        """Traite les commandes reçues via Zenoh."""
        try:
            # Validation JSON sécurité: max size pour éviter DoS
            payload = sample.payload.decode()
            if len(payload) > 1048576:  # 1MB max
                self.logger.warning(f"Payload trop volumineux: {len(payload)} bytes")
                await self._publish_error("Commande rejetée: payload trop volumineux")
                return

            command_data = json.loads(payload)
            # Validation: vérifier qu'il n'y a pas de secrets en clair
            if isinstance(command_data, dict):
                forbidden_keys = {
                    "password",
                    "secret",
                    "api_key",
                    "token",
                    "credential",
                }
                if any(k.lower() in forbidden_keys for k in command_data.keys()):
                    self.logger.warning(
                        "Tentative d'envoi de secret détectée dans commande",
                    )
                    await self._publish_error("Commande rejetée: secrets non autorisés")
                    return

            command = RobotCommand(**command_data)

            # Ajouter à la queue de traitement
            await self.command_queue.put(command)

        except json.JSONDecodeError as e:
            self.logger.error(f"Erreur décodage JSON: {e}")
            await self._publish_error(f"Erreur format JSON: {e}")
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

            except TimeoutError:
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
        """Commande goto_target conforme SDK Reachy Mini avec interpolation optimisée."""
        if not self.reachy_mini:
            return

        head_pose = params.get("head")
        antennas = params.get("antennas")
        duration = params.get("duration", 1.0)
        method = params.get("method", "minjerk")  # Minjerk recommandé SDK pour fluidité
        body_yaw = params.get("body_yaw")  # Mouvement corps synchronisé si spécifié

        if head_pose:
            # Convertir en numpy array si nécessaire
            if isinstance(head_pose, list):
                head_pose = np.array(head_pose)

            # OPTIMISATION EXPERTE: Mouvement combiné tête+corps si body_yaw spécifié
            # (plus expressif et fluide qu'appels séparés)
            if body_yaw is not None:
                self.reachy_mini.goto_target(
                    head=head_pose,
                    antennas=antennas,
                    body_yaw=body_yaw,
                    duration=duration,
                    method=method,
                )
            else:
                self.reachy_mini.goto_target(
                    head=head_pose,
                    antennas=antennas,
                    duration=duration,
                    method=method,
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
        """Commande set_emotion conforme SDK Reachy Mini."""
        emotion = params.get("emotion", "neutral")
        intensity = params.get("intensity", 0.5)

        # Mettre à jour l'état des émotions
        self.current_state.emotions[emotion] = intensity

        # Exécuter l'émotion sur le robot via SDK officiel
        if self.reachy_mini:
            try:
                # Valider les 6 émotions SDK officiel
                sdk_emotions = {"happy", "sad", "neutral", "excited", "curious", "calm"}

                if emotion in sdk_emotions:
                    # Utiliser set_emotion SDK si disponible
                    if hasattr(self.reachy_mini, "set_emotion"):
                        self.reachy_mini.set_emotion(emotion, intensity)
                    elif create_head_pose:
                        # Fallback: créer pose tête selon émotion
                        emotion_poses = {
                            "happy": create_head_pose(
                                pitch=0.1,
                                yaw=0.0,
                                degrees=False,
                            ),
                            "sad": create_head_pose(pitch=-0.1, yaw=0.0, degrees=False),
                            "excited": create_head_pose(
                                pitch=0.2,
                                yaw=0.1,
                                degrees=False,
                            ),
                            "curious": create_head_pose(
                                pitch=0.05,
                                yaw=0.2,
                                degrees=False,
                            ),
                            "calm": create_head_pose(
                                pitch=-0.05,
                                yaw=0.0,
                                degrees=False,
                            ),
                            "neutral": create_head_pose(
                                pitch=0.0,
                                yaw=0.0,
                                degrees=False,
                            ),
                        }
                        pose = emotion_poses.get(emotion, emotion_poses["neutral"])
                        # Appliquer intensité
                        pose[0, 0] *= intensity
                        if hasattr(self.reachy_mini, "set_target_head_pose"):
                            self.reachy_mini.set_target_head_pose(pose)
                else:
                    # Mapper émotion non-SDK vers émotion SDK la plus proche
                    emotion_map = {
                        "angry": "excited",
                        "surprised": "curious",
                        "fearful": "sad",
                        "confused": "curious",
                    }
                    sdk_emotion = emotion_map.get(emotion, "neutral")
                    if hasattr(self.reachy_mini, "set_emotion"):
                        self.reachy_mini.set_emotion(sdk_emotion, intensity)
                    self.logger.info(f"Émotion {emotion} mappée vers {sdk_emotion}")
            except Exception as e:
                self.logger.error(f"Erreur set_emotion: {e}")

    async def _cmd_play_audio(self, params: dict[str, Any]) -> None:
        """Commande play_audio conforme SDK Reachy Mini (media.play_audio)."""
        audio_data = params.get("audio_data")
        volume = params.get("volume", 0.7)

        if self.reachy_mini and audio_data:
            try:
                # Utiliser robot.media.play_audio si disponible
                if hasattr(self.reachy_mini, "media") and hasattr(
                    self.reachy_mini.media,
                    "play_audio",
                ):
                    # Convertir audio_data en bytes si nécessaire
                    if isinstance(audio_data, str):
                        # Chemin fichier
                        with open(audio_data, "rb") as f:
                            audio_bytes = f.read()
                    elif isinstance(audio_data, list):
                        # Liste → bytes
                        audio_bytes = bytes(audio_data)
                    else:
                        audio_bytes = audio_data

                    self.reachy_mini.media.play_audio(audio_bytes, volume=volume)
                    self.logger.info("Audio joué via robot.media.play_audio")
                else:
                    self.logger.warning("robot.media.play_audio non disponible")
            except Exception as e:
                self.logger.error(f"Erreur play_audio: {e}")

    async def _cmd_look_at(self, params: dict[str, Any]) -> None:
        """Commande look_at conforme SDK Reachy Mini (look_at_world/look_at_image)."""
        x = params.get("x", 0.0)
        y = params.get("y", 0.0)
        z = params.get("z", 0.0)
        duration = params.get("duration", 1.0)

        if not self.reachy_mini:
            return

        try:
            # CORRECTION EXPERTE: create_head_pose prend pitch/yaw/roll, pas x/y/z
            # Utiliser look_at_world() pour coordonnées 3D (méthode SDK recommandée)
            if hasattr(self.reachy_mini, "look_at_world"):
                # Validation coordonnées recommandées SDK (-2.0 ≤ x,y ≤ 2.0, 0.0 ≤ z ≤ 1.5)
                if abs(x) > 2.0 or abs(y) > 2.0 or z < 0.0 or z > 1.5:
                    self.logger.warning(
                        f"Coordonnées ({x}, {y}, {z}) hors limites SDK - clampage appliqué",
                    )
                    x = max(-2.0, min(2.0, x))
                    y = max(-2.0, min(2.0, y))
                    z = max(0.0, min(1.5, z))

                # Utiliser look_at_world SDK officiel
                self.reachy_mini.look_at_world(
                    x,
                    y,
                    z,
                    duration=duration,
                    perform_movement=True,
                )
                self.logger.info(f"Look_at_world SDK: ({x}, {y}, {z})")
            elif hasattr(self.reachy_mini, "look_at_image"):
                # Fallback: look_at_image si coordonnées image (u, v)
                self.reachy_mini.look_at_image(int(x), int(y))
                self.logger.info(f"Look_at_image SDK: ({int(x)}, {int(y)})")
            elif create_head_pose:
                # Fallback final: calculer pitch/yaw depuis x/y/z (approximation)
                # Note: Cette approximation est moins précise que look_at_world()
                pitch = z * 0.2  # Approximation verticale
                yaw = x * 0.3  # Approximation horizontale
                pose = create_head_pose(pitch=pitch, yaw=yaw, degrees=False)
                if hasattr(self.reachy_mini, "goto_target"):
                    self.reachy_mini.goto_target(
                        head=pose,
                        duration=duration,
                        method="minjerk",
                    )
                elif hasattr(self.reachy_mini, "set_target_head_pose"):
                    self.reachy_mini.set_target_head_pose(pose)
                self.logger.info(
                    f"Look_at fallback (pose calculée): pitch={pitch:.3f}, yaw={yaw:.3f}",
                )
        except Exception as e:
            self.logger.error(f"Erreur look_at: {e}")

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
            # Récupérer l'état des joints (API SDK officielle)
            joints_state: dict[str, float] = {}
            if hasattr(self.reachy_mini, "get_current_joint_positions"):
                try:
                    head_positions, antenna_positions = (
                        self.reachy_mini.get_current_joint_positions()
                    )
                    # Stewart joints (indices 0-5)
                    if isinstance(head_positions, list | tuple):
                        for i, val in enumerate(head_positions[:6]):
                            joints_state[f"stewart_{i+1}"] = float(val)
                    # Antennes (indices 0-1)
                    if isinstance(antenna_positions, list | tuple):
                        if len(antenna_positions) > 0:
                            joints_state["left_antenna"] = float(antenna_positions[0])
                        if len(antenna_positions) > 1:
                            joints_state["right_antenna"] = float(antenna_positions[1])
                except Exception as err:
                    self.logger.warning(
                        f"Lecture joints via get_current_joint_positions a échoué: {err}",
                    )

            # Optionnel: body_yaw si exposé par SDK
            try:
                if hasattr(self.reachy_mini, "get_current_body_yaw"):
                    yaw = self.reachy_mini.get_current_body_yaw()
                    if yaw is not None:
                        joints_state["yaw_body"] = float(yaw)
            except Exception:
                pass

            self.current_state.joints = joints_state

            # Récupérer l'état des capteurs (garder défensif: API non standardisée)
            sensors_state: dict[str, Any] = {}
            try:
                if hasattr(self.reachy_mini, "get_sensor_data"):
                    sensors_state = dict(self.reachy_mini.get_sensor_data())  # type: ignore[arg-type,no-any-return]  # SDK dynamique
                elif hasattr(self.reachy_mini, "io") and hasattr(
                    self.reachy_mini.io,
                    "get_imu",
                ):
                    imu_data = self.reachy_mini.io.get_imu()  # type: ignore[attr-defined]  # SDK dynamique
                    if imu_data is not None:
                        sensors_state["imu"] = imu_data
            except Exception as err:
                self.logger.warning(f"Lecture capteurs indisponible: {err}")

            self.current_state.sensors = sensors_state

            # Mettre à jour le timestamp
            self.current_state.timestamp = time.time()

        except Exception as e:
            self.logger.error(f"Erreur mise à jour état: {e}")

    async def _publish_state(self) -> None:
        """Publie l'état du robot."""
        if not self.publishers.get("state"):
            return

        try:
            state_data: dict[str, Any] = self.current_state.model_dump()  # type: ignore[no-any-return]
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

            command_data: dict[str, Any] = command.model_dump()  # type: ignore[no-any-return]
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
bridge: ZenohBridge | None = None


@app.on_event("startup")  # type: ignore[misc]
async def startup_event() -> None:
    """Démarre le bridge au démarrage de l'app."""
    global bridge
    bridge = ZenohBridge()
    await bridge.start()


@app.on_event("shutdown")  # type: ignore[misc]
async def shutdown_event() -> None:
    """Arrête le bridge à l'arrêt de l'app."""
    global bridge
    if bridge:
        await bridge.stop()


@app.post("/api/zenoh/command")  # type: ignore[misc]
async def send_robot_command(command: RobotCommand) -> dict[str, str]:
    """Envoie une commande au robot via Zenoh."""
    if not bridge or not bridge.is_connected():
        raise HTTPException(status_code=503, detail="Bridge Zenoh non connecté")

    success = await bridge.send_command(command)
    if not success:
        raise HTTPException(status_code=500, detail="Erreur envoi commande")

    return {"status": "success", "command": command.command}


@app.get("/api/zenoh/state")  # type: ignore[misc]
async def get_robot_state() -> dict[str, Any]:
    """Récupère l'état actuel du robot."""
    if not bridge or not bridge.is_connected():
        raise HTTPException(status_code=503, detail="Bridge Zenoh non connecté")

    state = bridge.get_current_state()
    state_dict: dict[str, Any] = state.model_dump()  # type: ignore[no-any-return]
    return state_dict


@app.get("/api/zenoh/status")  # type: ignore[misc]
async def get_bridge_status() -> dict[str, Any | bool]:
    """Récupère le statut du bridge."""
    return {
        "connected": bridge.is_connected() if bridge else False,
        "zenoh_available": ZENOH_AVAILABLE,
        "reachy_mini_available": REACHY_MINI_AVAILABLE,
    }


@app.websocket("/ws/zenoh")  # type: ignore[misc]
async def websocket_endpoint(websocket: WebSocket) -> None:
    """WebSocket pour communication temps réel avec Zenoh."""
    await websocket.accept()

    try:
        while True:
            # Recevoir des commandes via WebSocket
            data = await websocket.receive_text()
            command_data = json.loads(data)
            command = RobotCommand(**command_data)  # type: ignore[call-arg]  # Pydantic constructeur

            # Envoyer via Zenoh
            if bridge and bridge.is_connected():
                success = await bridge.send_command(command)
                await websocket.send_text(
                    json.dumps(
                        {
                            "status": "success" if success else "error",
                            "command": command.command,
                        },
                    ),
                )

            # Envoyer l'état actuel
            if bridge:
                state = bridge.get_current_state()
                await websocket.send_text(
                    json.dumps({"type": "state", "data": state.model_dump()}),  # type: ignore[no-any-return]
                )

    except WebSocketDisconnect:
        pass


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="127.0.0.1", port=8001)
