#!/usr/bin/env python3
"""BBIA Advanced Dashboard - Interface de contrôle sophistiquée
Dashboard web avancé avec métriques temps réel, visualisation 3D, et contrôle complet
"""

import asyncio
import json
import logging
import time
from datetime import datetime
from typing import Any

try:
    import uvicorn
    from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
    from fastapi.middleware.cors import CORSMiddleware
    from fastapi.responses import HTMLResponse

    FASTAPI_AVAILABLE = True
except ImportError:
    FASTAPI_AVAILABLE = False
    # Types fallback pour mypy
    FastAPI = Any  # type: ignore[assignment,misc]
    WebSocket = Any  # type: ignore[assignment,misc]
    Request = Any  # type: ignore[assignment,misc]

# Ajouter le chemin src pour les imports
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# BBIAVoice n'existe pas encore - utiliser les fonctions directement
from bbia_sim.bbia_behavior import BBIABehaviorManager
from bbia_sim.bbia_emotions import BBIAEmotions
from bbia_sim.bbia_vision import BBIAVision
from bbia_sim.robot_factory import RobotFactory
from bbia_sim.troubleshooting import (
    check_all,
    get_documentation_links,
    test_audio,
    test_camera,
    test_network_ping,
)

logger = logging.getLogger(__name__)


class BBIAAdvancedWebSocketManager:
    """Gestionnaire WebSocket avancé pour le dashboard BBIA."""

    def __init__(self) -> None:
        """Initialise le gestionnaire WebSocket avancé."""
        self.active_connections: list[WebSocket] = []
        self.robot: Any | None = None
        self.robot_backend = "mujoco"
        # Lock pour éviter les race conditions lors de l'initialisation du robot
        import threading

        self._robot_init_lock = threading.Lock()
        # OPTIMISATION RAM: Utiliser deque au lieu de liste pour limiter historique
        from collections import deque

        self.max_history = 1000  # Limite historique métriques
        self.metrics_history: deque[dict[str, Any]] = deque(maxlen=self.max_history)

        # OPTIMISATION RAM: Nettoyage connexions WebSocket inactives (>5 min)
        self._connection_last_activity: dict[WebSocket, float] = {}
        self._connection_cleanup_interval = 300.0  # 5 minutes

        # Modules BBIA
        self.emotions = BBIAEmotions()
        # OPTIMISATION RAM: Utiliser singleton BBIAVision si disponible
        try:
            from ..bbia_vision import get_bbia_vision_singleton

            self.vision = get_bbia_vision_singleton()
        except (ImportError, AttributeError):
            # Fallback si singleton non disponible
            self.vision = BBIAVision()
        # self.voice = BBIAVoice()  # Pas encore implémenté
        self.behavior_manager = BBIABehaviorManager()
        self.bbia_hf: Any | None = None  # Hugging Face chat module

        # Métriques temps réel
        self.current_metrics: dict[str, Any] = {
            "timestamp": time.time(),
            "robot_connected": False,
            "current_emotion": "neutral",
            "emotion_intensity": 0.0,
            "joint_positions": {},
            "performance": {
                "latency_ms": 0.0,
                "fps": 0.0,
                "cpu_usage": 0.0,
                "memory_usage": 0.0,
            },
            "vision": {
                "objects_detected": 0,
                "faces_detected": 0,
                "tracking_active": False,
            },
            "audio": {
                "microphone_active": False,
                "speaker_active": False,
                "volume_level": 0.0,
            },
        }

        # Démarrer la collecte de métriques
        # self._start_metrics_collection()  # Démarré lors de la première connexion WebSocket

        # Initialiser le robot automatiquement au démarrage
        self._initialize_robot_async()

    def _initialize_robot_async(self):
        """Initialise le robot de manière asynchrone."""

        def init_robot():
            with self._robot_init_lock:
                try:
                    if not self.robot:
                        logger.info(f"🔧 Initialisation robot {self.robot_backend}...")
                        self.robot = RobotFactory.create_backend(self.robot_backend)
                        if self.robot:
                            connected = self.robot.connect()
                            if connected:
                                if self.robot_backend == "mujoco":
                                    logger.info(
                                        f"✅ Robot {self.robot_backend} initialisé (mode simulation)"
                                    )
                                else:
                                    logger.info(
                                        f"✅ Robot {self.robot_backend} connecté automatiquement"
                                    )
                            else:
                                logger.warning(
                                    f"⚠️ Robot {self.robot_backend} en mode simulation"
                                )
                        else:
                            logger.error(
                                f"❌ RobotFactory.create_backend('{self.robot_backend}') a retourné None"
                            )
                except Exception as e:
                    logger.error(f"❌ Erreur initialisation robot: {e}", exc_info=True)
                    # En cas d'erreur, le dashboard fonctionne quand même en mode simulation
                    logger.info(
                        "ℹ️ Dashboard fonctionne en mode simulation (sans robot réel)"
                    )

        # Démarrer dans un thread pour ne pas bloquer
        import threading

        thread = threading.Thread(target=init_robot, daemon=True)
        thread.start()

    async def connect(self, websocket: WebSocket):
        """Accepte une nouvelle connexion WebSocket."""
        await websocket.accept()
        self.active_connections.append(websocket)
        # OPTIMISATION RAM: Enregistrer timestamp activité connexion
        self._connection_last_activity[websocket] = time.time()
        logger.info(
            f"🔌 WebSocket avancé connecté ({len(self.active_connections)} connexions)",
        )

        # Démarrer la collecte de métriques si c'est la première connexion
        if len(self.active_connections) == 1:
            self._start_metrics_collection()

        # S'assurer que le robot est initialisé - FORCER l'initialisation
        if not self.robot:
            logger.warning(
                "⚠️ Robot non initialisé lors de la connexion WebSocket - initialisation forcée"
            )
            try:
                self.robot = RobotFactory.create_backend(self.robot_backend)
                if self.robot:
                    connected = self.robot.connect()
                    if connected:
                        logger.info(f"✅ Robot {self.robot_backend} connecté (forcé)")
                        await self.send_log_message(
                            "info", f"✅ Robot {self.robot_backend} connecté"
                        )
                    else:
                        logger.warning(
                            f"⚠️ Robot {self.robot_backend} connect() a retourné False"
                        )
                        await self.send_log_message(
                            "warning",
                            f"⚠️ Robot {self.robot_backend} en mode simulation",
                        )
                else:
                    logger.error(
                        f"❌ RobotFactory.create_backend('{self.robot_backend}') a retourné None"
                    )
                    await self.send_log_message(
                        "error", f"❌ Impossible de créer le robot {self.robot_backend}"
                    )
            except Exception as e:
                logger.error(
                    f"❌ Erreur initialisation robot forcée: {e}", exc_info=True
                )
                await self.send_log_message("error", f"❌ Erreur robot: {e}")

        # Vérifier que le robot est vraiment connecté
        if self.robot:
            logger.info(
                f"✅ Robot présent: {type(self.robot).__name__}, is_connected={self.robot.is_connected}"
            )
            if not self.robot.is_connected:
                logger.warning(
                    "⚠️ Robot présent mais is_connected=False - reconnexion..."
                )
                self.robot.connect()
        else:
            logger.error("❌ Robot est None après vérification")

        # Envoyer état initial complet
        await self.send_complete_status()

    def disconnect(self, websocket: WebSocket):
        """Déconnecte un WebSocket."""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        # OPTIMISATION RAM: Supprimer du tracking activité
        if websocket in self._connection_last_activity:
            del self._connection_last_activity[websocket]
        logger.info(
            f"🔌 WebSocket avancé déconnecté ({len(self.active_connections)} connexions)",
        )

    def _cleanup_inactive_connections(self) -> None:
        """OPTIMISATION RAM: Nettoie les connexions WebSocket inactives (>5 min)."""
        current_time = time.time()
        inactive_connections: list[tuple[WebSocket, float]] = []

        for connection, last_activity in list(self._connection_last_activity.items()):
            inactivity = current_time - last_activity
            if inactivity > self._connection_cleanup_interval:
                inactive_connections.append((connection, inactivity))

        # Fermer connexions inactives
        for connection, inactivity in inactive_connections:
            try:
                # Tenter fermeture propre
                if connection in self.active_connections:
                    self.disconnect(connection)
                    logger.debug(
                        f"🗑️ Connexion WebSocket inactive fermée ({inactivity:.0f}s)"
                    )
            except Exception as e:
                logger.debug(f"Erreur nettoyage connexion inactive: {e}")

    async def broadcast(self, message: str):
        """Diffuse un message à toutes les connexions actives."""
        if not self.active_connections:
            return

        disconnected = []
        current_time = time.time()
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
                # OPTIMISATION RAM: Mettre à jour timestamp activité
                self._connection_last_activity[connection] = current_time
            except (ConnectionError, RuntimeError, WebSocketDisconnect):
                disconnected.append(
                    connection,
                )

        # Nettoyer les connexions fermées
        for connection in disconnected:
            self.disconnect(connection)

        # OPTIMISATION RAM: Nettoyer connexions inactives périodiquement
        self._cleanup_inactive_connections()

    async def send_complete_status(self):
        """Envoie le statut complet du système."""
        status_data = {
            "type": "complete_status",
            "timestamp": datetime.now().isoformat(),
            "robot": {
                "connected": self.robot is not None,
                "backend": self.robot_backend,
                "joints": self._get_available_joints(),
                "current_pose": self._get_current_pose(),
            },
            "bbia_modules": {
                "emotions": {
                    "current": self.emotions.current_emotion,
                    "intensity": self.emotions.emotion_intensity,
                    "available": list(self.emotions.emotions.keys()),
                },
                "vision": {
                    "active": self.vision.camera_active,
                    "quality": self.vision.vision_quality,
                    "objects": len(self.vision.objects_detected),
                    "faces": len(self.vision.faces_detected),
                },
                "voice": {
                    "tts_active": False,  # Pas encore implémenté
                    "stt_active": False,  # Pas encore implémenté
                    "volume": 0.0,
                },
                "behaviors": {
                    "available": list(self.behavior_manager.behaviors.keys()),
                    "current": getattr(self.behavior_manager, "current_behavior", None),
                },
            },
            "metrics": self.current_metrics,
            "history": list(self.metrics_history)[-50:],  # 50 dernières métriques
        }

        await self.broadcast(json.dumps(status_data))

    async def send_metrics_update(self):
        """Envoie une mise à jour des métriques."""
        metrics_data = {
            "type": "metrics_update",
            "timestamp": datetime.now().isoformat(),
            "metrics": self.current_metrics,
        }

        await self.broadcast(json.dumps(metrics_data))

    async def send_log_message(self, level: str, message: str):
        """Envoie un message de log."""
        log_data = {
            "type": "log",
            "level": level,
            "message": message,
            "timestamp": datetime.now().isoformat(),
        }

        await self.broadcast(json.dumps(log_data))

    def _get_available_joints(self) -> list[str]:
        """Récupère la liste des joints disponibles."""
        if self.robot:
            joints = self.robot.get_available_joints()
            if isinstance(joints, list):
                return [str(joint) for joint in joints if isinstance(joint, str | int)]
            return []
        return []

    def _get_current_pose(self) -> dict[str, float]:
        """Récupère la pose actuelle du robot."""
        if not self.robot:
            return {}

        pose = {}
        for joint in self._get_available_joints():
            try:
                pose[joint] = self.robot.get_joint_pos(joint)
            except (ConnectionError, RuntimeError, WebSocketDisconnect, Exception):
                # Gérer toutes les exceptions pour éviter les crashes
                pose[joint] = 0.0
        return pose

    def _start_metrics_collection(self):
        """Démarre la collecte automatique de métriques."""

        async def collect_metrics():
            while True:
                try:
                    # FAIRE AVANCER LA SIMULATION MuJoCo si robot connecté
                    if self.robot and hasattr(self.robot, "step"):
                        try:
                            # Faire un step de simulation pour que le robot bouge
                            self.robot.step()
                        except Exception as e:
                            logger.debug(f"Erreur step robot: {e}")

                    # Mettre à jour les métriques
                    self._update_metrics()

                    # OPTIMISATION RAM: Ajouter à l'historique (deque gère automatiquement maxlen)
                    self.metrics_history.append(self.current_metrics.copy())

                    # Envoyer mise à jour
                    await self.send_metrics_update()

                    # Attendre 100ms avant prochaine collecte
                    await asyncio.sleep(0.1)

                except Exception as e:
                    logger.error(f"Erreur collecte métriques: {e}")
                    await asyncio.sleep(1.0)

        # Démarrer la tâche en arrière-plan
        asyncio.create_task(collect_metrics())

    def _update_metrics(self):
        """Met à jour les métriques actuelles."""
        current_time = time.time()

        # Métriques robot
        self.current_metrics["timestamp"] = current_time
        self.current_metrics["robot_connected"] = self.robot is not None

        if self.robot:
            self.current_metrics["joint_positions"] = self._get_current_pose()

            # Métriques de performance (simulation)
            telemetry = self.robot.get_telemetry()
            self.current_metrics["performance"]["latency_ms"] = telemetry.get(
                "latency_ms",
                0.0,
            )
            self.current_metrics["performance"]["fps"] = telemetry.get("fps", 0.0)

        # Métriques BBIA
        self.current_metrics["current_emotion"] = self.emotions.current_emotion
        self.current_metrics["emotion_intensity"] = self.emotions.emotion_intensity

        self.current_metrics["vision"]["objects_detected"] = len(
            self.vision.objects_detected,
        )
        self.current_metrics["vision"]["faces_detected"] = len(
            self.vision.faces_detected,
        )
        self.current_metrics["vision"]["tracking_active"] = self.vision.tracking_active

        self.current_metrics["audio"][
            "microphone_active"
        ] = False  # Pas encore implémenté
        self.current_metrics["audio"]["speaker_active"] = False  # Pas encore implémenté
        self.current_metrics["audio"]["volume_level"] = 0.0


# Instance globale du gestionnaire avancé
advanced_websocket_manager = BBIAAdvancedWebSocketManager()

# Application FastAPI avancée
app: FastAPI | None
if FASTAPI_AVAILABLE:
    app = FastAPI(
        title="BBIA Advanced Dashboard",
        version="1.2.0",
        description="Interface de contrôle sophistiquée pour BBIA-SIM",
    )

    # Configuration CORS pour développement
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
else:
    app = None


def create_advanced_dashboard_app() -> FastAPI | None:
    """Crée l'application dashboard avancée FastAPI."""
    if not FASTAPI_AVAILABLE:
        logger.error("❌ FastAPI non disponible")
        return None

    return app


# HTML du dashboard avancé
ADVANCED_DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>BBIA Advanced Dashboard</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        * {
            scrollbar-width: thin;
            scrollbar-color: rgba(255, 255, 255, 0.3) transparent;
        }

        *::-webkit-scrollbar {
            width: 8px;
            height: 8px;
        }

        *::-webkit-scrollbar-track {
            background: transparent;
        }

        *::-webkit-scrollbar-thumb {
            background: rgba(255, 255, 255, 0.3);
            border-radius: 4px;
        }

        *::-webkit-scrollbar-thumb:hover {
            background: rgba(255, 255, 255, 0.5);
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', 'Oxygen', 'Ubuntu', 'Cantarell', 'Fira Sans', 'Droid Sans', 'Helvetica Neue', sans-serif;
            background: linear-gradient(135deg, #f5f7fa 0%, #e8ecf1 100%);
            color: #4D4D4D;
            min-height: 100vh;
            overflow-x: hidden;
            line-height: 1.6;
        }

        /* Animation d'entrée */
        @keyframes fadeInUp {
            from {
                opacity: 0;
                transform: translateY(20px);
            }
            to {
                opacity: 1;
                transform: translateY(0);
            }
        }

        .panel {
            animation: fadeInUp 0.5s ease-out;
        }

        .panel:nth-child(1) { animation-delay: 0.1s; }
        .panel:nth-child(2) { animation-delay: 0.2s; }
        .panel:nth-child(3) { animation-delay: 0.3s; }
        .panel:nth-child(4) { animation-delay: 0.4s; }

        .header {
            background: #ffffff;
            padding: 25px;
            text-align: center;
            border-bottom: 2px solid #4D4D4D;
            position: sticky;
            top: 0;
            z-index: 100;
            box-shadow: 0 2px 8px rgba(77, 77, 77, 0.1);
        }

        .header h1 {
            color: #4D4D4D;
            margin-bottom: 10px;
            font-weight: 700;
            font-size: 2.2em;
        }

        .header .subtitle {
            color: #4D4D4D;
            font-size: 0.95em;
            font-weight: 500;
            text-shadow: 0 1px 5px rgba(0, 0, 0, 0.1);
        }


        .main-container {
            display: grid;
            grid-template-columns: 1fr 1fr;
            grid-template-rows: auto auto auto;
            gap: 20px;
            padding: 20px;
            max-width: 1600px;
            margin: 0 auto;
        }

        .panel {
            background: #ffffff;
            border-radius: 8px;
            padding: 25px;
            border: 1px solid #E6E6E6;
            transition: all 0.3s ease;
            box-shadow: 0 2px 8px rgba(77, 77, 77, 0.08);
        }

        .panel:hover {
            box-shadow: 0 4px 12px rgba(77, 77, 77, 0.12);
            transform: translateY(-2px);
            border-color: #4D4D4D;
        }

        .panel h3 {
            margin-bottom: 20px;
            font-size: 1.3em;
            text-align: left;
            color: #4D4D4D;
            font-weight: 600;
            border-bottom: 2px solid #4D4D4D;
            padding-bottom: 10px;
            margin-top: 0;
        }

        .status-panel {
            grid-column: 1 / -1;
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
        }

        .status-item {
            background: #f5f5f5;
            padding: 20px;
            border-radius: 8px;
            text-align: center;
            transition: all 0.3s ease;
            border: 1px solid #E6E6E6;
        }

        .status-item:hover {
            background: #ffffff;
            border-color: #4D4D4D;
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(77, 77, 77, 0.15);
        }

        .status-value {
            font-size: 2em;
            font-weight: bold;
            margin-bottom: 5px;
            transition: transform 0.2s ease;
            color: #4D4D4D;
        }

        .status-value.updating {
            animation: pulse-value 0.5s ease;
        }

        @keyframes pulse-value {
            0%, 100% { transform: scale(1); }
            50% { transform: scale(1.1); }
        }

        .status-label {
            font-size: 0.9em;
            color: #4D4D4D;
            font-weight: 500;
            text-transform: uppercase;
            letter-spacing: 0.5px;
            font-size: 0.85em;
        }

        .connection-indicator {
            display: inline-block;
            width: 14px;
            height: 14px;
            border-radius: 50%;
            margin-right: 10px;
            animation: pulse 2s infinite;
            box-shadow: 0 0 8px currentColor;
        }

        .connected {
            background: #10b981;
            color: #10b981;
        }
        .disconnected {
            background: #ef4444;
            color: #ef4444;
        }

        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
        }

        .controls-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
            gap: 15px;
            margin-bottom: 20px;
        }

        .control-button {
            padding: 12px 20px;
            border: 1px solid #4D4D4D;
            border-radius: 6px;
            background: #ffffff;
            color: #4D4D4D;
            font-size: 14px;
            font-weight: 500;
            cursor: pointer;
            transition: all 0.2s ease;
            text-align: center;
            position: relative;
            overflow: hidden;
            box-shadow: 0 1px 3px rgba(77, 77, 77, 0.1);
        }

        .control-button::before {
            content: '';
            position: absolute;
            top: 50%;
            left: 50%;
            width: 0;
            height: 0;
            border-radius: 50%;
            background: rgba(255, 255, 255, 0.3);
            transform: translate(-50%, -50%);
            transition: width 0.6s, height 0.6s;
        }

        .control-button:active::before {
            width: 300px;
            height: 300px;
        }

        .control-button:hover {
            background: #4D4D4D;
            color: #ffffff;
            border-color: #4D4D4D;
            transform: translateY(-2px);
            box-shadow: 0 4px 8px rgba(77, 77, 77, 0.25);
        }

        .control-button:active {
            transform: translateY(-1px) scale(0.98);
        }

        .control-button:disabled {
            opacity: 0.5;
            cursor: not-allowed;
            transform: none;
        }

        .control-button.loading {
            pointer-events: none;
            position: relative;
        }

        .control-button.loading::after {
            content: '';
            position: absolute;
            top: 50%;
            left: 50%;
            width: 16px;
            height: 16px;
            margin: -8px 0 0 -8px;
            border: 2px solid rgba(255, 255, 255, 0.3);
            border-top-color: white;
            border-radius: 50%;
            animation: spin 0.6s linear infinite;
        }

        @keyframes spin {
            to { transform: rotate(360deg); }
        }

        /* Loading spinner standalone */
        .loading-spinner {
            display: inline-block;
            width: 20px;
            height: 20px;
            border: 3px solid rgba(77, 77, 77, 0.2);
            border-top-color: #4D4D4D;
            border-radius: 50%;
            animation: spin 0.8s linear infinite;
            vertical-align: middle;
            margin-right: 8px;
        }

        .emotion-button {
            border-color: rgba(77, 77, 77, 0.3);
            color: #4D4D4D;
        }

        .emotion-button:hover {
            background: linear-gradient(135deg, #4D4D4D 0%, #4D4D4D 100%);
            color: #ffffff;
        }

        .action-button {
            border-color: rgba(77, 77, 77, 0.3);
            color: #4D4D4D;
        }

        .action-button:hover {
            background: linear-gradient(135deg, #4D4D4D 0%, #4D4D4D 100%);
            color: #ffffff;
        }

        .behavior-button {
            border-color: rgba(77, 77, 77, 0.3);
            color: #4D4D4D;
        }

        .behavior-button:hover {
            background: linear-gradient(135deg, #4D4D4D 0%, #4D4D4D 100%);
            color: #ffffff;
        }

        .chart-container {
            position: relative;
            height: 300px;
            margin-top: 20px;
        }

        .joint-controls {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-top: 20px;
        }

        .joint-control {
            background: linear-gradient(135deg, #ffffff 0%, #f8f9fa 100%);
            padding: 18px;
            border-radius: 12px;
            border: 2px solid rgba(77, 77, 77, 0.15);
            transition: all 0.3s ease;
        }

        .joint-control:hover {
            border-color: rgba(77, 77, 77, 0.4);
            box-shadow: 0 4px 12px rgba(77, 77, 77, 0.15);
            transform: translateY(-2px);
        }

        .joint-name {
            font-weight: 600;
            margin-bottom: 10px;
            text-align: center;
            color: #4D4D4D;
            font-size: 0.9em;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        .joint-slider {
            width: 100%;
            margin-bottom: 10px;
        }

        .joint-value {
            text-align: center;
            font-size: 0.9em;
            color: #4D4D4D;
            font-weight: 500;
            transition: all 0.2s ease;
        }

        .joint-value.updating {
            color: #4D4D4D;
            font-weight: bold;
            transform: scale(1.1);
        }

        .logs-container {
            height: 400px;
            overflow-y: auto;
            background: linear-gradient(135deg, #f8f9fa 0%, #ffffff 100%);
            border: 2px solid rgba(77, 77, 77, 0.15);
            border-radius: 12px;
            padding: 18px;
            font-family: "SF Mono", "Monaco", "Inconsolata", "Fira Code", "Courier New", monospace;
            font-size: 13px;
        }

        .log-entry {
            margin-bottom: 10px;
            padding: 10px 14px;
            border-radius: 8px;
            background: #ffffff;
            border-left: 4px solid #e0e0e0;
            color: #4D4D4D;
            transition: all 0.2s ease;
            box-shadow: 0 1px 3px rgba(0, 0, 0, 0.05);
        }

        .log-entry:hover {
            transform: translateX(4px);
            box-shadow: 0 2px 6px rgba(0, 0, 0, 0.1);
        }

        .log-info {
            border-left-color: #10b981;
            background: linear-gradient(90deg, rgba(16, 185, 129, 0.05) 0%, #ffffff 100%);
        }
        .log-warning {
            border-left-color: #f59e0b;
            background: linear-gradient(90deg, rgba(245, 158, 11, 0.05) 0%, #ffffff 100%);
        }
        .log-error {
            border-left-color: #ef4444;
            background: linear-gradient(90deg, rgba(239, 68, 68, 0.05) 0%, #ffffff 100%);
        }
        .log-debug {
            border-left-color: #4D4D4D;
            background: linear-gradient(90deg, rgba(77, 77, 77, 0.05) 0%, #ffffff 100%);
        }

        .metrics-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-top: 20px;
        }

        .metric-item {
            background: linear-gradient(135deg, #ffffff 0%, #f8f9fa 100%);
            padding: 18px;
            border-radius: 12px;
            text-align: center;
            border: 2px solid rgba(77, 77, 77, 0.15);
            transition: all 0.3s ease;
        }

        .metric-item:hover {
            border-color: rgba(77, 77, 77, 0.4);
            box-shadow: 0 4px 12px rgba(77, 77, 77, 0.15);
            transform: translateY(-2px);
        }

        .metric-value {
            font-size: 1.5em;
            font-weight: bold;
            margin-bottom: 5px;
            color: #4D4D4D;
        }

        .metric-label {
            font-size: 0.85em;
            color: #4D4D4D;
            font-weight: 500;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        .vision-panel {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }

        .vision-item {
            background: linear-gradient(135deg, #ffffff 0%, #f8f9fa 100%);
            padding: 18px;
            border-radius: 12px;
            text-align: center;
            border: 2px solid rgba(77, 77, 77, 0.15);
            transition: all 0.3s ease;
        }

        .vision-item:hover {
            border-color: rgba(77, 77, 77, 0.4);
            box-shadow: 0 4px 12px rgba(77, 77, 77, 0.15);
            transform: translateY(-2px);
        }

        .vision-count {
            font-size: 2em;
            font-weight: bold;
            margin-bottom: 5px;
            color: #4D4D4D;
        }

        .vision-label {
            font-size: 0.85em;
            color: #4D4D4D;
            font-weight: 500;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        /* Styles Troubleshooting */
        .troubleshooting-panel {
            display: flex;
            flex-direction: column;
            gap: 15px;
        }

        .troubleshooting-actions {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
            gap: 10px;
            margin-bottom: 15px;
        }

        .troubleshooting-results {
            background: linear-gradient(135deg, #f8f9fa 0%, #ffffff 100%);
            border: 2px solid rgba(102, 126, 234, 0.15);
            border-radius: 12px;
            padding: 18px;
            max-height: 300px;
            overflow-y: auto;
            font-size: 0.9em;
        }

        .troubleshooting-placeholder {
            text-align: center;
            opacity: 0.6;
            font-style: italic;
            padding: 40px 20px;
        }

        .troubleshooting-placeholder::before {
            content: '🔍';
            display: block;
            font-size: 3em;
            margin-bottom: 10px;
            opacity: 0.5;
        }

        /* Toast notifications */
        .toast {
            position: fixed;
            bottom: 20px;
            right: 20px;
            background: rgba(0, 0, 0, 0.8);
            color: white;
            padding: 15px 20px;
            border-radius: 10px;
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
            z-index: 1000;
            animation: slideInRight 0.3s ease-out;
            max-width: 300px;
        }

        @keyframes slideInRight {
            from {
                transform: translateX(100%);
                opacity: 0;
            }
            to {
                transform: translateX(0);
                opacity: 1;
            }
        }

        .toast.success {
            border-left: 4px solid #4CAF50;
        }

        .toast.error {
            border-left: 4px solid #F44336;
        }

        .toast.warning {
            border-left: 4px solid #FF9800;
        }

        .toast.info {
            border-left: 4px solid #2196F3;
        }

        /* Empty states */
        .empty-state {
            text-align: center;
            padding: 40px 20px;
            opacity: 0.7;
        }

        .empty-state::before {
            content: '📭';
            display: block;
            font-size: 4em;
            margin-bottom: 15px;
            opacity: 0.5;
        }

        /* Tooltip */
        [data-tooltip] {
            position: relative;
            cursor: help;
        }

        [data-tooltip]:hover::after {
            content: attr(data-tooltip);
            position: absolute;
            bottom: 100%;
            left: 50%;
            transform: translateX(-50%);
            background: rgba(0, 0, 0, 0.9);
            color: white;
            padding: 8px 12px;
            border-radius: 6px;
            font-size: 12px;
            white-space: nowrap;
            z-index: 1000;
            margin-bottom: 5px;
            animation: fadeInUp 0.2s ease-out;
        }

        [data-tooltip]:hover::before {
            content: '';
            position: absolute;
            bottom: 100%;
            left: 50%;
            transform: translateX(-50%);
            border: 5px solid transparent;
            border-top-color: rgba(0, 0, 0, 0.9);
            z-index: 1000;
        }

        /* Responsive */
        @media (max-width: 768px) {
            .main-container {
                grid-template-columns: 1fr;
                padding: 10px;
                gap: 15px;
            }

            .header h1 {
                font-size: 1.8em;
            }

            .controls-grid {
                grid-template-columns: 1fr;
            }

            .status-panel {
                grid-template-columns: 1fr 1fr;
            }
        }

        /* Focus visible pour accessibilité */
        button:focus-visible,
        input:focus-visible {
            outline: 2px solid #4D4D4D;
            outline-offset: 2px;
        }

        /* Skeleton loading */
        .skeleton {
            background: linear-gradient(90deg, rgba(255,255,255,0.1) 25%, rgba(255,255,255,0.15) 50%, rgba(255,255,255,0.1) 75%);
            background-size: 200% 100%;
            animation: loading 1.5s ease-in-out infinite;
            border-radius: 4px;
        }

        @keyframes loading {
            0% { background-position: 200% 0; }
            100% { background-position: -200% 0; }
        }

        .troubleshooting-item {
            margin-bottom: 12px;
            padding: 14px;
            border-radius: 10px;
            background: #ffffff;
            border: 2px solid rgba(102, 126, 234, 0.1);
            transition: all 0.3s ease;
        }

        .troubleshooting-item:hover {
            box-shadow: 0 4px 12px rgba(102, 126, 234, 0.15);
            transform: translateX(4px);
        }

        .troubleshooting-item.ok {
            border-left: 3px solid #4CAF50;
        }

        .troubleshooting-item.warning {
            border-left: 3px solid #FF9800;
        }

        .troubleshooting-item.error {
            border-left: 3px solid #F44336;
        }

        .troubleshooting-item h4 {
            margin: 0 0 5px 0;
            font-size: 1em;
        }

        .troubleshooting-item p {
            margin: 5px 0;
            font-size: 0.9em;
        }

        .troubleshooting-fix {
            margin-top: 8px;
            padding: 10px;
            background: #f5f5f5;
            border-left: 3px solid #4D4D4D;
            border-radius: 4px;
            font-family: "Courier New", monospace;
            font-size: 0.85em;
            color: #4D4D4D;
        }

        .troubleshooting-docs {
            margin-top: 15px;
            padding: 15px;
            background: #f5f5f5;
            border: 1px solid #E6E6E6;
            border-radius: 8px;
        }

        .troubleshooting-docs h4 {
            margin-top: 0;
            margin-bottom: 10px;
        }

        .troubleshooting-docs a {
            color: #4D4D4D;
            text-decoration: none;
            display: block;
            padding: 5px 0;
            transition: color 0.2s ease;
        }

        .troubleshooting-docs a:hover {
            color: #008181;
            text-decoration: underline;
        }

        @media (max-width: 1200px) {
            .main-container {
                grid-template-columns: 1fr;
            }
        }

        /* Styles Chat BBIA */
        .chat-container {
            display: flex;
            flex-direction: column;
            height: 400px;
        }

        .chat-messages {
            background: linear-gradient(135deg, #f8f9fa 0%, #ffffff 100%);
            border: 2px solid rgba(102, 126, 234, 0.15);
            border-radius: 12px;
            padding: 18px;
            overflow-y: auto;
            flex: 1;
            margin-bottom: 15px;
            font-family: 'Segoe UI', sans-serif;
            font-size: 14px;
        }

        .chat-message {
            padding: 12px 15px;
            border-radius: 8px;
            margin-bottom: 10px;
            transition: all 0.3s ease;
            word-wrap: break-word;
        }

        .chat-message:hover {
            box-shadow: 0 2px 8px rgba(77, 77, 77, 0.1);
            transform: translateY(-1px);
        }

        .chat-message.chat-user {
            background: linear-gradient(135deg, #4D4D4D 0%, #4D4D4D 100%);
            color: #ffffff;
            border-left: 4px solid #4D4D4D;
            text-align: right;
            margin-left: 20%;
            box-shadow: 0 2px 8px rgba(77, 77, 77, 0.2);
        }

        .chat-message.chat-bbia {
            background: linear-gradient(135deg, #f8f9fa 0%, #ffffff 100%);
            color: #4D4D4D;
            border-left: 4px solid #4D4D4D;
            text-align: left;
            margin-right: 20%;
            box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
        }

        .chat-author {
            font-weight: 700;
            font-size: 11px;
            margin-bottom: 6px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }

        .chat-message.chat-user .chat-author {
            color: rgba(255, 255, 255, 0.9);
        }

        .chat-message.chat-bbia .chat-author {
            color: #4D4D4D;
        }

        .chat-text {
            font-size: 13px;
        }

        .chat-input-group {
            display: flex;
            gap: 10px;
        }

        .chat-input-group input {
            flex: 1;
            padding: 14px 18px;
            border: 2px solid rgba(102, 126, 234, 0.2);
            border-radius: 10px;
            background: #ffffff;
            color: #2c3e50;
            font-size: 14px;
            transition: all 0.3s ease;
            outline: none;
        }

        .chat-input-group input:focus {
            border-color: #667eea;
            box-shadow: 0 0 0 4px rgba(102, 126, 234, 0.15);
        }

        .chat-input-group input::placeholder {
            color: #999;
        }

        /* Toast Notifications - Style professionnel inspiré de Reachy Mini */
        .toast {
            position: relative;
            padding: 16px 24px;
            border-radius: 8px;
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
            min-width: 300px;
            max-width: 400px;
            font-size: 14px;
            font-weight: 500;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.2);
            pointer-events: auto;
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
        }

        .toast.success {
            background: linear-gradient(135deg, rgba(76, 175, 80, 0.95), rgba(56, 142, 60, 0.95));
            color: white;
        }

        .toast.error {
            background: linear-gradient(135deg, rgba(244, 67, 54, 0.95), rgba(198, 40, 40, 0.95));
            color: white;
        }

        .toast.warning {
            background: linear-gradient(135deg, rgba(255, 152, 0, 0.95), rgba(245, 124, 0, 0.95));
            color: white;
        }

        .toast.info {
            background: linear-gradient(135deg, rgba(33, 150, 243, 0.95), rgba(25, 118, 210, 0.95));
            color: white;
        }

        /* Amélioration du statut robot avec animations */
        .robot-status-container {
            position: relative;
            display: inline-block;
        }

        .robot-status-container::before {
            content: '';
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            width: 60px;
            height: 60px;
            border-radius: 50%;
            background: rgba(0, 204, 204, 0.2);
            animation: pulse-ring 2s ease-out infinite;
        }

        @keyframes pulse-ring {
            0% {
                transform: translate(-50%, -50%) scale(0.8);
                opacity: 1;
            }
            100% {
                transform: translate(-50%, -50%) scale(1.4);
                opacity: 0;
            }
        }

        /* Amélioration de l'accessibilité */
        @media (prefers-reduced-motion: reduce) {
            *,
            *::before,
            *::after {
                animation-duration: 0.01ms !important;
                animation-iteration-count: 1 !important;
                transition-duration: 0.01ms !important;
            }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>BBIA Advanced Dashboard</h1>
        <div class="subtitle">
            <span id="connection-indicator" class="connection-indicator disconnected"></span>
            <span id="connection-text">Déconnecté</span> |
            Backend: <span id="robot-backend">-</span> |
            Version: 1.2.0
        </div>
    </div>

    <div class="main-container">
        <!-- Panel de statut -->
        <div class="panel status-panel">
            <div class="status-item">
                <div class="status-value" id="robot-status">❌</div>
                <div class="status-label">Robot</div>
            </div>
            <div class="status-item">
                <div class="status-value" id="emotion-status">😐</div>
                <div class="status-label">Émotion</div>
            </div>
            <div class="status-item">
                <div class="status-value" id="latency-status">0ms</div>
                <div class="status-label">Latence</div>
            </div>
            <div class="status-item">
                <div class="status-value" id="fps-status">0</div>
                <div class="status-label">FPS</div>
            </div>
            <div class="status-item">
                <div class="status-value" id="objects-status">0</div>
                <div class="status-label">Objets</div>
            </div>
            <div class="status-item">
                <div class="status-value" id="faces-status">0</div>
                <div class="status-label">Visages</div>
            </div>
        </div>

        <!-- Panel de contrôle -->
        <div class="panel">
            <h3>Contrôles</h3>

            <div class="controls-grid">
                <button class="control-button emotion-button" onclick="setEmotion('happy', this)">Heureux</button>
                <button class="control-button emotion-button" onclick="setEmotion('sad', this)">Triste</button>
                <button class="control-button emotion-button" onclick="setEmotion('excited', this)">Excité</button>
                <button class="control-button emotion-button" onclick="setEmotion('angry', this)">Colère</button>
                <button class="control-button emotion-button" onclick="setEmotion('neutral', this)">Neutre</button>
                <button class="control-button emotion-button" onclick="setEmotion('curious', this)">Curieux</button>
            </div>

            <div class="controls-grid">
                <button class="control-button action-button" onclick="sendAction('look_at', this)">Regarder</button>
                <button class="control-button action-button" onclick="sendAction('greet', this)">Saluer</button>
                <button class="control-button action-button" onclick="sendAction('wake_up', this)">Réveil</button>
                <button class="control-button action-button" onclick="sendAction('sleep', this)">Dormir</button>
                <button class="control-button action-button" onclick="sendAction('nod', this)">Hocher</button>
                <button class="control-button action-button" onclick="sendAction('stop', this)">Arrêter</button>
            </div>

            <div class="controls-grid">
                <button class="control-button behavior-button" onclick="runBehavior('greeting', this)">Salutation</button>
                <button class="control-button behavior-button" onclick="runBehavior('exploration', this)">Exploration</button>
                <button class="control-button behavior-button" onclick="runBehavior('interaction', this)">Interaction</button>
                <button class="control-button behavior-button" onclick="runBehavior('demo', this)">Démo</button>
            </div>
        </div>

        <!-- Panel de métriques -->
        <div class="panel">
            <h3>Métriques Temps Réel</h3>
            <div class="chart-container">
                <canvas id="metricsChart"></canvas>
            </div>
            <div class="metrics-grid">
                <div class="metric-item">
                    <div class="metric-value" id="cpu-metric">0%</div>
                    <div class="metric-label">CPU</div>
                </div>
                <div class="metric-item">
                    <div class="metric-value" id="memory-metric">0%</div>
                    <div class="metric-label">Mémoire</div>
                </div>
                <div class="metric-item">
                    <div class="metric-value" id="volume-metric">0%</div>
                    <div class="metric-label">Volume</div>
                </div>
                <div class="metric-item">
                    <div class="metric-value" id="intensity-metric">0%</div>
                    <div class="metric-label">Intensité</div>
                </div>
            </div>
        </div>

        <!-- Panel de contrôle des joints -->
        <div class="panel">
            <h3>Contrôle des Joints</h3>
            <div class="joint-controls" id="joint-controls">
                <!-- Les contrôles de joints seront générés dynamiquement -->
            </div>
        </div>

        <!-- Panel de vision -->
        <div class="panel">
            <h3>Vision</h3>
            <div style="margin-bottom: 20px;">
                <video id="camera-stream" autoplay style="width: 100%; max-width: 640px; border-radius: 8px; border: 2px solid #4D4D4D; background: #000;"></video>
            </div>
            <div class="vision-panel">
                <div class="vision-item">
                    <div class="vision-count" id="objects-count">0</div>
                    <div class="vision-label">Objets Détectés</div>
                </div>
                <div class="vision-item">
                    <div class="vision-count" id="faces-count">0</div>
                    <div class="vision-label">Visages Détectés</div>
                </div>
            </div>
            <div class="controls-grid">
                <button class="control-button" onclick="toggleVision()">Basculer Vision</button>
                <button class="control-button" onclick="scanEnvironment()">Scanner</button>
                <button class="control-button" onclick="trackObject()">Suivre Objet</button>
            </div>
        </div>

        <!-- Panel Chat BBIA -->
        <div class="panel">
            <h3>Chat avec BBIA</h3>
            <div class="chat-container">
                <div class="chat-messages" id="chat-messages"></div>
                <div class="chat-input-group">
                    <input type="text" id="chat-input" placeholder="Tapez votre message..."
                           aria-label="Message chat BBIA"
                           data-tooltip="Appuyez sur Entrée pour envoyer">
                    <button class="control-button" id="chat-send-button" onclick="sendChatMessage()"
                            aria-label="Envoyer message"
                            data-tooltip="Envoyer le message">Envoyer</button>
                </div>
            </div>
        </div>

        <!-- Panel Troubleshooting -->
        <div class="panel">
            <h3>Troubleshooting</h3>
            <div class="troubleshooting-panel">
                <div class="troubleshooting-actions">
                    <button class="control-button" onclick="runAllChecks(this)">🔍 Vérifier Tout</button>
                    <button class="control-button" onclick="testCamera(this)">📷 Test Caméra</button>
                    <button class="control-button" onclick="testAudio(this)">🔊 Test Audio</button>
                    <button class="control-button" onclick="testNetwork(this)">🌐 Test Réseau</button>
                </div>
                <div class="troubleshooting-results" id="troubleshooting-results">
                    <p class="troubleshooting-placeholder">Cliquez sur "Vérifier Tout" pour commencer</p>
                </div>
                <div class="troubleshooting-docs" id="troubleshooting-docs" style="display: none;">
                    <h4>📚 Documentation</h4>
                    <div id="troubleshooting-docs-links"></div>
                </div>
            </div>
        </div>

        <!-- Panel de logs -->
        <div class="panel" style="grid-column: 1 / -1;">
            <h3>Logs Temps Réel</h3>
            <div class="logs-container" id="logs-container">
                <!-- Les logs seront ajoutés dynamiquement -->
            </div>
        </div>
    </div>

    <script>
        // Variables globales - Architecture modulaire inspirée de Reachy Mini
        const dashboard = {
            ws: null,
            reconnectInterval: null,
            metricsChart: null,
            metricsData: {
                labels: [],
                latency: [],
                fps: [],
                cpu: [],
                memory: []
            },
            currentStatus: {
                robot: { connected: false, backend: '-' },
                emotion: 'neutral',
                joints: [],
                vision: { objects: 0, faces: 0 }
            },
            lastMetricsUpdate: 0,
            METRICS_UPDATE_INTERVAL: 100,
            statusPollInterval: null,
            isPolling: false,
            videoErrorLogged: false,
            lastWSErrorLog: 0,
            lastCommandErrorLog: 0,
            lastChatErrorLog: 0
        };

        // Initialisation
        document.addEventListener('DOMContentLoaded', function() {
            console.log('🚀 Initialisation dashboard...');
            initializeChart();
            connect();
            startStatusPolling();
            startVideoStream();

            // Ajouter event listener pour Enter sur input chat
            const chatInput = document.getElementById('chat-input');
            if (chatInput) {
                chatInput.addEventListener('keypress', function(event) {
                    if (event.key === 'Enter' || event.keyCode === 13) {
                        event.preventDefault();
                        sendChatMessage();
                    }
                });
                console.log('✅ Event listener Enter ajouté au chat input');
            } else {
                console.error('❌ Input chat non trouvé à l\\'initialisation');
            }

            // Les fonctions sont maintenant assignées directement après leur définition
            // Plus besoin de setTimeout car elles sont définies avant d'être utilisées
            console.log('✅ Initialisation terminée - Toutes les fonctions sont globales');

            // Vérifier que sendChatMessage est accessible
            if (typeof window.sendChatMessage === 'function') {
                console.log('✅ sendChatMessage est accessible globalement');
            } else {
                console.error('❌ sendChatMessage n\\'est pas accessible globalement');
            }

            // Ajouter un event listener au bouton d'envoi comme backup
            const sendButton = document.getElementById('chat-send-button');
            if (sendButton) {
                sendButton.addEventListener('click', function(e) {
                    e.preventDefault();
                    console.log('🔵 Bouton cliqué - Appel sendChatMessage');
                    if (typeof window.sendChatMessage === 'function') {
                        window.sendChatMessage();
                    } else {
                        console.error('❌ sendChatMessage n\\'est pas une fonction');
                        showToast('❌ Erreur: fonction d\\'envoi non disponible', 'error', 3000);
                    }
                });
                console.log('✅ Event listener ajouté au bouton d\\'envoi');
            } else {
                console.warn('⚠️ Bouton d\\'envoi non trouvé');
            }
        });

        // Stream vidéo caméra
        function startVideoStream() {
            const video = document.getElementById('camera-stream');
            if (!video) {
                console.warn('❌ Élément video non trouvé');
                return;
            }

            console.log('📹 Démarrage stream vidéo...');

            // Utiliser endpoint MJPEG - gestion d'erreur silencieuse
            video.src = '/api/camera/stream';
            video.onloadstart = function() {
                console.log('✅ Stream vidéo démarré');
            };
            video.onerror = function(e) {
                // Erreur silencieuse - la caméra peut ne pas être disponible
                // Ne pas logger l'erreur pour éviter le bruit dans la console
                video.style.display = 'none';
                // Log uniquement une fois au démarrage
                if (!dashboard.videoErrorLogged) {
                    console.debug('⚠️ Stream vidéo non disponible (caméra peut être absente)');
                    dashboard.videoErrorLogged = true;
                }
                // Empêcher la propagation de l'erreur dans la console
                return false;
            };
            video.onloadeddata = function() {
                console.log('✅ Première frame vidéo chargée');
                video.style.display = 'block';
            };
        }

        // Connexion WebSocket - Gestion robuste comme Reachy Mini
        function connect() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.host}/ws`;

            try {
                dashboard.ws = new WebSocket(wsUrl);

                dashboard.ws.onopen = function(event) {
                    console.log('✅ WebSocket connecté');
                    updateConnectionStatus(true);
                    if (dashboard.reconnectInterval) {
                        clearInterval(dashboard.reconnectInterval);
                        dashboard.reconnectInterval = null;
                    }
                    addLog('info', 'Connexion WebSocket établie');
                    showToast('✅ Connecté au serveur', 'success', 2000);
                    // Message de bienvenue dans le chat
                    if (typeof addChatMessage === 'function') {
                        addChatMessage("bbia", "Bonjour ! Je suis BBIA. Comment puis-je vous aider aujourd'hui ?");
                    }
                };

                dashboard.ws.onmessage = function(event) {
                    try {
                        console.log('📥 [WS] Message reçu:', event.data);
                        const data = JSON.parse(event.data);
                        console.log('📥 [WS] Message parsé:', data);
                        handleMessage(data);
                    } catch (error) {
                        console.error('❌ [WS] Erreur parsing:', error, 'Data:', event.data);
                        addLog('error', `Erreur parsing: ${error.message}`);
                    }
                };

                dashboard.ws.onclose = function(event) {
                    console.log('⚠️ WebSocket fermé', event.code, event.reason);
                    updateConnectionStatus(false);
                    addLog('warning', `Connexion fermée (code: ${event.code})`);

                    // Reconnexion automatique avec backoff exponentiel
                    if (!dashboard.reconnectInterval) {
                        let delay = 3000; // Démarrer à 3 secondes
                        const maxDelay = 30000; // Max 30 secondes
                        let reconnectAttempts = 0;
                        const MAX_RECONNECT_ATTEMPTS = 20; // Arrêter après 20 tentatives

                        const reconnect = () => {
                            if (dashboard.ws && dashboard.ws.readyState === WebSocket.OPEN) {
                                reconnectAttempts = 0;
                                delay = 3000; // Réinitialiser le délai
                                if (dashboard.reconnectInterval) {
                                    clearInterval(dashboard.reconnectInterval);
                                    dashboard.reconnectInterval = null;
                                }
                                return;
                            }

                            reconnectAttempts++;

                            // Arrêter après trop de tentatives
                            if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
                                if (dashboard.reconnectInterval) {
                                    clearInterval(dashboard.reconnectInterval);
                                    dashboard.reconnectInterval = null;
                                }
                                console.debug('⚠️ Trop de tentatives de reconnexion WebSocket - arrêt temporaire');
                                // Réessayer après 30 secondes
                                setTimeout(() => {
                                    reconnectAttempts = 0;
                                    delay = 3000;
                                    if (!dashboard.reconnectInterval) {
                                        dashboard.reconnectInterval = setInterval(reconnect, delay);
                                    }
                                }, 30000);
                                return;
                            }

                            // Logger seulement toutes les 5 tentatives
                            if (reconnectAttempts === 1 || reconnectAttempts % 5 === 0) {
                                console.debug(`🔄 Tentative de reconnexion ${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS}...`);
                            }

                            setTimeout(() => {
                                if (dashboard.ws && dashboard.ws.readyState !== WebSocket.OPEN) {
                                    connect();
                                }
                                delay = Math.min(delay * 1.5, maxDelay);
                            }, delay);
                        };
                        dashboard.reconnectInterval = setInterval(reconnect, delay);
                    }
                };

                dashboard.ws.onerror = function(error) {
                    // Erreur silencieuse - ne pas logger l'objet Event complet
                    // Logger seulement une fois toutes les 5 secondes
                    const now = Date.now();
                    if (!dashboard.lastWSErrorLog || (now - dashboard.lastWSErrorLog) >= 5000) {
                        console.debug('⚠️ Erreur WebSocket (serveur peut être arrêté)');
                        dashboard.lastWSErrorLog = now;
                    }
                    updateConnectionStatus(false);
                    // Ne pas afficher de toast à chaque erreur pour éviter le spam
                    // Empêcher la propagation de l'erreur dans la console
                    return false;
                };
            } catch (error) {
                // Erreur silencieuse
                console.debug('⚠️ Erreur création WebSocket (serveur peut être arrêté)');
                updateConnectionStatus(false);
            }
        }

        // Polling de statut régulier avec backoff exponentiel
        let pollingErrorCount = 0;
        let lastPollingErrorLog = 0;
        let pollingInterval = 500; // Intervalle initial
        const POLLING_ERROR_LOG_INTERVAL = 10000; // Logger seulement toutes les 10 secondes
        const MAX_CONSECUTIVE_ERRORS = 10; // Arrêter après 10 erreurs consécutives
        const MAX_POLLING_INTERVAL = 30000; // Max 30 secondes entre les tentatives

        async function startStatusPolling() {
            if (dashboard.isPolling) return;
            dashboard.isPolling = true;

            const pollStatus = async () => {
                try {
                    // Utiliser un timeout manuel pour compatibilité
                    const controller = new AbortController();
                    const timeoutId = setTimeout(() => controller.abort(), 2000);

                    const response = await fetch('/api/status', {
                        signal: controller.signal
                    }).catch(() => null); // Intercepter toutes les erreurs silencieusement

                    clearTimeout(timeoutId);

                    if (response && response.ok) {
                        const data = await response.json();
                        updateStatusFromAPI(data);
                        // Reset en cas de succès
                        if (pollingErrorCount > 0) {
                            pollingErrorCount = 0;
                            pollingInterval = 500; // Réinitialiser l'intervalle
                            console.debug('✅ Connexion serveur rétablie');
                            // Redémarrer le polling à l'intervalle normal
                            if (dashboard.statusPollInterval) {
                                clearInterval(dashboard.statusPollInterval);
                            }
                            dashboard.statusPollInterval = setInterval(pollStatus, pollingInterval);
                        }
                    } else {
                        // Erreur HTTP ou pas de réponse
                        pollingErrorCount++;
                        handlePollingError();
                    }
                } catch (error) {
                    // Erreur silencieuse - ne pas propager
                    pollingErrorCount++;
                    handlePollingError();
                }
            };

            function handlePollingError() {
                // Arrêter le polling après trop d'erreurs
                if (pollingErrorCount >= MAX_CONSECUTIVE_ERRORS) {
                    if (dashboard.statusPollInterval) {
                        clearInterval(dashboard.statusPollInterval);
                        dashboard.statusPollInterval = null;
                    }
                    const now = Date.now();
                    if ((now - lastPollingErrorLog) >= POLLING_ERROR_LOG_INTERVAL) {
                        console.debug('⚠️ Serveur non disponible - polling suspendu. Reconnexion automatique...');
                        lastPollingErrorLog = now;
                    }
                    // Réessayer après un délai plus long
                    setTimeout(() => {
                        pollingErrorCount = 0;
                        pollingInterval = 500;
                        if (!dashboard.statusPollInterval) {
                            dashboard.statusPollInterval = setInterval(pollStatus, pollingInterval);
                        }
                    }, 10000); // Réessayer après 10 secondes
                    return;
                }

                // Augmenter l'intervalle avec backoff exponentiel
                pollingInterval = Math.min(pollingInterval * 1.5, MAX_POLLING_INTERVAL);

                // Redémarrer le polling avec le nouvel intervalle
                if (dashboard.statusPollInterval) {
                    clearInterval(dashboard.statusPollInterval);
                }
                dashboard.statusPollInterval = setInterval(pollStatus, pollingInterval);

                // Logger seulement toutes les 10 secondes
                const now = Date.now();
                if (pollingErrorCount === 1 || (now - lastPollingErrorLog) >= POLLING_ERROR_LOG_INTERVAL) {
                    console.debug(`⚠️ Serveur non disponible (${pollingErrorCount} erreurs, intervalle: ${Math.round(pollingInterval)}ms)`);
                    lastPollingErrorLog = now;
                }
            }

            // Polling initial
            await pollStatus();

            // Polling régulier avec intervalle adaptatif
            dashboard.statusPollInterval = setInterval(pollStatus, pollingInterval);
        }

        function updateStatusFromAPI(data) {
            // Mettre à jour le statut seulement si changé (comme Reachy Mini)
            const previousState = JSON.stringify(dashboard.currentStatus);

            if (data.robot_connected !== undefined) {
                dashboard.currentStatus.robot.connected = data.robot_connected;
            }
            if (data.backend) {
                dashboard.currentStatus.robot.backend = data.backend;
            }
            if (data.metrics) {
                updateMetrics(data.metrics);
            }

            const currentState = JSON.stringify(dashboard.currentStatus);
            if (previousState !== currentState) {
                updateUIFromStatus();
            }
        }

        function updateUIFromStatus() {
            // Mise à jour UI seulement si nécessaire
            const robotStatusEl = document.getElementById('robot-status');
            if (robotStatusEl) {
                robotStatusEl.textContent = dashboard.currentStatus.robot.connected ? '✅' : '❌';
            }
            const backendEl = document.getElementById('robot-backend');
            if (backendEl) {
                backendEl.textContent = dashboard.currentStatus.robot.backend || '-';
            }
        }

        // Gestion des messages
        function handleMessage(data) {
            switch(data.type) {
                case 'complete_status':
                    updateCompleteStatus(data);
                    break;
                case 'metrics_update':
                    updateMetrics(data.metrics);
                    break;
                case 'log':
                    addLog(data.level, data.message);
                    break;
                case 'chat_response':
                    // Réponse chat de BBIA
                    console.log('💬 [CHAT] Réponse reçue:', data);
                    if (data.sender && data.message) {
                        console.log(`✅ [CHAT] Ajout message ${data.sender}`);
                        addChatMessage(data.sender, data.message);
                        addLog('info', `Réponse ${data.sender} reçue`);
                    } else {
                        console.warn('⚠️ [CHAT] Données incomplètes:', data);
                        addChatMessage('bbia', 'Désolé, je n\\'ai pas pu traiter votre message correctement.');
                    }
                    break;
            }
        }

        // Mise à jour du statut complet
        function updateCompleteStatus(data) {
            if (!data) return;

            // Statut robot
            const robotStatusEl = document.getElementById('robot-status');
            if (robotStatusEl && data.robot) {
                robotStatusEl.textContent = data.robot.connected ? '✅' : '❌';
            }
            const backendEl = document.getElementById('robot-backend');
            if (backendEl && data.robot) {
                backendEl.textContent = data.robot.backend || '-';
            }

            // Statut émotion
            const emotionStatusEl = document.getElementById('emotion-status');
            if (emotionStatusEl && data.bbia_modules && data.bbia_modules.emotions) {
                emotionStatusEl.textContent = getEmotionEmoji(data.bbia_modules.emotions.current);
            }

            // Joints disponibles
            if (data.robot && data.robot.joints) {
                updateJointControls(data.robot.joints);
            }

            // Vision
            const objectsCountEl = document.getElementById('objects-count');
            if (objectsCountEl && data.bbia_modules && data.bbia_modules.vision) {
                objectsCountEl.textContent = data.bbia_modules.vision.objects || 0;
            }
            const facesCountEl = document.getElementById('faces-count');
            if (facesCountEl && data.bbia_modules && data.bbia_modules.vision) {
                facesCountEl.textContent = data.bbia_modules.vision.faces || 0;
            }
        }

        // Mise à jour des métriques (optimisée avec throttling) - Style Reachy Mini
        function updateMetrics(metrics) {
            if (!metrics) return;

            const now = Date.now();
            if (now - dashboard.lastMetricsUpdate < dashboard.METRICS_UPDATE_INTERVAL) {
                return; // Skip si trop tôt
            }
            dashboard.lastMetricsUpdate = now;

            // Utiliser requestAnimationFrame pour des mises à jour fluides
            requestAnimationFrame(() => {
                // Statut temps réel avec animation
                if (metrics.performance) {
                    updateValueWithAnimation('latency-status', Math.round(metrics.performance.latency_ms || 0) + 'ms');
                    updateValueWithAnimation('fps-status', Math.round(metrics.performance.fps || 0));
                    updateValueWithAnimation('cpu-metric', Math.round(metrics.performance.cpu_usage || 0) + '%');
                    updateValueWithAnimation('memory-metric', Math.round(metrics.performance.memory_usage || 0) + '%');
                }

                if (metrics.vision) {
                    updateValueWithAnimation('objects-status', metrics.vision.objects_detected || 0);
                    updateValueWithAnimation('faces-status', metrics.vision.faces_detected || 0);
                }

                if (metrics.audio) {
                    updateValueWithAnimation('volume-metric', Math.round((metrics.audio.volume_level || 0) * 100) + '%');
                }

                if (metrics.emotion_intensity !== undefined) {
                    updateValueWithAnimation('intensity-metric', Math.round(metrics.emotion_intensity * 100) + '%');
                }

                // Mise à jour du graphique
                updateChart(metrics);
            });
        }

        // Fonction helper pour mettre à jour avec animation
        function updateValueWithAnimation(elementId, newValue) {
            const element = document.getElementById(elementId);
            if (!element) return;

            if (element.textContent !== String(newValue)) {
                element.classList.add('updating');
                element.textContent = newValue;
                setTimeout(() => {
                    element.classList.remove('updating');
                }, 500);
            }
        }

        // Initialisation du graphique
        function initializeChart() {
            const canvas = document.getElementById('metricsChart');
            if (!canvas) return;

            const ctx = canvas.getContext('2d');
            dashboard.metricsChart = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: dashboard.metricsData.labels,
                    datasets: [
                        {
                            label: 'Latence (ms)',
                            data: dashboard.metricsData.latency,
                            borderColor: '#4D4D4D',
                            backgroundColor: 'rgba(77, 77, 77, 0.1)',
                            tension: 0.4,
                            fill: true
                        },
                        {
                            label: 'FPS',
                            data: dashboard.metricsData.fps,
                            borderColor: '#008181',
                            backgroundColor: 'rgba(0, 129, 129, 0.1)',
                            tension: 0.4,
                            fill: true
                        },
                        {
                            label: 'CPU (%)',
                            data: dashboard.metricsData.cpu,
                            borderColor: '#4D4D4D',
                            backgroundColor: 'rgba(77, 77, 77, 0.15)',
                            tension: 0.4,
                            fill: true
                        }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            beginAtZero: true,
                            grid: {
                                color: 'rgba(77, 77, 77, 0.1)'
                            },
                            ticks: {
                                color: '#4D4D4D'
                            }
                        },
                        x: {
                            grid: {
                                color: 'rgba(77, 77, 77, 0.1)'
                            },
                            ticks: {
                                color: '#4D4D4D'
                            }
                        }
                    },
                    plugins: {
                        legend: {
                            labels: {
                                color: '#4D4D4D',
                                font: {
                                    weight: '500'
                                }
                            }
                        }
                    }
                }
            });
        }

        // Mise à jour du graphique (optimisée) - Style professionnel
        function updateChart(metrics) {
            if (!dashboard.metricsChart || !metrics || !metrics.performance) return;

            const now = new Date().toLocaleTimeString();

            // Ajouter nouvelles données
            dashboard.metricsData.labels.push(now);
            dashboard.metricsData.latency.push(metrics.performance.latency_ms || 0);
            dashboard.metricsData.fps.push(metrics.performance.fps || 0);
            dashboard.metricsData.cpu.push(metrics.performance.cpu_usage || 0);
            dashboard.metricsData.memory.push(metrics.performance.memory_usage || 0);

            // Limiter à 30 points pour plus de données
            const maxPoints = 30;
            if (dashboard.metricsData.labels.length > maxPoints) {
                dashboard.metricsData.labels.shift();
                dashboard.metricsData.latency.shift();
                dashboard.metricsData.fps.shift();
                dashboard.metricsData.cpu.shift();
                dashboard.metricsData.memory.shift();
            }

            // Mettre à jour le graphique avec animation fluide
            dashboard.metricsChart.update({
                duration: 0, // Pas d'animation pour performance
                lazy: true
            });
        }

        // Mise à jour des contrôles de joints
        function updateJointControls(joints) {
            const container = document.getElementById('joint-controls');
            if (!container) return;

            container.innerHTML = '';

            if (!joints || joints.length === 0) {
                container.innerHTML = '<p style="text-align: center; opacity: 0.7;">Aucun joint disponible</p>';
                return;
            }

            joints.forEach(joint => {
                const control = document.createElement('div');
                control.className = 'joint-control';
                // Échapper les caractères spéciaux pour éviter les injections XSS
                const safeJoint = String(joint).replace(/[<>"']/g, '');
                control.innerHTML = `
                    <div class="joint-name">${safeJoint}</div>
                    <input type="range" class="joint-slider" id="slider-${safeJoint}"
                           min="-3.14" max="3.14" step="0.01" value="0"
                           onchange="setJointPosition('${safeJoint}', this.value)">
                    <div class="joint-value" id="value-${safeJoint}">0.00</div>
                `;
                container.appendChild(control);
            });
        }

        // Fonctions de contrôle avec feedback visuel
        function setEmotion(emotion, buttonElement) {
            const button = buttonElement || (window.event && window.event.target) || document.querySelector(`button[onclick*="setEmotion('${emotion}')"]`);
            if (button) {
                button.classList.add('loading');
                setTimeout(() => button.classList.remove('loading'), 500);
            }
            sendCommand('emotion', emotion);
            addLog('info', `Émotion définie: ${emotion}`);
        }
        window.setEmotion = setEmotion;

        function sendAction(action, buttonElement) {
            const button = buttonElement || (window.event && window.event.target) || document.querySelector(`button[onclick*="sendAction('${action}')"]`);
            if (button) {
                button.classList.add('loading');
                setTimeout(() => button.classList.remove('loading'), 500);
            }
            sendCommand('action', action);
            addLog('info', `Action envoyée: ${action}`);
        }
        window.sendAction = sendAction;

        function runBehavior(behavior, buttonElement) {
            const button = buttonElement || (window.event && window.event.target) || document.querySelector(`button[onclick*="runBehavior('${behavior}')"]`);
            if (button) {
                button.classList.add('loading');
                setTimeout(() => button.classList.remove('loading'), 1000);
            }
            sendCommand('behavior', behavior);
            addLog('info', `Comportement lancé: ${behavior}`);
        }
        window.runBehavior = runBehavior;

        // Debouncing pour les sliders de joints
        let jointUpdateTimeouts = {};

        function setJointPosition(joint, position) {
            const numPosition = parseFloat(position);
            if (isNaN(numPosition)) {
                addLog('error', `Position invalide pour joint ${joint}: ${position}`);
                return;
            }

            // Mise à jour immédiate de l'affichage
            const valueElement = document.getElementById(`value-${joint}`);
            if (valueElement) {
                valueElement.textContent = numPosition.toFixed(2);
                valueElement.classList.add('updating');
                setTimeout(() => valueElement.classList.remove('updating'), 300);
            }

            // Debounce: envoyer la commande après 100ms d'inactivité
            if (jointUpdateTimeouts[joint]) {
                clearTimeout(jointUpdateTimeouts[joint]);
            }

            jointUpdateTimeouts[joint] = setTimeout(() => {
                sendCommand('joint', { joint: joint, position: numPosition });
                delete jointUpdateTimeouts[joint];
            }, 100);
        }
        window.setJointPosition = setJointPosition;

        function toggleVision() {
            sendCommand('vision', 'toggle');
            addLog('info', 'Vision basculée');
        }
        window.toggleVision = toggleVision;

        function scanEnvironment() {
            sendCommand('vision', 'scan');
            addLog('info', 'Scan environnement lancé');
        }
        window.scanEnvironment = scanEnvironment;

        function trackObject() {
            sendCommand('vision', 'track');
            addLog('info', 'Suivi objet activé');
        }
        window.trackObject = trackObject;

        // Fonctions utilitaires
        function updateConnectionStatus(connected) {
            const indicator = document.getElementById('connection-indicator');
            const text = document.getElementById('connection-text');

            if (indicator) {
                indicator.className = 'connection-indicator ' + (connected ? 'connected' : 'disconnected');
            }
            if (text) {
                text.textContent = connected ? 'Connecté' : 'Déconnecté';
            }
        }

        function getEmotionEmoji(emotion) {
            const emojis = {
                'happy': '😊',
                'sad': '😢',
                'excited': '🤩',
                'angry': '😠',
                'neutral': '😐',
                'curious': '🤔',
                'calm': '😌',
                'surprised': '😲'
            };
            return emojis[emotion] || '😐';
        }

        function addLog(level, message) {
            const container = document.getElementById('logs-container');
            if (!container) return;

            const entry = document.createElement('div');
            entry.className = `log-entry log-${level}`;
            entry.style.opacity = '0';
            entry.style.transform = 'translateY(-10px)';
            entry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
            container.appendChild(entry);

            // Animation d'apparition
            requestAnimationFrame(() => {
                entry.style.transition = 'all 0.3s ease';
                entry.style.opacity = '1';
                entry.style.transform = 'translateY(0)';
            });

            // Scroll fluide
            container.scrollTo({
                top: container.scrollHeight,
                behavior: 'smooth'
            });

            // Limiter à 100 entrées (optimisé)
            if (container.children.length > 100) {
                const toRemove = container.children.length - 100;
                for (let i = 0; i < toRemove; i++) {
                    container.removeChild(container.firstChild);
                }
            }
        }

        // Toast notifications - Système professionnel inspiré de Reachy Mini
        const toastQueue = [];
        let toastContainer = null;

        function initToastContainer() {
            if (!toastContainer) {
                toastContainer = document.createElement('div');
                toastContainer.id = 'toast-container';
                toastContainer.style.cssText = 'position: fixed; top: 20px; right: 20px; z-index: 10000; display: flex; flex-direction: column; gap: 10px; pointer-events: none;';
                document.body.appendChild(toastContainer);
            }
        }

        function showToast(message, type = 'info', duration = 3000) {
            initToastContainer();

            const toast = document.createElement('div');
            toast.className = `toast ${type}`;
            toast.textContent = message;
            toast.style.opacity = '0';
            toast.style.transform = 'translateX(120%)';
            toastContainer.appendChild(toast);

            // Animation d'entrée
            requestAnimationFrame(() => {
                toast.style.opacity = '1';
                toast.style.transform = 'translateX(0)';
            });

            // Auto-dismiss
            const timeout = setTimeout(() => {
                toast.style.opacity = '0';
                toast.style.transform = 'translateX(120%)';
                setTimeout(() => {
                    if (toast.parentNode) {
                        toast.parentNode.removeChild(toast);
                    }
                }, 300);
            }, duration);

            // Permettre de fermer manuellement
            toast.style.cursor = 'pointer';
            toast.addEventListener('click', () => {
                clearTimeout(timeout);
                toast.style.opacity = '0';
                toast.style.transform = 'translateX(120%)';
                setTimeout(() => {
                    if (toast.parentNode) {
                        toast.parentNode.removeChild(toast);
                    }
                }, 300);
            });
        }

        function sendCommand(type, value) {
            if (dashboard.ws && dashboard.ws.readyState === WebSocket.OPEN) {
                try {
                    const command = {
                        type: 'command',
                        command_type: type,
                        value: value,
                        timestamp: new Date().toISOString()
                    };
                    console.log('Envoi commande:', command);
                    dashboard.ws.send(JSON.stringify(command));
                    showToast(`✅ Commande ${type} envoyée`, 'success', 2000);
                } catch (error) {
                    console.error('Erreur envoi commande:', error);
                    addLog('error', `Erreur envoi commande: ${error.message}`);
                    showToast(`❌ Erreur: ${error.message}`, 'error', 4000);
                }
            } else {
                // Logger seulement une fois toutes les 5 secondes pour éviter le spam
                const now = Date.now();
                if (!dashboard.lastCommandErrorLog || (now - dashboard.lastCommandErrorLog) >= 5000) {
                    console.debug('⚠️ WebSocket non connecté (serveur peut être arrêté)');
                    dashboard.lastCommandErrorLog = now;
                }
                // Ne pas afficher de toast pour éviter le spam
                if (!dashboard.reconnectInterval) {
                    connect();
                }
            }
        }
        window.sendCommand = sendCommand;

        // Fonctions Chat BBIA - Version simplifiée et robuste
        function sendChatMessage() {
            console.log('🔵 [CHAT] sendChatMessage appelé');

            // Récupérer l'input
            const input = document.getElementById('chat-input');
            if (!input) {
                console.error('❌ [CHAT] Input chat non trouvé');
                alert('Erreur: champ de saisie introuvable');
                return;
            }

            const message = input.value.trim();
            if (!message) {
                console.warn('⚠️ [CHAT] Message vide');
                return;
            }

            console.log('📤 [CHAT] Message:', message);
            console.log('🔌 [CHAT] WebSocket existe?', !!dashboard.ws);
            console.log('🔌 [CHAT] WebSocket state:', dashboard.ws ? dashboard.ws.readyState : 'null');

            // Vérifier WebSocket
            if (!dashboard.ws) {
                // Logger seulement une fois toutes les 5 secondes
                const now = Date.now();
                if (!dashboard.lastChatErrorLog || (now - dashboard.lastChatErrorLog) >= 5000) {
                    console.debug('⚠️ [CHAT] WebSocket non initialisé (serveur peut être arrêté)');
                    dashboard.lastChatErrorLog = now;
                }
                connect();
                // Réessayer après 1 seconde
                setTimeout(() => {
                    if (dashboard.ws && dashboard.ws.readyState === WebSocket.OPEN) {
                        sendChatMessage();
                    }
                }, 1000);
                return;
            }

            // Vérifier état WebSocket
            if (dashboard.ws.readyState !== WebSocket.OPEN) {
                // Logger seulement une fois toutes les 5 secondes
                const now = Date.now();
                if (!dashboard.lastChatErrorLog || (now - dashboard.lastChatErrorLog) >= 5000) {
                    console.debug('⚠️ [CHAT] WebSocket pas ouvert (serveur peut être arrêté)');
                    dashboard.lastChatErrorLog = now;
                }
                connect();
                setTimeout(() => {
                    if (dashboard.ws && dashboard.ws.readyState === WebSocket.OPEN) {
                        sendChatMessage();
                    }
                }, 1000);
                return;
            }

            // Envoyer le message
            try {
                const chatMessage = {
                    type: 'chat',
                    message: message,
                    timestamp: new Date().toISOString()
                };

                console.log('📨 [CHAT] Envoi:', JSON.stringify(chatMessage));
                dashboard.ws.send(JSON.stringify(chatMessage));
                console.log('✅ [CHAT] Message envoyé avec succès');

                // Afficher immédiatement le message utilisateur
                addChatMessage('user', message);

                // Vider l'input
                input.value = '';
                input.focus();

                addLog('info', `Message envoyé: ${message.substring(0, 30)}...`);

            } catch (error) {
                console.error('❌ [CHAT] Erreur:', error);
                alert('Erreur envoi: ' + error.message);
                addLog('error', `Erreur: ${error.message}`);
            }
        }

        // Rendre la fonction accessible globalement
        window.sendChatMessage = sendChatMessage;
        console.log('✅ Fonction sendChatMessage rendue globale');

        function addChatMessage(sender, message) {
            console.log('addChatMessage appelé:', sender, message);
            const container = document.getElementById('chat-messages');
            if (!container) {
                console.error('Container chat-messages non trouvé');
                return;
            }

            const entry = document.createElement('div');
            entry.className = `chat-message chat-${sender}`;
            entry.style.opacity = '0';
            entry.style.transform = sender === 'user' ? 'translateX(20px)' : 'translateX(-20px)';

            // Échapper HTML pour éviter XSS
            const safeMessage = String(message).replace(/[<>]/g, function(match) {
                return match === '<' ? '&lt;' : '&gt;';
            });
            entry.innerHTML = `
                <div class="chat-author">${sender === 'user' ? 'Vous' : 'BBIA'}</div>
                <div class="chat-text">${safeMessage}</div>
            `;
            container.appendChild(entry);

            // Animation d'apparition
            requestAnimationFrame(() => {
                entry.style.transition = 'all 0.3s ease';
                entry.style.opacity = '1';
                entry.style.transform = 'translateX(0)';
            });

            // Scroll fluide
            container.scrollTo({
                top: container.scrollHeight,
                behavior: 'smooth'
            });

            // Limiter à 50 messages
            if (container.children.length > 50) {
                const toRemove = container.children.length - 50;
                for (let i = 0; i < toRemove; i++) {
                    container.removeChild(container.firstChild);
                }
            }
        }

        // Rendre la fonction accessible globalement
        window.addChatMessage = addChatMessage;

        // Fonctions Troubleshooting avec feedback visuel
        async function runAllChecks(buttonElement) {
            const resultsDiv = document.getElementById('troubleshooting-results');
            if (!resultsDiv) return;
            const button = buttonElement || (window.event && window.event.target);

            if (button) {
                button.disabled = true;
                button.classList.add('loading');
            }

            resultsDiv.innerHTML = '<p style="text-align: center;"><span class="loading-spinner"></span> 🔍 Vérification en cours...</p>';

            try {
                const response = await fetch('/api/troubleshooting/check');
                const data = await response.json();

                if (data.success) {
                    displayTroubleshootingResults(data.results);
                    loadDocumentationLinks();
                } else {
                    resultsDiv.innerHTML = `<p class="troubleshooting-item error">❌ Erreur: ${data.error || 'Erreur inconnue'}</p>`;
                }
            } catch (error) {
                resultsDiv.innerHTML = `<p class="troubleshooting-item error">❌ Erreur réseau: ${error.message}</p>`;
            } finally {
                if (button) {
                    button.disabled = false;
                    button.classList.remove('loading');
                }
            }
        }
        window.runAllChecks = runAllChecks;

        async function testCamera(buttonElement) {
            const resultsDiv = document.getElementById('troubleshooting-results');
            if (!resultsDiv) return;
            const button = buttonElement || (window.event && window.event.target);

            if (button) {
                button.disabled = true;
                button.classList.add('loading');
            }

            resultsDiv.innerHTML = '<p style="text-align: center;"><span class="loading-spinner"></span> 📷 Test caméra en cours...</p>';

            try {
                const response = await fetch('/api/troubleshooting/test/camera', { method: 'POST' });
                const data = await response.json();

                if (data.success) {
                    const result = data.result;
                    const statusClass = result.status === 'ok' ? 'ok' : result.status === 'warning' ? 'warning' : 'error';
                    let html = `<div class="troubleshooting-item ${statusClass}">`;
                    html += `<h4>📷 Caméra</h4>`;
                    html += `<p>${result.message}</p>`;
                    if (result.fix) {
                        html += `<div class="troubleshooting-fix">💡 Fix: ${result.fix}</div>`;
                    }
                    if (result.frame_size) {
                        html += `<p>Taille: ${result.frame_size}</p>`;
                    }
                    html += `</div>`;
                    resultsDiv.innerHTML = html;
                } else {
                    resultsDiv.innerHTML = `<p class="troubleshooting-item error">❌ Erreur: ${data.error || 'Erreur inconnue'}</p>`;
                }
            } catch (error) {
                resultsDiv.innerHTML = `<p class="troubleshooting-item error">❌ Erreur réseau: ${error.message}</p>`;
            } finally {
                if (button) {
                    button.disabled = false;
                    button.classList.remove('loading');
                }
            }
        }
        window.testCamera = testCamera;

        async function testAudio(buttonElement) {
            const resultsDiv = document.getElementById('troubleshooting-results');
            if (!resultsDiv) return;
            const button = buttonElement || (window.event && window.event.target);

            if (button) {
                button.disabled = true;
                button.classList.add('loading');
            }

            resultsDiv.innerHTML = '<p style="text-align: center;"><span class="loading-spinner"></span> 🔊 Test audio en cours...</p>';

            try {
                const response = await fetch('/api/troubleshooting/test/audio', { method: 'POST' });
                const data = await response.json();

                if (data.success) {
                    const result = data.result;
                    const statusClass = result.status === 'ok' ? 'ok' : result.status === 'warning' ? 'warning' : 'error';
                    let html = `<div class="troubleshooting-item ${statusClass}">`;
                    html += `<h4>🔊 Audio</h4>`;
                    html += `<p>${result.message}</p>`;
                    if (result.fix) {
                        html += `<div class="troubleshooting-fix">💡 Fix: ${result.fix}</div>`;
                    }
                    if (result.devices && result.devices.length > 0) {
                        html += `<p>Devices disponibles: ${result.devices.length}</p>`;
                    }
                    html += `</div>`;
                    resultsDiv.innerHTML = html;
                } else {
                    resultsDiv.innerHTML = `<p class="troubleshooting-item error">❌ Erreur: ${data.error || 'Erreur inconnue'}</p>`;
                }
            } catch (error) {
                resultsDiv.innerHTML = `<p class="troubleshooting-item error">❌ Erreur réseau: ${error.message}</p>`;
            } finally {
                if (button) {
                    button.disabled = false;
                    button.classList.remove('loading');
                }
            }
        }
        window.testAudio = testAudio;

        async function testNetwork(buttonElement) {
            const resultsDiv = document.getElementById('troubleshooting-results');
            if (!resultsDiv) return;
            const button = buttonElement || (window.event && window.event.target);

            if (button) {
                button.disabled = true;
                button.classList.add('loading');
            }

            resultsDiv.innerHTML = '<p style="text-align: center;"><span class="loading-spinner"></span> 🌐 Test réseau en cours...</p>';

            try {
                const response = await fetch('/api/troubleshooting/test/network?host=8.8.8.8', { method: 'POST' });
                const data = await response.json();

                if (data.success) {
                    const result = data.result;
                    const statusClass = result.status === 'ok' ? 'ok' : 'error';
                    let html = `<div class="troubleshooting-item ${statusClass}">`;
                    html += `<h4>🌐 Réseau</h4>`;
                    html += `<p>${result.message}</p>`;
                    if (result.fix) {
                        html += `<div class="troubleshooting-fix">💡 Fix: ${result.fix}</div>`;
                    }
                    html += `</div>`;
                    resultsDiv.innerHTML = html;
                } else {
                    resultsDiv.innerHTML = `<p class="troubleshooting-item error">❌ Erreur: ${data.error || 'Erreur inconnue'}</p>`;
                }
            } catch (error) {
                resultsDiv.innerHTML = `<p class="troubleshooting-item error">❌ Erreur réseau: ${error.message}</p>`;
            } finally {
                if (button) {
                    button.disabled = false;
                    button.classList.remove('loading');
                }
            }
        }
        window.testNetwork = testNetwork;

        function displayTroubleshootingResults(results) {
            const resultsDiv = document.getElementById('troubleshooting-results');
            if (!resultsDiv || !results) return;
            let html = '';

            // Résumé
            if (results.summary) {
                const summary = results.summary;
                html += `<div class="troubleshooting-item ${summary.score >= 80 ? 'ok' : summary.score >= 50 ? 'warning' : 'error'}">`;
                html += `<h4>📊 Résumé</h4>`;
                html += `<p>Score: ${summary.score}% (${summary.passed}/${summary.total} checks OK)</p>`;
                html += `</div>`;
            }

            // Checks individuels
            const checks = ['python', 'dependencies', 'camera', 'audio', 'network', 'mujoco', 'ports', 'permissions'];
            for (const checkName of checks) {
                if (results[checkName]) {
                    const check = results[checkName];
                    const statusClass = check.status === 'ok' ? 'ok' : check.status === 'warning' ? 'warning' : 'error';
                    html += `<div class="troubleshooting-item ${statusClass}">`;
                    html += `<h4>${getCheckIcon(checkName)} ${checkName.charAt(0).toUpperCase() + checkName.slice(1)}</h4>`;
                    html += `<p>${check.message || check.status}</p>`;
                    if (check.fix) {
                        html += `<div class="troubleshooting-fix">💡 Fix: ${check.fix}</div>`;
                    }
                    html += `</div>`;
                }
            }

            resultsDiv.innerHTML = html;
        }

        function getCheckIcon(checkName) {
            const icons = {
                'python': '🐍',
                'dependencies': '📦',
                'camera': '📷',
                'audio': '🔊',
                'network': '🌐',
                'mujoco': '🎮',
                'ports': '🔌',
                'permissions': '🔐',
            };
            return icons[checkName] || '✓';
        }

        async function loadDocumentationLinks() {
            try {
                const response = await fetch('/api/troubleshooting/docs');
                const data = await response.json();

                if (data.success && data.links) {
                    const docsDiv = document.getElementById('troubleshooting-docs');
                    const linksDiv = document.getElementById('troubleshooting-docs-links');
                    let html = '';

                    for (const [name, path] of Object.entries(data.links)) {
                        // path est maintenant une URL complète depuis l'API
                        const displayName = name.replace(/_/g, ' ').replace(/\\b\\w/g, (l) => l.toUpperCase());
                        html += `<a href="${path}" target="_blank">📄 ${displayName}</a>`;
                    }

                    linksDiv.innerHTML = html;
                    docsDiv.style.display = 'block';
                }
            } catch (error) {
                console.error('Erreur chargement docs:', error);
            }
        }
    </script>
</body>
</html>
"""


# Routes FastAPI avancées
if FASTAPI_AVAILABLE:
    if app is None:
        raise RuntimeError("FastAPI app is None but FASTAPI_AVAILABLE is True")

    @app.get("/", response_class=HTMLResponse)
    async def advanced_dashboard():
        """Page principale du dashboard avancé."""
        return ADVANCED_DASHBOARD_HTML

    @app.get("/.well-known/appspecific/com.chrome.devtools.json")
    async def chrome_devtools_config():
        """Endpoint pour Chrome DevTools - évite les 404 dans les logs."""
        return {}  # Retourne un JSON vide pour Chrome DevTools

    @app.get("/api/status")
    async def get_status():
        """API endpoint pour récupérer le statut complet."""
        return {
            "status": "healthy",
            "timestamp": datetime.now().isoformat(),
            "version": "1.2.0",
            "robot_connected": advanced_websocket_manager.robot is not None,
            "backend": advanced_websocket_manager.robot_backend,
            "active_connections": len(advanced_websocket_manager.active_connections),
            "metrics": advanced_websocket_manager.current_metrics,
        }

    @app.get("/api/metrics")
    async def get_metrics():
        """API endpoint pour récupérer les métriques."""
        return {
            "current": advanced_websocket_manager.current_metrics,
            "history": list(advanced_websocket_manager.metrics_history)[
                -100:
            ],  # 100 dernières
        }

    @app.get("/api/joints")
    async def get_joints():
        """API endpoint pour récupérer les joints disponibles."""
        if advanced_websocket_manager.robot:
            return {
                "joints": advanced_websocket_manager.robot.get_available_joints(),
                "current_positions": advanced_websocket_manager._get_current_pose(),
            }
        return {"joints": [], "current_positions": {}}

    @app.post("/api/emotion")
    async def set_emotion(request: Request):
        """API endpoint pour définir une émotion."""
        try:
            emotion_data = await request.json()
            emotion = emotion_data.get("emotion", "neutral")
            intensity = emotion_data.get("intensity", 0.5)

            if advanced_websocket_manager.robot:
                success = advanced_websocket_manager.robot.set_emotion(
                    emotion,
                    intensity,
                )
                if success:
                    await advanced_websocket_manager.send_log_message(
                        "info",
                        f"Émotion définie: {emotion} (intensité: {intensity})",
                    )
                    return {"success": True, "emotion": emotion, "intensity": intensity}
                await advanced_websocket_manager.send_log_message(
                    "error",
                    f"Échec définition émotion: {emotion}",
                )
                return {"success": False, "error": "Failed to set emotion"}

            return {"success": False, "error": "Robot not connected"}
        except Exception as e:
            logger.error(f"Erreur set_emotion: {e}")
            return {"success": False, "error": str(e)}

    @app.post("/api/joint")
    async def set_joint_position(request: Request):
        """API endpoint pour définir la position d'un joint."""
        try:
            joint_data = await request.json()
            joint = joint_data.get("joint")
            position = joint_data.get("position", 0.0)

            if not joint:
                raise HTTPException(status_code=400, detail="Joint name required")

            if advanced_websocket_manager.robot:
                success = advanced_websocket_manager.robot.set_joint_pos(
                    joint, position
                )
                if success:
                    await advanced_websocket_manager.send_log_message(
                        "info",
                        f"Joint {joint} = {position}",
                    )
                    return {"success": True, "joint": joint, "position": position}
                await advanced_websocket_manager.send_log_message(
                    "error",
                    f"Échec contrôle joint {joint}",
                )
                return {"success": False, "error": "Failed to set joint position"}

            return {"success": False, "error": "Robot not connected"}
        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Erreur set_joint_position: {e}")
            return {"success": False, "error": str(e)}

    @app.get("/healthz")
    async def health_check():
        """Endpoint de santé pour CI."""
        return {
            "status": "healthy",
            "timestamp": datetime.now().isoformat(),
            "version": "1.2.0",
            "robot_connected": advanced_websocket_manager.robot is not None,
            "active_connections": len(advanced_websocket_manager.active_connections),
        }

    # Endpoints Troubleshooting
    @app.get("/api/troubleshooting/check")
    async def troubleshooting_check():
        """Exécute tous les checks de troubleshooting."""
        try:
            results = check_all()
            return {"success": True, "results": results}
        except Exception as e:
            logger.error(f"Erreur troubleshooting check: {e}")
            return {"success": False, "error": str(e)}

    @app.post("/api/troubleshooting/test/camera")
    async def troubleshooting_test_camera():
        """Test interactif de la caméra."""
        try:
            result = test_camera()
            return {"success": True, "result": result}
        except Exception as e:
            logger.error(f"Erreur test caméra: {e}")
            return {"success": False, "error": str(e)}

    @app.post("/api/troubleshooting/test/audio")
    async def troubleshooting_test_audio():
        """Test interactif de l'audio."""
        try:
            result = test_audio()
            return {"success": True, "result": result}
        except Exception as e:
            logger.error(f"Erreur test audio: {e}")
            return {"success": False, "error": str(e)}

    @app.post("/api/troubleshooting/test/network")
    async def troubleshooting_test_network(host: str = "8.8.8.8"):
        """Test interactif du réseau."""
        try:
            result = test_network_ping(host)
            return {"success": True, "result": result}
        except Exception as e:
            logger.error(f"Erreur test réseau: {e}")
            return {"success": False, "error": str(e)}

    @app.get("/api/troubleshooting/docs")
    async def troubleshooting_docs():
        """Retourne les liens vers la documentation."""
        try:
            links = get_documentation_links()
            # Convertir les chemins relatifs en URLs absolues
            base_url = "/api/docs/view"
            links_with_urls = {
                name: f"{base_url}?path={path}" for name, path in links.items()
            }
            return {"success": True, "links": links_with_urls}
        except Exception as e:
            logger.error(f"Erreur récupération docs: {e}")
            return {"success": False, "error": str(e)}

    @app.get("/api/docs/view")
    async def view_documentation(path: str):
        """Affiche un fichier de documentation en HTML formaté."""
        import html
        from pathlib import Path

        from fastapi.responses import FileResponse, HTMLResponse

        try:
            # Sécuriser le chemin pour éviter les accès non autorisés
            doc_path = Path(path)
            if ".." in str(doc_path) or doc_path.is_absolute():
                raise HTTPException(status_code=400, detail="Chemin invalide")

            # Construire le chemin complet depuis la racine du projet
            project_root = Path(__file__).parent.parent.parent
            full_path = project_root / doc_path

            # Vérifier que le fichier existe et est dans le dossier docs
            if not full_path.exists() or not str(full_path).startswith(
                str(project_root / "docs")
            ):
                raise HTTPException(status_code=404, detail="Fichier non trouvé")

            # Si c'est un fichier markdown, convertir en HTML
            if full_path.suffix == ".md":
                content = full_path.read_text(encoding="utf-8")
                # Échapper le HTML et convertir les retours à la ligne en <br>
                content_html = html.escape(content).replace("\n", "<br>\n")
                # Créer une page HTML simple avec le contenu
                html_page = f"""
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Documentation - {doc_path.name}</title>
    <style>
        body {{
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', sans-serif;
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
            line-height: 1.6;
            color: #4D4D4D;
            background: #f5f7fa;
        }}
        pre {{
            background: #ffffff;
            padding: 15px;
            border-radius: 8px;
            border: 1px solid #E6E6E6;
            overflow-x: auto;
            font-family: 'Courier New', monospace;
        }}
        code {{
            background: #f5f5f5;
            padding: 2px 6px;
            border-radius: 4px;
            font-family: 'Courier New', monospace;
        }}
        h1, h2, h3 {{
            color: #4D4D4D;
            border-bottom: 2px solid #4D4D4D;
            padding-bottom: 10px;
        }}
        a {{
            color: #008181;
            text-decoration: none;
        }}
        a:hover {{
            text-decoration: underline;
        }}
    </style>
</head>
<body>
    <h1>📚 {doc_path.name}</h1>
    <div style="background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1);">
        <pre style="white-space: pre-wrap; font-family: inherit;">{content_html}</pre>
    </div>
    <p style="margin-top: 20px; text-align: center;">
        <a href="/">← Retour au dashboard</a>
    </p>
</body>
</html>
"""
                return HTMLResponse(content=html_page)
            else:
                return FileResponse(full_path)
        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Erreur lecture documentation {path}: {e}")
            raise HTTPException(
                status_code=500, detail=f"Erreur lecture fichier: {e}"
            ) from e

    @app.get("/api/camera/stream")
    async def camera_stream():
        """Stream vidéo MJPEG depuis la caméra."""
        from fastapi.responses import StreamingResponse

        async def generate_frames():
            try:
                import cv2
                import numpy as np
            except ImportError:
                logger.warning("OpenCV non disponible pour stream vidéo")
                return

            vision = advanced_websocket_manager.vision
            frame_count = 0
            while True:
                try:
                    frame = None
                    if vision:
                        try:
                            # Utiliser la méthode privée _capture_image_from_camera
                            # qui gère SDK camera et OpenCV
                            frame = vision._capture_image_from_camera()
                        except Exception as e:
                            logger.debug(f"Erreur capture frame: {e}")

                    if frame is None:
                        # Frame de test avec texte si pas de caméra
                        frame = np.zeros((480, 640, 3), dtype=np.uint8)
                        # Ajouter texte "Caméra non disponible"
                        try:
                            cv2.putText(
                                frame,
                                "Camera not available",
                                (50, 240),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1,
                                (255, 255, 255),
                                2,
                            )
                        except (ConnectionError, RuntimeError, WebSocketDisconnect):
                            pass

                    # Encoder en JPEG
                    success, buffer = cv2.imencode(
                        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
                    )
                    if not success:
                        logger.warning("Échec encodage JPEG")
                        await asyncio.sleep(0.1)
                        continue

                    frame_bytes = buffer.tobytes()

                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
                    )

                    frame_count += 1
                    if frame_count % 30 == 0:
                        logger.debug(f"Stream vidéo: {frame_count} frames envoyées")

                    await asyncio.sleep(0.033)  # ~30 FPS
                except Exception as e:
                    logger.error(f"Erreur stream vidéo: {e}", exc_info=True)
                    await asyncio.sleep(1)

        return StreamingResponse(
            generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame"
        )

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        """Endpoint WebSocket pour communication temps réel."""
        await advanced_websocket_manager.connect(websocket)

        try:
            while True:
                # Recevoir messages du client
                data = await websocket.receive_text()
                message = json.loads(data)

                # Traiter commande ou chat
                logger.info(f"📨 [WS] Message reçu, type: {message.get('type')}")
                if message.get("type") == "command":
                    logger.info("🎯 [WS] Traitement commande")
                    await handle_advanced_robot_command(message)
                elif message.get("type") == "chat":
                    logger.info("💬 [WS] Traitement chat")
                    await handle_chat_message(message, websocket)
                else:
                    logger.warning(
                        f"⚠️ [WS] Type de message inconnu: {message.get('type')}"
                    )

        except WebSocketDisconnect:
            logger.info("🔌 WebSocket déconnecté normalement")
            advanced_websocket_manager.disconnect(websocket)
        except Exception as e:
            logger.error(f"❌ Erreur WebSocket: {e}", exc_info=True)
            advanced_websocket_manager.disconnect(websocket)


async def handle_advanced_robot_command(command_data: dict[str, Any]):
    """Traite une commande robot avancée reçue via WebSocket."""
    try:
        command_type = command_data.get("command_type")
        value = command_data.get("value")

        if not advanced_websocket_manager.robot:
            # Initialiser le robot si nécessaire avec lock
            logger.warning(
                "⚠️ Robot non initialisé lors de la commande - initialisation forcée"
            )
            with advanced_websocket_manager._robot_init_lock:
                # Double-check pattern
                if not advanced_websocket_manager.robot:
                    try:
                        logger.info(
                            f"🔧 Initialisation robot {advanced_websocket_manager.robot_backend} (forcé)..."
                        )
                        advanced_websocket_manager.robot = RobotFactory.create_backend(
                            advanced_websocket_manager.robot_backend,
                        )
                        if advanced_websocket_manager.robot:
                            connected = advanced_websocket_manager.robot.connect()
                            if connected:
                                logger.info(
                                    f"✅ Robot {advanced_websocket_manager.robot_backend} connecté (forcé)"
                                )
                                await advanced_websocket_manager.send_log_message(
                                    "info",
                                    f"✅ Robot {advanced_websocket_manager.robot_backend} connecté",
                                )
                            else:
                                logger.warning("⚠️ Robot connect() a retourné False")
                                await advanced_websocket_manager.send_log_message(
                                    "warning",
                                    f"⚠️ Robot {advanced_websocket_manager.robot_backend} en mode simulation",
                                )
                        else:
                            logger.error(
                                "❌ RobotFactory.create_backend a retourné None"
                            )
                            await advanced_websocket_manager.send_log_message(
                                "error",
                                "❌ Impossible de créer le robot",
                            )
                    except Exception as e:
                        logger.error(
                            f"❌ Erreur initialisation robot: {e}", exc_info=True
                        )
                        await advanced_websocket_manager.send_log_message(
                            "error",
                            f"❌ Erreur robot: {e}",
                        )

        if command_type == "emotion":
            # Définir émotion
            emotion = value
            if not emotion or not isinstance(emotion, str):
                await advanced_websocket_manager.send_log_message(
                    "error",
                    "Émotion invalide",
                )
                return

            intensity = 0.8  # Intensité par défaut
            if advanced_websocket_manager.robot:
                # Type narrowing après vérification isinstance
                if not isinstance(emotion, str):
                    raise TypeError(f"Expected emotion to be str, got {type(emotion)}")
                try:
                    logger.info(
                        f"🎭 [CMD] Exécution set_emotion: {emotion} (intensité: {intensity})"
                    )
                    success = advanced_websocket_manager.robot.set_emotion(
                        emotion,
                        intensity,
                    )
                    logger.info(f"🎭 [CMD] set_emotion retourné: {success}")

                    # Faire plusieurs steps pour que le changement soit visible
                    if hasattr(advanced_websocket_manager.robot, "step"):
                        for _ in range(5):
                            try:
                                advanced_websocket_manager.robot.step()
                            except (ConnectionError, RuntimeError, WebSocketDisconnect):
                                pass

                    if success:
                        logger.info(f"✅ [CMD] Émotion {emotion} appliquée avec succès")
                        await advanced_websocket_manager.send_log_message(
                            "info",
                            f"✅ Émotion définie: {emotion} (intensité: {intensity})",
                        )
                    else:
                        logger.warning("⚠️ [CMD] set_emotion a retourné False")
                        await advanced_websocket_manager.send_log_message(
                            "error",
                            f"❌ Échec émotion: {emotion}",
                        )
                except Exception as e:
                    logger.error(f"❌ [CMD] Erreur set_emotion: {e}", exc_info=True)
                    await advanced_websocket_manager.send_log_message(
                        "error",
                        f"❌ Erreur émotion: {e}",
                    )
            else:
                # Mode simulation - émotion simulée
                await advanced_websocket_manager.send_log_message(
                    "info",
                    f"ℹ️ Émotion simulée: {emotion} (robot non initialisé)",
                )

        elif command_type == "action":
            # Exécuter action
            action = value
            if not advanced_websocket_manager.robot:
                await advanced_websocket_manager.send_log_message(
                    "error",
                    "Robot non connecté",
                )
                return

            # Type narrowing après vérification robot
            if advanced_websocket_manager.robot is None:
                raise RuntimeError("Robot is None after check")
            robot = advanced_websocket_manager.robot

            # Initialiser success avant la fonction async
            success = False

            # Exécuter les actions de manière asynchrone pour ne pas bloquer
            async def execute_action():
                nonlocal success
                try:
                    if action == "look_at":
                        success = robot.look_at(0.5, 0.0, 0.0)
                    elif action == "greet":
                        # Exécuter en arrière-plan
                        await asyncio.to_thread(robot.run_behavior, "greeting", 3.0)
                        success = True
                    elif action == "wake_up":
                        await asyncio.to_thread(robot.run_behavior, "wake_up", 2.0)
                        success = True
                    elif action == "sleep":
                        await asyncio.to_thread(robot.run_behavior, "goto_sleep", 2.0)
                        success = True
                    elif action == "nod":
                        await asyncio.to_thread(robot.run_behavior, "nod", 1.0)
                        success = True
                    elif action == "stop":
                        # Arrêter tous les mouvements en cours
                        # Remettre tous les joints à leur position neutre
                        for joint in robot.get_available_joints():
                            try:
                                robot.set_joint_pos(joint, 0.0)
                            except (ConnectionError, RuntimeError, WebSocketDisconnect):
                                pass
                        robot.step()
                        success = True
                    else:
                        success = False
                except Exception as e:
                    logger.error(f"Erreur exécution action {action}: {e}")
                    success = False

            await execute_action()

            # Faire plusieurs steps pour que l'action soit visible
            if advanced_websocket_manager.robot and hasattr(
                advanced_websocket_manager.robot, "step"
            ):
                for _ in range(10):  # 10 steps pour que l'action soit visible
                    try:
                        advanced_websocket_manager.robot.step()
                    except (ConnectionError, RuntimeError, WebSocketDisconnect):
                        pass

            if success:
                await advanced_websocket_manager.send_log_message(
                    "info",
                    f"✅ Action exécutée: {action}",
                )
            else:
                await advanced_websocket_manager.send_log_message(
                    "error",
                    f"❌ Échec action: {action}",
                )

        elif command_type == "behavior":
            # Exécuter comportement
            behavior = value
            if not behavior or not isinstance(behavior, str):
                await advanced_websocket_manager.send_log_message(
                    "error",
                    "Comportement invalide",
                )
                return

            if not advanced_websocket_manager.robot:
                # Mode simulation - comportement simulé
                await advanced_websocket_manager.send_log_message(
                    "info",
                    f"Comportement simulé: {behavior} (mode simulation MuJoCo)",
                )
                return

            # Type narrowing après vérifications
            if not isinstance(behavior, str):
                raise TypeError(f"Expected behavior to be str, got {type(behavior)}")
            if advanced_websocket_manager.robot is None:
                raise RuntimeError("Robot is None after check")
            robot = advanced_websocket_manager.robot

            # Exécuter le comportement de manière asynchrone
            try:
                success = await asyncio.to_thread(robot.run_behavior, behavior, 5.0)
                # Faire plusieurs steps pour que le comportement soit visible
                if hasattr(robot, "step"):
                    for _ in range(
                        50
                    ):  # 50 steps pour que le comportement soit visible
                        try:
                            robot.step()
                        except (ConnectionError, RuntimeError, WebSocketDisconnect):
                            pass
            except (ValueError, RuntimeError, KeyError) as e:
                logger.error(f"Erreur exécution comportement {behavior}: {e}")
                success = False

            if success:
                await advanced_websocket_manager.send_log_message(
                    "info",
                    f"✅ Comportement lancé: {behavior}",
                )
            else:
                await advanced_websocket_manager.send_log_message(
                    "error",
                    f"❌ Échec comportement: {behavior}",
                )

        elif command_type == "joint":
            # Contrôler joint
            joint_data = value
            if not joint_data:
                await advanced_websocket_manager.send_log_message(
                    "error",
                    "Données joint manquantes",
                )
                return

            if not advanced_websocket_manager.robot:
                # Mode simulation - joint simulé
                joint = joint_data.get("joint")
                position = joint_data.get("position", 0.0)
                await advanced_websocket_manager.send_log_message(
                    "info",
                    f"Joint simulé: {joint} = {position} (mode simulation MuJoCo)",
                )
                return

            # Type narrowing après vérifications
            if joint_data is None:
                raise ValueError("joint_data is None after check")
            if advanced_websocket_manager.robot is None:
                raise RuntimeError("Robot is None after check")
            robot = advanced_websocket_manager.robot

            joint = joint_data.get("joint")
            position = joint_data.get("position", 0.0)
            try:
                logger.info(f"🔧 Exécution set_joint_pos: {joint} = {position}")
                success = robot.set_joint_pos(joint, position)
                logger.info(f"🔧 set_joint_pos retourné: {success}")

                # Faire plusieurs steps pour que le joint bouge vraiment
                if hasattr(robot, "step"):
                    for _ in range(5):
                        try:
                            robot.step()
                        except (ConnectionError, RuntimeError, WebSocketDisconnect):
                            pass

                if success:
                    logger.info(f"✅ Joint {joint} = {position:.2f} appliqué")
                    await advanced_websocket_manager.send_log_message(
                        "info",
                        f"✅ Joint {joint} = {position:.2f}",
                    )
                else:
                    logger.warning(f"⚠️ set_joint_pos a retourné False pour {joint}")
                    await advanced_websocket_manager.send_log_message(
                        "error",
                        f"❌ Échec joint {joint}",
                    )
            except Exception as e:
                logger.error(f"❌ Erreur set_joint_pos: {e}", exc_info=True)
                await advanced_websocket_manager.send_log_message(
                    "error",
                    f"❌ Erreur joint {joint}: {e}",
                )

        elif command_type == "vision":
            # Contrôler vision
            vision_action = value
            if vision_action == "toggle":
                advanced_websocket_manager.vision.camera_active = (
                    not advanced_websocket_manager.vision.camera_active
                )
                await advanced_websocket_manager.send_log_message(
                    "info",
                    f"Vision: {'activée' if advanced_websocket_manager.vision.camera_active else 'désactivée'}",
                )
            elif vision_action == "scan":
                try:
                    # Exécuter le scan de manière asynchrone
                    objects = await asyncio.to_thread(
                        advanced_websocket_manager.vision.scan_environment
                    )
                    num_objects = len(objects.get("objects", []))
                    await advanced_websocket_manager.send_log_message(
                        "info",
                        f"Scan: {num_objects} objets détectés",
                    )
                except Exception as e:
                    logger.error(f"Erreur scan environnement: {e}")
                    await advanced_websocket_manager.send_log_message(
                        "error",
                        f"Erreur scan: {e}",
                    )
            elif vision_action == "track":
                advanced_websocket_manager.vision.tracking_active = (
                    not advanced_websocket_manager.vision.tracking_active
                )
                await advanced_websocket_manager.send_log_message(
                    "info",
                    f"Suivi: {'activé' if advanced_websocket_manager.vision.tracking_active else 'désactivé'}",
                )

        # Envoyer mise à jour du statut
        await advanced_websocket_manager.send_complete_status()

    except Exception as e:
        logger.error(f"❌ Erreur commande avancée: {e}")
        await advanced_websocket_manager.send_log_message("error", f"Erreur: {e!s}")


async def handle_chat_message(message_data: dict[str, Any], websocket: WebSocket):
    """Traite un message chat reçu via WebSocket.

    Args:
        message_data: Données du message chat
        websocket: Connexion WebSocket pour réponse

    """
    try:
        logger.info(f"📨 [CHAT] Message reçu: {message_data}")
        user_message = message_data.get("message", "")
        logger.info(f"📨 [CHAT] Texte extrait: '{user_message}'")

        if not user_message:
            logger.warning("Message chat vide reçu")
            await advanced_websocket_manager.send_log_message(
                "error",
                "Message chat vide",
            )
            # Envoyer quand même une réponse d'erreur
            error_response = {
                "type": "chat_response",
                "sender": "bbia",
                "message": "Désolé, votre message est vide. Pouvez-vous réessayer ?",
                "timestamp": datetime.now().isoformat(),
            }
            await websocket.send_text(json.dumps(error_response))
            return

        # Importer BBIAHuggingFace et initialiser une seule fois
        if (
            not hasattr(advanced_websocket_manager, "bbia_hf")
            or advanced_websocket_manager.bbia_hf is None
        ):
            try:
                from bbia_sim.bbia_huggingface import BBIAHuggingFace

                advanced_websocket_manager.bbia_hf = BBIAHuggingFace()
                logger.info("🤗 Module BBIAHuggingFace initialisé pour chat")
            except ImportError as e:
                logger.warning(f"⚠️ Hugging Face non disponible: {e}")
                advanced_websocket_manager.bbia_hf = None
            except Exception as e:
                logger.error(f"❌ Erreur initialisation BBIAHuggingFace: {e}")
                advanced_websocket_manager.bbia_hf = None

        # NE PAS renvoyer le message utilisateur (déjà affiché côté client)
        # Générer directement la réponse BBIA

        # Générer réponse BBIA
        if (
            hasattr(advanced_websocket_manager, "bbia_hf")
            and advanced_websocket_manager.bbia_hf is not None
        ):
            try:
                logger.info(f"🤖 Génération réponse BBIA pour: {user_message[:50]}...")
                bbia_response = advanced_websocket_manager.bbia_hf.chat(user_message)
                logger.info(f"✅ Réponse BBIA générée: {bbia_response[:50]}...")
            except Exception as e:
                logger.error(f"❌ Erreur génération réponse BBIA: {e}")
                bbia_response = f"Désolé, une erreur s'est produite lors de la génération de la réponse: {str(e)}"

            # Envoyer réponse BBIA
            chat_response_bbia = {
                "type": "chat_response",
                "sender": "bbia",
                "message": bbia_response,
                "timestamp": datetime.now().isoformat(),
            }
            logger.info(
                f"📤 [CHAT] Envoi réponse BBIA ({len(bbia_response)} caractères)"
            )
            response_json = json.dumps(chat_response_bbia)
            logger.debug(f"📤 [CHAT] JSON réponse: {response_json[:100]}...")
            await websocket.send_text(response_json)
            logger.info("✅ [CHAT] Réponse envoyée avec succès")

            await advanced_websocket_manager.send_log_message(
                "info",
                f"Chat: {len(user_message)} caractères → réponse BBIA ({len(bbia_response)} caractères)",
            )
        else:
            # Réponse fallback si HF indisponible
            fallback_response = f"Bonjour ! J'ai bien reçu votre message : '{user_message}'. Le module Hugging Face n'est pas disponible actuellement, mais je peux toujours interagir avec vous via les commandes du dashboard."
            chat_response_bbia = {
                "type": "chat_response",
                "sender": "bbia",
                "message": fallback_response,
                "timestamp": datetime.now().isoformat(),
            }
            logger.info("📤 [CHAT] Envoi réponse fallback")
            await websocket.send_text(json.dumps(chat_response_bbia))
            logger.info("✅ [CHAT] Réponse fallback envoyée")
            await advanced_websocket_manager.send_log_message(
                "info",
                f"Chat fallback: {len(user_message)} caractères",
            )

    except Exception as e:
        logger.error(f"❌ Erreur chat: {e}", exc_info=True)
        try:
            await advanced_websocket_manager.send_log_message(
                "error",
                f"Erreur chat: {e!s}",
            )
            # Envoyer une réponse d'erreur au client
            error_response = {
                "type": "chat_response",
                "sender": "bbia",
                "message": f"Désolé, une erreur s'est produite: {str(e)}",
                "timestamp": datetime.now().isoformat(),
            }
            await websocket.send_text(json.dumps(error_response))
        except Exception as e2:
            logger.error(f"❌ Erreur lors de l'envoi du message d'erreur: {e2}")


def run_advanced_dashboard(
    host: str = "127.0.0.1",
    port: int = 8000,
    backend: str = "mujoco",
):
    """Lance le dashboard BBIA avancé.

    Args:
        host: Adresse d'écoute
        port: Port d'écoute
        backend: Backend robot à utiliser

    """
    if not FASTAPI_AVAILABLE:
        logger.error("❌ FastAPI non disponible")
        return

    advanced_websocket_manager.robot_backend = backend

    logger.info(f"🚀 Lancement dashboard BBIA avancé sur {host}:{port}")
    logger.info(f"🔗 URL: http://{host}:{port}")
    logger.info(f"🤖 Backend robot: {backend}")
    logger.info("📊 Métriques temps réel activées")
    logger.info("🎮 Contrôles avancés disponibles")

    if app is None:
        logger.error("❌ Application FastAPI non disponible")
        return
    uvicorn.run(app, host=host, port=port, log_level="info")


# Test rapide
if __name__ == "__main__":
    run_advanced_dashboard()
