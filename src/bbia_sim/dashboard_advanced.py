#!/usr/bin/env python3
"""
BBIA Advanced Dashboard - Interface de contrôle sophistiquée
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
    from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
    from fastapi.middleware.cors import CORSMiddleware
    from fastapi.responses import HTMLResponse

    FASTAPI_AVAILABLE = True
except ImportError:
    FASTAPI_AVAILABLE = False
    FastAPI = None  # type: ignore
    WebSocket = None  # type: ignore

# Ajouter le chemin src pour les imports
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# BBIAVoice n'existe pas encore - utiliser les fonctions directement
from bbia_sim.bbia_behavior import BBIABehaviorManager
from bbia_sim.bbia_emotions import BBIAEmotions
from bbia_sim.bbia_vision import BBIAVision
from bbia_sim.robot_factory import RobotFactory

logger = logging.getLogger(__name__)


class BBIAAdvancedWebSocketManager:
    """Gestionnaire WebSocket avancé pour le dashboard BBIA."""

    def __init__(self) -> None:
        """Initialise le gestionnaire WebSocket avancé."""
        self.active_connections: list[WebSocket] = []
        self.robot: Any | None = None
        self.robot_backend = "mujoco"
        self.metrics_history: list[dict[str, Any]] = []
        self.max_history = 1000  # Limite historique métriques

        # Modules BBIA
        self.emotions = BBIAEmotions()
        self.vision = BBIAVision()
        # self.voice = BBIAVoice()  # Pas encore implémenté
        self.behavior_manager = BBIABehaviorManager()
        self.bbia_hf: Any | None = None  # Hugging Face chat module

        # Métriques temps réel
        self.current_metrics = {
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

    async def connect(self, websocket: WebSocket):
        """Accepte une nouvelle connexion WebSocket."""
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(
            f"🔌 WebSocket avancé connecté ({len(self.active_connections)} connexions)"
        )

        # Démarrer la collecte de métriques si c'est la première connexion
        if len(self.active_connections) == 1:
            self._start_metrics_collection()

        # Envoyer état initial complet
        await self.send_complete_status()

    def disconnect(self, websocket: WebSocket):
        """Déconnecte un WebSocket."""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(
            f"🔌 WebSocket avancé déconnecté ({len(self.active_connections)} connexions)"
        )

    async def broadcast(self, message: str):
        """Diffuse un message à toutes les connexions actives."""
        if not self.active_connections:
            return

        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception:
                disconnected.append(connection)

        # Nettoyer les connexions fermées
        for connection in disconnected:
            self.disconnect(connection)

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
            "history": self.metrics_history[-50:],  # 50 dernières métriques
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
            except Exception:
                pose[joint] = 0.0
        return pose

    def _start_metrics_collection(self):
        """Démarre la collecte automatique de métriques."""

        async def collect_metrics():
            while True:
                try:
                    # Mettre à jour les métriques
                    self._update_metrics()

                    # Ajouter à l'historique
                    self.metrics_history.append(self.current_metrics.copy())
                    if len(self.metrics_history) > self.max_history:
                        self.metrics_history.pop(0)

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
                "latency_ms", 0.0
            )
            self.current_metrics["performance"]["fps"] = telemetry.get("fps", 0.0)

        # Métriques BBIA
        self.current_metrics["current_emotion"] = self.emotions.current_emotion
        self.current_metrics["emotion_intensity"] = self.emotions.emotion_intensity

        self.current_metrics["vision"]["objects_detected"] = len(
            self.vision.objects_detected
        )
        self.current_metrics["vision"]["faces_detected"] = len(
            self.vision.faces_detected
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
    app = None  # type: ignore


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

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            min-height: 100vh;
            overflow-x: hidden;
        }

        .header {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            padding: 20px;
            text-align: center;
            border-bottom: 1px solid rgba(255, 255, 255, 0.2);
        }

        .header h1 {
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
            margin-bottom: 10px;
        }

        .header .subtitle {
            font-size: 1.2em;
            opacity: 0.8;
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
            background: rgba(255, 255, 255, 0.1);
            border-radius: 15px;
            padding: 25px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.2);
        }

        .panel h3 {
            margin-bottom: 20px;
            font-size: 1.4em;
            text-align: center;
            text-shadow: 1px 1px 2px rgba(0,0,0,0.3);
        }

        .status-panel {
            grid-column: 1 / -1;
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
        }

        .status-item {
            background: rgba(255, 255, 255, 0.1);
            padding: 15px;
            border-radius: 10px;
            text-align: center;
        }

        .status-value {
            font-size: 2em;
            font-weight: bold;
            margin-bottom: 5px;
        }

        .status-label {
            font-size: 0.9em;
            opacity: 0.8;
        }

        .connection-indicator {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 10px;
            animation: pulse 2s infinite;
        }

        .connected { background: #4CAF50; }
        .disconnected { background: #F44336; }

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
            padding: 15px 10px;
            border: none;
            border-radius: 10px;
            background: rgba(255, 255, 255, 0.2);
            color: white;
            font-size: 14px;
            cursor: pointer;
            transition: all 0.3s ease;
            backdrop-filter: blur(5px);
            text-align: center;
        }

        .control-button:hover {
            background: rgba(255, 255, 255, 0.3);
            transform: translateY(-2px);
        }

        .control-button:active {
            transform: translateY(0);
        }

        .emotion-button {
            background: linear-gradient(45deg, #ff6b6b, #ee5a24);
        }

        .action-button {
            background: linear-gradient(45deg, #4ecdc4, #44a08d);
        }

        .behavior-button {
            background: linear-gradient(45deg, #a8edea, #fed6e3);
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
            background: rgba(255, 255, 255, 0.1);
            padding: 15px;
            border-radius: 10px;
        }

        .joint-name {
            font-weight: bold;
            margin-bottom: 10px;
            text-align: center;
        }

        .joint-slider {
            width: 100%;
            margin-bottom: 10px;
        }

        .joint-value {
            text-align: center;
            font-size: 0.9em;
            opacity: 0.8;
        }

        .logs-container {
            height: 400px;
            overflow-y: auto;
            background: rgba(0, 0, 0, 0.3);
            border-radius: 10px;
            padding: 15px;
            font-family: 'Courier New', monospace;
            font-size: 13px;
        }

        .log-entry {
            margin-bottom: 8px;
            padding: 5px;
            border-radius: 5px;
            background: rgba(255, 255, 255, 0.05);
        }

        .log-info { color: #4CAF50; }
        .log-warning { color: #FF9800; }
        .log-error { color: #F44336; }
        .log-debug { color: #2196F3; }

        .metrics-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-top: 20px;
        }

        .metric-item {
            background: rgba(255, 255, 255, 0.1);
            padding: 15px;
            border-radius: 10px;
            text-align: center;
        }

        .metric-value {
            font-size: 1.5em;
            font-weight: bold;
            margin-bottom: 5px;
        }

        .metric-label {
            font-size: 0.8em;
            opacity: 0.8;
        }

        .vision-panel {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }

        .vision-item {
            background: rgba(255, 255, 255, 0.1);
            padding: 15px;
            border-radius: 10px;
            text-align: center;
        }

        .vision-count {
            font-size: 2em;
            font-weight: bold;
            margin-bottom: 5px;
        }

        .vision-label {
            font-size: 0.9em;
            opacity: 0.8;
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
            background: rgba(0, 0, 0, 0.3);
            border-radius: 10px;
            padding: 15px;
            overflow-y: auto;
            flex: 1;
            margin-bottom: 15px;
            font-family: 'Courier New', monospace;
            font-size: 13px;
        }

        .chat-message {
            background: rgba(255, 255, 255, 0.1);
            padding: 10px;
            border-radius: 8px;
            margin-bottom: 10px;
        }

        .chat-message.chat-user {
            background: rgba(78, 205, 196, 0.3);
            text-align: right;
        }

        .chat-message.chat-bbia {
            background: rgba(255, 107, 107, 0.3);
            text-align: left;
        }

        .chat-author {
            font-weight: bold;
            font-size: 11px;
            opacity: 0.7;
            margin-bottom: 5px;
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
            padding: 12px;
            border: none;
            border-radius: 10px;
            background: rgba(255, 255, 255, 0.1);
            color: white;
            font-size: 14px;
        }

        .chat-input-group input::placeholder {
            color: rgba(255, 255, 255, 0.5);
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 BBIA Advanced Dashboard</h1>
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
            <h3>🎮 Contrôles</h3>

            <div class="controls-grid">
                <button class="control-button emotion-button" onclick="setEmotion('happy')">😊 Heureux</button>
                <button class="control-button emotion-button" onclick="setEmotion('sad')">😢 Triste</button>
                <button class="control-button emotion-button" onclick="setEmotion('excited')">🤩 Excité</button>
                <button class="control-button emotion-button" onclick="setEmotion('angry')">😠 Colère</button>
                <button class="control-button emotion-button" onclick="setEmotion('neutral')">😐 Neutre</button>
                <button class="control-button emotion-button" onclick="setEmotion('curious')">🤔 Curieux</button>
            </div>

            <div class="controls-grid">
                <button class="control-button action-button" onclick="sendAction('look_at')">👀 Regarder</button>
                <button class="control-button action-button" onclick="sendAction('greet')">👋 Saluer</button>
                <button class="control-button action-button" onclick="sendAction('wake_up')">🌅 Réveil</button>
                <button class="control-button action-button" onclick="sendAction('sleep')">😴 Dormir</button>
                <button class="control-button action-button" onclick="sendAction('nod')">👍 Hocher</button>
                <button class="control-button action-button" onclick="sendAction('stop')">⏹️ Arrêter</button>
            </div>

            <div class="controls-grid">
                <button class="control-button behavior-button" onclick="runBehavior('greeting')">👋 Salutation</button>
                <button class="control-button behavior-button" onclick="runBehavior('exploration')">🔍 Exploration</button>
                <button class="control-button behavior-button" onclick="runBehavior('interaction')">💬 Interaction</button>
                <button class="control-button behavior-button" onclick="runBehavior('demo')">🎪 Démo</button>
            </div>
        </div>

        <!-- Panel de métriques -->
        <div class="panel">
            <h3>📊 Métriques Temps Réel</h3>
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
            <h3>🔧 Contrôle des Joints</h3>
            <div class="joint-controls" id="joint-controls">
                <!-- Les contrôles de joints seront générés dynamiquement -->
            </div>
        </div>

        <!-- Panel de vision -->
        <div class="panel">
            <h3>👁️ Vision</h3>
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
                <button class="control-button" onclick="toggleVision()">👁️ Basculer Vision</button>
                <button class="control-button" onclick="scanEnvironment()">🔍 Scanner</button>
                <button class="control-button" onclick="trackObject()">🎯 Suivre Objet</button>
            </div>
        </div>

        <!-- Panel Chat BBIA -->
        <div class="panel">
            <h3>💬 Chat avec BBIA</h3>
            <div class="chat-container">
                <div class="chat-messages" id="chat-messages"></div>
                <div class="chat-input-group">
                    <input type="text" id="chat-input" placeholder="Tapez votre message..."
                           onkeypress="if(event.key==='Enter') sendChatMessage()">
                    <button class="control-button" onclick="sendChatMessage()">Envoyer</button>
                </div>
            </div>
        </div>

        <!-- Panel de logs -->
        <div class="panel" style="grid-column: 1 / -1;">
            <h3>📝 Logs Temps Réel</h3>
            <div class="logs-container" id="logs-container">
                <!-- Les logs seront ajoutés dynamiquement -->
            </div>
        </div>
    </div>

    <script>
        // Variables globales
        let ws = null;
        let reconnectInterval = null;
        let metricsChart = null;
        let metricsData = {
            labels: [],
            latency: [],
            fps: [],
            cpu: [],
            memory: []
        };

        // Initialisation
        document.addEventListener('DOMContentLoaded', function() {
            initializeChart();
            connect();
        });

        // Connexion WebSocket
        function connect() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.host}/ws`;

            ws = new WebSocket(wsUrl);

            ws.onopen = function(event) {
                console.log('WebSocket connecté');
                updateConnectionStatus(true);
                clearInterval(reconnectInterval);
                addLog('info', 'Connexion WebSocket établie');
            };

            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                handleMessage(data);
            };

            ws.onclose = function(event) {
                console.log('WebSocket fermé');
                updateConnectionStatus(false);
                addLog('error', 'Connexion WebSocket fermée');
                reconnectInterval = setInterval(connect, 3000);
            };

            ws.onerror = function(error) {
                console.error('Erreur WebSocket:', error);
                addLog('error', 'Erreur WebSocket');
            };
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
                    // Réponse chat de BBIA ou confirmation message user
                    addChatMessage(data.sender, data.message);
                    break;
            }
        }

        // Mise à jour du statut complet
        function updateCompleteStatus(data) {
            // Statut robot
            document.getElementById('robot-status').textContent =
                data.robot.connected ? '✅' : '❌';
            document.getElementById('robot-backend').textContent =
                data.robot.backend || '-';

            // Statut émotion
            document.getElementById('emotion-status').textContent =
                getEmotionEmoji(data.bbia_modules.emotions.current);

            // Joints disponibles
            updateJointControls(data.robot.joints);

            // Vision
            document.getElementById('objects-count').textContent =
                data.bbia_modules.vision.objects;
            document.getElementById('faces-count').textContent =
                data.bbia_modules.vision.faces;
        }

        // Mise à jour des métriques
        function updateMetrics(metrics) {
            // Statut temps réel
            document.getElementById('latency-status').textContent =
                Math.round(metrics.performance.latency_ms) + 'ms';
            document.getElementById('fps-status').textContent =
                Math.round(metrics.performance.fps);
            document.getElementById('objects-status').textContent =
                metrics.vision.objects_detected;
            document.getElementById('faces-status').textContent =
                metrics.vision.faces_detected;

            // Métriques détaillées
            document.getElementById('cpu-metric').textContent =
                Math.round(metrics.performance.cpu_usage) + '%';
            document.getElementById('memory-metric').textContent =
                Math.round(metrics.performance.memory_usage) + '%';
            document.getElementById('volume-metric').textContent =
                Math.round(metrics.audio.volume_level * 100) + '%';
            document.getElementById('intensity-metric').textContent =
                Math.round(metrics.emotion_intensity * 100) + '%';

            // Mise à jour du graphique
            updateChart(metrics);
        }

        // Initialisation du graphique
        function initializeChart() {
            const ctx = document.getElementById('metricsChart').getContext('2d');
            metricsChart = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: metricsData.labels,
                    datasets: [
                        {
                            label: 'Latence (ms)',
                            data: metricsData.latency,
                            borderColor: '#ff6b6b',
                            backgroundColor: 'rgba(255, 107, 107, 0.1)',
                            tension: 0.4
                        },
                        {
                            label: 'FPS',
                            data: metricsData.fps,
                            borderColor: '#4ecdc4',
                            backgroundColor: 'rgba(78, 205, 196, 0.1)',
                            tension: 0.4
                        },
                        {
                            label: 'CPU (%)',
                            data: metricsData.cpu,
                            borderColor: '#ffa726',
                            backgroundColor: 'rgba(255, 167, 38, 0.1)',
                            tension: 0.4
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
                                color: 'rgba(255, 255, 255, 0.1)'
                            },
                            ticks: {
                                color: 'white'
                            }
                        },
                        x: {
                            grid: {
                                color: 'rgba(255, 255, 255, 0.1)'
                            },
                            ticks: {
                                color: 'white'
                            }
                        }
                    },
                    plugins: {
                        legend: {
                            labels: {
                                color: 'white'
                            }
                        }
                    }
                }
            });
        }

        // Mise à jour du graphique
        function updateChart(metrics) {
            const now = new Date().toLocaleTimeString();

            // Ajouter nouvelles données
            metricsData.labels.push(now);
            metricsData.latency.push(metrics.performance.latency_ms);
            metricsData.fps.push(metrics.performance.fps);
            metricsData.cpu.push(metrics.performance.cpu_usage);
            metricsData.memory.push(metrics.performance.memory_usage);

            // Limiter à 20 points
            if (metricsData.labels.length > 20) {
                metricsData.labels.shift();
                metricsData.latency.shift();
                metricsData.fps.shift();
                metricsData.cpu.shift();
                metricsData.memory.shift();
            }

            // Mettre à jour le graphique
            metricsChart.update('none');
        }

        // Mise à jour des contrôles de joints
        function updateJointControls(joints) {
            const container = document.getElementById('joint-controls');
            container.innerHTML = '';

            joints.forEach(joint => {
                const control = document.createElement('div');
                control.className = 'joint-control';
                control.innerHTML = `
                    <div class="joint-name">${joint}</div>
                    <input type="range" class="joint-slider" id="slider-${joint}"
                           min="-3.14" max="3.14" step="0.01" value="0"
                           onchange="setJointPosition('${joint}', this.value)">
                    <div class="joint-value" id="value-${joint}">0.00</div>
                `;
                container.appendChild(control);
            });
        }

        // Fonctions de contrôle
        function setEmotion(emotion) {
            sendCommand('emotion', emotion);
            addLog('info', `Émotion définie: ${emotion}`);
        }

        function sendAction(action) {
            sendCommand('action', action);
            addLog('info', `Action envoyée: ${action}`);
        }

        function runBehavior(behavior) {
            sendCommand('behavior', behavior);
            addLog('info', `Comportement lancé: ${behavior}`);
        }

        function setJointPosition(joint, position) {
            sendCommand('joint', { joint: joint, position: parseFloat(position) });
            document.getElementById(`value-${joint}`).textContent = parseFloat(position).toFixed(2);
        }

        function toggleVision() {
            sendCommand('vision', 'toggle');
            addLog('info', 'Vision basculée');
        }

        function scanEnvironment() {
            sendCommand('vision', 'scan');
            addLog('info', 'Scan environnement lancé');
        }

        function trackObject() {
            sendCommand('vision', 'track');
            addLog('info', 'Suivi objet activé');
        }

        // Fonctions utilitaires
        function updateConnectionStatus(connected) {
            const indicator = document.getElementById('connection-indicator');
            const text = document.getElementById('connection-text');

            if (connected) {
                indicator.className = 'connection-indicator connected';
                text.textContent = 'Connecté';
            } else {
                indicator.className = 'connection-indicator disconnected';
                text.textContent = 'Déconnecté';
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
            const entry = document.createElement('div');
            entry.className = `log-entry log-${level}`;
            entry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
            container.appendChild(entry);
            container.scrollTop = container.scrollHeight;

            // Limiter à 100 entrées
            while (container.children.length > 100) {
                container.removeChild(container.firstChild);
            }
        }

        function sendCommand(type, value) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                const command = {
                    type: 'command',
                    command_type: type,
                    value: value,
                    timestamp: new Date().toISOString()
                };
                ws.send(JSON.stringify(command));
            } else {
                addLog('error', 'WebSocket non connecté');
            }
        }

        // Fonctions Chat BBIA
        function sendChatMessage() {
            const input = document.getElementById('chat-input');
            if (!input) return;
            const message = input.value.trim();
            if (message && ws && ws.readyState === WebSocket.OPEN) {
                const command = {
                    type: 'chat',
                    message: message,
                    timestamp: new Date().toISOString()
                };
                ws.send(JSON.stringify(command));
                addChatMessage('user', message);
                input.value = '';
            } else {
                addLog('warning', 'Impossible d\'envoyer le message');
            }
        }

        function addChatMessage(sender, message) {
            const container = document.getElementById('chat-messages');
            if (!container) return;

            const entry = document.createElement('div');
            entry.className = `chat-message chat-${sender}`;
            entry.innerHTML = `
                <div class="chat-author">${sender === 'user' ? 'Vous' : 'BBIA'}</div>
                <div class="chat-text">${message}</div>
            `;
            container.appendChild(entry);
            container.scrollTop = container.scrollHeight;

            // Limiter à 50 messages
            while (container.children.length > 50) {
                container.removeChild(container.firstChild);
            }
        }
    </script>
</body>
</html>
"""


# Routes FastAPI avancées
if FASTAPI_AVAILABLE:

    @app.get("/", response_class=HTMLResponse)
    async def advanced_dashboard():
        """Page principale du dashboard avancé."""
        return ADVANCED_DASHBOARD_HTML

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
            "history": advanced_websocket_manager.metrics_history[
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
    async def set_emotion(emotion_data: dict):
        """API endpoint pour définir une émotion."""
        emotion = emotion_data.get("emotion", "neutral")
        intensity = emotion_data.get("intensity", 0.5)

        if advanced_websocket_manager.robot:
            success = await advanced_websocket_manager.robot.set_emotion(
                emotion, intensity
            )
            if success:
                await advanced_websocket_manager.send_log_message(
                    "info", f"Émotion définie: {emotion} (intensité: {intensity})"
                )
                return {"success": True, "emotion": emotion, "intensity": intensity}
            else:
                await advanced_websocket_manager.send_log_message(
                    "error", f"Échec définition émotion: {emotion}"
                )
                return {"success": False, "error": "Failed to set emotion"}

        return {"success": False, "error": "Robot not connected"}

    @app.post("/api/joint")
    async def set_joint_position(joint_data: dict):
        """API endpoint pour définir la position d'un joint."""
        joint = joint_data.get("joint")
        position = joint_data.get("position", 0.0)

        if not joint:
            raise HTTPException(status_code=400, detail="Joint name required")

        if advanced_websocket_manager.robot:
            success = advanced_websocket_manager.robot.set_joint_pos(joint, position)
            if success:
                await advanced_websocket_manager.send_log_message(
                    "info", f"Joint {joint} = {position}"
                )
                return {"success": True, "joint": joint, "position": position}
            else:
                await advanced_websocket_manager.send_log_message(
                    "error", f"Échec contrôle joint {joint}"
                )
                return {"success": False, "error": "Failed to set joint position"}

        return {"success": False, "error": "Robot not connected"}

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
                if message.get("type") == "command":
                    await handle_advanced_robot_command(message)
                elif message.get("type") == "chat":
                    await handle_chat_message(message, websocket)

        except WebSocketDisconnect:
            advanced_websocket_manager.disconnect(websocket)
        except Exception as e:
            logger.error(f"❌ Erreur WebSocket avancé: {e}")
            advanced_websocket_manager.disconnect(websocket)


async def handle_advanced_robot_command(command_data: dict[str, Any]):
    """Traite une commande robot avancée reçue via WebSocket."""
    try:
        command_type = command_data.get("command_type")
        value = command_data.get("value")

        if not advanced_websocket_manager.robot:
            # Initialiser le robot si nécessaire
            advanced_websocket_manager.robot = RobotFactory.create_backend(
                advanced_websocket_manager.robot_backend
            )
            if advanced_websocket_manager.robot:
                connected = advanced_websocket_manager.robot.connect()
                if connected:
                    await advanced_websocket_manager.send_log_message(
                        "info",
                        f"Robot {advanced_websocket_manager.robot_backend} connecté",
                    )
                else:
                    await advanced_websocket_manager.send_log_message(
                        "warning",
                        f"Robot {advanced_websocket_manager.robot_backend} en mode simulation",
                    )

        if command_type == "emotion":
            # Définir émotion
            emotion = value
            if not emotion or not isinstance(emotion, str):
                await advanced_websocket_manager.send_log_message(
                    "error", "Émotion invalide"
                )
                return

            intensity = 0.8  # Intensité par défaut
            if advanced_websocket_manager.robot:
                # Type narrowing après vérification isinstance
                assert isinstance(
                    emotion, str
                )  # noqa: B101 - Type narrowing après vérification isinstance
                success = advanced_websocket_manager.robot.set_emotion(
                    emotion, intensity
                )
                if success:
                    await advanced_websocket_manager.send_log_message(
                        "info", f"Émotion définie: {emotion}"
                    )
                else:
                    await advanced_websocket_manager.send_log_message(
                        "error", f"Échec émotion: {emotion}"
                    )
            else:
                await advanced_websocket_manager.send_log_message(
                    "error", "Robot non connecté"
                )

        elif command_type == "action":
            # Exécuter action
            action = value
            if not advanced_websocket_manager.robot:
                await advanced_websocket_manager.send_log_message(
                    "error", "Robot non connecté"
                )
                return

            # Type narrowing après vérification robot
            assert (
                advanced_websocket_manager.robot is not None
            )  # noqa: B101 - Type narrowing après vérification
            robot = advanced_websocket_manager.robot

            if action == "look_at":
                success = robot.look_at(0.5, 0.0, 0.0)
            elif action == "greet":
                success = robot.run_behavior("greeting", 3.0)
            elif action == "wake_up":
                success = robot.run_behavior("wake_up", 2.0)
            elif action == "sleep":
                success = robot.run_behavior("goto_sleep", 2.0)
            elif action == "nod":
                success = robot.run_behavior("nod", 1.0)
            elif action == "stop":
                success = True  # Arrêt immédiat
            else:
                success = False

            if success:
                await advanced_websocket_manager.send_log_message(
                    "info", f"Action exécutée: {action}"
                )
            else:
                await advanced_websocket_manager.send_log_message(
                    "error", f"Échec action: {action}"
                )

        elif command_type == "behavior":
            # Exécuter comportement
            behavior = value
            if not behavior or not isinstance(behavior, str):
                await advanced_websocket_manager.send_log_message(
                    "error", "Comportement invalide"
                )
                return

            if not advanced_websocket_manager.robot:
                await advanced_websocket_manager.send_log_message(
                    "error", "Robot non connecté"
                )
                return

            # Type narrowing après vérifications
            assert isinstance(
                behavior, str
            )  # noqa: B101 - Type narrowing après vérification isinstance
            assert (
                advanced_websocket_manager.robot is not None
            )  # noqa: B101 - Type narrowing après vérification
            robot = advanced_websocket_manager.robot

            success = robot.run_behavior(behavior, 5.0)
            if success:
                await advanced_websocket_manager.send_log_message(
                    "info", f"Comportement lancé: {behavior}"
                )
            else:
                await advanced_websocket_manager.send_log_message(
                    "error", f"Échec comportement: {behavior}"
                )

        elif command_type == "joint":
            # Contrôler joint
            joint_data = value
            if not joint_data:
                await advanced_websocket_manager.send_log_message(
                    "error", "Données joint manquantes"
                )
                return

            if not advanced_websocket_manager.robot:
                await advanced_websocket_manager.send_log_message(
                    "error", "Robot non connecté"
                )
                return

            # Type narrowing après vérifications
            assert (
                joint_data is not None
            )  # noqa: B101 - Type narrowing après vérification
            assert (
                advanced_websocket_manager.robot is not None
            )  # noqa: B101 - Type narrowing après vérification
            robot = advanced_websocket_manager.robot

            joint = joint_data.get("joint")
            position = joint_data.get("position", 0.0)
            success = robot.set_joint_pos(joint, position)
            if success:
                await advanced_websocket_manager.send_log_message(
                    "info", f"Joint {joint} = {position}"
                )
            else:
                await advanced_websocket_manager.send_log_message(
                    "error", f"Échec joint {joint}"
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
                objects = advanced_websocket_manager.vision.scan_environment()
                await advanced_websocket_manager.send_log_message(
                    "info", f"Scan: {len(objects.get('objects', []))} objets détectés"
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
        await advanced_websocket_manager.send_log_message("error", f"Erreur: {str(e)}")


async def handle_chat_message(message_data: dict[str, Any], websocket: WebSocket):
    """Traite un message chat reçu via WebSocket.

    Args:
        message_data: Données du message chat
        websocket: Connexion WebSocket pour réponse
    """
    try:
        user_message = message_data.get("message", "")
        timestamp = message_data.get("timestamp", "")

        if not user_message:
            await advanced_websocket_manager.send_log_message(
                "error", "Message chat vide"
            )
            return

        # Importer BBIAHuggingFace et initialiser une seule fois
        if not hasattr(advanced_websocket_manager, "bbia_hf"):
            try:
                from bbia_sim.bbia_huggingface import BBIAHuggingFace

                advanced_websocket_manager.bbia_hf = BBIAHuggingFace()
                logger.info("🤗 Module BBIAHuggingFace initialisé pour chat")
            except ImportError:
                logger.warning("⚠️ Hugging Face non disponible, chat limité")
                advanced_websocket_manager.bbia_hf = None

        # Envoyer message utilisateur
        chat_response = {
            "type": "chat_response",
            "sender": "user",
            "message": user_message,
            "timestamp": timestamp,
        }
        await websocket.send_text(json.dumps(chat_response))

        # Générer réponse BBIA
        if (
            hasattr(advanced_websocket_manager, "bbia_hf")
            and advanced_websocket_manager.bbia_hf
        ):
            bbia_response = advanced_websocket_manager.bbia_hf.chat(user_message)

            # Envoyer réponse BBIA
            chat_response_bbia = {
                "type": "chat_response",
                "sender": "bbia",
                "message": bbia_response,
                "timestamp": datetime.now().isoformat(),
            }
            await websocket.send_text(json.dumps(chat_response_bbia))

            await advanced_websocket_manager.send_log_message(
                "info", f"Chat: {len(user_message)} caractères → réponse BBIA"
            )
        else:
            # Réponse fallback si HF indisponible
            fallback_response = f"Je comprends: {user_message}. Hugging Face non disponible pour réponse intelligente."
            chat_response_bbia = {
                "type": "chat_response",
                "sender": "bbia",
                "message": fallback_response,
                "timestamp": datetime.now().isoformat(),
            }
            await websocket.send_text(json.dumps(chat_response_bbia))

    except Exception as e:
        logger.error(f"❌ Erreur chat: {e}")
        await advanced_websocket_manager.send_log_message(
            "error", f"Erreur chat: {str(e)}"
        )


def run_advanced_dashboard(
    host: str = "127.0.0.1", port: int = 8000, backend: str = "mujoco"
):
    """
    Lance le dashboard BBIA avancé.

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

    uvicorn.run(app, host=host, port=port, log_level="info")


# Test rapide
if __name__ == "__main__":
    run_advanced_dashboard()
