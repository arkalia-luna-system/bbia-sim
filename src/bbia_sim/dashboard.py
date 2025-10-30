#!/usr/bin/env python3
"""
bbia_dashboard.py - Dashboard web minimal pour BBIA
Interface web simple avec FastAPI + WebSocket pour contrôler le robot
"""

import asyncio
import json
import logging
from datetime import datetime
from typing import Any

try:
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
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

from bbia_sim.robot_factory import RobotFactory

logger = logging.getLogger(__name__)


class BBIAWebSocketManager:
    """Gestionnaire WebSocket pour le dashboard BBIA."""

    def __init__(self) -> None:
        """Initialise le gestionnaire WebSocket."""
        self.active_connections: list[WebSocket] = []
        self.robot: Any | None = None
        self.robot_backend = "mujoco"

    async def connect(self, websocket: WebSocket):
        """Accepte une nouvelle connexion WebSocket."""
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(
            f"🔌 WebSocket connecté ({len(self.active_connections)} connexions)"
        )

        # Envoyer état initial
        await self.send_status_update()

    def disconnect(self, websocket: WebSocket):
        """Déconnecte un WebSocket."""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(
            f"🔌 WebSocket déconnecté ({len(self.active_connections)} connexions)"
        )

    async def send_personal_message(self, message: str, websocket: WebSocket):
        """Envoie un message à un WebSocket spécifique."""
        try:
            await websocket.send_text(message)
        except Exception as e:
            logger.error(f"❌ Erreur envoi message: {e}")

    async def broadcast(self, message: str):
        """Diffuse un message à tous les WebSockets connectés."""
        if not self.active_connections:
            return

        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.error(f"❌ Erreur broadcast: {e}")
                disconnected.append(connection)

        # Nettoyer les connexions fermées
        for connection in disconnected:
            self.disconnect(connection)

    async def send_status_update(self):
        """Envoie une mise à jour de statut."""
        status = {
            "type": "status",
            "timestamp": datetime.now().isoformat(),
            "robot_connected": self.robot is not None,
            "robot_backend": self.robot_backend,
            "active_connections": len(self.active_connections),
        }
        await self.broadcast(json.dumps(status))

    async def send_log_message(self, level: str, message: str):
        """Envoie un message de log."""
        log_data = {
            "type": "log",
            "timestamp": datetime.now().isoformat(),
            "level": level,
            "message": message,
        }
        await self.broadcast(json.dumps(log_data))


# Instance globale du gestionnaire
websocket_manager = BBIAWebSocketManager()

# Application FastAPI
if FASTAPI_AVAILABLE:
    app = FastAPI(title="BBIA Dashboard", version="1.2.0")
else:
    app = None  # type: ignore


def create_dashboard_app() -> FastAPI | None:
    """Crée l'application dashboard FastAPI."""
    if not FASTAPI_AVAILABLE:
        logger.error("❌ FastAPI non disponible")
        return None

    return app


# HTML du dashboard
DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>BBIA Dashboard</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            min-height: 100vh;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
            background: rgba(255, 255, 255, 0.1);
            border-radius: 15px;
            padding: 30px;
            backdrop-filter: blur(10px);
        }
        h1 {
            text-align: center;
            margin-bottom: 30px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .status {
            background: rgba(255, 255, 255, 0.2);
            padding: 15px;
            border-radius: 10px;
            margin-bottom: 20px;
        }
        .controls {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-bottom: 30px;
        }
        button {
            padding: 15px 20px;
            border: none;
            border-radius: 10px;
            background: rgba(255, 255, 255, 0.2);
            color: white;
            font-size: 16px;
            cursor: pointer;
            transition: all 0.3s ease;
            backdrop-filter: blur(5px);
        }
        button:hover {
            background: rgba(255, 255, 255, 0.3);
            transform: translateY(-2px);
        }
        button:active {
            transform: translateY(0);
        }
        .logs {
            background: rgba(0, 0, 0, 0.3);
            padding: 15px;
            border-radius: 10px;
            height: 300px;
            overflow-y: auto;
            font-family: 'Courier New', monospace;
            font-size: 14px;
        }
        .log-entry {
            margin-bottom: 5px;
            padding: 2px 0;
        }
        .log-info { color: #4CAF50; }
        .log-warning { color: #FF9800; }
        .log-error { color: #F44336; }
        .connection-status {
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            margin-right: 10px;
        }
        .connected { background: #4CAF50; }
        .disconnected { background: #F44336; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 BBIA Dashboard</h1>

        <div class="status">
            <h3>📊 Statut</h3>
            <p><span id="connection-status" class="connection-status disconnected"></span>
               <span id="connection-text">Déconnecté</span></p>
            <p>Robot: <span id="robot-status">Non connecté</span></p>
            <p>Backend: <span id="robot-backend">-</span></p>
        </div>

        <div class="controls">
            <button onclick="sendCommand('emotion', 'happy')">😊 Heureux</button>
            <button onclick="sendCommand('emotion', 'sad')">😢 Triste</button>
            <button onclick="sendCommand('emotion', 'excited')">🤩 Excité</button>
            <button onclick="sendCommand('emotion', 'neutral')">😐 Neutre</button>
            <button onclick="sendCommand('action', 'look_at')">👀 Regarder</button>
            <button onclick="sendCommand('action', 'greet')">👋 Saluer</button>
            <button onclick="sendCommand('action', 'wake_up')">🌅 Réveil</button>
            <button onclick="sendCommand('action', 'stop')">⏹️ Arrêter</button>
        </div>

        <div class="logs">
            <h3>📝 Logs</h3>
            <div id="log-container"></div>
        </div>
    </div>

    <script>
        let ws = null;
        let reconnectInterval = null;

        function connect() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.host}/ws`;

            ws = new WebSocket(wsUrl);

            ws.onopen = function(event) {
                console.log('WebSocket connecté');
                updateConnectionStatus(true);
                clearInterval(reconnectInterval);
            };

            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                handleMessage(data);
            };

            ws.onclose = function(event) {
                console.log('WebSocket fermé');
                updateConnectionStatus(false);
                // Reconnexion automatique
                reconnectInterval = setInterval(connect, 3000);
            };

            ws.onerror = function(error) {
                console.error('Erreur WebSocket:', error);
            };
        }

        function updateConnectionStatus(connected) {
            const statusEl = document.getElementById('connection-status');
            const textEl = document.getElementById('connection-text');

            if (connected) {
                statusEl.className = 'connection-status connected';
                textEl.textContent = 'Connecté';
            } else {
                statusEl.className = 'connection-status disconnected';
                textEl.textContent = 'Déconnecté';
            }
        }

        function handleMessage(data) {
            if (data.type === 'status') {
                document.getElementById('robot-status').textContent =
                    data.robot_connected ? 'Connecté' : 'Non connecté';
                document.getElementById('robot-backend').textContent =
                    data.robot_backend || '-';
            } else if (data.type === 'log') {
                addLogEntry(data.level, data.message);
            }
        }

        function addLogEntry(level, message) {
            const container = document.getElementById('log-container');
            const entry = document.createElement('div');
            entry.className = `log-entry log-${level}`;
            entry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
            container.appendChild(entry);
            container.scrollTop = container.scrollHeight;
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
                addLogEntry('info', `Commande envoyée: ${type} = ${value}`);
            } else {
                addLogEntry('error', 'WebSocket non connecté');
            }
        }

        // Connexion initiale
        connect();
    </script>
</body>
</html>
"""


# Routes FastAPI
if FASTAPI_AVAILABLE:

    @app.get("/", response_class=HTMLResponse)
    async def dashboard():
        """Page principale du dashboard."""
        return DASHBOARD_HTML

    @app.get("/healthz")
    async def health_check():
        """Endpoint de santé pour CI."""
        return {
            "status": "healthy",
            "timestamp": datetime.now().isoformat(),
            "version": "1.2.0",
            "robot_connected": websocket_manager.robot is not None,
        }

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        """Endpoint WebSocket pour communication temps réel."""
        await websocket_manager.connect(websocket)

        try:
            while True:
                # Recevoir messages du client
                data = await websocket.receive_text()
                message = json.loads(data)

                # Traiter commande
                if message.get("type") == "command":
                    await handle_robot_command(message)

        except WebSocketDisconnect:
            websocket_manager.disconnect(websocket)
        except Exception as e:
            logger.error(f"❌ Erreur WebSocket: {e}")
            websocket_manager.disconnect(websocket)


async def handle_robot_command(command_data: dict[str, Any]):
    """Traite une commande robot reçue via WebSocket."""
    try:
        command_type = command_data.get("command_type")
        value = command_data.get("value")

        logger.info(f"🎯 Commande reçue: {command_type} = {value}")

        # Initialiser robot si nécessaire
        if not websocket_manager.robot:
            websocket_manager.robot = RobotFactory.create_backend(
                websocket_manager.robot_backend
            )
            if websocket_manager.robot:
                websocket_manager.robot.connect()
                await websocket_manager.send_log_message(
                    "info", f"Robot {websocket_manager.robot_backend} connecté"
                )

        # Exécuter commande
        if websocket_manager.robot:
            if command_type == "emotion":
                websocket_manager.robot.set_emotion(str(value), intensity=0.8)
                await websocket_manager.send_log_message(
                    "info", f"Émotion définie: {value}"
                )

            elif command_type == "action":
                if value == "look_at":
                    websocket_manager.robot.set_joint_pos("yaw_body", 0.2)
                    await asyncio.sleep(1)
                    websocket_manager.robot.set_joint_pos("yaw_body", -0.2)
                    await asyncio.sleep(1)
                    websocket_manager.robot.set_joint_pos("yaw_body", 0.0)
                    await websocket_manager.send_log_message("info", "Action: regarder")

                elif value == "greet":
                    websocket_manager.robot.set_emotion("happy", intensity=0.9)
                    await asyncio.sleep(2)
                    websocket_manager.robot.set_emotion("neutral", intensity=0.5)
                    await websocket_manager.send_log_message("info", "Action: saluer")

                elif value == "wake_up":
                    websocket_manager.robot.set_emotion("excited", intensity=0.7)
                    await asyncio.sleep(1)
                    websocket_manager.robot.set_joint_pos("yaw_body", 0.1)
                    await asyncio.sleep(0.5)
                    websocket_manager.robot.set_joint_pos("yaw_body", -0.1)
                    await asyncio.sleep(0.5)
                    websocket_manager.robot.set_joint_pos("yaw_body", 0.0)
                    websocket_manager.robot.set_emotion("happy", intensity=0.8)
                    await websocket_manager.send_log_message("info", "Action: réveil")

                elif value == "stop":
                    websocket_manager.robot.set_emotion("neutral", intensity=0.5)
                    websocket_manager.robot.set_joint_pos("yaw_body", 0.0)
                    await websocket_manager.send_log_message("info", "Action: arrêt")

            # Mise à jour statut
            await websocket_manager.send_status_update()

    except Exception as e:
        logger.error(f"❌ Erreur commande robot: {e}")
        await websocket_manager.send_log_message("error", f"Erreur: {str(e)}")


def run_dashboard(host: str = "127.0.0.1", port: int = 8000, backend: str = "mujoco"):
    """
    Lance le dashboard BBIA.

    Args:
        host: Adresse d'écoute
        port: Port d'écoute
        backend: Backend robot à utiliser
    """
    if not FASTAPI_AVAILABLE:
        logger.error("❌ FastAPI non disponible")
        return

    websocket_manager.robot_backend = backend

    logger.info(f"🚀 Lancement dashboard BBIA sur {host}:{port}")
    logger.info(f"🔗 URL: http://{host}:{port}")
    logger.info(f"🤖 Backend robot: {backend}")

    uvicorn.run(app, host=host, port=port, log_level="info")


# Test rapide
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("🧪 Test Dashboard BBIA")
    print("=" * 40)

    print(f"FastAPI disponible: {FASTAPI_AVAILABLE}")

    if FASTAPI_AVAILABLE:
        print("✅ Dashboard prêt")
        print("🚀 Pour lancer: python scripts/bbia_dashboard.py")
    else:
        print("❌ FastAPI requis pour le dashboard")
