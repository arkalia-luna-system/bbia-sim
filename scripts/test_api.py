#!/usr/bin/env python3
"""Script de test pour l'API BBIA-SIM."""

import requests
import json
import time
import websockets
import asyncio
from typing import Dict, Any

API_BASE = "http://localhost:8000"
WS_BASE = "ws://localhost:8000"
API_TOKEN = "bbia-secret-key-dev"

def test_rest_endpoints():
    """Test des endpoints REST."""
    print("🧪 Test des endpoints REST...")
    
    headers = {"Authorization": f"Bearer {API_TOKEN}"}
    
    # Test endpoint racine
    try:
        response = requests.get(f"{API_BASE}/")
        print(f"✅ GET / : {response.status_code}")
        print(f"   Response: {response.json()['message']}")
    except Exception as e:
        print(f"❌ GET / : {e}")
    
    # Test health check
    try:
        response = requests.get(f"{API_BASE}/health")
        print(f"✅ GET /health : {response.status_code}")
        print(f"   Status: {response.json()['status']}")
    except Exception as e:
        print(f"❌ GET /health : {e}")
    
    # Test état du robot
    try:
        response = requests.get(f"{API_BASE}/api/state/full", headers=headers)
        print(f"✅ GET /api/state/full : {response.status_code}")
        state = response.json()
        print(f"   Robot status: {state['status']}, Battery: {state['battery']}%")
    except Exception as e:
        print(f"❌ GET /api/state/full : {e}")
    
    # Test batterie
    try:
        response = requests.get(f"{API_BASE}/api/state/battery", headers=headers)
        print(f"✅ GET /api/state/battery : {response.status_code}")
        battery = response.json()
        print(f"   Battery: {battery['level']}% ({battery['status']})")
    except Exception as e:
        print(f"❌ GET /api/state/battery : {e}")
    
    # Test mouvement
    try:
        pose_data = {"x": 0.1, "y": 0.2, "z": 0.3}
        response = requests.post(f"{API_BASE}/api/motion/goto_pose", 
                               json=pose_data, headers=headers)
        print(f"✅ POST /api/motion/goto_pose : {response.status_code}")
        motion = response.json()
        print(f"   Status: {motion['status']}, Time: {motion['estimated_time']}s")
    except Exception as e:
        print(f"❌ POST /api/motion/goto_pose : {e}")
    
    # Test retour à la position d'origine
    try:
        response = requests.post(f"{API_BASE}/api/motion/home", headers=headers)
        print(f"✅ POST /api/motion/home : {response.status_code}")
        home = response.json()
        print(f"   Status: {home['status']}, Time: {home['estimated_time']}s")
    except Exception as e:
        print(f"❌ POST /api/motion/home : {e}")


async def test_websocket():
    """Test du WebSocket."""
    print("\n🌐 Test du WebSocket...")
    
    try:
        async with websockets.connect(f"{WS_BASE}/ws/telemetry") as websocket:
            print("✅ Connexion WebSocket établie")
            
            # Test ping/pong
            ping_msg = json.dumps({"type": "ping"})
            await websocket.send(ping_msg)
            print("📤 Message ping envoyé")
            
            # Réception des messages
            for i in range(5):
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                    data = json.loads(message)
                    
                    if data.get("type") == "pong":
                        print("✅ Pong reçu")
                    else:
                        print(f"📊 Télémétrie reçue: {data.get('timestamp', 'N/A')}")
                        if "robot" in data:
                            pos = data["robot"]["position"]
                            print(f"   Position: x={pos['x']:.3f}, y={pos['y']:.3f}, z={pos['z']:.3f}")
                
                except asyncio.TimeoutError:
                    print("⏰ Timeout de réception")
                    break
            
            print("✅ Test WebSocket terminé")
    
    except Exception as e:
        print(f"❌ Erreur WebSocket : {e}")


def test_authentication():
    """Test de l'authentification."""
    print("\n🔐 Test de l'authentification...")
    
    # Test sans token
    try:
        response = requests.get(f"{API_BASE}/api/state/full")
        print(f"❌ GET sans token : {response.status_code} (attendu: 401)")
    except Exception as e:
        print(f"❌ GET sans token : {e}")
    
    # Test avec mauvais token
    try:
        headers = {"Authorization": "Bearer wrong-token"}
        response = requests.get(f"{API_BASE}/api/state/full", headers=headers)
        print(f"❌ GET avec mauvais token : {response.status_code} (attendu: 401)")
    except Exception as e:
        print(f"❌ GET avec mauvais token : {e}")
    
    # Test avec bon token
    try:
        headers = {"Authorization": f"Bearer {API_TOKEN}"}
        response = requests.get(f"{API_BASE}/api/state/full", headers=headers)
        print(f"✅ GET avec bon token : {response.status_code}")
    except Exception as e:
        print(f"❌ GET avec bon token : {e}")


def main():
    """Point d'entrée principal."""
    print("🧪 Tests de l'API BBIA-SIM")
    print("=" * 50)
    
    # Attente que l'API soit prête
    print("⏳ Attente que l'API soit prête...")
    time.sleep(2)
    
    # Tests REST
    test_rest_endpoints()
    
    # Tests d'authentification
    test_authentication()
    
    # Tests WebSocket
    try:
        asyncio.run(test_websocket())
    except Exception as e:
        print(f"❌ Erreur lors du test WebSocket : {e}")
    
    print("\n✅ Tests terminés")


if __name__ == "__main__":
    main()
