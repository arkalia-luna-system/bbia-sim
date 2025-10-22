#!/usr/bin/env python3
"""Script de test pour l'API BBIA-SIM."""

import asyncio
import json

import httpx
import websockets


class BBIAAPITester:
    """Testeur pour l'API BBIA-SIM."""

    def __init__(
        self,
        base_url: str = "http://localhost:8000",
        token: str = "bbia-secret-key-dev",
    ):
        """Initialise le testeur.

        Args:
            base_url: URL de base de l'API
            token: Token d'authentification
        """
        self.base_url = base_url
        self.token = token
        self.headers = {"Authorization": f"Bearer {token}"}

    async def test_health_check(self) -> bool:
        """Teste l'endpoint de santé."""
        print("🔍 Test de santé...")
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(f"{self.base_url}/health")
                if response.status_code == 200:
                    data = response.json()
                    print(f"✅ Santé OK: {data}")
                    return True
                else:
                    print(f"❌ Santé KO: {response.status_code}")
                    return False
        except Exception as e:
            print(f"❌ Erreur santé: {e}")
            return False

    async def test_api_info(self) -> bool:
        """Teste l'endpoint d'informations API."""
        print("🔍 Test info API...")
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(f"{self.base_url}/api/info")
                if response.status_code == 200:
                    data = response.json()
                    print(f"✅ Info API OK: {data['name']} v{data['version']}")
                    return True
                else:
                    print(f"❌ Info API KO: {response.status_code}")
                    return False
        except Exception as e:
            print(f"❌ Erreur info API: {e}")
            return False

    async def test_state_endpoints(self) -> bool:
        """Teste les endpoints d'état."""
        print("🔍 Test endpoints d'état...")

        endpoints = [
            "/api/state/full",
            "/api/state/battery",
            "/api/state/joints",
            "/api/state/status",
        ]

        success_count = 0
        async with httpx.AsyncClient() as client:
            for endpoint in endpoints:
                try:
                    response = await client.get(
                        f"{self.base_url}{endpoint}", headers=self.headers
                    )
                    if response.status_code == 200:
                        response.json()
                        print(f"✅ {endpoint}: OK")
                        success_count += 1
                    else:
                        print(f"❌ {endpoint}: {response.status_code}")
                except Exception as e:
                    print(f"❌ Erreur {endpoint}: {e}")

        return success_count == len(endpoints)

    async def test_motion_endpoints(self) -> bool:
        """Teste les endpoints de mouvement."""
        print("🔍 Test endpoints de mouvement...")

        success_count = 0
        async with httpx.AsyncClient() as client:
            # Test goto_pose
            try:
                pose_data = {"x": 0.1, "y": 0.0, "z": 0.2}
                response = await client.post(
                    f"{self.base_url}/api/motion/goto_pose",
                    headers=self.headers,
                    json=pose_data,
                )
                if response.status_code == 200:
                    print("✅ goto_pose: OK")
                    success_count += 1
                else:
                    print(f"❌ goto_pose: {response.status_code}")
            except Exception as e:
                print(f"❌ Erreur goto_pose: {e}")

            # Test home
            try:
                response = await client.post(
                    f"{self.base_url}/api/motion/home", headers=self.headers
                )
                if response.status_code == 200:
                    print("✅ home: OK")
                    success_count += 1
                else:
                    print(f"❌ home: {response.status_code}")
            except Exception as e:
                print(f"❌ Erreur home: {e}")

            # Test joints
            try:
                joints_data = [
                    {"joint_name": "neck_yaw", "position": 0.5},
                    {"joint_name": "right_shoulder_pitch", "position": -0.3},
                ]
                response = await client.post(
                    f"{self.base_url}/api/motion/joints",
                    headers=self.headers,
                    json=joints_data,
                )
                if response.status_code == 200:
                    print("✅ joints: OK")
                    success_count += 1
                else:
                    print(f"❌ joints: {response.status_code}")
            except Exception as e:
                print(f"❌ Erreur joints: {e}")

            # Test head
            try:
                response = await client.post(
                    f"{self.base_url}/api/motion/head?yaw=0.2&pitch=0.1",
                    headers=self.headers,
                )
                if response.status_code == 200:
                    print("✅ head: OK")
                    success_count += 1
                else:
                    print(f"❌ head: {response.status_code}")
            except Exception as e:
                print(f"❌ Erreur head: {e}")

        return success_count >= 3  # Au moins 3 sur 4

    async def test_websocket_telemetry(self) -> bool:
        """Teste le WebSocket de télémétrie."""
        print("🔍 Test WebSocket télémétrie...")

        try:
            ws_url = self.base_url.replace("http", "ws") + "/ws/telemetry"
            async with websockets.connect(ws_url) as websocket:
                print("✅ Connexion WebSocket établie")

                # Test ping
                ping_message = {"type": "ping"}
                await websocket.send(json.dumps(ping_message))

                # Attente de la réponse
                response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                data = json.loads(response)

                if data.get("type") == "pong":
                    print("✅ Ping/Pong OK")
                else:
                    print(f"⚠️ Réponse inattendue: {data}")

                # Test réception de télémétrie
                print("📡 Attente de données de télémétrie...")
                telemetry_received = False

                for _ in range(5):  # Attendre 5 messages max
                    try:
                        message = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                        data = json.loads(message)

                        if "joints" in data and "timestamp" in data:
                            print(
                                f"✅ Télémétrie reçue: {len(data['joints'])} articulations"
                            )
                            telemetry_received = True
                            break
                    except asyncio.TimeoutError:
                        print("⏰ Timeout attente télémétrie")
                        break

                return telemetry_received

        except Exception as e:
            print(f"❌ Erreur WebSocket: {e}")
            return False

    async def run_all_tests(self) -> dict[str, bool]:
        """Exécute tous les tests."""
        print("🚀 Démarrage des tests BBIA-SIM API")
        print("=" * 50)

        tests = {
            "health": await self.test_health_check(),
            "api_info": await self.test_api_info(),
            "state_endpoints": await self.test_state_endpoints(),
            "motion_endpoints": await self.test_motion_endpoints(),
            "websocket_telemetry": await self.test_websocket_telemetry(),
        }

        print("=" * 50)
        print("📊 Résultats des tests:")

        passed = 0
        for test_name, result in tests.items():
            status = "✅ PASS" if result else "❌ FAIL"
            print(f"  {test_name}: {status}")
            if result:
                passed += 1

        print(f"\n🎯 Score: {passed}/{len(tests)} tests passés")

        if passed == len(tests):
            print("🎉 Tous les tests sont passés !")
        else:
            print("⚠️ Certains tests ont échoué")

        return tests


async def main():
    """Fonction principale."""
    tester = BBIAAPITester()
    await tester.run_all_tests()


if __name__ == "__main__":
    asyncio.run(main())
