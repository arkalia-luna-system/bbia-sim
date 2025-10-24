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
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(f"{self.base_url}/health")
                if response.status_code == 200:
                    response.json()
                    return True
                else:
                    return False
        except Exception:
            return False

    async def test_api_info(self) -> bool:
        """Teste l'endpoint d'informations API."""
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(f"{self.base_url}/api/info")
                if response.status_code == 200:
                    response.json()
                    return True
                else:
                    return False
        except Exception:
            return False

    async def test_state_endpoints(self) -> bool:
        """Teste les endpoints d'état."""
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
                        success_count += 1
                    else:
                        pass
                except Exception:
                    pass

        return success_count == len(endpoints)

    async def test_motion_endpoints(self) -> bool:
        """Teste les endpoints de mouvement."""
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
                    success_count += 1
                else:
                    pass
            except Exception:
                pass

            # Test home
            try:
                response = await client.post(
                    f"{self.base_url}/api/motion/home", headers=self.headers
                )
                if response.status_code == 200:
                    success_count += 1
                else:
                    pass
            except Exception:
                pass

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
                    success_count += 1
                else:
                    pass
            except Exception:
                pass

            # Test head
            try:
                response = await client.post(
                    f"{self.base_url}/api/motion/head?yaw=0.2&pitch=0.1",
                    headers=self.headers,
                )
                if response.status_code == 200:
                    success_count += 1
                else:
                    pass
            except Exception:
                pass

        return success_count >= 3  # Au moins 3 sur 4

    async def test_websocket_telemetry(self) -> bool:
        """Teste le WebSocket de télémétrie."""
        try:
            ws_url = self.base_url.replace("http", "ws") + "/ws/telemetry"
            async with websockets.connect(ws_url) as websocket:

                # Test ping
                ping_message = {"type": "ping"}
                await websocket.send(json.dumps(ping_message))

                # Attente de la réponse
                response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                data = json.loads(response)

                if data.get("type") == "pong":
                    pass
                else:
                    pass

                # Test réception de télémétrie
                telemetry_received = False

                for _ in range(5):  # Attendre 5 messages max
                    try:
                        message = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                        data = json.loads(message)

                        if "joints" in data and "timestamp" in data:
                            telemetry_received = True
                            break
                    except asyncio.TimeoutError:
                        break

                return telemetry_received

        except Exception:
            return False

    async def run_all_tests(self) -> dict[str, bool]:
        """Exécute tous les tests."""
        tests = {
            "health": await self.test_health_check(),
            "api_info": await self.test_api_info(),
            "state_endpoints": await self.test_state_endpoints(),
            "motion_endpoints": await self.test_motion_endpoints(),
            "websocket_telemetry": await self.test_websocket_telemetry(),
        }

        passed = 0
        for _test_name, result in tests.items():
            if result:
                passed += 1

        if passed == len(tests):
            pass
        else:
            pass

        return tests


async def main():
    """Fonction principale."""
    tester = BBIAAPITester()
    await tester.run_all_tests()


if __name__ == "__main__":
    asyncio.run(main())
