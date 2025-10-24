"""
Tests d'intégration end-to-end pour BBIA-SIM.

Ces tests vérifient le cycle complet :
API → Simulateur → Mouvement → Télémétrie
"""

import asyncio
import json
import logging
import time
from typing import Any

import httpx
import pytest
import pytest_asyncio
import websockets

# Configuration du logging pour les tests
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration des tests
API_URL = "http://localhost:8000"
API_TOKEN = "bbia-secret-key-dev"
HEADERS = {"Authorization": f"Bearer {API_TOKEN}"}
TIMEOUT = 10.0


class BBIAE2ETestClient:
    """Client de test pour les tests e2e BBIA-SIM."""

    def __init__(self, api_url: str, token: str):
        """Initialise le client de test.

        Args:
            api_url: URL de l'API
            token: Token d'authentification
        """
        self.api_url = api_url.rstrip("/")
        self.token = token
        self.headers = {"Authorization": f"Bearer {token}"}
        self.client = httpx.AsyncClient(timeout=TIMEOUT)

    async def check_health(self) -> bool:
        """Vérifie que l'API est accessible.

        Returns:
            True si l'API est accessible
        """
        try:
            response = await self.client.get(f"{self.api_url}/health")
            return response.status_code == 200
        except Exception:
            return False

    async def get_joint_positions(self) -> dict[str, Any]:
        """Récupère les positions actuelles des joints.

        Returns:
            Dict avec les positions des joints
        """
        response = await self.client.get(
            f"{self.api_url}/api/state/joints", headers=self.headers
        )
        response.raise_for_status()
        return response.json()

    async def set_joint_positions(self, joints: list[dict[str, Any]]) -> dict[str, Any]:
        """Définit les positions des joints.

        Args:
            joints: Liste des joints avec leurs positions

        Returns:
            Réponse de l'API
        """
        response = await self.client.post(
            f"{self.api_url}/api/motion/joints",
            json=joints,
            headers=self.headers,
        )
        response.raise_for_status()
        return response.json()

    async def connect_websocket(self):
        """Se connecte au WebSocket de télémétrie.

        Returns:
            Client WebSocket connecté
        """
        ws_url = f"{self.api_url.replace('http', 'ws')}/ws/telemetry"
        return await websockets.connect(ws_url, additional_headers=self.headers)

    async def close(self) -> None:
        """Ferme les connexions."""
        await self.client.aclose()


@pytest.mark.asyncio
class TestMotionRoundtrip:
    """Tests du cycle complet mouvement."""

    @pytest_asyncio.fixture
    async def client(self):
        """Fixture pour le client de test."""
        client = BBIAE2ETestClient(API_URL, API_TOKEN)
        yield client
        await client.close()

    async def test_api_health(self, client: BBIAE2ETestClient):
        """Test que l'API est accessible."""
        if not await client.check_health():
            pytest.skip("API non accessible - serveur non démarré")

    async def test_joint_position_roundtrip(self, client: BBIAE2ETestClient):
        """Test du cycle complet : GET → SET → GET avec vérification du changement."""

        # Vérifier que l'API est accessible
        if not await client.check_health():
            pytest.skip("API non accessible - serveur non démarré")

        # 1. Récupération de la position initiale
        initial_state = await client.get_joint_positions()
        initial_neck_yaw = initial_state["joints"]["neck_yaw"]["position"]

        logger.info(f"Position initiale neck_yaw: {initial_neck_yaw:.3f} rad")

        # 2. Calcul de la position cible (décalage de 0.2 rad)
        target_position = initial_neck_yaw + 0.2

        # 3. Envoi de la commande de mouvement
        joints_command = [{"joint_name": "neck_yaw", "position": target_position}]

        start_time = time.time()
        motion_response = await client.set_joint_positions(joints_command)
        motion_time = time.time() - start_time

        logger.info(f"Mouvement exécuté en {motion_time:.3f}s")
        logger.info(f"Réponse mouvement: {motion_response}")

        # Vérification de la réponse
        assert motion_response["status"] == "moving"
        assert motion_response["success_count"] == 1
        assert motion_response["total_count"] == 1

        # 4. Attente pour que le mouvement se stabilise
        await asyncio.sleep(0.5)

        # 5. Vérification de la nouvelle position
        final_state = await client.get_joint_positions()
        final_neck_yaw = final_state["joints"]["neck_yaw"]["position"]

        logger.info(f"Position finale neck_yaw: {final_neck_yaw:.3f} rad")

        # 6. Vérification que le changement est significatif
        position_change = abs(final_neck_yaw - initial_neck_yaw)
        assert (
            position_change >= 0.1
        ), f"Changement trop faible: {position_change:.3f} rad"

        logger.info(f"✅ Changement de position détecté: {position_change:.3f} rad")

    async def test_websocket_telemetry_rate(self, client: BBIAE2ETestClient):
        """Test que le WebSocket émet des messages à la bonne fréquence."""

        # Vérifier que l'API est accessible
        if not await client.check_health():
            pytest.skip("API non accessible - serveur non démarré")

        # Connexion WebSocket
        websocket = await client.connect_websocket()

        messages_received = 0
        start_time = time.time()
        test_duration = 1.0  # Test sur 1 seconde

        try:
            while time.time() - start_time < test_duration:
                try:
                    # Attente d'un message avec timeout
                    message = await asyncio.wait_for(websocket.recv(), timeout=0.2)

                    data = json.loads(message)

                    # Vérification du format du message
                    assert "timestamp" in data
                    assert "joints" in data
                    assert isinstance(data["joints"], dict)

                    messages_received += 1
                    logger.debug(
                        f"Message télémétrie #{messages_received}: {data['timestamp']}"
                    )

                except asyncio.TimeoutError:
                    # Timeout normal, continue
                    continue
                except Exception as e:
                    logger.error(f"Erreur réception WebSocket: {e}")
                    break

        finally:
            await websocket.close()

        # Vérification du taux de messages
        expected_min_messages = 5  # Au moins 5 messages en 1 seconde
        assert (
            messages_received >= expected_min_messages
        ), f"Trop peu de messages reçus: {messages_received} < {expected_min_messages}"

        rate = messages_received / test_duration
        logger.info(
            f"✅ Taux télémétrie: {rate:.1f} Hz ({messages_received} messages en {test_duration}s)"
        )

    async def test_invalid_joint_rejection(self, client: BBIAE2ETestClient):
        """Test que les joints invalides sont rejetés avec 422."""

        # Vérifier que l'API est accessible
        if not await client.check_health():
            pytest.skip("API non accessible - serveur non démarré")

        # Tentative avec un joint inexistant
        invalid_joints = [{"joint_name": "invalid_joint", "position": 0.5}]

        try:
            await client.set_joint_positions(invalid_joints)
            raise AssertionError("Devrait lever une exception pour joint invalide")
        except httpx.HTTPStatusError as e:
            assert (
                e.response.status_code == 422
            ), f"Mauvais code d'erreur: {e.response.status_code}"
            logger.info("✅ Joint invalide correctement rejeté avec 422")

    async def test_joint_angle_clamping(self, client: BBIAE2ETestClient):
        """Test que les angles hors limites sont clampés."""

        # Vérifier que l'API est accessible
        if not await client.check_health():
            pytest.skip("API non accessible - serveur non démarré")

        # Angle très élevé (devrait être clampé)
        extreme_angle = 10.0  # Bien au-delà des limites
        joints_command = [{"joint_name": "neck_yaw", "position": extreme_angle}]

        try:
            # La commande devrait réussir mais avec clamp
            response = await client.set_joint_positions(joints_command)

            assert response["status"] == "moving"
            assert response["success_count"] == 1

            # Vérification que la position finale est dans les limites
            await asyncio.sleep(0.5)
            final_state = await client.get_joint_positions()
            final_position = final_state["joints"]["neck_yaw"]["position"]

            # La position devrait être dans les limites raisonnables
            assert (
                -2.0 <= final_position <= 2.0
            ), f"Position non clampée: {final_position}"

            logger.info(
                f"✅ Angle clampé de {extreme_angle} à {final_position:.3f} rad"
            )

        except httpx.HTTPStatusError as e:
            if e.response.status_code == 422:
                # Si l'API retourne 422, vérifier que c'est pour une raison valide
                error_detail = e.response.json().get("detail", "")
                logger.warning(f"API retourne 422: {error_detail}")
                # Pour ce test, on accepte l'erreur 422 comme comportement valide
                assert "non valide" in error_detail or "Unprocessable Entity" in str(e)
                logger.info("✅ Angle extrême correctement rejeté par l'API")
            else:
                raise


@pytest.mark.asyncio
class TestPerformance:
    """Tests de performance des composants."""

    @pytest_asyncio.fixture
    async def client(self):
        """Fixture pour le client de test."""
        client = BBIAE2ETestClient(API_URL, API_TOKEN)
        yield client
        await client.close()

    async def test_motion_response_time(self, client: BBIAE2ETestClient):
        """Test que les réponses de mouvement sont rapides."""

        # Vérifier que l'API est accessible
        if not await client.check_health():
            pytest.skip("API non accessible - serveur non démarré")

        joints_command = [{"joint_name": "neck_yaw", "position": 0.1}]

        start_time = time.time()
        await client.set_joint_positions(joints_command)
        response_time = time.time() - start_time

        # La réponse devrait être rapide (< 1 seconde)
        assert response_time < 1.0, f"Réponse trop lente: {response_time:.3f}s"

        logger.info(f"✅ Temps de réponse mouvement: {response_time:.3f}s")

    async def test_concurrent_motions(self, client: BBIAE2ETestClient):
        """Test que plusieurs mouvements simultanés fonctionnent."""

        # Vérifier que l'API est accessible
        if not await client.check_health():
            pytest.skip("API non accessible - serveur non démarré")

        # Préparation de plusieurs commandes
        commands = [
            [{"joint_name": "neck_yaw", "position": 0.2}],
            [{"joint_name": "neck_yaw", "position": -0.2}],
            [{"joint_name": "neck_yaw", "position": 0.0}],
        ]

        # Exécution concurrente
        start_time = time.time()
        tasks = [client.set_joint_positions(cmd) for cmd in commands]
        responses = await asyncio.gather(*tasks)
        total_time = time.time() - start_time

        # Vérification que toutes les commandes ont réussi
        for i, response in enumerate(responses):
            assert response["status"] == "moving", f"Commande {i} échouée"

        # Le temps total devrait être raisonnable
        assert total_time < 3.0, f"Temps total trop long: {total_time:.3f}s"

        logger.info(f"✅ {len(commands)} mouvements concurrents en {total_time:.3f}s")


# Configuration pytest pour les tests e2e
pytestmark = [
    pytest.mark.e2e,
    pytest.mark.asyncio,
]
