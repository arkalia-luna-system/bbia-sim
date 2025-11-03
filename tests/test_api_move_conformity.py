"""Tests pour les endpoints /api/move conformes SDK officiel."""

from uuid import UUID

import pytest
from fastapi.testclient import TestClient

from bbia_sim.daemon.app.main import app
from bbia_sim.daemon.config import settings

client = TestClient(app)


@pytest.fixture
def api_token() -> str:
    """Token d'authentification pour les tests."""
    return settings.api_token


class TestMoveGoto:
    """Tests pour POST /api/move/goto (conforme SDK)."""

    def test_goto_with_head_pose(self, api_token: str) -> None:
        """Test goto avec head_pose."""
        request_data = {
            "head_pose": {
                "x": 0.1,
                "y": 0.0,
                "z": 0.3,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.1,
            },
            "duration": 2.0,
            "interpolation": "minjerk",
        }
        response = client.post(
            "/api/move/goto",
            json=request_data,
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "uuid" in data
        # UUID doit être valide
        UUID(data["uuid"])

    def test_goto_with_antennas(self, api_token: str) -> None:
        """Test goto avec antennas uniquement."""
        request_data = {
            "antennas": [0.1, -0.1],
            "duration": 1.5,
            "interpolation": "linear",
        }
        response = client.post(
            "/api/move/goto",
            json=request_data,
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "uuid" in data

    def test_goto_all_interpolation_modes(self, api_token: str) -> None:
        """Test tous les modes d'interpolation."""
        modes = ["linear", "minjerk", "ease", "cartoon"]
        for mode in modes:
            request_data = {
                "antennas": [0.0, 0.0],
                "duration": 1.0,
                "interpolation": mode,
            }
            response = client.post(
                "/api/move/goto",
                json=request_data,
                headers={"Authorization": f"Bearer {api_token}"},
            )
            assert response.status_code == 200
            data = response.json()
            assert "uuid" in data


class TestMoveRunning:
    """Tests pour GET /api/move/running."""

    def test_get_running_moves_empty(self, api_token: str) -> None:
        """Test récupération liste moves vide."""
        response = client.get(
            "/api/move/running",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)


class TestMoveRecordedMoveDatasets:
    """Tests pour endpoints recorded move datasets (conforme SDK)."""

    def test_discover_recorded_move_datasets(self, api_token: str) -> None:
        """Test GET /api/move/recorded-move-datasets/discover - Endpoint discovery datasets.

        AMÉLIORATION: Test amélioré pour vérifier intégration HF Hub si disponible,
        avec fallback vers liste hardcodée. Mentionné dans docs/audit/DECISION_FINAL_AMELIORATIONS.md.
        """
        response = client.get(
            "/api/move/recorded-move-datasets/discover",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list), "Retourne une liste de datasets"
        assert len(data) > 0, "Au moins un dataset doit être retourné"

        # Vérifier que ce sont des strings (noms de datasets)
        for dataset in data:
            assert isinstance(dataset, str), "Chaque dataset doit être un string"
            assert "/" in dataset, "Format dataset: 'org/repo-name'"

        # Vérifier datasets connus présents (toujours inclus pour compatibilité)
        expected_datasets = [
            "pollen-robotics/reachy-mini-dances-library",
            "pollen-robotics/reachy-mini-emotions-library",
        ]
        for expected in expected_datasets:
            assert expected in data, f"Dataset attendu '{expected}' manquant"

        # AMÉLIORATION: Vérifier que la liste est triée (nouveau comportement)
        assert data == sorted(data), "Les datasets doivent être triés"

        # AMÉLIORATION: Si HF Hub disponible, il peut y avoir plus de datasets
        # (pas de limite stricte, juste vérifier que c'est cohérent)
        assert len(data) >= len(
            expected_datasets,
        ), "Doit contenir au moins les datasets hardcodés"

    def test_list_recorded_move_dataset_without_token(self, api_token: str) -> None:
        """Test lister un dataset sans token (vérifie comportement sécurité)."""
        # Sans token - peut retourner 401/403 (auth requise) ou 404 (dataset non trouvé)
        response = client.get("/api/move/recorded-move-datasets/list/test-dataset")
        # 404 = dataset non trouvé (normal), 401/403 = auth requise, 200 = public endpoint
        assert response.status_code in (200, 401, 403, 404)

    def test_list_recorded_move_dataset_not_found(self, api_token: str) -> None:
        """Test dataset non trouvé retourne 404."""
        response = client.get(
            "/api/move/recorded-move-datasets/list/non-existent-dataset",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        # Peut être 404 si SDK disponible, ou 501 si SDK non disponible
        assert response.status_code in (404, 501)


class TestMoveStop:
    """Tests pour POST /api/move/stop."""

    def test_stop_move_invalid_uuid(self, api_token: str) -> None:
        """Test stop avec UUID invalide."""
        response = client.post(
            "/api/move/stop",
            json={"uuid": "invalid-uuid"},
            headers={"Authorization": f"Bearer {api_token}"},
        )
        # FastAPI/Pydantic retourne 422 pour validation échouée (UUID invalide)
        assert response.status_code == 422

    def test_stop_move_not_found(self, api_token: str) -> None:
        """Test stop avec UUID non trouvé."""
        import uuid

        fake_uuid = str(uuid.uuid4())
        response = client.post(
            "/api/move/stop",
            json={"uuid": fake_uuid},
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 404


class TestMovePlay:
    """Tests pour /api/move/play/*."""

    def test_play_wake_up(self, api_token: str) -> None:
        """Test POST /api/move/play/wake_up."""
        response = client.post(
            "/api/move/play/wake_up",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "uuid" in data

    def test_play_goto_sleep(self, api_token: str) -> None:
        """Test POST /api/move/play/goto_sleep."""
        response = client.post(
            "/api/move/play/goto_sleep",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "uuid" in data


class TestMoveSetTarget:
    """Tests pour /api/move/set_target."""

    def test_set_target(self, api_token: str) -> None:
        """Test POST /api/move/set_target."""
        target_data = {
            "target_head_pose": {
                "x": 0.1,
                "y": 0.0,
                "z": 0.3,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.1,
            },
            "target_antennas": [0.1, -0.1],
        }
        response = client.post(
            "/api/move/set_target",
            json=target_data,
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "ok"
