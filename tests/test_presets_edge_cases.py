"""Tests edge cases pour les presets d'émotions."""

import json
from unittest.mock import MagicMock, patch

import pytest
from fastapi import HTTPException

from bbia_sim.daemon.app.routers.presets import (
    EmotionPreset,
    apply_preset,
    create_preset,
    delete_preset,
    get_preset,
    list_presets,
)


@pytest.fixture
def temp_presets_dir(tmp_path):
    """Créer un dossier temporaire pour les presets."""
    with patch("bbia_sim.daemon.app.routers.presets.PRESETS_DIR", tmp_path):
        yield tmp_path


@pytest.fixture
def sample_preset():
    """Preset de test valide."""
    return EmotionPreset(
        name="test_preset",
        emotions={"happy": 0.8, "sad": 0.2},
        description="Preset de test",
    )


class TestPresetsEdgeCases:
    """Tests edge cases pour les presets."""

    @pytest.mark.asyncio
    async def test_create_preset_invalid_emotion_intensity_negative(
        self, temp_presets_dir, sample_preset
    ):
        """Test création preset avec intensité négative."""
        preset = EmotionPreset(
            name="invalid_preset",
            emotions={"happy": -0.5},  # Intensité invalide
        )

        with pytest.raises(HTTPException):
            await create_preset(preset)

    @pytest.mark.asyncio
    async def test_create_preset_invalid_emotion_intensity_too_high(
        self, temp_presets_dir, sample_preset
    ):
        """Test création preset avec intensité > 1."""
        preset = EmotionPreset(
            name="invalid_preset",
            emotions={"happy": 1.5},  # Intensité invalide
        )

        with pytest.raises(HTTPException):
            await create_preset(preset)

    @pytest.mark.asyncio
    async def test_create_preset_invalid_name_path_traversal(self, temp_presets_dir):
        """Test création preset avec nom contenant path traversal."""
        preset = EmotionPreset(
            name="../malicious",  # Path traversal
            emotions={"happy": 0.5},
        )

        with pytest.raises(HTTPException):
            await create_preset(preset)

    @pytest.mark.asyncio
    async def test_get_preset_not_found(self, temp_presets_dir):
        """Test récupération preset inexistant."""
        with pytest.raises(HTTPException) as exc_info:
            await get_preset("nonexistent_preset")
        assert exc_info.value.status_code == 404

    @pytest.mark.asyncio
    async def test_get_preset_invalid_name_path_traversal(self, temp_presets_dir):
        """Test récupération preset avec nom path traversal."""
        with pytest.raises(HTTPException) as exc_info:
            await get_preset("../malicious")
        assert exc_info.value.status_code == 400

    @pytest.mark.asyncio
    async def test_apply_preset_empty_emotions(self, temp_presets_dir):
        """Test application preset vide (pas d'émotions)."""
        # Créer un preset vide
        preset_file = temp_presets_dir / "empty_preset.json"
        with open(preset_file, "w", encoding="utf-8") as f:
            json.dump({"name": "empty_preset", "emotions": {}}, f)

        with pytest.raises(HTTPException) as exc_info:
            await apply_preset("empty_preset")
        assert exc_info.value.status_code == 400

    @pytest.mark.asyncio
    async def test_apply_preset_not_found(self, temp_presets_dir):
        """Test application preset inexistant."""
        with pytest.raises(HTTPException) as exc_info:
            await apply_preset("nonexistent_preset")
        assert exc_info.value.status_code == 404

    @pytest.mark.asyncio
    async def test_apply_preset_corrupted_json(self, temp_presets_dir):
        """Test application preset avec JSON corrompu."""
        # Créer un fichier JSON corrompu
        preset_file = temp_presets_dir / "corrupted.json"
        with open(preset_file, "w", encoding="utf-8") as f:
            f.write("{ invalid json }")

        with pytest.raises(HTTPException) as exc_info:
            await apply_preset("corrupted")
        assert exc_info.value.status_code == 500

    @pytest.mark.asyncio
    async def test_delete_preset_not_found(self, temp_presets_dir):
        """Test suppression preset inexistant."""
        with pytest.raises(HTTPException) as exc_info:
            await delete_preset("nonexistent_preset")
        assert exc_info.value.status_code == 404

    @pytest.mark.asyncio
    async def test_delete_preset_invalid_name_path_traversal(self, temp_presets_dir):
        """Test suppression preset avec nom path traversal."""
        with pytest.raises(HTTPException) as exc_info:
            await delete_preset("../malicious")
        assert exc_info.value.status_code == 400

    @pytest.mark.asyncio
    async def test_list_presets_empty_directory(self, temp_presets_dir):
        """Test liste presets dans dossier vide."""
        result = await list_presets()
        assert result["count"] == 0
        assert result["presets"] == []

    @pytest.mark.asyncio
    async def test_list_presets_with_corrupted_files(self, temp_presets_dir):
        """Test liste presets avec fichiers corrompus."""
        # Créer un preset valide
        valid_file = temp_presets_dir / "valid.json"
        with open(valid_file, "w", encoding="utf-8") as f:
            json.dump({"name": "valid", "emotions": {"happy": 0.5}}, f)

        # Créer un fichier corrompu
        corrupted_file = temp_presets_dir / "corrupted.json"
        with open(corrupted_file, "w", encoding="utf-8") as f:
            f.write("{ invalid json }")

        # La liste doit ignorer le fichier corrompu
        result = await list_presets()
        assert result["count"] == 1
        assert result["presets"][0]["name"] == "valid"

    @pytest.mark.asyncio
    async def test_apply_preset_robot_error(self, temp_presets_dir):
        """Test application preset avec erreur robot."""
        # Créer un preset valide
        preset_file = temp_presets_dir / "test_preset.json"
        with open(preset_file, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "name": "test_preset",
                    "emotions": {"happy": 0.8, "sad": 0.2},
                },
                f,
            )

        # Mocker RobotFactory pour simuler une erreur
        with patch("bbia_sim.robot_factory.RobotFactory") as mock_factory:
            mock_factory.create_backend.return_value = None

            with pytest.raises(HTTPException) as exc_info:
                await apply_preset("test_preset")
            assert exc_info.value.status_code == 500

    @pytest.mark.asyncio
    async def test_apply_preset_invalid_intensity_in_file(self, temp_presets_dir):
        """Test application preset avec intensité invalide dans fichier."""
        # Créer un preset avec intensité invalide
        preset_file = temp_presets_dir / "invalid_intensity.json"
        with open(preset_file, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "name": "invalid_intensity",
                    "emotions": {"happy": 1.5},  # Intensité > 1
                },
                f,
            )

        # Mocker RobotFactory
        with patch("bbia_sim.robot_factory.RobotFactory") as mock_factory:
            mock_robot = MagicMock()
            mock_robot.set_emotion = MagicMock(return_value=True)
            mock_factory.create_backend.return_value = mock_robot

            # L'application doit gérer l'erreur et continuer avec les autres émotions
            result = await apply_preset("invalid_intensity")
            # Vérifier que le résultat contient des erreurs ou qu'aucune émotion n'a été appliquée
            assert "errors" in result or result.get("applied_count", 0) == 0
