"""Router pour les endpoints de presets d'émotions."""

import json
import logging
from pathlib import Path
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

logger = logging.getLogger(__name__)

router = APIRouter()

# Dossier pour stocker les presets
PRESETS_DIR = Path.home() / ".bbia_sim" / "presets"
PRESETS_DIR.mkdir(parents=True, exist_ok=True)


class EmotionPreset(BaseModel):
    """Modèle pour un preset d'émotion."""

    name: str
    emotions: dict[str, float]  # {emotion: intensity (0-1)}
    description: str | None = None


@router.get("/presets")
async def list_presets() -> dict[str, Any]:
    """Liste tous les presets disponibles.

    Returns:
        Liste des presets avec leurs noms et descriptions
    """
    try:
        presets = []
        for preset_file in PRESETS_DIR.glob("*.json"):
            try:
                with open(preset_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    presets.append(
                        {
                            "name": data.get("name", preset_file.stem),
                            "filename": preset_file.name,
                            "description": data.get("description", ""),
                            "emotions": data.get("emotions", {}),
                        }
                    )
            except (json.JSONDecodeError, IOError) as e:
                logger.warning(f"Erreur lecture preset {preset_file}: {e}")
                continue

        return {"presets": presets, "count": len(presets)}
    except Exception as e:
        logger.exception("Erreur lors de la liste des presets")
        raise HTTPException(status_code=500, detail=f"Erreur: {e!s}")


@router.get("/presets/{preset_name}")
async def get_preset(preset_name: str) -> dict[str, Any]:
    """Récupère un preset spécifique.

    Args:
        preset_name: Nom du preset (sans extension .json)

    Returns:
        Données du preset
    """
    preset_file = PRESETS_DIR / f"{preset_name}.json"
    if not preset_file.exists():
        raise HTTPException(status_code=404, detail=f"Preset '{preset_name}' introuvable")

    try:
        with open(preset_file, "r", encoding="utf-8") as f:
            return json.load(f)
    except (json.JSONDecodeError, IOError) as e:
        logger.exception(f"Erreur lecture preset {preset_name}")
        raise HTTPException(status_code=500, detail=f"Erreur lecture: {e!s}")


@router.post("/presets")
async def create_preset(preset: EmotionPreset) -> dict[str, Any]:
    """Crée ou met à jour un preset.

    Args:
        preset: Données du preset

    Returns:
        Confirmation de création
    """
    # Valider les intensités (0-1)
    for emotion, intensity in preset.emotions.items():
        if not 0 <= intensity <= 1:
            raise HTTPException(
                status_code=400,
                detail=f"Intensité invalide pour {emotion}: {intensity} (doit être entre 0 et 1)",
            )

    preset_file = PRESETS_DIR / f"{preset.name}.json"
    try:
        with open(preset_file, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "name": preset.name,
                    "emotions": preset.emotions,
                    "description": preset.description or "",
                },
                f,
                indent=2,
                ensure_ascii=False,
            )

        logger.info(f"✅ Preset créé: {preset.name}")
        return {
            "success": True,
            "message": f"Preset '{preset.name}' créé avec succès",
            "filename": preset_file.name,
        }
    except IOError as e:
        logger.exception(f"Erreur création preset {preset.name}")
        raise HTTPException(status_code=500, detail=f"Erreur écriture: {e!s}")


@router.post("/presets/{preset_name}/apply")
async def apply_preset(preset_name: str) -> dict[str, Any]:
    """Applique un preset (définit toutes les émotions).

    Args:
        preset_name: Nom du preset à appliquer

    Returns:
        Confirmation d'application
    """
    preset_file = PRESETS_DIR / f"{preset_name}.json"
    if not preset_file.exists():
        raise HTTPException(status_code=404, detail=f"Preset '{preset_name}' introuvable")

    try:
        with open(preset_file, "r", encoding="utf-8") as f:
            preset_data = json.load(f)

        emotions = preset_data.get("emotions", {})
        if not emotions:
            raise HTTPException(status_code=400, detail="Preset vide (pas d'émotions)")

        # Appliquer chaque émotion via l'API ecosystem
        try:
            from bbia_sim.robot_factory import RobotFactory

            robot = RobotFactory.create_backend("mujoco")
            if robot:
                robot.connect()
                for emotion, intensity in emotions.items():
                    if hasattr(robot, "set_emotion"):
                        robot.set_emotion(emotion, intensity)
                robot.disconnect()

            logger.info(f"✅ Preset '{preset_name}' appliqué: {len(emotions)} émotions")
            return {
                "success": True,
                "message": f"Preset '{preset_name}' appliqué avec succès",
                "emotions_applied": emotions,
            }
        except Exception as e:
            logger.exception(f"Erreur application preset {preset_name}")
            raise HTTPException(status_code=500, detail=f"Erreur application: {e!s}")
    except (json.JSONDecodeError, IOError) as e:
        logger.exception(f"Erreur lecture preset {preset_name}")
        raise HTTPException(status_code=500, detail=f"Erreur lecture: {e!s}")


@router.delete("/presets/{preset_name}")
async def delete_preset(preset_name: str) -> dict[str, Any]:
    """Supprime un preset.

    Args:
        preset_name: Nom du preset à supprimer

    Returns:
        Confirmation de suppression
    """
    preset_file = PRESETS_DIR / f"{preset_name}.json"
    if not preset_file.exists():
        raise HTTPException(status_code=404, detail=f"Preset '{preset_name}' introuvable")

    try:
        preset_file.unlink()
        logger.info(f"✅ Preset supprimé: {preset_name}")
        return {
            "success": True,
            "message": f"Preset '{preset_name}' supprimé avec succès",
        }
    except IOError as e:
        logger.exception(f"Erreur suppression preset {preset_name}")
        raise HTTPException(status_code=500, detail=f"Erreur suppression: {e!s}")

