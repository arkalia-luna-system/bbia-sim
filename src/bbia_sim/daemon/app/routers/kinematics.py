"""Router pour les endpoints de cinématique du robot (conforme SDK officiel)."""

import logging
from pathlib import Path
from typing import Any

from fastapi import APIRouter, Depends, HTTPException, Response

from ..backend_adapter import BackendAdapter, get_backend_adapter

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/kinematics")

# Chemin vers les assets STL (conforme SDK officiel)
STL_ASSETS_DIR = (
    Path(__file__).parent.parent.parent.parent.parent
    / "src"
    / "bbia_sim"
    / "sim"
    / "assets"
    / "reachy_official"
)


@router.get("/info")
async def get_kinematics_info(
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> dict[str, Any]:
    """Récupère les informations sur la cinématique du robot (conforme SDK)."""
    # Le backend doit exposer kinematics_engine et check_collision
    # Pour l'instant, utiliser valeurs par défaut si non disponibles
    kinematics_engine = (
        getattr(backend, "kinematics_engine", None) or "AnalyticalKinematics"
    )
    check_collision = getattr(backend, "check_collision", False)

    return {
        "info": {
            "engine": str(kinematics_engine),
            "collision_check": bool(check_collision),
        }
    }


@router.get("/urdf")
async def get_urdf(
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> dict[str, str]:
    """Récupère la représentation URDF du robot (conforme SDK)."""
    # Le backend doit avoir une méthode get_urdf()
    if hasattr(backend, "get_urdf"):
        return {"urdf": backend.get_urdf()}
    elif hasattr(backend, "_robot") and hasattr(backend._robot, "get_urdf"):
        return {"urdf": backend._robot.get_urdf()}
    else:
        return {"urdf": ""}


@router.get("/stl/{filename}")
async def get_stl_file(filename: str) -> Response:
    """Récupère un fichier STL depuis les assets (conforme SDK officiel).

    Args:
        filename: Nom du fichier STL (ex: body_down_3dprint.stl)

    Returns:
        Fichier STL en binaire avec type MIME approprié

    Raises:
        HTTPException: Si le fichier n'est pas trouvé ou extension invalide
    """
    # Sécurisation : ne permettre que les fichiers .stl
    if not filename.endswith(".stl"):
        raise HTTPException(
            status_code=400, detail="Seuls les fichiers .stl sont autorisés"
        )

    # Nettoyer le chemin pour éviter les directory traversal (conforme SDK)
    from pathlib import Path

    safe_filename = Path(filename).name
    file_path = STL_ASSETS_DIR / safe_filename

    if not file_path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"Fichier STL non trouvé: {safe_filename}",
        )

    try:
        with open(file_path, "rb") as file:
            content = file.read()
            return Response(content, media_type="model/stl")
    except Exception as e:
        logger.error(f"Erreur lors de la lecture du fichier STL {filename}: {e}")
        raise HTTPException(
            status_code=500, detail=f"Erreur lors de la lecture du fichier: {str(e)}"
        ) from e
