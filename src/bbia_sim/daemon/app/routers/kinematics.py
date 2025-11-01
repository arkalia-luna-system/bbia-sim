"""Router pour les endpoints de cinématique du robot (conforme SDK officiel)."""

from pathlib import Path
from typing import Any

from fastapi import APIRouter, Depends, HTTPException, Response

from ..backend_adapter import BackendAdapter, get_backend_adapter

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
            "collision check": bool(check_collision),
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
async def get_stl_file(filename: Path) -> Response:
    """Récupère un fichier STL depuis les assets (conforme SDK officiel)."""
    file_path = STL_ASSETS_DIR / filename

    if not file_path.exists():
        raise HTTPException(status_code=404, detail=f"STL file not found {file_path}")

    try:
        with open(file_path, "rb") as file:
            content = file.read()
            return Response(content, media_type="model/stl")
    except FileNotFoundError as e:
        raise HTTPException(
            status_code=404, detail=f"STL file not found {file_path}"
        ) from e
