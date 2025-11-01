"""Router pour les endpoints de cinématique du robot."""

import logging
from pathlib import Path
from typing import Any

from fastapi import APIRouter, HTTPException, Response

from ....robot_factory import RobotFactory

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/kinematics", tags=["kinematics"])

# Chemin vers les assets STL
STL_ASSETS_DIR = (
    Path(__file__).parent.parent.parent.parent.parent
    / "src"
    / "bbia_sim"
    / "sim"
    / "assets"
    / "reachy_official"
)


@router.get("/info")
async def get_kinematics_info() -> dict[str, Any]:
    """Récupère les informations sur la cinématique du robot.

    Returns:
        Informations sur le moteur cinématique et la détection de collisions
    """
    try:
        # Tentative de récupération depuis le backend
        robot = RobotFactory.create_backend("mujoco")
        if robot:
            robot.connect()

            # Vérifier si le backend expose des infos cinématiques
            kinematics_engine = "MuJoCo"  # Par défaut pour simulation
            check_collision = False  # Par défaut

            if hasattr(robot, "kinematics_engine"):
                kinematics_engine = str(robot.kinematics_engine)
            elif hasattr(robot, "_backend") and hasattr(robot._backend, "kinematics_engine"):
                kinematics_engine = str(robot._backend.kinematics_engine)

            if hasattr(robot, "check_collision"):
                check_collision = bool(robot.check_collision)
            elif hasattr(robot, "_backend") and hasattr(robot._backend, "check_collision"):
                check_collision = bool(robot._backend.check_collision)

            robot.disconnect()

            return {
                "info": {
                    "engine": kinematics_engine,
                    "collision_check": check_collision,
                }
            }

    except Exception as e:
        logger.warning(f"Erreur lors de la récupération info cinématique: {e}")

    # Fallback : valeurs par défaut
    return {
        "info": {
            "engine": "MuJoCo",
            "collision_check": False,
        }
    }


@router.get("/urdf")
async def get_urdf() -> dict[str, str]:
    """Récupère la représentation URDF du robot.

    Returns:
        URDF du robot (peut être vide si non disponible)
    """
    try:
        robot = RobotFactory.create_backend("mujoco")
        if robot:
            robot.connect()

            urdf_content = ""
            if hasattr(robot, "get_urdf"):
                urdf_content = str(robot.get_urdf())
            elif hasattr(robot, "_backend") and hasattr(robot._backend, "get_urdf"):
                urdf_content = str(robot._backend.get_urdf())
            elif hasattr(robot, "urdf") or (
                hasattr(robot, "_backend") and hasattr(robot._backend, "urdf")
            ):
                urdf_attr = getattr(robot, "urdf", None) or (
                    getattr(robot._backend, "urdf", None) if hasattr(robot, "_backend") else None
                )
                if urdf_attr:
                    urdf_content = str(urdf_attr)

            robot.disconnect()

            # Si URDF trouvé, le retourner
            if urdf_content:
                return {"urdf": urdf_content}

    except Exception as e:
        logger.warning(f"Erreur lors de la récupération URDF: {e}")

    # Fallback : URDF vide (indique que non disponible actuellement)
    return {"urdf": ""}


@router.get("/stl/{filename:path}")
async def get_stl_file(filename: str) -> Response:
    """Récupère un fichier STL depuis les assets.

    Args:
        filename: Nom du fichier STL (chemin relatif depuis assets/reachy_official)

    Returns:
        Fichier STL en binaire avec type MIME approprié

    Raises:
        HTTPException: Si le fichier n'est pas trouvé
    """
    # Sécurisation : ne permettre que les fichiers .stl
    if not filename.endswith(".stl"):
        raise HTTPException(status_code=400, detail="Seuls les fichiers .stl sont autorisés")

    # Nettoyer le chemin pour éviter les directory traversal
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
