#!/usr/bin/env python3
"""Router FastAPI pour contrôles media (speaker, microphone, camera).

Endpoints pour contrôler le volume des haut-parleurs et microphone,
et activer/désactiver la caméra.
"""

import logging
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from ....robot_factory import RobotFactory

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/development/api/media", tags=["media"])


# Modèles Pydantic pour validation
class VolumeRequest(BaseModel):
    """Requête pour changer le volume."""

    volume: float = Field(..., ge=0.0, le=1.0, description="Volume entre 0.0 et 1.0")


class CameraToggleRequest(BaseModel):
    """Requête pour activer/désactiver la caméra."""

    enabled: bool = Field(
        ..., description="Activer (True) ou désactiver (False) la caméra"
    )


class MediaStatusResponse(BaseModel):
    """Réponse avec statut media."""

    speaker_volume: float = Field(..., ge=0.0, le=1.0)
    microphone_volume: float = Field(..., ge=0.0, le=1.0)
    camera_enabled: bool
    speaker_active: bool
    microphone_active: bool


# Variables globales pour état (fallback simulation)
_speaker_volume: float = 0.5
_microphone_volume: float = 0.5
_camera_enabled: bool = True
_speaker_active: bool = True
_microphone_active: bool = True


def _get_robot_media() -> Any | None:
    """Récupère robot.media si disponible.

    Returns:
        robot.media ou None si non disponible (simulation)

    """
    try:
        # Essayer d'abord avec MuJoCo backend
        robot = RobotFactory.create_backend("mujoco")
        if robot is None:
            # Fallback: essayer avec reachy_mini backend
            robot = RobotFactory.create_backend("reachy_mini")

        if robot is None:
            logger.debug("Aucun backend robot disponible (simulation)")
            return None

        # Connecter si nécessaire
        if not hasattr(robot, "_connected") or not robot._connected:
            try:
                robot.connect()
            except Exception as e:
                logger.debug("Connexion robot échouée (simulation): %s", e)
                return None

        # Récupérer robot.media
        # Le SDK Reachy Mini expose robot.media
        # Pour MuJoCo, on utilise le shim simulation_shims.SimulationMediaModule
        if hasattr(robot, "robot") and robot.robot is not None:
            # Backend reachy_mini: robot.robot.media
            if hasattr(robot.robot, "media"):
                return robot.robot.media
        elif hasattr(robot, "media"):
            # Backend MuJoCo avec shim: robot.media directement
            return robot.media

        logger.debug("robot.media non disponible (simulation)")
        return None

    except Exception as e:
        logger.debug("Erreur récupération robot.media (simulation): %s", e)
        return None


@router.post("/speaker/volume", response_model=dict[str, Any])
async def set_speaker_volume(request: VolumeRequest) -> dict[str, Any]:
    """Définit le volume du haut-parleur.

    Args:
        request: Requête avec volume (0.0-1.0)

    Returns:
        Confirmation avec nouveau volume

    Raises:
        HTTPException: Si erreur lors de la mise à jour
    """
    try:
        global _speaker_volume
        _speaker_volume = request.volume

        # Appliquer volume au robot réel si disponible
        media = _get_robot_media()
        if media:
            speaker = getattr(media, "speaker", None)
            if speaker:
                try:
                    if hasattr(speaker, "set_volume"):
                        speaker.set_volume(request.volume)
                    elif hasattr(speaker, "volume"):
                        speaker.volume = request.volume
                    logger.debug("Volume speaker appliqué au robot: %s", request.volume)
                except Exception as e:
                    logger.warning("Erreur application volume speaker au robot: %s", e)

        logger.info("Volume haut-parleur défini à %s", request.volume)
        return {"status": "success", "volume": _speaker_volume}
    except Exception as e:
        logger.exception("Erreur lors de la définition du volume: %s", e)
        raise HTTPException(status_code=500, detail=str(e)) from e


@router.post("/microphone/volume", response_model=dict[str, Any])
async def set_microphone_volume(request: VolumeRequest) -> dict[str, Any]:
    """Définit le volume du microphone.

    Args:
        request: Requête avec volume (0.0-1.0)

    Returns:
        Confirmation avec nouveau volume

    Raises:
        HTTPException: Si erreur lors de la mise à jour
    """
    try:
        global _microphone_volume
        _microphone_volume = request.volume

        # Appliquer volume au robot réel si disponible
        media = _get_robot_media()
        if media:
            microphone = getattr(media, "microphone", None)
            if microphone:
                try:
                    if hasattr(microphone, "set_volume"):
                        microphone.set_volume(request.volume)
                    elif hasattr(microphone, "volume"):
                        microphone.volume = request.volume
                    logger.debug(
                        f"Volume microphone appliqué au robot: {request.volume}"
                    )
                except Exception as e:
                    logger.warning(
                        f"Erreur application volume microphone au robot: {e}"
                    )

        logger.info("Volume microphone défini à %s", request.volume)
        return {"status": "success", "volume": _microphone_volume}
    except Exception as e:
        logger.exception("Erreur lors de la définition du volume: %s", e)
        raise HTTPException(status_code=500, detail=str(e)) from e


@router.post("/camera/toggle", response_model=dict[str, Any])
async def toggle_camera(request: CameraToggleRequest) -> dict[str, Any]:
    """Active ou désactive la caméra.

    Args:
        request: Requête avec enabled (True/False)

    Returns:
        Confirmation avec nouvel état

    Raises:
        HTTPException: Si erreur lors de la mise à jour
    """
    try:
        global _camera_enabled
        _camera_enabled = request.enabled

        # Activer/désactiver caméra au robot réel si disponible
        media = _get_robot_media()
        if media:
            camera = getattr(media, "camera", None)
            if camera:
                try:
                    if request.enabled:
                        if hasattr(camera, "start"):
                            camera.start()
                        elif hasattr(camera, "enable"):
                            camera.enable()
                        elif hasattr(camera, "enabled"):
                            camera.enabled = True
                    else:
                        if hasattr(camera, "stop"):
                            camera.stop()
                        elif hasattr(camera, "disable"):
                            camera.disable()
                        elif hasattr(camera, "enabled"):
                            camera.enabled = False
                    logger.debug(
                        f"Caméra {'activée' if request.enabled else 'désactivée'} au robot"
                    )
                except Exception as e:
                    logger.warning("Erreur toggle caméra au robot: %s", e)

        logger.info("Caméra %s", "activée" if _camera_enabled else "désactivée")
        return {"status": "success", "enabled": _camera_enabled}
    except Exception as e:
        logger.exception("Erreur lors du toggle caméra: %s", e)
        raise HTTPException(status_code=500, detail=str(e)) from e


@router.get("/status", response_model=MediaStatusResponse)
async def get_media_status() -> MediaStatusResponse:
    """Récupère le statut actuel des contrôles media.

    Returns:
        Statut complet (volumes, états activés/désactivés)

    Raises:
        HTTPException: Si erreur lors de la récupération
    """
    try:
        # Récupérer statut réel du robot si disponible
        media = _get_robot_media()
        speaker_volume = _speaker_volume
        microphone_volume = _microphone_volume
        camera_enabled = _camera_enabled
        speaker_active = _speaker_active
        microphone_active = _microphone_active

        if media:
            # Récupérer statut speaker
            speaker = getattr(media, "speaker", None)
            if speaker:
                try:
                    if hasattr(speaker, "get_volume"):
                        speaker_volume = float(speaker.get_volume())
                    elif hasattr(speaker, "volume"):
                        speaker_volume = float(speaker.volume)
                    if hasattr(speaker, "is_active"):
                        speaker_active = speaker.is_active()
                    elif hasattr(speaker, "active"):
                        speaker_active = speaker.active
                except Exception as e:
                    logger.debug("Erreur lecture statut speaker: %s", e)

            # Récupérer statut microphone
            microphone = getattr(media, "microphone", None)
            if microphone:
                try:
                    if hasattr(microphone, "get_volume"):
                        microphone_volume = float(microphone.get_volume())
                    elif hasattr(microphone, "volume"):
                        microphone_volume = float(microphone.volume)
                    if hasattr(microphone, "is_active"):
                        microphone_active = microphone.is_active()
                    elif hasattr(microphone, "active"):
                        microphone_active = microphone.active
                except Exception as e:
                    logger.debug("Erreur lecture statut microphone: %s", e)

            # Récupérer statut camera
            camera = getattr(media, "camera", None)
            if camera:
                try:
                    if hasattr(camera, "is_enabled"):
                        camera_enabled = camera.is_enabled()
                    elif hasattr(camera, "enabled"):
                        camera_enabled = camera.enabled
                except Exception as e:
                    logger.debug("Erreur lecture statut camera: %s", e)

        return MediaStatusResponse(
            speaker_volume=speaker_volume,
            microphone_volume=microphone_volume,
            camera_enabled=camera_enabled,
            speaker_active=speaker_active,
            microphone_active=microphone_active,
        )
    except Exception as e:
        logger.exception("Erreur lors de la récupération du statut: %s", e)
        raise HTTPException(status_code=500, detail=str(e)) from e
