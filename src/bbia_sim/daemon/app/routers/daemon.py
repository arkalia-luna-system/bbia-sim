"""Router pour les endpoints de contrôle du daemon."""

import logging
from datetime import datetime
from typing import Any

from fastapi import APIRouter, HTTPException, Query

from ...simulation_service import simulation_service

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/daemon")


class DaemonStatus:
    """Statut du daemon."""

    def __init__(
        self,
        status: str,
        simulation_running: bool,
        error: str | None = None,
    ) -> None:
        self.status = status
        self.simulation_running = simulation_running
        self.error = error

    def dict(self) -> dict[str, Any]:
        """Convertit en dictionnaire."""
        return {
            "status": self.status,
            "simulation_running": self.simulation_running,
            "error": self.error,
            "timestamp": datetime.now().isoformat(),
        }


@router.post("/start")
async def start_daemon(
    wake_up: bool = Query(False, description="Réveiller le robot au démarrage"),
) -> dict[str, Any]:
    """Démarre le daemon et la simulation.

    Args:
        wake_up: Si True, réveille le robot au démarrage

    Returns:
        Statut du démarrage

    Raises:
        HTTPException: En cas d'erreur
    """
    try:
        if simulation_service.is_simulation_ready():
            return {
                "status": "already_running",
                "message": "Daemon déjà démarré",
                "timestamp": datetime.now().isoformat(),
            }

        success = await simulation_service.start_simulation(headless=True)
        if not success:
            raise HTTPException(
                status_code=500, detail="Échec du démarrage de la simulation"
            )

        # Si wake_up demandé, utiliser comportement wake_up
        if wake_up:
            try:
                from ....robot_factory import RobotFactory

                robot = RobotFactory.create_backend("mujoco")
                if robot:
                    robot.connect()
                    # Utiliser comportement wake_up si disponible
                    if hasattr(robot, "wake_up"):
                        robot.wake_up()
                    robot.disconnect()
            except Exception as e:
                logger.warning(f"Échec wake_up: {e}")

        return {
            "status": "started",
            "message": "Daemon démarré avec succès",
            "wake_up": wake_up,
            "timestamp": datetime.now().isoformat(),
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Erreur lors du démarrage du daemon: {e}")
        raise HTTPException(status_code=500, detail=f"Erreur: {str(e)}") from e


@router.post("/stop")
async def stop_daemon(
    goto_sleep: bool = Query(False, description="Mettre le robot en veille à l'arrêt"),
) -> dict[str, Any]:
    """Arrête le daemon et la simulation.

    Args:
        goto_sleep: Si True, met le robot en veille avant arrêt

    Returns:
        Statut de l'arrêt

    Raises:
        HTTPException: En cas d'erreur
    """
    try:
        if goto_sleep:
            try:
                from ....robot_factory import RobotFactory

                robot = RobotFactory.create_backend("mujoco")
                if robot:
                    robot.connect()
                    # Utiliser comportement goto_sleep si disponible
                    if hasattr(robot, "goto_sleep"):
                        robot.goto_sleep()
                    robot.disconnect()
            except Exception as e:
                logger.warning(f"Échec goto_sleep: {e}")

        await simulation_service.stop_simulation()

        return {
            "status": "stopped",
            "message": "Daemon arrêté avec succès",
            "goto_sleep": goto_sleep,
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.error(f"Erreur lors de l'arrêt du daemon: {e}")
        raise HTTPException(status_code=500, detail=f"Erreur: {str(e)}") from e


@router.post("/restart")
async def restart_daemon() -> dict[str, Any]:
    """Redémarre le daemon.

    Returns:
        Statut du redémarrage

    Raises:
        HTTPException: En cas d'erreur
    """
    try:
        # Arrêter d'abord
        if simulation_service.is_simulation_ready():
            await simulation_service.stop_simulation()

        # Attendre un peu
        import asyncio

        await asyncio.sleep(0.5)

        # Redémarrer
        success = await simulation_service.start_simulation(headless=True)
        if not success:
            raise HTTPException(
                status_code=500, detail="Échec du redémarrage de la simulation"
            )

        return {
            "status": "restarted",
            "message": "Daemon redémarré avec succès",
            "timestamp": datetime.now().isoformat(),
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Erreur lors du redémarrage du daemon: {e}")
        raise HTTPException(status_code=500, detail=f"Erreur: {str(e)}") from e


@router.get("/status")
async def get_daemon_status() -> dict[str, Any]:
    """Récupère le statut actuel du daemon.

    Returns:
        Statut du daemon
    """
    try:
        is_running = simulation_service.is_simulation_ready()
        status = DaemonStatus(
            status="running" if is_running else "stopped",
            simulation_running=is_running,
            error=None,
        )
        return status.dict()
    except Exception as e:
        logger.error(f"Erreur lors de la récupération du statut: {e}")
        return DaemonStatus(
            status="error",
            simulation_running=False,
            error=str(e),
        ).dict()
