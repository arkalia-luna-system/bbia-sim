"""Router pour les endpoints d'état du robot."""

import logging
import os
from datetime import datetime
from typing import Any

from fastapi import APIRouter
from pydantic import BaseModel

from ....robot_factory import RobotFactory
from ...simulation_service import simulation_service

logger = logging.getLogger(__name__)

router = APIRouter()


class RobotState(BaseModel):
    """Modèle pour l'état du robot."""

    position: dict[str, float]
    status: str
    battery: float
    temperature: float
    timestamp: str


def _read_sdk_telemetry() -> dict[str, Any] | None:
    """Lit la télémétrie réelle via le SDK Reachy Mini si disponible.

    Comportement:
    - Activé uniquement si l'env BBIA_TELEMETRY_SDK est défini à true/1/yes
    - Timeout configurable via BBIA_TELEMETRY_TIMEOUT (secondes, défaut 1.0)
    - Aucun échec bloquant: en cas d'erreur ou indisponibilité, retourne None
    """
    try:
        use_sdk = os.environ.get("BBIA_TELEMETRY_SDK", "false").lower() in {
            "1",
            "true",
            "yes",
        }
        if not use_sdk:
            return None

        try:
            timeout = float(os.environ.get("BBIA_TELEMETRY_TIMEOUT", "1.0") or 1.0)
        except Exception:
            timeout = 1.0  # noqa: B110 - Valeur par défaut si parsing timeout échoue

        # Créer un backend SDK avec tentative connexion rapide
        backend = RobotFactory.create_backend(
            "reachy_mini",
            use_sim=False,
            timeout=timeout,
            spawn_daemon=False,
        )
        if backend is None:
            return None

        try:
            backend.connect()
        except Exception:
            # Ne pas bloquer si la connexion échoue
            return (
                None  # noqa: B110 - Retourner None si connexion SDK échoue (comportement attendu)
            )

        try:
            # Lecture batterie/IMU si exposés via robot.media
            data: dict[str, Any] = {}
            # Certains backends exposent un objet robot; typage dynamique pour mypy
            robot_obj: Any = getattr(backend, "robot", None)
            if robot_obj is None:
                return None

            # Le SDK expose généralement robot.media
            media_mgr = getattr(robot_obj, "media", None)
            if media_mgr is None:
                return None

            # Batterie (si disponible)
            battery_level = None
            if hasattr(media_mgr, "get_battery_level"):
                try:
                    battery_level = float(media_mgr.get_battery_level())
                except Exception:
                    battery_level = (
                        None  # noqa: B110 - Valeur par défaut si lecture batterie échoue
                    )
            elif hasattr(media_mgr, "battery"):
                try:
                    battery_level = float(media_mgr.battery)
                except Exception:
                    battery_level = (
                        None  # noqa: B110 - Valeur par défaut si lecture batterie échoue
                    )

            if battery_level is not None:
                data["battery"] = battery_level

            # Température (optionnelle)
            temperature = None
            if hasattr(media_mgr, "get_temperature"):
                try:
                    temperature = float(media_mgr.get_temperature())
                except Exception:
                    temperature = (
                        None  # noqa: B110 - Valeur par défaut si lecture température échoue
                    )
            if temperature is not None:
                data["temperature"] = temperature

            # IMU (si exposé via robot.io ou robot.media)
            imu = None
            if hasattr(robot_obj, "io") and robot_obj.io:
                io_mgr = robot_obj.io
                try:
                    if hasattr(io_mgr, "get_imu"):
                        imu = io_mgr.get_imu()  # doit renvoyer dict-like
                except Exception:
                    imu = None  # noqa: B110 - Valeur par défaut si lecture IMU échoue
            if imu and isinstance(imu, dict):
                data["imu"] = imu

            return data if data else None
        finally:
            try:
                backend.disconnect()
            except Exception:
                pass  # noqa: B110 - Ignorer erreur déconnexion (déjà déconnecté ou non critique)
    except Exception:
        return None  # noqa: B110 - Retourner None si lecture télémétrie SDK échoue (fallback simulation)


class BatteryInfo(BaseModel):
    """Modèle pour les informations de batterie."""

    level: float
    unit: str
    status: str
    estimated_time: str


@router.get("/full", response_model=RobotState)
async def get_full_state() -> RobotState:
    """Récupère l'état complet du robot.

    Returns:
        État complet du robot

    """
    logger.info("Récupération de l'état complet du robot")

    # Récupération des données depuis la simulation
    robot_state = simulation_service.get_robot_state()
    robot_state.get("joint_positions", {})

    # Valeurs par défaut (simulation)
    battery_level: float = 85.5
    temperature_c: float = 25.5

    # Tentative lecture SDK (non bloquant)
    sdk = _read_sdk_telemetry()
    if sdk:
        battery_level = float(sdk.get("battery", battery_level))
        temperature_c = float(sdk.get("temperature", temperature_c))

    return RobotState(
        position={"x": 0.0, "y": 0.0, "z": 0.0},
        status="ready" if simulation_service.is_simulation_ready() else "not_ready",
        battery=battery_level,
        temperature=temperature_c,
        timestamp=datetime.now().isoformat(),
    )


@router.get("/position")
async def get_position() -> dict[str, Any]:
    """Récupère la position actuelle du robot.

    Returns:
        Position du robot

    """
    logger.info("Récupération de la position du robot")

    return {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "timestamp": datetime.now().isoformat(),
    }


@router.get("/battery", response_model=BatteryInfo)
async def get_battery_level() -> BatteryInfo:
    """Récupère le niveau de batterie.

    Returns:
        Informations sur la batterie

    """
    logger.info("Récupération du niveau de batterie")

    # Par défaut simulation
    battery_level = 85.5

    # Tentative lecture SDK
    sdk = _read_sdk_telemetry()
    if sdk and "battery" in sdk:
        try:
            battery_level = float(sdk["battery"])  # type: ignore[index]
        except Exception:
            pass  # noqa: B110 - Ignorer erreur parsing batterie (utiliser valeur par défaut)
    status = "good" if battery_level > 20 else "low" if battery_level > 10 else "critical"
    estimated_time = f"{battery_level * 0.8:.1f}h" if battery_level > 20 else "Recharge nécessaire"

    return BatteryInfo(
        level=battery_level,
        unit="percent",
        status=status,
        estimated_time=estimated_time,
    )


@router.get("/temperature")
async def get_temperature() -> dict[str, Any]:
    """Récupère la température du robot.

    Returns:
        Température du robot

    """
    logger.info("Récupération de la température")

    # Par défaut simulation
    temperature_c = 25.5

    # Tentative lecture SDK
    sdk = _read_sdk_telemetry()
    if sdk and "temperature" in sdk:
        try:
            temperature_c = float(sdk["temperature"])  # type: ignore[index]
        except Exception:
            pass  # noqa: B110 - Ignorer erreur parsing température (utiliser valeur par défaut)

    return {
        "temperature": temperature_c,
        "unit": "celsius",
        "status": "normal",
        "timestamp": datetime.now().isoformat(),
    }


@router.get("/status")
async def get_status() -> dict[str, Any]:
    """Récupère le statut général du robot.

    Returns:
        Statut du robot

    """
    logger.info("Récupération du statut du robot")

    return {
        "status": "ready",
        "mode": "autonomous",
        "errors": [],
        "warnings": [],
        "timestamp": datetime.now().isoformat(),
    }


@router.post("/simulation/start")
async def start_simulation() -> dict[str, Any]:
    """Démarre la simulation MuJoCo."""
    logger.info("Démarrage de la simulation MuJoCo")

    try:
        success = await simulation_service.start_simulation(headless=True)
        if success:
            return {
                "status": "started",
                "message": "Simulation MuJoCo démarrée avec succès",
                "timestamp": datetime.now().isoformat(),
            }
        else:
            return {
                "status": "error",
                "message": "Échec du démarrage de la simulation",
                "timestamp": datetime.now().isoformat(),
            }
    except Exception as e:
        logger.error(f"Erreur lors du démarrage de la simulation : {e}")
        return {
            "status": "error",
            "message": f"Erreur : {str(e)}",
            "timestamp": datetime.now().isoformat(),
        }


@router.post("/simulation/stop")
async def stop_simulation() -> dict[str, Any]:
    """Arrête la simulation MuJoCo."""
    logger.info("Arrêt de la simulation MuJoCo")

    try:
        await simulation_service.stop_simulation()
        return {
            "status": "stopped",
            "message": "Simulation MuJoCo arrêtée avec succès",
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.error(f"Erreur lors de l'arrêt de la simulation : {e}")
        return {
            "status": "error",
            "message": f"Erreur : {str(e)}",
            "timestamp": datetime.now().isoformat(),
        }


@router.get("/joints")
async def get_joint_states() -> dict[str, Any]:
    logger.info("Récupération de l'état des articulations")

    # Récupération des positions depuis la simulation
    joint_positions = simulation_service.get_joint_positions()

    # Formatage des données pour l'API
    joints = {}
    for joint_name, position in joint_positions.items():
        joints[joint_name] = {
            "position": position,
            "velocity": 0.0,  # À implémenter
            "effort": 0.0,  # À implémenter
        }

    return {"joints": joints, "timestamp": datetime.now().isoformat()}


@router.get("/present_head_pose")
async def get_present_head_pose() -> dict[str, Any]:
    """Récupère la pose actuelle de la tête.

    Returns:
        Pose de la tête (4x4 matrix ou xyz + roll/pitch/yaw)
    """
    logger.info("Récupération de la pose actuelle de la tête")
    try:
        from ....robot_factory import RobotFactory

        robot = RobotFactory.create_backend("mujoco")
        if robot:
            robot.connect()
            if hasattr(robot, "get_current_head_pose"):
                pose = robot.get_current_head_pose()
                robot.disconnect()
                return {
                    "head_pose": pose.tolist() if hasattr(pose, "tolist") else pose,
                    "timestamp": datetime.now().isoformat(),
                }

        # Fallback: simulation
        return {
            "head_pose": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.error(f"Erreur lors de la récupération de la pose tête: {e}")
        return {
            "head_pose": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "timestamp": datetime.now().isoformat(),
        }


@router.get("/present_body_yaw")
async def get_present_body_yaw() -> dict[str, Any]:
    """Récupère le yaw actuel du corps (en radians).

    Returns:
        Yaw du corps en radians
    """
    logger.info("Récupération du yaw corps")
    try:
        from ....robot_factory import RobotFactory

        robot = RobotFactory.create_backend("mujoco")
        if robot:
            robot.connect()
            # Essayer get_current_body_yaw ou via get_joint_positions
            if hasattr(robot, "get_current_body_yaw"):
                yaw = robot.get_current_body_yaw()
            elif hasattr(robot, "get_joint_positions"):
                positions = robot.get_joint_positions()
                yaw = positions.get("yaw_body", 0.0)
            else:
                yaw = 0.0
            robot.disconnect()
            return {
                "body_yaw": float(yaw),
                "unit": "radians",
                "timestamp": datetime.now().isoformat(),
            }

        # Fallback: simulation
        return {
            "body_yaw": 0.0,
            "unit": "radians",
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.error(f"Erreur lors de la récupération du yaw corps: {e}")
        return {
            "body_yaw": 0.0,
            "unit": "radians",
            "timestamp": datetime.now().isoformat(),
        }


@router.get("/present_antenna_joint_positions")
async def get_present_antenna_joint_positions() -> dict[str, Any]:
    """Récupère les positions actuelles des antennes (en radians).

    Returns:
        Positions des antennes (left, right) en radians
    """
    logger.info("Récupération des positions antennes")
    try:
        from ....robot_factory import RobotFactory

        robot = RobotFactory.create_backend("mujoco")
        if robot:
            robot.connect()
            if hasattr(robot, "get_present_antenna_joint_positions"):
                positions = robot.get_present_antenna_joint_positions()
            elif hasattr(robot, "get_joint_positions"):
                all_pos = robot.get_joint_positions()
                positions = [
                    all_pos.get("left_antenna", 0.0),
                    all_pos.get("right_antenna", 0.0),
                ]
            else:
                positions = [0.0, 0.0]
            robot.disconnect()

            # Format tuple comme officiel (left, right)
            left = float(positions[0] if isinstance(positions, list) else positions[0])
            right = float(positions[1] if isinstance(positions, list) else positions[1])

            return {
                "antennas": (left, right),
                "left": left,
                "right": right,
                "unit": "radians",
                "timestamp": datetime.now().isoformat(),
            }

        # Fallback: simulation
        return {
            "antennas": (0.0, 0.0),
            "left": 0.0,
            "right": 0.0,
            "unit": "radians",
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.error(f"Erreur lors de la récupération des positions antennes: {e}")
        return {
            "antennas": (0.0, 0.0),
            "left": 0.0,
            "right": 0.0,
            "unit": "radians",
            "timestamp": datetime.now().isoformat(),
        }


@router.get("/sensors")
async def get_sensor_data() -> dict[str, Any]:
    """Récupère les données des capteurs.

    Returns:
        Données des capteurs

    """
    logger.info("Récupération des données des capteurs")

    imu_data = {
        "acceleration": {"x": 0.0, "y": 0.0, "z": 9.81},
        "gyroscope": {"x": 0.0, "y": 0.0, "z": 0.0},
        "magnetometer": {"x": 0.0, "y": 0.0, "z": 0.0},
    }

    sdk = _read_sdk_telemetry()
    if sdk and "imu" in sdk and isinstance(sdk["imu"], dict):  # type: ignore[index]
        try:
            imu_data = sdk["imu"]  # type: ignore[assignment,index]
        except Exception:
            pass  # noqa: B110 - Ignorer erreur parsing IMU (utiliser valeur par défaut)

    return {
        "camera": {"status": "active", "resolution": "640x480", "fps": 30},
        "microphone": {"status": "active", "level": 0.3},
        "imu": imu_data,
        "timestamp": datetime.now().isoformat(),
    }
