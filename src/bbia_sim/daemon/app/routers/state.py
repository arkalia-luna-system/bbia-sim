"""Router pour les endpoints d'état du robot."""

import logging
import os
from datetime import datetime
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
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
            return None  # noqa: B110 - Retourner None si connexion SDK échoue (comportement attendu)

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
                    battery_level = None  # noqa: B110 - Valeur par défaut si lecture batterie échoue
            elif hasattr(media_mgr, "battery"):
                try:
                    battery_level = float(media_mgr.battery)
                except Exception:
                    battery_level = None  # noqa: B110 - Valeur par défaut si lecture batterie échoue

            if battery_level is not None:
                data["battery"] = battery_level

            # Température (optionnelle)
            temperature = None
            if hasattr(media_mgr, "get_temperature"):
                try:
                    temperature = float(media_mgr.get_temperature())
                except Exception:
                    temperature = None  # noqa: B110 - Valeur par défaut si lecture température échoue
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


@router.get("/full")
async def get_full_state(
    with_control_mode: bool = True,
    with_head_pose: bool = True,
    with_target_head_pose: bool = False,
    with_head_joints: bool = False,
    with_target_head_joints: bool = False,
    with_body_yaw: bool = True,
    with_target_body_yaw: bool = False,
    with_antenna_positions: bool = True,
    with_target_antenna_positions: bool = False,
    with_passive_joints: bool = False,
    use_pose_matrix: bool = False,
) -> dict[str, Any]:
    """Récupère l'état complet du robot avec paramètres optionnels (conforme SDK).

    Args:
        with_control_mode: Inclure le mode de contrôle moteur
        with_head_pose: Inclure la pose de la tête
        with_target_head_pose: Inclure la pose cible de la tête
        with_head_joints: Inclure les joints de la tête
        with_target_head_joints: Inclure les joints cibles de la tête
        with_body_yaw: Inclure le yaw du corps
        with_target_body_yaw: Inclure le yaw cible du corps
        with_antenna_positions: Inclure les positions des antennes
        with_target_antenna_positions: Inclure les positions cibles des antennes
        with_passive_joints: Inclure les joints passifs
        use_pose_matrix: Utiliser format matrice 4x4 pour les poses

    Returns:
        État complet du robot (champs optionnels selon paramètres)
    """

    from ...models import Matrix4x4Pose, XYZRPYPose

    logger.info("Récupération de l'état complet du robot")
    result: dict[str, Any] = {}

    try:
        from ....robot_factory import RobotFactory

        robot = RobotFactory.create_backend("mujoco")
        if robot:
            robot.connect()

            # Control mode
            if with_control_mode:
                if hasattr(robot, "get_motor_control_mode"):
                    mode = robot.get_motor_control_mode()
                    result["control_mode"] = (
                        mode.value if hasattr(mode, "value") else str(mode)
                    )

            # Head pose
            if with_head_pose:
                if hasattr(robot, "get_current_head_pose"):
                    pose = robot.get_current_head_pose()
                    if use_pose_matrix:
                        result["head_pose"] = Matrix4x4Pose.from_pose_array(
                            pose
                        ).model_dump()
                    else:
                        result["head_pose"] = XYZRPYPose.from_pose_array(
                            pose
                        ).model_dump()

            # Body yaw
            if with_body_yaw:
                if hasattr(robot, "get_current_body_yaw"):
                    result["body_yaw"] = float(robot.get_current_body_yaw())

            # Antenna positions
            if with_antenna_positions:
                if hasattr(robot, "get_present_antenna_joint_positions"):
                    positions = robot.get_present_antenna_joint_positions()
                    result["antennas_position"] = [
                        float(positions[0]),
                        float(positions[1]),
                    ]

            robot.disconnect()
    except Exception as e:
        logger.error(f"Erreur lors de la récupération de l'état: {e}")

    # Valeurs par défaut si manquantes
    if with_control_mode and "control_mode" not in result:
        result["control_mode"] = "enabled"
    if with_head_pose and "head_pose" not in result:
        if use_pose_matrix:
            result["head_pose"] = Matrix4x4Pose(
                m=(
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                )
            ).model_dump()
        else:
            result["head_pose"] = XYZRPYPose().model_dump()
    if with_body_yaw and "body_yaw" not in result:
        result["body_yaw"] = 0.0
    if with_antenna_positions and "antennas_position" not in result:
        result["antennas_position"] = [0.0, 0.0]

    result["timestamp"] = datetime.now().isoformat()
    return result


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
    status = (
        "good" if battery_level > 20 else "low" if battery_level > 10 else "critical"
    )
    estimated_time = (
        f"{battery_level * 0.8:.1f}h" if battery_level > 20 else "Recharge nécessaire"
    )

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
async def get_present_head_pose(
    use_pose_matrix: bool = False,
) -> dict[str, Any]:
    """Récupère la pose actuelle de la tête (conforme SDK).

    Args:
        use_pose_matrix: Si True, retourne matrice 4x4, sinon xyz+rpy

    Returns:
        Pose de la tête (format choisi)
    """

    logger.info(
        f"Récupération de la pose actuelle de la tête (matrix={use_pose_matrix})"
    )
    try:
        import numpy as np

        from ....robot_factory import RobotFactory
        from ...models import Matrix4x4Pose, XYZRPYPose

        robot = RobotFactory.create_backend("mujoco")
        if robot:
            robot.connect()
            if hasattr(robot, "get_current_head_pose"):
                pose = robot.get_current_head_pose()
                robot.disconnect()

                # Convertir en format demandé
                if use_pose_matrix:
                    if isinstance(pose, np.ndarray):
                        pose_matrix = Matrix4x4Pose.from_pose_array(pose)
                    else:
                        pose_matrix = Matrix4x4Pose(
                            m=(
                                1.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                1.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                1.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                1.0,
                            )
                        )
                    return pose_matrix.model_dump()
                else:
                    if isinstance(pose, np.ndarray):
                        pose_xyz = XYZRPYPose.from_pose_array(pose)
                    else:
                        pose_xyz = XYZRPYPose(x=0.0, y=0.0, z=0.0)
                    return pose_xyz.model_dump()

        # Fallback: simulation
        if use_pose_matrix:
            return {
                "m": (
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                )
            }
        return {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    except Exception as e:
        logger.error(f"Erreur lors de la récupération de la pose tête: {e}")
        if use_pose_matrix:
            return {
                "m": (
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                )
            }
        return {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


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


@router.websocket("/ws/full")
async def ws_full_state(
    websocket: WebSocket,
    frequency: float = 10.0,
    with_head_pose: bool = True,
    with_target_head_pose: bool = False,
    with_head_joints: bool = False,
    with_target_head_joints: bool = False,
    with_body_yaw: bool = True,
    with_target_body_yaw: bool = False,
    with_antenna_positions: bool = True,
    with_target_antenna_positions: bool = False,
    with_passive_joints: bool = False,
    use_pose_matrix: bool = False,
) -> None:
    """WebSocket endpoint pour stream l'état complet du robot (conforme SDK).

    Args:
        websocket: Connexion WebSocket
        frequency: Fréquence d'envoi en Hz (défaut: 10.0)
        with_head_pose: Inclure pose de la tête
        with_target_head_pose: Inclure pose cible de la tête
        with_head_joints: Inclure joints de la tête
        with_target_head_joints: Inclure joints cibles de la tête
        with_body_yaw: Inclure yaw du corps
        with_target_body_yaw: Inclure yaw cible du corps
        with_antenna_positions: Inclure positions des antennes
        with_target_antenna_positions: Inclure positions cibles des antennes
        with_passive_joints: Inclure joints passifs
        use_pose_matrix: Utiliser format matrice 4x4
    """
    import asyncio

    await websocket.accept()
    period = 1.0 / frequency

    try:
        while True:
            # Utiliser get_full_state pour cohérence
            full_state = await get_full_state(
                with_control_mode=True,
                with_head_pose=with_head_pose,
                with_target_head_pose=with_target_head_pose,
                with_head_joints=with_head_joints,
                with_target_head_joints=with_target_head_joints,
                with_body_yaw=with_body_yaw,
                with_target_body_yaw=with_target_body_yaw,
                with_antenna_positions=with_antenna_positions,
                with_target_antenna_positions=with_target_antenna_positions,
                with_passive_joints=with_passive_joints,
                use_pose_matrix=use_pose_matrix,
            )
            await websocket.send_json(full_state)
            await asyncio.sleep(period)

    except WebSocketDisconnect:
        logger.info("Client WebSocket déconnecté")
    except Exception as e:
        logger.error(f"Erreur WebSocket: {e}")


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
