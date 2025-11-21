"""Router pour les endpoints de l'écosystème BBIA-SIM - Phase 3."""

import asyncio
import logging
import time
from datetime import datetime
from typing import Any

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel, Field

from ....bbia_behavior import BBIABehaviorManager
from ....bbia_emotions import BBIAEmotions
from ....robot_factory import RobotFactory

logger = logging.getLogger(__name__)

router = APIRouter()

# Temps de démarrage de l'application pour calcul uptime
_app_start_time: float | None = None

# Référence au gestionnaire WebSocket (sera initialisé à la demande)
_ws_manager: Any | None = None


def get_app_start_time() -> float:
    """Récupère ou initialise le temps de démarrage de l'application."""
    global _app_start_time
    if _app_start_time is None:
        _app_start_time = time.time()
    return _app_start_time


def format_uptime(seconds: float) -> str:
    """Formate le temps en format HH:MM:SS."""
    hours = int(seconds // 3600)
    minutes = int((seconds % 3600) // 60)
    secs = int(seconds % 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


# OPTIMISATION RAM: Import module-level au lieu de dynamique (évite imports répétés)
try:
    from ...ws.telemetry import manager as _telemetry_manager_import
except ImportError:
    _telemetry_manager_import = None  # type: ignore[assignment]


def get_ws_manager() -> Any | None:
    """Récupère le gestionnaire WebSocket de télémétrie.

    OPTIMISATION RAM: Cache résultat + import module-level au lieu de dynamique.
    """
    global _ws_manager
    if _ws_manager is None:
        # OPTIMISATION RAM: Utiliser import module-level au lieu de dynamique
        if _telemetry_manager_import is not None:
            _ws_manager = _telemetry_manager_import
        else:
            logger.debug("Gestionnaire WebSocket non disponible")
            _ws_manager = None
    return _ws_manager


# OPTIMISATION RAM: Cache résultat avec TTL 1s (évite appels répétés)
_active_connections_cache: int | None = None
_active_connections_cache_time: float = 0.0
_ACTIVE_CONNECTIONS_CACHE_TTL = 1.0  # 1 seconde


def get_active_connections() -> int:
    """Récupère le nombre de connexions WebSocket actives.

    OPTIMISATION RAM: Cache résultat avec TTL 1s pour éviter appels répétés.

    Returns:
        Nombre de connexions WebSocket actives (toutes routes confondues)

    """
    global _active_connections_cache, _active_connections_cache_time
    current_time = time.time()

    # Utiliser cache si valide (TTL 1s)
    if (
        _active_connections_cache is not None
        and (current_time - _active_connections_cache_time)
        < _ACTIVE_CONNECTIONS_CACHE_TTL
    ):
        return _active_connections_cache

    # Calculer valeur réelle
    manager = get_ws_manager()
    result = 0
    if manager is not None and hasattr(manager, "active_connections"):
        result = len(manager.active_connections)

    # Mettre en cache
    _active_connections_cache = result
    _active_connections_cache_time = current_time
    return result


# Modèles Pydantic pour l'API publique
class EmotionResponse(BaseModel):
    """Réponse pour les émotions BBIA."""

    emotion: str = Field(..., description="Nom de l'émotion appliquée")
    intensity: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Intensité de l'émotion (0.0-1.0)",
    )
    duration: float = Field(..., gt=0, description="Durée en secondes")
    joints_affected: list[str] = Field(..., description="Joints affectés par l'émotion")
    timestamp: str = Field(..., description="Timestamp de l'application")
    status: str = Field(..., description="Statut de l'opération")


class BehaviorResponse(BaseModel):
    """Réponse pour les comportements BBIA."""

    behavior: str = Field(..., description="Nom du comportement exécuté")
    parameters: dict[str, Any] = Field(..., description="Paramètres du comportement")
    estimated_duration: float = Field(..., description="Durée estimée en secondes")
    timestamp: str = Field(..., description="Timestamp de l'exécution")
    status: str = Field(..., description="Statut de l'opération")


class RobotCapabilities(BaseModel):
    """Capacités du robot Reachy Mini."""

    model: str = Field(..., description="Modèle du robot")
    joints: int = Field(..., description="Nombre d'articulations")
    emotions: list[str] = Field(..., description="Émotions disponibles")
    behaviors: list[str] = Field(..., description="Comportements disponibles")
    backends: list[str] = Field(..., description="Backends disponibles")
    simulation_mode: bool = Field(..., description="Mode simulation activé")


class APIStatus(BaseModel):
    """Statut de l'API BBIA-SIM."""

    version: str = Field(..., description="Version de l'API")
    status: str = Field(..., description="Statut général")
    uptime: str = Field(..., description="Temps de fonctionnement")
    robot_connected: bool = Field(..., description="Robot connecté")
    simulation_running: bool = Field(..., description="Simulation en cours")
    active_connections: int = Field(..., description="Connexions WebSocket actives")


@router.get("/capabilities", response_model=RobotCapabilities)
async def get_robot_capabilities() -> RobotCapabilities:
    """Récupère les capacités du robot Reachy Mini.

    Returns:
        Informations sur les capacités du robot

    Raises:
        HTTPException: En cas d'erreur

    """
    try:
        # Initialiser les modules BBIA
        emotions_module = BBIAEmotions()
        behavior_manager = BBIABehaviorManager()

        return RobotCapabilities(
            model="Reachy Mini Wireless",
            joints=16,
            emotions=list(emotions_module.emotions.keys()),
            behaviors=list(behavior_manager.behaviors.keys()),
            backends=["mujoco", "reachy_mini", "reachy"],
            simulation_mode=True,
        )
    except Exception as e:
        logger.exception("Erreur lors de la récupération des capacités: %s", e)
        raise HTTPException(
            status_code=500,
            detail="Erreur lors de la récupération des capacités",
        ) from e


@router.get("/status", response_model=APIStatus)
async def get_api_status() -> APIStatus:
    """Récupère le statut de l'API BBIA-SIM.

    Returns:
        Statut détaillé de l'API

    Raises:
        HTTPException: En cas d'erreur

    """
    try:
        # Vérifier la connexion robot
        robot_connected = False
        simulation_running = False

        try:
            robot = RobotFactory.create_backend("mujoco")
            robot_connected = robot is not None
            simulation_running = True
        except (ImportError, RuntimeError, ValueError):
            robot_connected = False
            simulation_running = False

        # Calculer uptime réel
        start_time = get_app_start_time()
        uptime_seconds = time.time() - start_time
        uptime_formatted = format_uptime(uptime_seconds)

        # Récupérer nombre de connexions actives
        active_conn = get_active_connections()

        return APIStatus(
            version="1.2.0",
            status="running",
            uptime=uptime_formatted,
            robot_connected=robot_connected,
            simulation_running=simulation_running,
            active_connections=active_conn,
        )
    except Exception as e:
        logger.exception("Erreur lors de la récupération du statut: %s", e)
        raise HTTPException(
            status_code=500,
            detail="Erreur lors de la récupération du statut",
        ) from e


@router.post("/emotions/apply", response_model=EmotionResponse)
async def apply_emotion(
    emotion: str = Query(..., description="Nom de l'émotion à appliquer"),
    intensity: float = Query(0.5, ge=0.0, le=1.0, description="Intensité de l'émotion"),
    duration: float = Query(5.0, gt=0, description="Durée en secondes"),
    joint: str | None = Query(None, description="Joint spécifique à animer"),
) -> EmotionResponse:
    """Applique une émotion BBIA au robot.

    Args:
        emotion: Nom de l'émotion (happy, sad, angry, etc.)
        intensity: Intensité de l'émotion (0.0-1.0)
        duration: Durée en secondes
        joint: Joint spécifique à animer (optionnel)

    Returns:
        Confirmation de l'application de l'émotion

    Raises:
        HTTPException: En cas d'erreur

    """
    try:
        # Initialiser le module émotions
        emotions_module = BBIAEmotions()

        # Vérifier que l'émotion existe
        available_emotions = list(emotions_module.emotions.keys())
        if emotion not in available_emotions:
            raise HTTPException(
                status_code=400,
                detail=f"Émotion '{emotion}' non disponible. Émotions disponibles: {available_emotions}",
            )

        # Appliquer l'émotion (simulation)
        emotions_module.current_emotion = emotion
        emotions_module.emotion_intensity = intensity

        # Simuler les joints affectés
        joints_affected = ["yaw_body"]  # Joint principal pour les émotions

        return EmotionResponse(
            emotion=emotion,
            intensity=intensity,
            duration=duration,
            joints_affected=joints_affected,
            timestamp=datetime.now().isoformat(),
            status="applied",
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.exception("Erreur lors de l'application de l'émotion: %s", e)
        raise HTTPException(
            status_code=500,
            detail="Erreur lors de l'application de l'émotion",
        ) from e


@router.post("/behaviors/execute", response_model=BehaviorResponse)
async def execute_behavior(
    behavior: str = Query(..., description="Nom du comportement à exécuter"),
    intensity: float = Query(
        1.0,
        ge=0.0,
        le=2.0,
        description="Intensité du comportement",
    ),
    duration: float | None = Query(
        None,
        gt=0,
        description="Durée personnalisée en secondes",
    ),
) -> BehaviorResponse:
    """Exécute un comportement BBIA.

    Args:
        behavior: Nom du comportement (wake_up, greeting, etc.)
        intensity: Intensité du comportement (0.0-2.0)
        duration: Durée personnalisée (optionnel)

    Returns:
        Confirmation de l'exécution du comportement

    Raises:
        HTTPException: En cas d'erreur

    """
    try:
        # Initialiser le module comportements
        behavior_manager = BBIABehaviorManager()

        # Vérifier que le comportement existe
        available_behaviors = list(behavior_manager.behaviors.keys())
        if behavior not in available_behaviors:
            raise HTTPException(
                status_code=400,
                detail=f"Comportement '{behavior}' non disponible. Comportements disponibles: {available_behaviors}",
            )

        # Exécuter le comportement (simulation)
        success = behavior_manager.execute_behavior(behavior)
        estimated_duration = 5.0  # Durée estimée par défaut

        return BehaviorResponse(
            behavior=behavior,
            parameters={"intensity": intensity, "duration": duration},
            estimated_duration=estimated_duration,
            timestamp=datetime.now().isoformat(),
            status="executed" if success else "failed",
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.exception("Erreur lors de l'exécution du comportement: %s", e)
        raise HTTPException(
            status_code=500,
            detail="Erreur lors de l'exécution du comportement",
        ) from e


@router.get("/emotions/available")
async def get_available_emotions() -> dict[str, Any]:
    """Récupère la liste des émotions disponibles.

    Returns:
        Liste des émotions disponibles avec descriptions

    Raises:
        HTTPException: En cas d'erreur

    """
    try:
        emotions_module = BBIAEmotions()
        emotions = list(emotions_module.emotions.keys())

        # Descriptions des émotions
        descriptions = {
            "neutral": "État neutre, pas d'émotion particulière",
            "happy": "Joie et bonheur, mouvements énergiques",
            "sad": "Tristesse, mouvements lents et mélancoliques",
            "angry": "Colère, mouvements brusques et agressifs",
            "surprised": "Surprise, mouvements rapides et saccadés",
            "confused": "Confusion, mouvements hésitants",
            "determined": "Détermination, mouvements fermes et précis",
            "nostalgic": "Nostalgie, mouvements doux et rêveurs",
            "proud": "Fierté, mouvements majestueux",
            "curious": "Curiosité, mouvements d'exploration",
            "excited": "Excitation, mouvements rapides et énergiques",
            "fearful": "Peur, mouvements tremblants et reculés",
        }

        return {
            "emotions": emotions,
            "descriptions": {
                emotion: descriptions.get(emotion, "Description non disponible")
                for emotion in emotions
            },
            "total_count": len(emotions),
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.exception("Erreur lors de la récupération des émotions: %s", e)
        raise HTTPException(
            status_code=500,
            detail="Erreur lors de la récupération des émotions",
        ) from e


@router.get("/behaviors/available")
async def get_available_behaviors() -> dict[str, Any]:
    """Récupère la liste des comportements disponibles.

    Returns:
        Liste des comportements disponibles avec descriptions

    Raises:
        HTTPException: En cas d'erreur

    """
    try:
        behavior_manager = BBIABehaviorManager()
        behaviors = list(behavior_manager.behaviors.keys())

        # Descriptions des comportements
        descriptions = {
            "wake_up": "Séquence de réveil du robot",
            "greeting": "Salutation amicale",
            "nod": "Hochement de tête",
            "shake_head": "Mouvement de dénégation",
            "look_around": "Regarder autour de soi",
            "rest": "Position de repos",
            "dance": "Séquence de danse",
            "wave": "Faire un signe de la main",
        }

        return {
            "behaviors": behaviors,
            "descriptions": {
                behavior: descriptions.get(behavior, "Description non disponible")
                for behavior in behaviors
            },
            "total_count": len(behaviors),
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.exception("Erreur lors de la récupération des comportements: %s", e)
        raise HTTPException(
            status_code=500,
            detail="Erreur lors de la récupération des comportements",
        ) from e


@router.get("/demo/modes")
async def get_demo_modes() -> dict[str, Any]:
    """Récupère les modes de démonstration disponibles.

    Returns:
        Modes de démonstration disponibles

    Raises:
        HTTPException: En cas d'erreur

    """
    try:
        demo_modes = {
            "simulation": {
                "name": "Mode Simulation",
                "description": "Démonstration avec simulation MuJoCo",
                "backend": "mujoco",
                "features": ["3D Viewer", "Physique réaliste", "Contrôle complet"],
                "requirements": ["MuJoCo", "OpenGL"],
            },
            "robot_real": {
                "name": "Mode Robot Réel",
                "description": "Démonstration avec robot Reachy Mini physique",
                "backend": "reachy_mini",
                "features": ["Robot physique", "SDK officiel", "Contrôle sécurisé"],
                "requirements": ["Robot Reachy Mini", "SDK officiel"],
            },
            "mixed": {
                "name": "Mode Mixte",
                "description": "Basculement entre simulation et robot réel",
                "backend": "auto",
                "features": ["Switch automatique", "Comparaison", "Tests"],
                "requirements": ["Les deux modes"],
            },
        }

        return {
            "demo_modes": demo_modes,
            "recommended": "simulation",
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.exception("Erreur lors de la récupération des modes de démo: %s", e)
        raise HTTPException(
            status_code=500,
            detail="Erreur lors de la récupération des modes de démo",
        ) from e


@router.post("/demo/start")
async def start_demo_mode(
    mode: str = Query(..., description="Mode de démonstration"),
    duration: float = Query(30.0, gt=0, description="Durée en secondes"),
    emotion: str | None = Query(None, description="Émotion à démontrer"),
) -> dict[str, Any]:
    """Démarre un mode de démonstration.

    Args:
        mode: Mode de démonstration (simulation, robot_real, mixed)
        duration: Durée en secondes
        emotion: Émotion à démontrer (optionnel)

    Returns:
        Confirmation du démarrage de la démo

    Raises:
        HTTPException: En cas d'erreur

    """
    try:
        # Vérifier le mode
        valid_modes = ["simulation", "robot_real", "mixed"]
        if mode not in valid_modes:
            raise HTTPException(
                status_code=400,
                detail=f"Mode '{mode}' non valide. Modes disponibles: {valid_modes}",
            )

        # Implémentation logique démarrage démo
        demo_info: dict[str, Any] = {
            "mode": mode,
            "duration": duration,
            "emotion": emotion,
            "status": "started",
            "timestamp": datetime.now().isoformat(),
        }

        try:
            # Démarrer simulation si nécessaire
            if mode in ["simulation", "mixed"]:
                from ...simulation_service import simulation_service

                if not simulation_service.is_simulation_ready():
                    success = await simulation_service.start_simulation(headless=True)
                    if not success:
                        raise HTTPException(
                            status_code=500,
                            detail="Échec démarrage simulation",
                        )
                    demo_info["simulation"] = "started"

            # Appliquer émotion si demandée
            if emotion:
                try:
                    from ....bbia_emotions import BBIAEmotions

                    emotions_module = BBIAEmotions()
                    # Valider émotion - utiliser toutes les émotions disponibles
                    valid_emotions = list(emotions_module.emotions.keys())
                    if emotion.lower() in valid_emotions:
                        # Créer robot pour appliquer émotion
                        robot = RobotFactory.create_backend(
                            "mujoco" if mode != "robot_real" else "reachy",
                        )
                        if robot:
                            robot.connect()
                            # Utiliser set_emotion du robot si disponible
                            if hasattr(robot, "set_emotion"):
                                robot.set_emotion(emotion.lower(), 0.7)
                            else:
                                # Sinon, mettre à jour état émotions
                                emotions_module.set_emotion(emotion.lower(), 0.7)
                            demo_info["emotion_applied"] = emotion.lower()
                            robot.disconnect()
                except Exception as e:
                    logger.warning("Émotion non appliquée: %s", e)
                    demo_info["emotion_error"] = str(e)

            # Planifier arrêt après durée
            if duration > 0:

                async def stop_demo_after_duration() -> None:
                    await asyncio.sleep(duration)
                    try:
                        if mode in ["simulation", "mixed"]:
                            from ...simulation_service import (
                                simulation_service as sim_service,
                            )

                            await sim_service.stop_simulation()
                    except Exception as e:
                        logger.warning("Erreur arrêt démo: %s", e)

                asyncio.create_task(stop_demo_after_duration())
                demo_info["auto_stop"] = True

            demo_info["message"] = (
                f"Démonstration {mode} démarrée pour {duration} secondes"
            )

        except HTTPException:
            raise
        except Exception as e:
            logger.exception("Erreur logique démo: %s", e)
            demo_info["status"] = "error"
            demo_info["error"] = str(e)

        return demo_info
    except HTTPException:
        raise
    except Exception as e:
        logger.exception("Erreur lors du démarrage de la démo: %s", e)
        raise HTTPException(
            status_code=500,
            detail="Erreur lors du démarrage de la démo",
        ) from e
