"""Router pour les endpoints de l'écosystème BBIA-SIM - Phase 3."""

import logging
from datetime import datetime
from typing import Any, Optional

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel, Field

from ....bbia_behavior import BBIABehaviorManager
from ....bbia_emotions import BBIAEmotions
from ....robot_factory import RobotFactory

logger = logging.getLogger(__name__)

router = APIRouter()


# Modèles Pydantic pour l'API publique
class EmotionResponse(BaseModel):
    """Réponse pour les émotions BBIA."""

    emotion: str = Field(..., description="Nom de l'émotion appliquée")
    intensity: float = Field(
        ..., ge=0.0, le=1.0, description="Intensité de l'émotion (0.0-1.0)"
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
        logger.error(f"Erreur lors de la récupération des capacités: {e}")
        raise HTTPException(
            status_code=500, detail="Erreur lors de la récupération des capacités"
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
        except Exception:
            robot_connected = False
            simulation_running = False

        return APIStatus(
            version="1.2.0",
            status="running",
            uptime="00:00:00",  # TODO: Calculer le temps réel
            robot_connected=robot_connected,
            simulation_running=simulation_running,
            active_connections=0,  # TODO: Compter les connexions WebSocket
        )
    except Exception as e:
        logger.error(f"Erreur lors de la récupération du statut: {e}")
        raise HTTPException(
            status_code=500, detail="Erreur lors de la récupération du statut"
        ) from e


@router.post("/emotions/apply", response_model=EmotionResponse)
async def apply_emotion(
    emotion: str = Query(..., description="Nom de l'émotion à appliquer"),
    intensity: float = Query(0.5, ge=0.0, le=1.0, description="Intensité de l'émotion"),
    duration: float = Query(5.0, gt=0, description="Durée en secondes"),
    joint: Optional[str] = Query(None, description="Joint spécifique à animer"),
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
        logger.error(f"Erreur lors de l'application de l'émotion: {e}")
        raise HTTPException(
            status_code=500, detail="Erreur lors de l'application de l'émotion"
        ) from e


@router.post("/behaviors/execute", response_model=BehaviorResponse)
async def execute_behavior(
    behavior: str = Query(..., description="Nom du comportement à exécuter"),
    intensity: float = Query(
        1.0, ge=0.0, le=2.0, description="Intensité du comportement"
    ),
    duration: Optional[float] = Query(
        None, gt=0, description="Durée personnalisée en secondes"
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
        logger.error(f"Erreur lors de l'exécution du comportement: {e}")
        raise HTTPException(
            status_code=500, detail="Erreur lors de l'exécution du comportement"
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
        logger.error(f"Erreur lors de la récupération des émotions: {e}")
        raise HTTPException(
            status_code=500, detail="Erreur lors de la récupération des émotions"
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
        logger.error(f"Erreur lors de la récupération des comportements: {e}")
        raise HTTPException(
            status_code=500, detail="Erreur lors de la récupération des comportements"
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
        logger.error(f"Erreur lors de la récupération des modes de démo: {e}")
        raise HTTPException(
            status_code=500, detail="Erreur lors de la récupération des modes de démo"
        ) from e


@router.post("/demo/start")
async def start_demo_mode(
    mode: str = Query(..., description="Mode de démonstration"),
    duration: float = Query(30.0, gt=0, description="Durée en secondes"),
    emotion: Optional[str] = Query(None, description="Émotion à démontrer"),
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

        # TODO: Implémenter la logique de démarrage de démo
        # Pour l'instant, on retourne une confirmation

        return {
            "mode": mode,
            "duration": duration,
            "emotion": emotion,
            "status": "started",
            "message": f"Démonstration {mode} démarrée pour {duration} secondes",
            "timestamp": datetime.now().isoformat(),
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Erreur lors du démarrage de la démo: {e}")
        raise HTTPException(
            status_code=500, detail="Erreur lors du démarrage de la démo"
        ) from e
