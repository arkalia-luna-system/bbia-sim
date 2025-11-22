"""Router pour la gestion des applications HuggingFace."""

import logging
from typing import Any

from fastapi import APIRouter, HTTPException, WebSocket

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/apps")


class AppInfo:
    """Information sur une application."""

    def __init__(
        self,
        name: str,
        source_kind: str = "huggingface",
        installed: bool = False,
        status: str = "stopped",
    ) -> None:
        """Initialise les informations d'une app.

        Args:
            name: Nom de l'application
            source_kind: Source (huggingface, local, etc.)
            installed: Si l'app est installée
            status: Statut (running, stopped, etc.)

        """
        self.name = name
        self.source_kind = source_kind
        self.installed = installed
        self.status = status

    def model_dump(self) -> dict[str, Any]:
        """Convertit en dictionnaire."""
        return {
            "name": self.name,
            "source_kind": self.source_kind,
            "installed": self.installed,
            "status": self.status,
        }


class AppStatus:
    """Statut d'une application."""

    def __init__(
        self, name: str, status: str = "stopped", running: bool = False
    ) -> None:
        """Initialise le statut d'une app.

        Args:
            name: Nom de l'application
            status: Statut (running, stopped, error)
            running: Si l'app est en cours d'exécution

        """
        self.name = name
        self.status = status
        self.running = running

    def model_dump(self) -> dict[str, Any]:
        """Convertit en dictionnaire."""
        return {
            "name": self.name,
            "status": self.status,
            "running": self.running,
        }


# Liste des apps créées par les testeurs bêta (125 unités bêta - Novembre 2024)
# Source: Email Pollen Robotics + communauté Hugging Face Spaces
_BETA_TESTER_APPS: list[dict[str, Any]] = [
    {
        "name": "reachy-mini-conversation",
        "source_kind": "hf_space",
        "description": "Application conversationnelle avec reconnaissance vocale",
        "author": "beta-tester",
        "category": "conversation",
        "hf_space": "reachy-mini-conversation",
    },
    {
        "name": "reachy-mini-vision-demo",
        "source_kind": "hf_space",
        "description": "Démonstration vision avec détection d'objets",
        "author": "beta-tester",
        "category": "vision",
        "hf_space": "reachy-mini-vision-demo",
    },
    {
        "name": "reachy-mini-movements",
        "source_kind": "hf_space",
        "description": "Bibliothèque de mouvements créés par la communauté",
        "author": "beta-tester",
        "category": "movements",
        "hf_space": "reachy-mini-movements",
    },
    {
        "name": "reachy-mini-ai-assistant",
        "source_kind": "hf_space",
        "description": "Assistant IA avec Hugging Face models",
        "author": "beta-tester",
        "category": "ai",
        "hf_space": "reachy-mini-ai-assistant",
    },
]

# État global pour les apps BBIA
_bbia_apps_manager: dict[str, Any] = {
    "installed_apps": [],
    "current_app": None,
    "available_apps": [
        {"name": "bbia_vision", "source_kind": "local", "installed": True},
        {"name": "bbia_chat", "source_kind": "huggingface", "installed": True},
        {"name": "bbia_emotions", "source_kind": "local", "installed": True},
    ],
}


@router.get("/list-available/{source_kind}")
async def list_available_apps(source_kind: str) -> list[dict[str, Any]]:
    """Liste les applications disponibles pour une source.

    Args:
        source_kind: Type de source (huggingface, local, etc.)

    Returns:
        Liste des applications disponibles

    """
    apps: list[dict[str, Any]] = _bbia_apps_manager["available_apps"]
    filtered: list[dict[str, Any]] = [
        app for app in apps if app.get("source_kind") == source_kind
    ]
    return filtered


@router.get("/list-available")
async def list_all_available_apps() -> list[dict[str, Any]]:
    """Liste toutes les applications disponibles.

    Returns:
        Liste de toutes les applications disponibles (inclut apps locales, HF Hub, et testeurs bêta)

    """
    apps: list[dict[str, Any]] = _bbia_apps_manager["available_apps"].copy()

    # Ajouter les apps testeurs bêta
    apps.extend(_BETA_TESTER_APPS)

    # Essayer de découvrir des apps depuis HF Hub
    try:
        from huggingface_hub import HfApi

        api = HfApi()
        # Rechercher spaces avec préfixe "reachy-mini"
        hf_spaces = api.list_spaces(
            search="reachy-mini",
            limit=20,
        )

        # Filtrer et ajouter les spaces trouvés (éviter doublons)
        existing_names = {app.get("name") for app in apps}
        for space in hf_spaces:
            if space.id and space.id not in existing_names:
                apps.append(
                    {
                        "name": space.id,
                        "source_kind": "hf_space",
                        "description": getattr(space, "cardData", {}).get(
                            "description",
                            "",
                        ),
                        "author": "community",
                        "category": "community",
                        "hf_space": space.id,
                    },
                )
                existing_names.add(space.id)

        logger.info(
            "✅ Découverte %d apps HF Hub (total: %d apps disponibles)",
            len(hf_spaces),
            len(apps),
        )
    except (ImportError, AttributeError) as e:
        logger.debug("huggingface_hub non disponible ou erreur: %s", e)
    except Exception as e:  # noqa: BLE001
        logger.warning("Erreur récupération apps HF Hub: %s", e)

    return apps


@router.get("/list-community")
async def list_community_apps() -> list[dict[str, Any]]:
    """Liste les applications créées par la communauté (testeurs bêta + HF Hub).

    Returns:
        Liste des applications de la communauté

    """
    community_apps: list[dict[str, Any]] = _BETA_TESTER_APPS.copy()

    # Ajouter apps découvertes depuis HF Hub
    try:
        from huggingface_hub import HfApi

        api = HfApi()
        hf_spaces = api.list_spaces(
            search="reachy-mini",
            limit=20,
        )

        existing_names = {app.get("name") for app in community_apps}
        for space in hf_spaces:
            if space.id and space.id not in existing_names:
                community_apps.append(
                    {
                        "name": space.id,
                        "source_kind": "hf_space",
                        "description": getattr(space, "cardData", {}).get(
                            "description",
                            "",
                        ),
                        "author": "community",
                        "category": "community",
                        "hf_space": space.id,
                    },
                )
                existing_names.add(space.id)

    except (ImportError, AttributeError) as e:
        logger.debug("huggingface_hub non disponible ou erreur: %s", e)
    except Exception as e:  # noqa: BLE001
        logger.warning("Erreur récupération apps communauté HF Hub: %s", e)

    return community_apps


@router.post("/install")
async def install_app(app_info: dict[str, Any]) -> dict[str, str]:
    """Installe une nouvelle application (simulation - background job).

    Args:
        app_info: Informations sur l'application à installer

    Returns:
        ID du job en arrière-plan

    """
    app_name = app_info.get("name", "unknown")
    logger.info("Installation de l'application: %s", app_name)

    # Simulation d'un job ID
    job_id = f"install_{app_name}_{hash(app_name) % 10000}"

    # Ajouter à la liste des apps installées
    if app_name not in [app["name"] for app in _bbia_apps_manager["installed_apps"]]:
        _bbia_apps_manager["installed_apps"].append(
            {
                "name": app_name,
                "source_kind": app_info.get("source_kind", "huggingface"),
                "installed": True,
            },
        )

    return {"job_id": job_id}


@router.post("/remove/{app_name}")
async def remove_app(app_name: str) -> dict[str, str]:
    """Supprime une application installée (simulation - background job).

    Args:
        app_name: Nom de l'application à supprimer

    Returns:
        ID du job en arrière-plan

    """
    logger.info("Suppression de l'application: %s", app_name)

    # Simulation d'un job ID
    job_id = f"remove_{app_name}_{hash(app_name) % 10000}"

    # Retirer de la liste
    _bbia_apps_manager["installed_apps"] = [
        app for app in _bbia_apps_manager["installed_apps"] if app["name"] != app_name
    ]

    # Arrêter si c'était l'app courante
    if _bbia_apps_manager["current_app"] == app_name:
        _bbia_apps_manager["current_app"] = None

    return {"job_id": job_id}


@router.get("/job-status/{job_id}")
async def job_status(job_id: str) -> dict[str, Any]:
    """Récupère le statut/logs d'un job.

    Args:
        job_id: ID du job

    Returns:
        Informations sur le job (status, logs, etc.)

    """
    # Simulation d'un job avec statut "completed" ou "running"
    return {
        "job_id": job_id,
        "status": "completed",
        "logs": [f"Job {job_id} terminé avec succès"],
        "progress": 100,
    }


@router.websocket("/ws/apps-manager/{job_id}")
async def ws_apps_manager(websocket: WebSocket, job_id: str) -> None:
    """WebSocket pour streamer le statut/logs d'un job en temps réel.

    Args:
        websocket: Connexion WebSocket
        job_id: ID du job à suivre

    """
    import asyncio

    from fastapi import WebSocketDisconnect

    await websocket.accept()

    try:
        # Simulation : envoyer quelques mises à jour
        for i in range(3):
            await websocket.send_json(
                {
                    "job_id": job_id,
                    "status": "running" if i < 2 else "completed",
                    "progress": (i + 1) * 33,
                    "log": f"Étape {i + 1}/3 terminée",
                },
            )
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        logger.info("Client WebSocket déconnecté pour job %s", job_id)
    finally:
        await websocket.close()


@router.post("/start-app/{app_name}")
async def start_app(app_name: str) -> dict[str, Any]:
    """Démarre une application par son nom.

    Args:
        app_name: Nom de l'application

    Returns:
        Statut de l'application

    Raises:
        HTTPException: Si l'app n'est pas installée ou erreur au démarrage

    """
    # Vérifier si l'app est installée
    installed = [app["name"] for app in _bbia_apps_manager["installed_apps"]] + [
        app["name"]
        for app in _bbia_apps_manager["available_apps"]
        if app.get("installed")
    ]

    if app_name not in installed:
        raise HTTPException(
            status_code=400,
            detail=f"Application '{app_name}' non installée",
        )

    # Démarrer l'app
    _bbia_apps_manager["current_app"] = app_name

    logger.info("Démarrage de l'application: %s", app_name)

    return AppStatus(name=app_name, status="running", running=True).model_dump()


@router.post("/restart-current-app")
async def restart_app() -> dict[str, Any]:
    """Redémarre l'application actuellement en cours d'exécution.

    Returns:
        Statut de l'application

    Raises:
        HTTPException: Si aucune app n'est en cours d'exécution

    """
    current_app = _bbia_apps_manager.get("current_app")
    if not current_app:
        raise HTTPException(
            status_code=400,
            detail="Aucune application en cours d'exécution",
        )

    logger.info("Redémarrage de l'application: %s", current_app)

    return AppStatus(name=current_app, status="running", running=True).model_dump()


@router.post("/stop-current-app")
async def stop_app() -> dict[str, Any] | None:
    """Arrête l'application actuellement en cours d'exécution.

    Returns:
        None si succès

    Raises:
        HTTPException: Si aucune app n'est en cours d'exécution

    """
    current_app = _bbia_apps_manager.get("current_app")
    if not current_app:
        raise HTTPException(
            status_code=400,
            detail="Aucune application en cours d'exécution",
        )

    logger.info("Arrêt de l'application: %s", current_app)
    _bbia_apps_manager["current_app"] = None

    return None


@router.get("/current-app-status")
async def current_app_status() -> dict[str, Any] | None:
    """Récupère le statut de l'application en cours d'exécution, si présente.

    Returns:
        Statut de l'app ou None si aucune app en cours

    """
    current_app = _bbia_apps_manager.get("current_app")
    if current_app:
        return AppStatus(name=current_app, status="running", running=True).model_dump()
    return None
