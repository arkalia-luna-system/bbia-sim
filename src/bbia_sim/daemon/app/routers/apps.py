"""Router pour la gestion des applications HuggingFace."""

import asyncio
import logging
import time
from collections import deque
from typing import Any
from uuid import uuid4

from fastapi import APIRouter, HTTPException, WebSocket

from bbia_sim.daemon.app.hf_app_installer import HFAppInstaller

logger = logging.getLogger(__name__)

# Installer global
_hf_app_installer = HFAppInstaller()

# Jobs d'installation en cours
# OPTIMISATION RAM: Limiter nombre de jobs en mémoire (max 50 jobs)
_installation_jobs: dict[str, dict[str, Any]] = {}
_MAX_JOBS_IN_MEMORY = 50
_MAX_JOB_LOGS = 500  # Maximum 500 logs par job

router = APIRouter(prefix="/apps")


def _cleanup_old_jobs() -> None:
    """Nettoie les jobs terminés pour libérer la RAM.

    Supprime les jobs terminés (completed, failed, already_installed)
    si le nombre de jobs dépasse la limite.
    """
    global _installation_jobs

    # Compter jobs terminés
    finished_jobs = [
        job_id
        for job_id, job in _installation_jobs.items()
        if job.get("status") in ("completed", "failed", "already_installed")
    ]

    # Si on dépasse la limite, supprimer les jobs terminés les plus anciens
    if len(_installation_jobs) > _MAX_JOBS_IN_MEMORY:
        # Trier par timestamp (si disponible) ou supprimer les plus anciens
        jobs_to_remove = finished_jobs[: len(_installation_jobs) - _MAX_JOBS_IN_MEMORY]
        for job_id in jobs_to_remove:
            del _installation_jobs[job_id]
            logger.debug("🗑️ Job terminé supprimé pour libérer RAM: %s", job_id)

        logger.info(
            "🧹 Nettoyage jobs: %d jobs supprimés (reste: %d)",
            len(jobs_to_remove),
            len(_installation_jobs),
        )


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

    # Ajouter les apps installées depuis HF Spaces
    installed_hf_apps = _hf_app_installer.list_installed_apps()
    for installed_app in installed_hf_apps:
        # Marquer comme installée si pas déjà dans la liste
        existing = next(
            (app for app in apps if app.get("name") == installed_app["name"]),
            None,
        )
        if existing:
            existing["installed"] = True
            existing["app_path"] = installed_app.get("app_path")
        else:
            apps.append(
                {
                    "name": installed_app["name"],
                    "source_kind": "hf_space",
                    "installed": True,
                    "app_path": installed_app.get("app_path"),
                },
            )

    # Essayer de découvrir des apps depuis HF Hub
    try:
        from huggingface_hub import HfApi

        api = HfApi()
        # OPTIMISATION RAM: Limiter le nombre de spaces récupérés
        # (limite déjà à 20, mais on peut réduire si nécessaire)
        hf_spaces = api.list_spaces(
            search="reachy-mini",
            limit=20,  # Maximum 20 spaces pour éviter surcharge mémoire
        )

        # Filtrer et ajouter les spaces trouvés (éviter doublons)
        existing_names = {app.get("name") for app in apps}
        for space in hf_spaces:
            if space.id and space.id not in existing_names:
                # Vérifier si déjà installée
                is_installed = _hf_app_installer.is_installed(space.id.split("/")[-1])
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
                        "installed": is_installed,
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
    """Installe une nouvelle application depuis Hugging Face Spaces.

    Args:
        app_info: Informations sur l'application à installer
                  - name: Nom de l'app
                  - hf_space: ID du Space HF (format: "username/space-name")
                  - source_kind: Type de source (hf_space, local, etc.)

    Returns:
        ID du job en arrière-plan

    Raises:
        HTTPException: Si l'installation échoue immédiatement (avant job)
    """
    app_name = app_info.get("name", "unknown")
    hf_space_id = app_info.get("hf_space") or app_info.get("name")
    source_kind = app_info.get("source_kind", "hf_space")

    # Normaliser "huggingface" vers "hf_space" pour compatibilité
    if source_kind == "huggingface":
        source_kind = "hf_space"

    logger.info("Installation de l'application: %s (source: %s)", app_name, source_kind)

    # Générer un job ID unique
    job_id = str(uuid4())

    # Pour les apps locales, installation immédiate (pas de job)
    if source_kind == "local":
        if app_name not in [
            app["name"] for app in _bbia_apps_manager["installed_apps"]
        ]:
            _bbia_apps_manager["installed_apps"].append(
                {
                    "name": app_name,
                    "source_kind": source_kind,
                    "installed": True,
                },
            )
        return {"job_id": job_id, "status": "completed"}

    # Pour les apps HF Spaces, installation asynchrone
    if source_kind == "hf_space" and hf_space_id:
        # Vérifier si déjà installé
        if _hf_app_installer.is_installed(app_name):
            logger.info("App %s déjà installée", app_name)
            return {"job_id": job_id, "status": "already_installed"}

        # Créer job d'installation
        _installation_jobs[job_id] = {
            "status": "running",
            "app_name": app_name,
            "hf_space_id": hf_space_id,
            "logs": [],
            "progress": 0,
        }

        # Lancer installation en arrière-plan
        asyncio.create_task(_run_installation_job(job_id, hf_space_id, app_name))

        return {"job_id": job_id}

    raise HTTPException(
        status_code=400,
        detail=f"Impossible d'installer: hf_space manquant pour {app_name}",
    )


async def _run_installation_job(
    job_id: str,
    hf_space_id: str,
    app_name: str,
) -> None:
    """Exécute un job d'installation en arrière-plan.

    Args:
        job_id: ID du job
        hf_space_id: ID du Space HF
        app_name: Nom de l'app
    """
    job = _installation_jobs.get(job_id)
    if not job:
        return

    try:
        job["logs"].append(f"🚀 Démarrage installation {app_name}...")
        job["progress"] = 10

        # Installer l'app
        result = await _hf_app_installer.install_app(
            hf_space_id=hf_space_id,
            app_name=app_name,
        )

        job["logs"].append(f"✅ Installation terminée: {result['status']}")
        job["progress"] = 100
        job["status"] = "completed"

        # Ajouter à la liste des apps installées
        if app_name not in [
            app["name"] for app in _bbia_apps_manager["installed_apps"]
        ]:
            _bbia_apps_manager["installed_apps"].append(
                {
                    "name": app_name,
                    "source_kind": "hf_space",
                    "installed": True,
                    "app_path": result.get("app_path"),
                },
            )

        logger.info("✅ Job %s terminé avec succès", job_id)

        # OPTIMISATION RAM: Nettoyer les vieux jobs après chaque job terminé
        _cleanup_old_jobs()

    except Exception as e:
        logger.exception("❌ Erreur job %s: %s", job_id, e)
        job["status"] = "failed"
        job["logs"].append(f"❌ Erreur: {str(e)}")
        job["error"] = str(e)

        # OPTIMISATION RAM: Nettoyer les vieux jobs même en cas d'erreur
        _cleanup_old_jobs()


async def _run_uninstallation_job(
    job_id: str,
    app_name: str,
) -> None:
    """Exécute un job de désinstallation en arrière-plan.

    Args:
        job_id: ID du job
        app_name: Nom de l'app
    """
    job = _installation_jobs.get(job_id)
    if not job:
        return

    try:
        job["logs"].append(f"🗑️ Démarrage désinstallation {app_name}...")
        job["progress"] = 10

        # Vérifier si installée depuis HF Spaces
        if _hf_app_installer.is_installed(app_name):
            # Désinstaller l'app
            await _hf_app_installer.uninstall_app(app_name)
            job["logs"].append("✅ Désinstallation terminée")
        else:
            job["logs"].append("ℹ️ App non trouvée dans le système de fichiers")

        job["progress"] = 50

        # Retirer de la liste
        _bbia_apps_manager["installed_apps"] = [
            app for app in _bbia_apps_manager["installed_apps"] if app["name"] != app_name
        ]

        job["progress"] = 75

        # Arrêter si c'était l'app courante
        if _bbia_apps_manager["current_app"] == app_name:
            _bbia_apps_manager["current_app"] = None

        job["progress"] = 100
        job["status"] = "completed"
        job["logs"].append(f"✅ App {app_name} supprimée avec succès")

        logger.info("✅ Job désinstallation %s terminé avec succès", job_id)

        # OPTIMISATION RAM: Nettoyer les vieux jobs après chaque job terminé
        _cleanup_old_jobs()

    except Exception as e:
        logger.exception("❌ Erreur job désinstallation %s: %s", job_id, e)
        job["status"] = "failed"
        job["logs"].append(f"❌ Erreur: {str(e)}")
        job["error"] = str(e)

        # OPTIMISATION RAM: Nettoyer les vieux jobs même en cas d'erreur
        _cleanup_old_jobs()


@router.post("/remove/{app_name}")
async def remove_app(app_name: str) -> dict[str, str]:
    """Supprime une application installée.

    Args:
        app_name: Nom de l'application à supprimer

    Returns:
        ID du job en arrière-plan

    Raises:
        HTTPException: Si l'app n'est pas installée
    """
    logger.info("Suppression de l'application: %s", app_name)

    # Vérifier si l'app est installée
    is_installed = _hf_app_installer.is_installed(app_name)
    is_in_list = app_name in [
        app["name"] for app in _bbia_apps_manager["installed_apps"]
    ]

    # Si l'app n'est ni installée ni dans la liste, on laisse quand même créer le job
    # pour permettre la suppression même si elle n'est pas trouvée (nettoyage)
    if not is_installed and not is_in_list:
        logger.warning("App %s non trouvée, création job de nettoyage", app_name)

    # Générer un job ID unique
    job_id = str(uuid4())

    # Créer job de désinstallation
    _installation_jobs[job_id] = {
        "status": "running",
        "app_name": app_name,
        "logs": [],
        "progress": 0,
    }

    # Lancer désinstallation en arrière-plan
    asyncio.create_task(_run_uninstallation_job(job_id, app_name))

    return {"job_id": job_id}


@router.get("/job-status/{job_id}")
async def job_status(job_id: str) -> dict[str, Any]:
    """Récupère le statut/logs d'un job.

    Args:
        job_id: ID du job

    Returns:
        Informations sur le job (status, logs, progress, etc.)

    Raises:
        HTTPException: Si le job n'existe pas
    """
    job = _installation_jobs.get(job_id)
    if not job:
        raise HTTPException(status_code=404, detail=f"Job {job_id} non trouvé")

    # OPTIMISATION RAM: Convertir deque en liste pour la sérialisation JSON
    logs = job.get("logs", [])
    if isinstance(logs, deque):
        logs = list(logs)

    return {
        "job_id": job_id,
        "status": job.get("status", "unknown"),
        "logs": logs,
        "progress": job.get("progress", 0),
        "app_name": job.get("app_name"),
        "error": job.get("error"),
    }


@router.websocket("/ws/apps-manager/{job_id}")
async def ws_apps_manager(websocket: WebSocket, job_id: str) -> None:
    """WebSocket pour streamer le statut/logs d'un job en temps réel.

    Args:
        websocket: Connexion WebSocket
        job_id: ID du job à suivre

    """
    from fastapi import WebSocketDisconnect

    await websocket.accept()

    try:
        # Envoyer mises à jour en temps réel
        last_log_count = 0
        while True:
            job = _installation_jobs.get(job_id)
            if not job:
                await websocket.send_json(
                    {
                        "job_id": job_id,
                        "status": "not_found",
                        "error": "Job non trouvé",
                    },
                )
                break

            # Envoyer nouveaux logs
            # OPTIMISATION RAM: Convertir deque en liste si nécessaire
            current_logs = job.get("logs", [])
            if isinstance(current_logs, deque):
                current_logs = list(current_logs)

            if len(current_logs) > last_log_count:
                for log in current_logs[last_log_count:]:
                    await websocket.send_json(
                        {
                            "job_id": job_id,
                            "status": job.get("status", "running"),
                            "progress": job.get("progress", 0),
                            "log": log,
                        },
                    )
                last_log_count = len(current_logs)

            # Si terminé, envoyer statut final et fermer
            if job.get("status") in ("completed", "failed", "already_installed"):
                await websocket.send_json(
                    {
                        "job_id": job_id,
                        "status": job.get("status"),
                        "progress": job.get("progress", 100),
                        "done": True,
                    },
                )
                break

            await asyncio.sleep(0.5)  # Poll toutes les 500ms

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


# Fonctions utilitaires pour compatibilité avec les tests
def format_uptime(seconds: float) -> str:
    """Formate le temps en format HH:MM:SS.

    Args:
        seconds: Nombre de secondes

    Returns:
        Chaîne formatée HH:MM:SS
    """
    hours = int(seconds // 3600)
    minutes = int((seconds % 3600) // 60)
    secs = int(seconds % 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


# État global pour le temps de démarrage des apps
_app_start_times: dict[str, float] = {}


def get_app_start_time(app_name: str) -> float:
    """Récupère le temps de démarrage d'une application.

    Args:
        app_name: Nom de l'application

    Returns:
        Timestamp de démarrage ou temps actuel si première fois
    """
    if app_name not in _app_start_times:
        _app_start_times[app_name] = time.time()
    return _app_start_times[app_name]
