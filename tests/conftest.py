#!/usr/bin/env python3
"""
Configuration globale pytest avec système de verrouillage pour éviter
l'exécution simultanée de plusieurs instances de tests.
OPTIMISATION RAM: Force utilisation de mocks pour modèles lourds.
"""

import atexit
import fcntl
import gc
import logging
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest  # type: ignore[import-untyped]

# OPTIMISATION RAM: Forcer mode mock pour tests (évite chargement modèles lourds)
os.environ.setdefault("BBIA_DISABLE_AUDIO", "1")
os.environ.setdefault("BBIA_DISABLE_VISION_MODELS", "1")
os.environ.setdefault("BBIA_FORCE_MOCK_MODELS", "1")


def _suppress_portaudio_error() -> None:
    """Supprime l'exception PortAudio dans le handler atexit de sounddevice."""
    try:
        import sounddevice as sd

        # Remplacer le handler atexit de sounddevice pour supprimer l'exception
        # L'exception se produit quand sounddevice essaie de terminer PortAudio
        # alors qu'il n'a pas été initialisé (CI/headless)
        if hasattr(sd, "_exit_handler"):
            # Supprimer le handler existant
            try:
                atexit.unregister(sd._exit_handler)
            except (ValueError, AttributeError):
                pass

            # Créer un nouveau handler qui ignore l'exception
            def _safe_exit_handler() -> None:
                try:
                    if hasattr(sd, "_lib") and sd._lib is not None:
                        # Vérifier si PortAudio est initialisé avant de terminer
                        try:
                            sd._lib.Pa_Terminate()
                        except Exception:
                            # Ignorer l'exception PortAudio (non bloquant)
                            pass
                except Exception:
                    # Ignorer toutes les exceptions (non bloquant)
                    pass

            # Enregistrer le nouveau handler
            atexit.register(_safe_exit_handler)
    except (ImportError, AttributeError):
        # sounddevice non disponible, rien à faire
        pass


# Supprimer l'exception PortAudio au démarrage
_suppress_portaudio_error()

# Chemin vers le fichier de lock
LOCK_FILE = Path(__file__).parent.parent / ".pytest.lock"
LOCK_TIMEOUT = 300  # 5 minutes max pour un run de tests
_MAX_RECURSION = 3  # Protection contre récursion infinie

# Stocker le file descriptor du lock globalement pour pouvoir le libérer
# même en cas d'interruption ou de blocage
_lock_fd: int | None = None


class ExpectedErrorFilter(logging.Filter):
    """
    Filtre de logging pour réduire le bruit des erreurs attendues dans les tests.

    Les erreurs suivantes sont attendues et peuvent être réduites au niveau WARNING :
    - ModuleNotFoundError pour dépendances optionnelles (reachy_mini, etc.)
    - RuntimeError pour eSpeak non installé
    - Erreurs de reconnaissance vocale dans les tests de gestion d'erreurs
    - Erreurs de synthèse vocale dans les tests de gestion d'erreurs
    - Erreurs de processeurs de modèles non disponibles
    """

    # Patterns d'erreur attendus (chaînes simples pour recherche rapide)
    EXPECTED_ERROR_PATTERNS = [
        "ModuleNotFoundError: No module named 'reachy_mini'",
        "RuntimeError: This means you probably do not have eSpeak",
        "Erreur de reconnaissance vocale",
        "Erreur de synthèse vocale",
        "Aucun moteur TTS disponible",
        "Erreur initialisation fallback",
        "Erreur création Move",
    ]

    # Patterns regex pour correspondances plus flexibles
    EXPECTED_ERROR_REGEX = [
        re.compile(r"Processeur.*non disponible", re.IGNORECASE),
        re.compile(r"ModuleNotFoundError.*reachy", re.IGNORECASE),
        re.compile(r"eSpeak.*not.*installed", re.IGNORECASE),
    ]

    def filter(self, record: logging.LogRecord) -> bool:
        """
        Filtre les messages d'erreur attendus.

        Returns:
            True si le message doit être affiché, False sinon.
        """
        # Si ce n'est pas un message ERROR, ne pas filtrer
        if record.levelno < logging.ERROR:
            return True

        # Obtenir le message complet
        message = record.getMessage()

        # Vérifier les patterns simples
        for pattern in self.EXPECTED_ERROR_PATTERNS:
            if pattern in message:
                record.levelno = logging.WARNING
                record.levelname = "WARNING"
                return True

        # Vérifier les patterns regex
        for regex_pattern in self.EXPECTED_ERROR_REGEX:
            if regex_pattern.search(message):
                record.levelno = logging.WARNING
                record.levelname = "WARNING"
                return True

        # Vérifier aussi dans les exceptions capturées
        if record.exc_info and record.exc_info[1]:
            exc_message = str(record.exc_info[1])
            for pattern in self.EXPECTED_ERROR_PATTERNS:
                if pattern in exc_message:
                    record.levelno = logging.WARNING
                    record.levelname = "WARNING"
                    return True
            for regex_pattern in self.EXPECTED_ERROR_REGEX:
                if regex_pattern.search(exc_message):
                    record.levelno = logging.WARNING
                    record.levelname = "WARNING"
                    return True

        # Garder toutes les autres erreurs
        return True


def acquire_lock(recursion_level: int = 0) -> bool:
    """
    Acquiert un verrou exclusif pour empêcher l'exécution simultanée.

    Args:
        recursion_level: Niveau de récursion (protection contre boucles infinies)

    Returns:
        True si le lock est acquis, False sinon.
    """
    # Protection contre récursion infinie
    if recursion_level >= _MAX_RECURSION:
        print(
            f"⚠️  Trop de tentatives de récupération lock ({recursion_level}). "
            f"Abandon."
        )
        return False

    if not LOCK_FILE.parent.exists():
        LOCK_FILE.parent.mkdir(parents=True, exist_ok=True)

    try:
        # Ouvrir le fichier en mode append
        lock_fd = os.open(str(LOCK_FILE), os.O_CREAT | os.O_WRONLY | os.O_TRUNC)

        # Tenter d'acquérir le lock exclusif (non-bloquant)
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)

            # Écrire le PID et timestamp
            pid = os.getpid()
            timestamp = time.time()
            os.write(lock_fd, f"{pid}:{timestamp}\n".encode())
            os.fsync(lock_fd)

            # Stocker le file descriptor globalement
            global _lock_fd
            _lock_fd = lock_fd

            # Enregistrer la libération du lock à la fin (plusieurs handlers pour sécurité)
            atexit.register(release_lock, lock_fd)

            # Handler pour SIGINT (Ctrl+C) et SIGTERM
            def signal_handler(signum, frame):
                release_lock(lock_fd)
                sys.exit(1)

            signal.signal(signal.SIGINT, signal_handler)
            signal.signal(signal.SIGTERM, signal_handler)

            return True
        except BlockingIOError:
            # Lock déjà acquis par un autre processus
            os.close(lock_fd)

            # Lire qui a le lock
            try:
                with open(LOCK_FILE) as f:
                    lock_info = f.read().strip()
                    if ":" not in lock_info:
                        # Format invalide, nettoyer
                        print("⚠️  Lock au format invalide. Suppression...")
                        os.remove(LOCK_FILE)
                        return acquire_lock(recursion_level + 1)
                    pid_str, timestamp_str = lock_info.split(":", 1)
                    try:
                        pid_int = int(pid_str)
                        lock_timestamp = float(timestamp_str)
                    except (ValueError, TypeError):
                        # Format invalide, nettoyer
                        print(
                            f"⚠️  Lock contient données invalides: {lock_info}. "
                            f"Suppression..."
                        )
                        os.remove(LOCK_FILE)
                        return acquire_lock(recursion_level + 1)

                    # Vérifier si le processus existe encore
                    try:
                        os.kill(pid_int, 0)  # Vérifier si processus existe
                        elapsed = time.time() - lock_timestamp

                        if elapsed > LOCK_TIMEOUT:
                            # Lock expiré (processus probablement mort)
                            print(
                                f"⚠️  Lock expiré (>{LOCK_TIMEOUT}s). "
                                f"Processus {pid_int} pourrait être mort. "
                                f"Suppression du lock..."
                            )
                            os.remove(LOCK_FILE)
                            # Réessayer une fois (avec incrément récursion)
                            return acquire_lock(recursion_level + 1)
                        else:
                            print(
                                f"❌ Tests déjà en cours d'exécution !\n"
                                f"   Processus PID: {pid_int}\n"
                                f"   Lock acquis il y a: {elapsed:.1f}s\n"
                                f"   Fichier lock: {LOCK_FILE}\n\n"
                                f"💡 Solutions:\n"
                                f"   1. Attendre la fin de l'autre processus\n"
                                f"   2. Vérifier: ps aux | grep {pid_int}\n"
                                f"   3. Si processus mort: rm {LOCK_FILE}\n"
                                f"   4. Timeout automatique après {LOCK_TIMEOUT}s\n"
                            )
                            return False
                    except ProcessLookupError:
                        # Processus n'existe plus, lock orphelin
                        print(
                            f"⚠️  Lock orphelin détecté (processus {pid_int} n'existe plus). "
                            f"Suppression..."
                        )
                        os.remove(LOCK_FILE)
                        # Réessayer une fois (avec incrément récursion)
                        return acquire_lock(recursion_level + 1)
            except Exception as e:
                print(f"⚠️  Erreur lecture lock: {e}. Suppression du lock...")
                try:
                    os.remove(LOCK_FILE)
                except Exception:
                    pass
                return False

    except Exception as e:
        print(f"⚠️  Erreur création lock: {e}")
        return False


def release_lock(lock_fd: int) -> None:
    """Libère le lock à la fin des tests."""
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_UN)
        os.close(lock_fd)
        try:
            os.remove(LOCK_FILE)
        except Exception:
            pass
    except Exception:
        pass


def force_cleanup_all_resources() -> None:
    """
    Force le nettoyage de toutes les ressources qui peuvent empêcher pytest de se terminer.
    - Arrête tous les threads actifs (sauf le thread principal)
    - Ferme toutes les boucles asyncio
    - Déconnecte tous les backends actifs
    - Ferme toutes les connexions WebSocket/HTTP
    """
    try:
        # 1. Nettoyer tous les backends actifs (threads watchdog)
        try:

            # Forcer la déconnexion de tous les backends qui pourraient être actifs
            # Note: On ne peut pas vraiment lister toutes les instances, mais on force
            # le nettoyage des threads watchdog qui peuvent rester actifs
            pass  # Les backends doivent être nettoyés dans les tests individuels
        except Exception:
            pass

        # 2. Fermer toutes les boucles asyncio qui peuvent rester ouvertes
        try:
            import asyncio

            # Annuler toutes les tâches asyncio en attente
            try:
                # Essayer d'obtenir la boucle courante
                loop = asyncio.get_running_loop()
            except RuntimeError:
                # Pas de boucle en cours, essayer de nettoyer les boucles fermées
                try:
                    # Essayer de récupérer la boucle par défaut (peut être fermée)
                    loop = asyncio.get_event_loop()
                    if loop.is_closed():
                        # Créer une nouvelle boucle pour nettoyer puis la fermer
                        loop = asyncio.new_event_loop()
                        asyncio.set_event_loop(loop)
                except RuntimeError:
                    loop = None

            if loop is not None and not loop.is_closed():
                # Annuler toutes les tâches en cours
                try:
                    tasks = [t for t in asyncio.all_tasks(loop) if not t.done()]
                    for task in tasks:
                        task.cancel()
                except Exception:
                    pass

                # Essayer de fermer proprement la boucle (sans bloquer)
                try:
                    if not loop.is_closed():
                        # Arrêter la boucle sans bloquer
                        try:
                            loop.call_soon_threadsafe(loop.stop)
                        except Exception:
                            pass
                except Exception:
                    pass
        except Exception:
            pass

        # 3. Forcer l'arrêt des threads non-daemon qui peuvent bloquer pytest
        # Note: Les threads daemon sont automatiquement terminés, mais on peut
        # forcer l'arrêt propre si nécessaire
        try:
            # Lister tous les threads actifs sauf le thread principal
            all_threads = threading.enumerate()
            main_thread = threading.main_thread()

            for thread in all_threads:
                if thread is main_thread:
                    continue

                # Si le thread est toujours actif, essayer de le joindre avec timeout
                if thread.is_alive():
                    thread_name = getattr(thread, "name", "Unknown")
                    # Ignorer les threads système Python (garbage collector, etc.)
                    if "MainThread" in thread_name or "Thread" not in thread_name:
                        continue

                    # Joindre avec timeout très court
                    thread.join(timeout=0.1)
        except Exception:
            pass

        # 4. Force garbage collection pour libérer les ressources
        try:
            gc.collect()
        except Exception:
            pass

    except Exception:
        # Ignorer toutes les erreurs pour ne pas bloquer la fin de pytest
        pass


@pytest.hookimpl(tryfirst=True)
def pytest_collection_modifyitems(config: pytest.Config, items: list) -> None:
    """
    Hook pour modifier les tests collectés - skip automatique en CI pour tests lents/heavy.
    """
    # Skip automatique en CI pour tests marqués slow/heavy si variable d'environnement définie
    skip_slow_in_ci = os.environ.get("BBIA_SKIP_SLOW_TESTS", "0") == "1"
    is_ci = os.environ.get("CI", "false").lower() == "true"

    if is_ci and skip_slow_in_ci:
        for item in items:
            # Skip tests marqués slow ou heavy en CI si flag activé
            if item.get_closest_marker("slow") or item.get_closest_marker("heavy"):
                skip_marker = pytest.mark.skip(
                    reason="Test désactivé en CI (BBIA_SKIP_SLOW_TESTS=1)"
                )
                item.add_marker(skip_marker)


def _clear_heavy_model_caches_if_loaded() -> None:
    """Vide les caches ML seulement si les modules sont déjà dans sys.modules.

    Évite l'import eager de vision_yolo (ultralytics→torch) au pytest_configure,
    qui ralentit / casse les tests unitaires légers.
    """
    import sys

    vision_yolo = sys.modules.get("bbia_sim.vision_yolo")
    if vision_yolo is not None:
        yolo_lock = getattr(vision_yolo, "_yolo_cache_lock", None)
        yolo_cache = getattr(vision_yolo, "_yolo_model_cache", None)
        if yolo_lock is not None and isinstance(yolo_cache, dict):
            with yolo_lock:
                yolo_cache.clear()
        mp_lock = getattr(vision_yolo, "_mediapipe_cache_lock", None)
        if mp_lock is not None:
            with mp_lock:
                vision_yolo._mediapipe_face_detection_cache = None

    voice_whisper = sys.modules.get("bbia_sim.voice_whisper")
    if voice_whisper is not None:
        w_lock = getattr(voice_whisper, "_whisper_model_cache_lock", None)
        w_cache = getattr(voice_whisper, "_whisper_models_cache", None)
        if w_lock is not None and isinstance(w_cache, dict):
            with w_lock:
                w_cache.clear()

    hf_mod = sys.modules.get("bbia_sim.bbia_huggingface")
    if hf_mod is not None:
        BBIAHuggingFace = getattr(hf_mod, "BBIAHuggingFace", None)
        if BBIAHuggingFace is None:
            return
        try:
            with BBIAHuggingFace._shared_unload_thread_lock:
                thread = BBIAHuggingFace._shared_unload_thread
                if thread is not None and thread.is_alive():
                    BBIAHuggingFace._shared_unload_thread_stop.set()
                    thread.join(timeout=1.0)
                BBIAHuggingFace._shared_instances.clear()
                BBIAHuggingFace._shared_unload_thread = None
        except (AttributeError, RuntimeError):
            pass
        clear_cache = getattr(BBIAHuggingFace, "_clear_cache", None)
        if callable(clear_cache):
            clear_cache()


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config: pytest.Config) -> None:
    """
    Hook pytest qui s'exécute au démarrage.
    Vérifie le lock avant de lancer les tests.
    OPTIMISATION RAM: Nettoie les caches de modèles avant les tests.
    """
    # Acquérir le lock uniquement dans pytest
    # (ce hook ne s'exécute que si on est vraiment dans pytest)
    if not acquire_lock():
        print("\n❌ Impossible d'acquérir le verrou d'exécution.")
        print("   Un autre processus exécute déjà les tests.\n")
        sys.exit(1)

    print("✅ Verrou d'exécution acquis. Tests sécurisés.\n")

    # Configurer le filtre de logging pour réduire le bruit des erreurs attendues
    # Appliquer le filtre aux loggers principaux du projet
    expected_error_filter = ExpectedErrorFilter()
    logger_names = [
        "bbia_sim",
        "bbia_sim.bbia_voice",
        "bbia_sim.bbia_voice_advanced",
        "bbia_sim.bbia_huggingface",
        "bbia_sim.backends.reachy_mini_backend",
        "root",
    ]
    for logger_name in logger_names:
        logger = logging.getLogger(logger_name)
        # Ajouter le filtre uniquement s'il n'est pas déjà présent
        if not any(isinstance(f, ExpectedErrorFilter) for f in logger.filters):
            logger.addFilter(expected_error_filter)

    # OPTIMISATION RAM: Nettoyer caches modèles UNIQUEMENT s'ils sont déjà chargés.
    # Ne jamais importer vision_yolo / whisper / HF ici : cela tire ultralytics→torch
    # et peut bloquer/tuer la session pytest (SystemExit via signal pendant import).
    try:
        _clear_heavy_model_caches_if_loaded()
        print("🧹 Caches modèles nettoyés si déjà chargés (pas d'import torch/YOLO)\n")
    except Exception as e:
        # Ignorer erreurs de nettoyage (non bloquant)
        print(f"⚠️  Erreur nettoyage cache (non bloquant): {e}\n")


@pytest.hookimpl(trylast=True)
def pytest_unconfigure(config: pytest.Config) -> None:
    """Hook pytest qui s'exécute à la fin des tests."""
    # Nettoyer toutes les ressources avant de libérer le lock
    force_cleanup_all_resources()

    # Libérer le lock de manière explicite
    global _lock_fd
    if _lock_fd is not None:
        try:
            release_lock(_lock_fd)
        except Exception:
            pass
        _lock_fd = None

    # Nettoyer aussi le fichier lock au cas où
    try:
        if LOCK_FILE.exists():
            os.remove(LOCK_FILE)
    except Exception:
        pass

    # OPTIMISATION RAM: Nettoyer caches après tests sans forcer import YOLO/torch
    try:
        _clear_heavy_model_caches_if_loaded()
    except Exception:
        pass


def run_cleanup_scripts() -> None:
    """
    Exécute les scripts de nettoyage après les tests.
    Nettoie les fichiers cache, métadonnées macOS, et processus gourmands.

    Peut être désactivé via la variable d'environnement BBIA_DISABLE_AUTO_CLEANUP=1
    """
    # Vérifier si le nettoyage automatique est désactivé
    if os.environ.get("BBIA_DISABLE_AUTO_CLEANUP", "0") == "1":
        return

    try:
        # Chemin vers le script de nettoyage
        project_root = Path(__file__).parent.parent
        cleanup_script = project_root / "scripts" / "cleanup_all.sh"

        if not cleanup_script.exists():
            print(f"⚠️  Script de nettoyage non trouvé: {cleanup_script}")
            return

        # S'assurer que le script a les permissions d'exécution
        try:
            os.chmod(cleanup_script, 0o755)
        except Exception:
            # Ignorer si chmod échoue (peut être normal en CI)
            pass

        # Exécuter le script de nettoyage en mode non-interactif (cache uniquement)
        # Ne pas nettoyer la RAM automatiquement (peut être dangereux)
        print("\n🧹 Exécution du nettoyage automatique après les tests...")
        try:
            # Utiliser bash explicitement pour éviter les problèmes de permissions
            result = subprocess.run(
                ["bash", str(cleanup_script), "--cache-only", "--yes"],
                cwd=str(project_root),
                capture_output=True,
                text=True,
                timeout=30,  # Timeout de 30 secondes max (optimisé)
            )
            if result.returncode == 0:
                print("✅ Nettoyage automatique terminé avec succès")
            else:
                print(f"⚠️  Nettoyage terminé avec code {result.returncode}")
                if result.stderr:
                    print(f"   Erreur: {result.stderr[:200]}")
        except subprocess.TimeoutExpired:
            print("⚠️  Nettoyage timeout (ignoré)")
        except Exception as e:
            print(f"⚠️  Erreur lors du nettoyage (non bloquant): {e}")
    except Exception as e:
        # Ignorer toutes les erreurs pour ne pas bloquer la fin de pytest
        print(f"⚠️  Erreur nettoyage scripts (non bloquant): {e}")


@pytest.hookimpl(trylast=True)
def pytest_sessionfinish(session: pytest.Session, exitstatus: int) -> None:
    """
    Hook pytest qui s'exécute à la fin de la session complète.
    Force le nettoyage de toutes les ressources pour éviter que pytest reste bloqué.
    """
    force_cleanup_all_resources()

    # Exécuter les scripts de nettoyage après les tests
    run_cleanup_scripts()


@pytest.fixture(autouse=True)
def clear_model_caches_after_test():
    """
    Fixture automatique: nettoie les caches de modèles après chaque test.
    OPTIMISATION RAM: Libère mémoire après chaque test.
    N'importe pas HF/YOLO si le module n'a jamais été chargé.
    """
    yield
    import sys

    hf_mod = sys.modules.get("bbia_sim.bbia_huggingface")
    if hf_mod is None:
        return
    BBIAHuggingFace = getattr(hf_mod, "BBIAHuggingFace", None)
    if BBIAHuggingFace is None:
        return
    try:
        with BBIAHuggingFace._shared_unload_thread_lock:
            # Arrêter thread partagé si actif et plus d'instances
            if (
                not BBIAHuggingFace._shared_instances
                and BBIAHuggingFace._shared_unload_thread
                and BBIAHuggingFace._shared_unload_thread.is_alive()
            ):
                BBIAHuggingFace._shared_unload_thread_stop.set()
                BBIAHuggingFace._shared_unload_thread.join(timeout=0.5)
    except (AttributeError, RuntimeError):
        # Ignorer si attributs absents ou erreurs
        pass

    # Nettoyer cache MuJoCo après chaque test
    try:
        from bbia_sim.mujoco_model_cache import clear_mujoco_cache

        clear_mujoco_cache()
    except (ImportError, AttributeError):
        pass

    # Nettoyer après chaque test
    try:
        gc.collect()  # Force garbage collection

        # Nettoyer les boucles asyncio qui peuvent rester ouvertes
        try:
            import asyncio

            # Fermer toutes les boucles qui ne sont plus utilisées
            try:
                asyncio.get_running_loop()
                # Ne pas fermer si la boucle est en cours d'utilisation
            except RuntimeError:
                # Pas de boucle en cours, rien à faire
                pass
        except Exception:
            pass
    except Exception:
        pass


@pytest.fixture(scope="session")
def mock_yolo_detector():
    """
    Fixture session: Mock YOLO detector partagé (évite rechargement).
    OPTIMISATION RAM: Un seul mock pour toute la session de tests.
    """
    from unittest.mock import MagicMock

    mock = MagicMock()
    mock.is_loaded = True
    mock.model_size = "n"
    return mock


@pytest.fixture(scope="session")
def mock_whisper_stt():
    """
    Fixture session: Mock Whisper STT partagé (évite rechargement).
    OPTIMISATION RAM: Un seul mock pour toute la session de tests.
    """
    from unittest.mock import MagicMock

    mock = MagicMock()
    mock.is_loaded = True
    mock.model_size = "tiny"
    return mock
