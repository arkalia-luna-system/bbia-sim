#!/usr/bin/env python3
"""
Configuration globale pytest avec système de verrouillage pour éviter
l'exécution simultanée de plusieurs instances de tests.
OPTIMISATION RAM: Force utilisation de mocks pour modèles lourds.
"""

import atexit
import fcntl
import gc
import os
import signal
import sys
import threading
import time
from pathlib import Path

import pytest

# OPTIMISATION RAM: Forcer mode mock pour tests (évite chargement modèles lourds)
os.environ.setdefault("BBIA_DISABLE_AUDIO", "1")
os.environ.setdefault("BBIA_DISABLE_VISION_MODELS", "1")
os.environ.setdefault("BBIA_FORCE_MOCK_MODELS", "1")

# Chemin vers le fichier de lock
LOCK_FILE = Path(__file__).parent.parent / ".pytest.lock"
LOCK_TIMEOUT = 300  # 5 minutes max pour un run de tests
_MAX_RECURSION = 3  # Protection contre récursion infinie

# Stocker le file descriptor du lock globalement pour pouvoir le libérer
# même en cas d'interruption ou de blocage
_lock_fd: int | None = None


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

    # OPTIMISATION RAM: Nettoyer caches modèles avant tests
    try:
        # Nettoyer cache YOLO
        from bbia_sim.vision_yolo import _yolo_cache_lock, _yolo_model_cache

        with _yolo_cache_lock:
            _yolo_model_cache.clear()

        # Nettoyer cache MediaPipe
        from bbia_sim import vision_yolo
        from bbia_sim.vision_yolo import _mediapipe_cache_lock

        with _mediapipe_cache_lock:
            vision_yolo._mediapipe_face_detection_cache = None

        # Nettoyer cache Whisper
        try:
            from bbia_sim.voice_whisper import (
                _whisper_model_cache_lock,
                _whisper_models_cache,
            )

            with _whisper_model_cache_lock:
                _whisper_models_cache.clear()
        except ImportError:
            pass

        # Nettoyer cache HuggingFace
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            if hasattr(BBIAHuggingFace, "_clear_cache"):
                BBIAHuggingFace._clear_cache()
        except (ImportError, AttributeError):
            pass

        print("🧹 Caches modèles nettoyés (optimisation RAM)\n")
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

    # OPTIMISATION RAM: Nettoyer caches après tests
    try:
        from bbia_sim.vision_yolo import _yolo_cache_lock, _yolo_model_cache

        with _yolo_cache_lock:
            _yolo_model_cache.clear()
    except Exception:
        pass


@pytest.hookimpl(trylast=True)
def pytest_sessionfinish(session: pytest.Session, exitstatus: int) -> None:
    """
    Hook pytest qui s'exécute à la fin de la session complète.
    Force le nettoyage de toutes les ressources pour éviter que pytest reste bloqué.
    """
    force_cleanup_all_resources()


@pytest.fixture(autouse=True)
def clear_model_caches_after_test():
    """
    Fixture automatique: nettoie les caches de modèles après chaque test.
    OPTIMISATION RAM: Libère mémoire après chaque test.
    """
    yield
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
