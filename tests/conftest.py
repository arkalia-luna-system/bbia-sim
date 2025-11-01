#!/usr/bin/env python3
"""
Configuration globale pytest avec système de verrouillage pour éviter
l'exécution simultanée de plusieurs instances de tests.
"""

import atexit
import fcntl
import os
import sys
import time
from pathlib import Path

import pytest

# Chemin vers le fichier de lock
LOCK_FILE = Path(__file__).parent.parent / ".pytest.lock"
LOCK_TIMEOUT = 300  # 5 minutes max pour un run de tests


def acquire_lock() -> bool:
    """
    Acquiert un verrou exclusif pour empêcher l'exécution simultanée.

    Returns:
        True si le lock est acquis, False sinon.
    """
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

            # Enregistrer la libération du lock à la fin
            atexit.register(release_lock, lock_fd)

            return True
        except BlockingIOError:
            # Lock déjà acquis par un autre processus
            os.close(lock_fd)

            # Lire qui a le lock
            try:
                with open(LOCK_FILE) as f:
                    lock_info = f.read().strip()
                    pid, timestamp_str = lock_info.split(":")
                    lock_timestamp = float(timestamp_str)

                    # Vérifier si le processus existe encore
                    try:
                        os.kill(int(pid), 0)  # Vérifier si processus existe
                        elapsed = time.time() - lock_timestamp

                        if elapsed > LOCK_TIMEOUT:
                            # Lock expiré (processus probablement mort)
                            print(
                                f"⚠️  Lock expiré (>{LOCK_TIMEOUT}s). "
                                f"Processus {pid} pourrait être mort. "
                                f"Suppression du lock..."
                            )
                            os.remove(LOCK_FILE)
                            # Réessayer une fois
                            return acquire_lock()
                        else:
                            print(
                                f"❌ Tests déjà en cours d'exécution !\n"
                                f"   Processus PID: {pid}\n"
                                f"   Lock acquis il y a: {elapsed:.1f}s\n"
                                f"   Fichier lock: {LOCK_FILE}\n\n"
                                f"💡 Solutions:\n"
                                f"   1. Attendre la fin de l'autre processus\n"
                                f"   2. Vérifier: ps aux | grep {pid}\n"
                                f"   3. Si processus mort: rm {LOCK_FILE}\n"
                                f"   4. Timeout automatique après {LOCK_TIMEOUT}s\n"
                            )
                            return False
                    except ProcessLookupError:
                        # Processus n'existe plus, lock orphelin
                        print(
                            f"⚠️  Lock orphelin détecté (processus {pid} n'existe plus). "
                            f"Suppression..."
                        )
                        os.remove(LOCK_FILE)
                        # Réessayer une fois
                        return acquire_lock()
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


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config: pytest.Config) -> None:
    """
    Hook pytest qui s'exécute au démarrage.
    Vérifie le lock avant de lancer les tests.
    """
    # Acquérir le lock uniquement dans pytest
    # (ce hook ne s'exécute que si on est vraiment dans pytest)
    if not acquire_lock():
        print("\n❌ Impossible d'acquérir le verrou d'exécution.")
        print("   Un autre processus exécute déjà les tests.\n")
        sys.exit(1)

    print("✅ Verrou d'exécution acquis. Tests sécurisés.\n")


@pytest.hookimpl(trylast=True)
def pytest_unconfigure(config: pytest.Config) -> None:
    """Hook pytest qui s'exécute à la fin des tests."""
    # Le lock sera libéré par atexit, mais on peut aussi le faire ici
    try:
        if LOCK_FILE.exists():
            os.remove(LOCK_FILE)
    except Exception:
        pass
