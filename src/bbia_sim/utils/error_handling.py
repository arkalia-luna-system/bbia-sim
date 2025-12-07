"""Module de gestion centralisée des erreurs pour BBIA.

Ce module fournit des fonctions utilitaires pour gérer les erreurs de manière
cohérente dans tout le projet, réduisant la duplication de code et facilitant
le debugging.
"""

import logging
from collections.abc import Callable
from typing import Any, TypeVar

T = TypeVar("T")

logger = logging.getLogger(__name__)


def safe_execute(
    func: Callable[[], T],
    fallback: T | None = None,
    logger_instance: logging.Logger | None = None,
    error_msg: str = "Erreur lors de l'exécution",
    critical: bool = False,
    reraise: bool = False,
) -> T | None:
    """Exécute une fonction avec gestion d'erreurs centralisée.

    Args:
        func: Fonction à exécuter (sans arguments)
        fallback: Valeur de retour en cas d'erreur
        logger_instance: Logger à utiliser (si None, utilise logger du module)
        error_msg: Message d'erreur personnalisé
        critical: Si True, log en ERROR, sinon DEBUG
        reraise: Si True, relève l'exception après logging

    Returns:
        Résultat de func ou fallback en cas d'erreur

    Example:
        >>> from bbia_sim.utils.error_handling import safe_execute
        >>> result = safe_execute(
        ...     lambda: os.environ.setdefault("KEY", "value"),
        ...     fallback=None,
        ...     logger=logger,
        ...     error_msg="Impossible de configurer variable d'environnement",
        ...     critical=False
        ... )
    """
    log = logger_instance if logger_instance is not None else logger
    try:
        return func()
    except Exception as e:
        if log:
            log_func = log.error if critical else log.debug
            log_func(f"{error_msg}: {e}")
        if reraise:
            raise
        return fallback


def safe_import(
    module_name: str,
    logger_instance: logging.Logger | None = None,
) -> Any | None:
    """Importe un module avec gestion d'erreurs.

    Args:
        module_name: Nom du module (ex: "mediapipe")
        logger_instance: Logger optionnel

    Returns:
        Module importé ou None si échec

    Example:
        >>> from bbia_sim.utils.error_handling import safe_import
        >>> mediapipe = safe_import("mediapipe", logger=logger)
        >>> if mediapipe:
        ...     # Utiliser mediapipe
        ...     pass
    """
    log = logger_instance if logger_instance is not None else logger
    try:
        return __import__(module_name)
    except ImportError as e:
        if log:
            log.debug(f"Module {module_name} non disponible: {e}")
        return None
    except Exception as e:
        if log:
            log.warning(f"Erreur inattendue lors de l'import de {module_name}: {e}")
        return None


def safe_execute_with_exceptions(
    func: Callable[[], T],
    expected_exceptions: tuple[type[Exception], ...],
    fallback: T | None = None,
    logger_instance: logging.Logger | None = None,
    error_msg: str = "Erreur lors de l'exécution",
    critical: bool = False,
) -> T | None:
    """Exécute une fonction en gérant spécifiquement certaines exceptions.

    Args:
        func: Fonction à exécuter
        expected_exceptions: Tuple d'exceptions attendues à gérer spécifiquement
        fallback: Valeur de retour en cas d'erreur
        logger_instance: Logger à utiliser
        error_msg: Message d'erreur personnalisé
        critical: Si True, log en ERROR, sinon DEBUG

    Returns:
        Résultat de func ou fallback en cas d'erreur

    Example:
        >>> from bbia_sim.utils.error_handling import safe_execute_with_exceptions
        >>> result = safe_execute_with_exceptions(
        ...     lambda: robot.set_emotion("happy", 0.8),
        ...     expected_exceptions=(ConnectionError, RuntimeError),
        ...     fallback=False,
        ...     logger=logger,
        ...     error_msg="Erreur lors de la définition de l'émotion",
        ...     critical=True
        ... )
    """
    log = logger_instance if logger_instance is not None else logger
    try:
        return func()
    except expected_exceptions as e:
        if log:
            log_func = log.error if critical else log.debug
            log_func(f"{error_msg} ({type(e).__name__}): {e}")
        return fallback
    except Exception as e:
        if log:
            log.warning(f"{error_msg} (erreur inattendue {type(e).__name__}): {e}")
        return fallback
