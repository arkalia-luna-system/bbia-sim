#!/usr/bin/env python3
"""Timeout / reconnect configurables pour le client SDK → daemon Pollen.

Anticipe Pollen #1494 (suite) + #1466/#1465 (relay Wireless HS) côté BBIA :
pas de patch firmware — bornes et retries sur la connexion commande.

Variables d'environnement (préfixe BBIA_)::

    BBIA_SDK_CONNECT_TIMEOUT=5.0     # secondes (défaut 5)
    BBIA_SDK_RECONNECT_ATTEMPTS=3    # tentatives totales (défaut 3, min 1)
    BBIA_SDK_RECONNECT_DELAY=0.5     # pause entre tentatives (s)
    BBIA_SDK_HEARTBEAT_TIMEOUT=2.0   # watchdog sans réponse (s)
"""

from __future__ import annotations

import logging
import os
import time
from collections.abc import Callable
from typing import Final, TypeVar

logger = logging.getLogger(__name__)

T = TypeVar("T")

ENV_CONNECT_TIMEOUT: Final[str] = "BBIA_SDK_CONNECT_TIMEOUT"
ENV_RECONNECT_ATTEMPTS: Final[str] = "BBIA_SDK_RECONNECT_ATTEMPTS"
ENV_RECONNECT_DELAY: Final[str] = "BBIA_SDK_RECONNECT_DELAY"
ENV_HEARTBEAT_TIMEOUT: Final[str] = "BBIA_SDK_HEARTBEAT_TIMEOUT"

_DEFAULT_CONNECT_TIMEOUT: Final[float] = 5.0
_DEFAULT_RECONNECT_ATTEMPTS: Final[int] = 3
_DEFAULT_RECONNECT_DELAY: Final[float] = 0.5
_DEFAULT_HEARTBEAT_TIMEOUT: Final[float] = 2.0


def _parse_float(env_name: str, default: float, *, minimum: float = 0.0) -> float:
    raw = os.environ.get(env_name, "").strip()
    if not raw:
        return default
    try:
        value = float(raw)
    except ValueError:
        logger.warning("%s invalide (%r) → défaut %s", env_name, raw, default)
        return default
    if value < minimum:
        logger.warning("%s=%s < %s → clamp", env_name, value, minimum)
        return minimum
    return value


def _parse_int(env_name: str, default: int, *, minimum: int = 1) -> int:
    raw = os.environ.get(env_name, "").strip()
    if not raw:
        return default
    try:
        value = int(raw)
    except ValueError:
        logger.warning("%s invalide (%r) → défaut %s", env_name, raw, default)
        return default
    if value < minimum:
        logger.warning("%s=%s < %s → clamp", env_name, value, minimum)
        return minimum
    return value


def get_sdk_connect_timeout(fallback: float | None = None) -> float:
    """Timeout connexion SDK (secondes)."""
    default = float(fallback) if fallback is not None else _DEFAULT_CONNECT_TIMEOUT
    if default <= 0:
        default = _DEFAULT_CONNECT_TIMEOUT
    return _parse_float(ENV_CONNECT_TIMEOUT, default, minimum=0.1)


def get_sdk_reconnect_attempts() -> int:
    """Nombre total de tentatives de connexion (inclut le 1er essai)."""
    return _parse_int(ENV_RECONNECT_ATTEMPTS, _DEFAULT_RECONNECT_ATTEMPTS, minimum=1)


def get_sdk_reconnect_delay() -> float:
    """Délai entre tentatives (secondes)."""
    return _parse_float(ENV_RECONNECT_DELAY, _DEFAULT_RECONNECT_DELAY, minimum=0.0)


def get_sdk_heartbeat_timeout(fallback: float | None = None) -> float:
    """Timeout heartbeat watchdog (secondes) — soft-detect relay down."""
    default = float(fallback) if fallback is not None else _DEFAULT_HEARTBEAT_TIMEOUT
    if default <= 0:
        default = _DEFAULT_HEARTBEAT_TIMEOUT
    return _parse_float(ENV_HEARTBEAT_TIMEOUT, default, minimum=0.2)


def call_with_reconnect(
    factory: Callable[[], T],
    *,
    attempts: int | None = None,
    delay: float | None = None,
    label: str = "SDK connect",
) -> T:
    """Exécute ``factory`` avec retries sur erreurs de connexion / timeout.

    Raises:
        Dernière exception si toutes les tentatives échouent.
    """
    total = get_sdk_reconnect_attempts() if attempts is None else max(1, int(attempts))
    pause = get_sdk_reconnect_delay() if delay is None else max(0.0, float(delay))
    last_exc: BaseException | None = None
    for attempt in range(1, total + 1):
        try:
            result = factory()
            if attempt > 1:
                logger.info("%s réussi après %s tentative(s)", label, attempt)
            return result
        except (
            OSError,
            TimeoutError,
            ConnectionError,
            RuntimeError,
            ValueError,
        ) as exc:
            last_exc = exc
            logger.warning(
                "%s échec %s/%s: %s",
                label,
                attempt,
                total,
                exc,
            )
            if attempt < total and pause > 0:
                time.sleep(pause)
    assert last_exc is not None
    raise last_exc
