#!/usr/bin/env python3
"""Gardes runtime audio / media — anticipation Pollen #1507 / #1525 / #1521.

BBIA n'utilise pas GStreamer ni ``amixer`` via shell pour le volume.
Ce module force ``no_media`` / refuse les backends dangereux quand
``BBIA_DISABLE_AUDIO=1``, et détecte les chemins à risque.

Variables::

    BBIA_DISABLE_AUDIO=1   # force no_media + skip audio host
"""

from __future__ import annotations

import logging
import os
import re
from typing import Final

logger = logging.getLogger(__name__)

DISABLE_AUDIO_ENV: Final[str] = "BBIA_DISABLE_AUDIO"
_TRUTHY: Final[frozenset[str]] = frozenset({"1", "true", "yes", "on"})

# Backends / tokens connus pour chemin USB audio / GStreamer Pollen
_DANGEROUS_MEDIA_TOKENS: Final[frozenset[str]] = frozenset(
    {
        "gstreamer",
        "gst",
        "gst-launch",
        "audiomixer",
        "alsa_amixer",
        "amixer",
        "pulse_usb",
        "xmos",
        "usb_audio",
        "usb_xmos",
        "reachy_mini_audio",
        "reachy_audio",
        "no_reachy_mini_audio",
    }
)

# Indices explicites « device USB Reachy Mini Audio » (#1507)
_USB_AUDIO_HINTS: Final[tuple[str, ...]] = (
    "usb",
    "xmos",
    "reachy_mini_audio",
    "reachy_audio",
    "pulse_usb",
)

_SAFE_MEDIA_BACKENDS: Final[frozenset[str]] = frozenset(
    {
        "no_media",
        "none",
        "off",
        "disabled",
        "default",  # autorisé seulement si audio non désactivé
        "sounddevice",
        "opencv",
    }
)

_SHELL_AMIXER_RE: Final[re.Pattern[str]] = re.compile(
    r"(?:^|[\s;/|&])amixer(?:\s|$)|gst-launch|audiomixer",
    re.IGNORECASE,
)


def is_audio_disabled() -> bool:
    """True si BBIA_DISABLE_AUDIO est activé (CI / host sans USB audio stable)."""
    return os.environ.get(DISABLE_AUDIO_ENV, "").strip().lower() in _TRUTHY


def is_dangerous_media_backend(media_backend: str | None) -> bool:
    """True si le backend demandé ressemble à GStreamer / ALSA shell / USB XMOS."""
    if not media_backend:
        return False
    token = media_backend.strip().lower().replace("-", "_")
    if token in _DANGEROUS_MEDIA_TOKENS:
        return True
    return any(danger in token for danger in _DANGEROUS_MEDIA_TOKENS)


def looks_like_usb_reachy_audio(media_backend: str | None) -> bool:
    """True si le nom évoque le device USB « Reachy Mini Audio » (Pollen #1507).

    BBIA ne peut pas garantir la présence du device host ; on détecte le
    *chemin demandé* pour refuser clairement plutôt que laisser échouer
    en « No Reachy Mini Audio » côté GStreamer/ALSA.
    """
    if not media_backend:
        return False
    token = media_backend.strip().lower().replace("-", "_").replace(" ", "_")
    return any(hint in token for hint in _USB_AUDIO_HINTS)


def looks_like_shell_amixer(command: str | None) -> bool:
    """Détecte une commande shell amixer / gst-launch (Pollen #1521)."""
    if not command:
        return False
    return bool(_SHELL_AMIXER_RE.search(command))


def refuse_shell_audio_command(command: str | None, *, context: str = "audio") -> None:
    """Lève ValueError si la commande ressemble à amixer/GStreamer shell."""
    if looks_like_shell_amixer(command):
        raise ValueError(
            f"{context}: commande shell amixer/GStreamer refusée (Pollen #1521/#1525). "
            "Utiliser l'API volume Pydantic [0,1] ou media_backend=no_media."
        )


def resolve_media_backend(
    requested: str | None,
    *,
    context: str = "media",
    force_no_media_when_disabled: bool = True,
) -> str:
    """Normalise le media_backend ; force ``no_media`` si audio désactivé / dangereux.

    Returns:
        Backend sûr à passer au SDK (souvent ``no_media``).
    """
    raw = (requested or "no_media").strip() or "no_media"
    lowered = raw.lower()

    if is_audio_disabled() and force_no_media_when_disabled:
        if lowered not in {"no_media", "none", "off", "disabled"}:
            logger.warning(
                "%s: %s=1 → media_backend %r forcé à no_media "
                "(anticipation USB audio / GStreamer #1507/#1525)",
                context,
                DISABLE_AUDIO_ENV,
                raw,
            )
        return "no_media"

    if is_dangerous_media_backend(raw) or looks_like_usb_reachy_audio(raw):
        reason = (
            "USB Reachy Mini Audio / XMOS (#1507 — device host souvent absent)"
            if looks_like_usb_reachy_audio(raw)
            else "GStreamer/ALSA shell (#1525/#1521)"
        )
        logger.warning(
            "%s: media_backend %r refusé → no_media (%s). "
            "Contournement: media_backend=no_media ou BBIA_DISABLE_AUDIO=1. "
            "Le firmware/host Pollen reste exposé si audio USB forcé ailleurs.",
            context,
            raw,
            reason,
        )
        return "no_media"

    if lowered in {"none", "off", "disabled"}:
        return "no_media"

    if lowered not in _SAFE_MEDIA_BACKENDS and not lowered.startswith("custom_"):
        logger.warning(
            "%s: media_backend inconnu %r → no_media (refus défensif)",
            context,
            raw,
        )
        return "no_media"

    return raw if lowered != "none" else "no_media"
