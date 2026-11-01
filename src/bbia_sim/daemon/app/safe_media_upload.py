"""Validation stricte des uploads media (anticipation GHSA-m2pc-3q4q-w6jr).

BBIA n'expose pas ``/api/media/sounds/upload`` du daemon Pollen. Ce module
sert de garde-fou si une surface d'upload est ajoutée plus tard, et de
référence pour les tests de non-régression.
"""

from __future__ import annotations

import os
from pathlib import Path

# Allowlist audio uniquement — jamais .sh / .py / sans extension
ALLOWED_SOUND_EXTENSIONS = frozenset({".wav", ".ogg", ".flac", ".mp3"})
# ~2 Mo par défaut (évite dump binaire arbitraire)
DEFAULT_MAX_SOUND_BYTES = 2 * 1024 * 1024


def sanitize_sound_filename(filename: str | None) -> str:
    """Retourne un basename sûr avec extension allowlistée.

    Raises:
        ValueError: nom vide, traversal, ou extension interdite.
    """
    if not filename or not str(filename).strip():
        raise ValueError("Filename is required")
    raw = str(filename).strip()
    # Null byte = extension spoofing / truncation OS (ex: tone.wav\x00.sh)
    if "\x00" in raw:
        raise ValueError("Invalid filename")
    if "/" in raw or "\\" in raw or raw in (".", ".."):
        raise ValueError("Path traversal rejected")
    safe = Path(raw).name
    if not safe or safe != raw or "\x00" in safe:
        raise ValueError("Invalid filename")
    ext = Path(safe).suffix.lower()
    if ext not in ALLOWED_SOUND_EXTENSIONS:
        raise ValueError(
            f"Extension interdite: {ext!r} (autorisées: "
            f"{sorted(ALLOWED_SOUND_EXTENSIONS)})"
        )
    return safe


def validate_sound_payload(
    filename: str | None,
    content: bytes,
    *,
    max_bytes: int = DEFAULT_MAX_SOUND_BYTES,
) -> str:
    """Valide nom + taille. Retourne le basename sanitizé."""
    safe = sanitize_sound_filename(filename)
    if not content:
        raise ValueError("Empty file rejected")
    if len(content) > max_bytes:
        raise ValueError(f"File too large (max {max_bytes} bytes)")
    # Magic numbers légers (WAV / OGG / FLAC / ID3/MP3)
    head = content[:12]
    ext = Path(safe).suffix.lower()
    if ext == ".wav" and not head.startswith(b"RIFF"):
        raise ValueError("Content does not look like WAV")
    if ext == ".ogg" and not head.startswith(b"OggS"):
        raise ValueError("Content does not look like OGG")
    if ext == ".flac" and not head.startswith(b"fLaC"):
        raise ValueError("Content does not look like FLAC")
    if ext == ".mp3" and not (
        head.startswith(b"ID3") or head[:2] in (b"\xff\xfb", b"\xff\xfa", b"\xff\xf3")
    ):
        raise ValueError("Content does not look like MP3")
    return safe


def safe_dest_under(base_dir: Path | str, filename: str) -> Path:
    """Construit un chemin confiné sous base_dir (anti path-join escape)."""
    safe_name = sanitize_sound_filename(filename)
    base = Path(os.path.realpath(str(base_dir)))
    candidate = Path(os.path.realpath(str(base / safe_name)))
    if os.path.commonpath([str(base), str(candidate)]) != str(base):
        raise ValueError("Destination escapes base directory")
    return candidate
