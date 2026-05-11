"""Cache pour assets STL - Chargement lazy et optimisation mémoire."""

import logging
from collections import OrderedDict
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# Cache global pour fichiers STL chargés
_stl_cache: OrderedDict[str, bytes] = OrderedDict()
_cache_max_size: int = 20  # Maximum 20 fichiers STL en cache
_cache_max_size_bytes: int = 50 * 1024 * 1024  # 50 MB max
_STL_ROOT_DIR = (
    Path(__file__).resolve().parent / "sim" / "assets" / "reachy_official"
).resolve()
_ALLOWED_STL_FILES: dict[str, Path] = {
    path.name: path.resolve() for path in _STL_ROOT_DIR.glob("*.stl") if path.is_file()
}


def get_cached_stl(stl_name: str) -> bytes:
    """Récupère un fichier STL autorisé depuis le cache ou le charge si absent.

    Args:
        stl_name: Nom du fichier STL (ex: body_down_3dprint.stl)

    Returns:
        Contenu binaire du fichier STL

    Raises:
        FileNotFoundError: Si le fichier STL n'existe pas
        OSError: Si le fichier ne peut pas être lu
    """
    safe_name = Path(stl_name).name
    if safe_name != stl_name:
        logger.warning("Nom STL invalide refusé: %s", stl_name)
        raise FileNotFoundError("Nom de fichier STL invalide")

    stl_path_resolved = _ALLOWED_STL_FILES.get(safe_name)
    if stl_path_resolved is None:
        logger.warning("Fichier STL non autorisé: %s", safe_name)
        raise FileNotFoundError("Fichier STL non autorisé")

    stl_path_str = str(stl_path_resolved)

    # Vérifier si le STL est déjà en cache
    if stl_path_str in _stl_cache:
        # Déplacer en fin (LRU)
        _stl_cache.move_to_end(stl_path_str)
        logger.debug("✅ STL récupéré depuis cache: %s", stl_path_str)
        return _stl_cache[stl_path_str]

    # Vérifier que le fichier existe
    if not stl_path_resolved.exists():
        logger.error("Fichier STL introuvable: %s", stl_path_resolved)
        raise FileNotFoundError(f"Fichier STL introuvable: {stl_path_resolved}")

    # Charger le fichier STL
    logger.info("📦 Chargement STL: %s", stl_path_str)
    try:
        with open(stl_path_resolved, "rb") as f:
            stl_content = f.read()
    except OSError as e:
        logger.error("Erreur lecture STL %s: %s", stl_path_resolved, e)
        raise

    # Vérifier la taille du cache avant d'ajouter
    current_size = sum(len(content) for content in _stl_cache.values())
    stl_size = len(stl_content)

    # Si le nouveau fichier dépasse la limite, évincer les plus anciens
    while (
        len(_stl_cache) >= _cache_max_size
        or (current_size + stl_size) > _cache_max_size_bytes
    ):
        if not _stl_cache:
            break
        oldest_key = next(iter(_stl_cache))
        evicted_size = len(_stl_cache[oldest_key])
        del _stl_cache[oldest_key]
        current_size -= evicted_size
        logger.debug("🗑️ STL évincé du cache: %s", oldest_key)

    # Ajouter au cache
    _stl_cache[stl_path_str] = stl_content
    _stl_cache.move_to_end(stl_path_str)
    current_size += stl_size

    logger.debug(
        "✅ STL ajouté au cache (fichiers: %d/%d, taille: %.1f MB/%.1f MB)",
        len(_stl_cache),
        _cache_max_size,
        current_size / (1024 * 1024),
        _cache_max_size_bytes / (1024 * 1024),
    )
    return stl_content


def clear_stl_cache() -> None:
    """Vide le cache des fichiers STL."""
    global _stl_cache
    count = len(_stl_cache)
    total_size = sum(len(content) for content in _stl_cache.values())
    _stl_cache.clear()
    logger.info(
        "🗑️ Cache STL vidé (%d fichiers, %.1f MB supprimés)",
        count,
        total_size / (1024 * 1024),
    )


def get_stl_cache_stats() -> dict[str, Any]:
    """Retourne les statistiques du cache STL.

    Returns:
        Dictionnaire avec statistiques (size, max_size, cached_files, total_size_mb)
    """
    total_size = sum(len(content) for content in _stl_cache.values())
    return {
        "file_count": len(_stl_cache),
        "max_file_count": _cache_max_size,
        "total_size_bytes": total_size,
        "total_size_mb": total_size / (1024 * 1024),
        "max_size_mb": _cache_max_size_bytes / (1024 * 1024),
        "cached_files": list(_stl_cache.keys()),
    }


def set_stl_cache_max_size(max_files: int, max_size_mb: int = 50) -> None:
    """Définit les limites du cache STL.

    Args:
        max_files: Nombre maximum de fichiers en cache
        max_size_mb: Taille maximale du cache en MB

    Raises:
        ValueError: Si max_files < 1 ou max_size_mb < 1
    """
    global _cache_max_size, _cache_max_size_bytes
    if max_files < 1:
        raise ValueError("max_files doit être >= 1")
    if max_size_mb < 1:
        raise ValueError("max_size_mb doit être >= 1")

    old_max_files = _cache_max_size
    old_max_mb = _cache_max_size_bytes / (1024 * 1024)
    _cache_max_size = max_files
    _cache_max_size_bytes = max_size_mb * 1024 * 1024

    # Évincer les fichiers en trop si nécessaire
    current_size = sum(len(content) for content in _stl_cache.values())
    while len(_stl_cache) > _cache_max_size or current_size > _cache_max_size_bytes:
        if not _stl_cache:
            break
        oldest_key = next(iter(_stl_cache))
        evicted_size = len(_stl_cache[oldest_key])
        del _stl_cache[oldest_key]
        current_size -= evicted_size
        logger.debug("🗑️ STL évincé (réduction cache): %s", oldest_key)

    logger.info(
        "📊 Limites cache STL modifiées: %d fichiers/%d MB → %d fichiers/%d MB",
        old_max_files,
        int(old_max_mb),
        _cache_max_size,
        max_size_mb,
    )
