"""Cache LRU pour modèles MuJoCo - Optimisation chargement modèles."""

import logging
from collections import OrderedDict
from pathlib import Path
from typing import Any

import mujoco

logger = logging.getLogger(__name__)

# Cache global pour modèles MuJoCo
_mujoco_model_cache: OrderedDict[str, mujoco.MjModel] = OrderedDict()
_cache_max_size: int = 5  # Maximum 5 modèles en cache


def get_cached_mujoco_model(model_path: str | Path) -> mujoco.MjModel:
    """Récupère un modèle MuJoCo depuis le cache ou le charge si absent.

    Args:
        model_path: Chemin vers le fichier XML du modèle MuJoCo

    Returns:
        Instance MjModel (depuis cache ou chargée)

    Raises:
        FileNotFoundError: Si le fichier modèle n'existe pas
        mujoco.FatalError: Si le modèle MJCF est invalide
    """
    model_path_str = str(Path(model_path).resolve())

    # Vérifier si le modèle est déjà en cache
    if model_path_str in _mujoco_model_cache:
        # Déplacer en fin (LRU - Least Recently Used)
        _mujoco_model_cache.move_to_end(model_path_str)
        logger.debug("✅ Modèle MuJoCo récupéré depuis cache: %s", model_path_str)
        return _mujoco_model_cache[model_path_str]

    # Charger le modèle
    model_path_obj = Path(model_path)
    if not model_path_obj.exists():
        logger.error("Modèle MuJoCo introuvable: %s", model_path_obj)
        raise FileNotFoundError(f"Modèle MuJoCo introuvable: {model_path_obj}")

    logger.info("📦 Chargement modèle MuJoCo: %s", model_path_str)
    model = mujoco.MjModel.from_xml_path(str(model_path_obj))

    # Ajouter au cache
    _mujoco_model_cache[model_path_str] = model
    _mujoco_model_cache.move_to_end(model_path_str)

    # Évincer le plus ancien si cache plein
    if len(_mujoco_model_cache) > _cache_max_size:
        oldest_key = next(iter(_mujoco_model_cache))
        del _mujoco_model_cache[oldest_key]
        logger.debug("🗑️ Modèle évincé du cache: %s", oldest_key)

    logger.debug(
        "✅ Modèle ajouté au cache (taille: %d/%d)",
        len(_mujoco_model_cache),
        _cache_max_size,
    )
    return model


def clear_mujoco_cache() -> None:
    """Vide le cache des modèles MuJoCo."""
    global _mujoco_model_cache
    count = len(_mujoco_model_cache)
    _mujoco_model_cache.clear()
    logger.info("🗑️ Cache MuJoCo vidé (%d modèles supprimés)", count)


def get_cache_stats() -> dict[str, Any]:
    """Retourne les statistiques du cache.

    Returns:
        Dictionnaire avec statistiques (size, max_size, cached_models)
    """
    return {
        "size": len(_mujoco_model_cache),
        "max_size": _cache_max_size,
        "cached_models": list(_mujoco_model_cache.keys()),
    }


def set_cache_max_size(max_size: int) -> None:
    """Définit la taille maximale du cache.

    Args:
        max_size: Nombre maximum de modèles en cache

    Raises:
        ValueError: Si max_size < 1
    """
    global _cache_max_size
    if max_size < 1:
        raise ValueError("max_size doit être >= 1")

    old_max = _cache_max_size
    _cache_max_size = max_size

    # Évincer les modèles en trop si nécessaire
    while len(_mujoco_model_cache) > _cache_max_size:
        oldest_key = next(iter(_mujoco_model_cache))
        del _mujoco_model_cache[oldest_key]
        logger.debug("🗑️ Modèle évincé (réduction cache): %s", oldest_key)

    logger.info(
        "📊 Taille cache MuJoCo modifiée: %d → %d",
        old_max,
        _cache_max_size,
    )
