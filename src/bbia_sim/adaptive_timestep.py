"""Timestep adaptatif pour simulation MuJoCo - Optimisation performance selon complexité."""

import logging
from typing import Any

import mujoco

logger = logging.getLogger(__name__)

# Limites du timestep adaptatif
MIN_TIMESTEP: float = 0.005  # 200 Hz (scènes simples)
MAX_TIMESTEP: float = 0.02  # 50 Hz (scènes complexes)
DEFAULT_TIMESTEP: float = 0.01  # 100 Hz (par défaut)


def calculate_scene_complexity(model: mujoco.MjModel) -> float:
    """Calcule un score de complexité de la scène (0.0 = simple, 1.0 = complexe).

    Args:
        model: Modèle MuJoCo

    Returns:
        Score de complexité normalisé entre 0.0 et 1.0
    """
    # Facteurs de complexité
    n_joints = model.njnt
    n_bodies = model.nbody
    n_geoms = model.ngeom
    n_actuators = model.nu
    n_constraints = model.nv

    # Poids pour chaque facteur (ajustables selon expérience)
    weights = {
        "joints": 0.3,
        "bodies": 0.2,
        "geoms": 0.2,
        "actuators": 0.15,
        "constraints": 0.15,
    }

    # Normalisation approximative (basée sur valeurs typiques)
    # Reachy Mini : ~16 joints, ~20 bodies, ~30 geoms
    max_typical = {
        "joints": 50,
        "bodies": 100,
        "geoms": 150,
        "actuators": 30,
        "constraints": 50,
    }

    # Calcul du score pondéré
    complexity = (
        weights["joints"] * min(n_joints / max_typical["joints"], 1.0)
        + weights["bodies"] * min(n_bodies / max_typical["bodies"], 1.0)
        + weights["geoms"] * min(n_geoms / max_typical["geoms"], 1.0)
        + weights["actuators"] * min(n_actuators / max_typical["actuators"], 1.0)
        + weights["constraints"] * min(n_constraints / max_typical["constraints"], 1.0)
    )

    # Normaliser entre 0.0 et 1.0
    return min(max(complexity, 0.0), 1.0)


def calculate_adaptive_timestep(
    model: mujoco.MjModel,
    min_timestep: float = MIN_TIMESTEP,
    max_timestep: float = MAX_TIMESTEP,
) -> float:
    """Calcule un timestep adaptatif selon la complexité de la scène.

    Args:
        model: Modèle MuJoCo
        min_timestep: Timestep minimum (scènes simples)
        max_timestep: Timestep maximum (scènes complexes)

    Returns:
        Timestep adaptatif entre min_timestep et max_timestep
    """
    complexity = calculate_scene_complexity(model)

    # Interpolation linéaire : complexité faible → timestep petit (fréquence élevée)
    # complexité élevée → timestep grand (fréquence faible)
    # Inverser : complexité 0 → min_timestep, complexité 1 → max_timestep
    adaptive_timestep = min_timestep + complexity * (max_timestep - min_timestep)

    logger.debug(
        "Complexité scène: %.2f → Timestep adaptatif: %.4fs (%.1f Hz)",
        complexity,
        adaptive_timestep,
        1.0 / adaptive_timestep,
    )

    return adaptive_timestep


def apply_adaptive_timestep(
    model: mujoco.MjModel,
    min_timestep: float = MIN_TIMESTEP,
    max_timestep: float = MAX_TIMESTEP,
) -> float:
    """Applique un timestep adaptatif au modèle MuJoCo.

    Args:
        model: Modèle MuJoCo (sera modifié)
        min_timestep: Timestep minimum
        max_timestep: Timestep maximum

    Returns:
        Timestep appliqué
    """
    adaptive_timestep = calculate_adaptive_timestep(model, min_timestep, max_timestep)

    # Appliquer le timestep au modèle
    model.opt.timestep = adaptive_timestep

    logger.info(
        "✅ Timestep adaptatif appliqué: %.4fs (%.1f Hz) - Complexité: %.2f",
        adaptive_timestep,
        1.0 / adaptive_timestep,
        calculate_scene_complexity(model),
    )

    return adaptive_timestep


def get_timestep_info(model: mujoco.MjModel) -> dict[str, Any]:
    """Retourne les informations sur le timestep actuel.

    Args:
        model: Modèle MuJoCo

    Returns:
        Dictionnaire avec informations timestep
    """
    current_timestep = model.opt.timestep
    complexity = calculate_scene_complexity(model)
    adaptive_timestep = calculate_adaptive_timestep(model)

    return {
        "current_timestep": current_timestep,
        "current_frequency_hz": 1.0 / current_timestep if current_timestep > 0 else 0,
        "complexity_score": complexity,
        "recommended_timestep": adaptive_timestep,
        "recommended_frequency_hz": (
            1.0 / adaptive_timestep if adaptive_timestep > 0 else 0
        ),
        "min_timestep": MIN_TIMESTEP,
        "max_timestep": MAX_TIMESTEP,
        "scene_stats": {
            "joints": model.njnt,
            "bodies": model.nbody,
            "geoms": model.ngeom,
            "actuators": model.nu,
            "constraints": model.nv,
        },
    }
