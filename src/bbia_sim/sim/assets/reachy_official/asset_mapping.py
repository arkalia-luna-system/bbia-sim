"""Mapping des assets officiels Reachy Mini."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class AssetMapping:
    """Mapping d'un asset officiel."""

    component_name: str
    official_stl_path: Optional[str]  # None si pas encore disponible
    placeholder_path: str
    description: str


# Mapping des assets officiels Reachy Mini
OFFICIAL_ASSETS: dict[str, AssetMapping] = {
    "torso": AssetMapping(
        component_name="torso",
        official_stl_path=None,  # TODO: ajouter le chemin officiel
        placeholder_path="meshes/torso.stl",
        description="Torse principal du robot"
    ),
    "head": AssetMapping(
        component_name="head",
        official_stl_path=None,  # TODO: ajouter le chemin officiel
        placeholder_path="meshes/head.stl",
        description="Tête avec caméra"
    ),
    "upper_arm": AssetMapping(
        component_name="upper_arm",
        official_stl_path=None,  # TODO: ajouter le chemin officiel
        placeholder_path="meshes/upper_arm.stl",
        description="Bras supérieur (épaule)"
    ),
    "forearm": AssetMapping(
        component_name="forearm",
        official_stl_path=None,  # TODO: ajouter le chemin officiel
        placeholder_path="meshes/forearm.stl",
        description="Avant-bras (coude)"
    ),
    "gripper": AssetMapping(
        component_name="gripper",
        official_stl_path=None,  # TODO: ajouter le chemin officiel
        placeholder_path="meshes/gripper.stl",
        description="Pince/gripper"
    ),
}


def get_asset_path(component_name: str) -> str:
    """Retourne le chemin de l'asset (officiel si disponible, sinon placeholder)."""
    if component_name not in OFFICIAL_ASSETS:
        raise ValueError(f"Composant inconnu: {component_name}")

    asset = OFFICIAL_ASSETS[component_name]
    return asset.official_stl_path or asset.placeholder_path


def get_available_assets() -> dict[str, str]:
    """Retourne les assets disponibles avec leur chemin."""
    return {
        name: get_asset_path(name)
        for name in OFFICIAL_ASSETS.keys()
    }


def get_official_assets() -> dict[str, str]:
    """Retourne uniquement les assets officiels disponibles."""
    return {
        name: asset.official_stl_path
        for name, asset in OFFICIAL_ASSETS.items()
        if asset.official_stl_path is not None
    }


def get_placeholder_assets() -> dict[str, str]:
    """Retourne les assets placeholder encore utilisés."""
    return {
        name: asset.placeholder_path
        for name, asset in OFFICIAL_ASSETS.items()
        if asset.official_stl_path is None
    }
