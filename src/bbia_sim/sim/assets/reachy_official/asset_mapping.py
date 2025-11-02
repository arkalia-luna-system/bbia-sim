"""Mapping des assets officiels Reachy Mini."""

from dataclasses import dataclass


@dataclass
class AssetMapping:
    """Mapping d'un asset officiel."""

    component_name: str
    official_stl_path: str | None  # None si pas encore disponible
    placeholder_path: str
    description: str


# Mapping des assets officiels Reachy Mini
OFFICIAL_ASSETS: dict[str, AssetMapping] = {
    "torso": AssetMapping(
        component_name="torso",
        official_stl_path="body_top_3dprint.stl",
        placeholder_path="meshes/torso.stl",
        description="Torse principal du robot (partie supérieure)",
    ),
    "torso_base": AssetMapping(
        component_name="torso_base",
        official_stl_path="body_down_3dprint.stl",
        placeholder_path="meshes/torso_base.stl",
        description="Base du torse du robot",
    ),
    "head": AssetMapping(
        component_name="head",
        official_stl_path="head_front_3dprint.stl",
        placeholder_path="meshes/head.stl",
        description="Tête avec caméra (avant)",
    ),
    "head_back": AssetMapping(
        component_name="head_back",
        official_stl_path="head_back_3dprint.stl",
        placeholder_path="meshes/head_back.stl",
        description="Arrière de la tête",
    ),
    "stewart_arm": AssetMapping(
        component_name="stewart_arm",
        official_stl_path="mp01062_stewart_arm_3.stl",
        placeholder_path="meshes/stewart_arm.stl",
        description="Bras Stewart principal",
    ),
    "stewart_link": AssetMapping(
        component_name="stewart_link",
        official_stl_path="stewart_link_rod.stl",
        placeholder_path="meshes/stewart_link.stl",
        description="Tige de liaison Stewart",
    ),
    "stewart_gripper": AssetMapping(
        component_name="stewart_gripper",
        official_stl_path="stewart_tricap_3dprint.stl",
        placeholder_path="meshes/stewart_gripper.stl",
        description="Capuchon triangulaire Stewart (gripper)",
    ),
    "stewart_plate": AssetMapping(
        component_name="stewart_plate",
        official_stl_path="stewart_main_plate_3dprint.stl",
        placeholder_path="meshes/stewart_plate.stl",
        description="Plaque principale Stewart",
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
    return {name: get_asset_path(name) for name in OFFICIAL_ASSETS.keys()}


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
