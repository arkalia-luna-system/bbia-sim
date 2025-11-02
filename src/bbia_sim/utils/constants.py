"""Utility constants for the bbia_sim package."""

from importlib.resources import files

import bbia_sim

# Chemins adaptés pour BBIA
URDF_ROOT_PATH: str = str(files(bbia_sim).joinpath("sim/models"))
ASSETS_ROOT_PATH: str = str(files(bbia_sim).joinpath("sim/assets"))
MODELS_ROOT_PATH: str = str(files(bbia_sim).joinpath("sim/assets"))
