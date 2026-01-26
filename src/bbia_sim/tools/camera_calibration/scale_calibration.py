#!/usr/bin/env python3
"""Calibration d'échelle pour résolutions multiples - Reachy Mini.

Usage:
    python -m bbia_sim.tools.camera_calibration.scale_calibration \
        --calibration ./camera_calibration.json \
        --resolution 640x480 \
        --crop 100,100,640,480 \
        --output ./camera_calibration_scaled.json
"""

import argparse
import json
from datetime import datetime
from pathlib import Path

import numpy as np


def scale_intrinsics(
    camera_matrix: np.ndarray,
    original_resolution: tuple[int, int],
    new_resolution: tuple[int, int],
    crop: tuple[int, int, int, int] | None = None,
) -> np.ndarray:
    """Met à l'échelle les intrinsics pour une nouvelle résolution.

    Args:
        camera_matrix: Matrice caméra originale
        original_resolution: Résolution originale (width, height)
        new_resolution: Nouvelle résolution (width, height)
        crop: Zone de crop (x, y, width, height) ou None

    Returns:
        Matrice caméra mise à l'échelle
    """
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]

    orig_w, orig_h = original_resolution
    new_w, new_h = new_resolution

    if crop:
        crop_x, crop_y, crop_w, crop_h = crop
        # Ajuster pour le crop
        scale_x = new_w / crop_w
        scale_y = new_h / crop_h
        fx_scaled = fx * scale_x
        fy_scaled = fy * scale_y
        cx_scaled = (cx - crop_x) * scale_x
        cy_scaled = (cy - crop_y) * scale_y
    else:
        # Mise à l'échelle simple
        scale_x = new_w / orig_w
        scale_y = new_h / orig_h
        fx_scaled = fx * scale_x
        fy_scaled = fy * scale_y
        cx_scaled = cx * scale_x
        cy_scaled = cy * scale_y

    scaled_matrix = np.array([[fx_scaled, 0, cx_scaled], [0, fy_scaled, cy_scaled], [0, 0, 1]])

    return scaled_matrix


def scale_calibration(
    calibration_file: Path,
    output_file: Path,
    resolution: tuple[int, int],
    crop: tuple[int, int, int, int] | None = None,
) -> None:
    """Met à l'échelle une calibration pour une nouvelle résolution.

    Args:
        calibration_file: Fichier calibration original
        output_file: Fichier sortie
        resolution: Nouvelle résolution (width, height)
        crop: Zone de crop (x, y, width, height) ou None
    """
    # Charger calibration originale
    with open(calibration_file) as f:
        calib_data = json.load(f)

    camera_matrix = np.array(calib_data["camera_matrix"])
    orig_res = (calib_data["resolution"]["width"], calib_data["resolution"]["height"])

    # Mettre à l'échelle
    scaled_matrix = scale_intrinsics(camera_matrix, orig_res, resolution, crop)

    # Créer nouvelle calibration
    scaled_data = calib_data.copy()
    scaled_data["camera_matrix"] = scaled_matrix.tolist()
    scaled_data["resolution"] = {"width": resolution[0], "height": resolution[1]}
    if crop:
        scaled_data["crop"] = {"x": crop[0], "y": crop[1], "width": crop[2], "height": crop[3]}
    scaled_data["original_resolution"] = orig_res
    scaled_data["timestamp"] = datetime.now().isoformat()

    # Sauvegarder
    output_file.parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, "w") as f:
        json.dump(scaled_data, f, indent=2)

    print("✅ Calibration mise à l'échelle !")
    print(f"   📁 Fichier: {output_file}")
    print(f"   📐 Résolution: {resolution[0]}x{resolution[1]}")
    if crop:
        print(f"   ✂️  Crop: {crop[0]},{crop[1]},{crop[2]},{crop[3]}")


def main() -> None:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(description="Calibration d'échelle pour résolutions multiples")
    parser.add_argument("--calibration", type=str, required=True, help="Fichier calibration original")
    parser.add_argument("--resolution", type=str, required=True, help="Nouvelle résolution (ex: 640x480)")
    parser.add_argument("--crop", type=str, help="Zone crop (ex: 100,100,640,480)")
    parser.add_argument("--output", type=str, required=True, help="Fichier sortie")

    args = parser.parse_args()

    resolution = tuple(map(int, args.resolution.split("x")))
    crop = None
    if args.crop:
        crop = tuple(map(int, args.crop.split(",")))

    scale_calibration(
        calibration_file=Path(args.calibration),
        output_file=Path(args.output),
        resolution=resolution,
        crop=crop,
    )


if __name__ == "__main__":
    main()
