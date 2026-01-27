#!/usr/bin/env python3
"""Calibration d'échelle pour résolutions multiples avec crop/zoom - Reachy Mini.

Usage:
    python -m bbia_sim.tools.camera_calibration.scale_calibration \
        --calibration ./camera_calibration.json \
        --resolution 640x480 \
        --crop 100,100,640,480 \
        --output ./camera_calibration_scaled.json
"""

import argparse
import json
from pathlib import Path


def scale_intrinsics(
    camera_matrix: list[list[float]],
    original_resolution: tuple[int, int],
    target_resolution: tuple[int, int],
    crop: tuple[int, int, int, int] | None = None,
) -> tuple[list[list[float]], tuple[int, int]]:
    """Met à l'échelle les intrinsics pour une nouvelle résolution.

    Args:
        camera_matrix: Matrice caméra originale
        original_resolution: Résolution originale (width, height)
        target_resolution: Résolution cible (width, height)
        crop: Zone de crop (x, y, width, height) ou None

    Returns:
        Tuple (matrice caméra mise à l'échelle, résolution finale)
    """
    fx, fy = camera_matrix[0][0], camera_matrix[1][1]
    cx, cy = camera_matrix[0][2], camera_matrix[1][2]

    orig_w, orig_h = original_resolution
    target_w, target_h = target_resolution

    # Facteurs d'échelle
    scale_x = target_w / orig_w
    scale_y = target_h / orig_h

    # Appliquer crop si nécessaire
    if crop:
        crop_x, crop_y, crop_w, crop_h = crop
        # Ajuster le centre optique pour le crop
        cx = cx - crop_x
        cy = cy - crop_y
        # Nouvelle résolution après crop
        final_w, final_h = crop_w, crop_h
    else:
        final_w, final_h = target_w, target_h

    # Mettre à l'échelle les intrinsics
    fx_scaled = fx * scale_x
    fy_scaled = fy * scale_y
    cx_scaled = cx * scale_x
    cy_scaled = cy * scale_y

    # Nouvelle matrice caméra
    scaled_matrix = [
        [fx_scaled, 0.0, cx_scaled],
        [0.0, fy_scaled, cy_scaled],
        [0.0, 0.0, 1.0],
    ]

    return scaled_matrix, (final_w, final_h)


def scale_calibration(
    calibration_file: Path,
    output_file: Path,
    resolution: tuple[int, int],
    crop: tuple[int, int, int, int] | None = None,
) -> None:
    """Met à l'échelle une calibration pour une nouvelle résolution.

    Args:
        calibration_file: Fichier de calibration original
        output_file: Fichier de sortie
        resolution: Résolution cible (width, height)
        crop: Zone de crop (x, y, width, height) ou None
    """
    # Charger calibration originale
    with open(calibration_file) as f:
        calib_data = json.load(f)

    orig_matrix = calib_data["camera_matrix"]
    orig_res = (
        calib_data["resolution"]["width"],
        calib_data["resolution"]["height"],
    )

    # Mettre à l'échelle
    scaled_matrix, final_res = scale_intrinsics(orig_matrix, orig_res, resolution, crop)

    # Créer nouvelle calibration
    scaled_calib = {
        "camera_matrix": scaled_matrix,
        "distortion_coefficients": calib_data["distortion_coefficients"],
        "resolution": {"width": final_res[0], "height": final_res[1]},
        "reprojection_error": calib_data.get("reprojection_error", 0.0),
        "original_resolution": {
            "width": orig_res[0],
            "height": orig_res[1],
        },
        "scale_factors": {
            "scale_x": final_res[0] / orig_res[0],
            "scale_y": final_res[1] / orig_res[1],
        },
    }

    if crop:
        scaled_calib["crop"] = {
            "x": crop[0],
            "y": crop[1],
            "width": crop[2],
            "height": crop[3],
        }

    # Sauvegarder
    output_file.parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, "w") as f:
        json.dump(scaled_calib, f, indent=2)

    print("✅ Calibration mise à l'échelle !")
    print(f"   📁 Fichier: {output_file}")
    print(f"   📐 Résolution originale: {orig_res[0]}x{orig_res[1]}")
    print(f"   📐 Résolution finale: {final_res[0]}x{final_res[1]}")
    if crop:
        print(f"   ✂️  Crop: {crop[0]},{crop[1]},{crop[2]},{crop[3]}")


def main() -> None:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description="Calibration d'échelle pour résolutions multiples"
    )
    parser.add_argument(
        "--calibration",
        type=str,
        required=True,
        help="Fichier de calibration original",
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Fichier de sortie",
    )
    parser.add_argument(
        "--resolution",
        type=str,
        required=True,
        help="Résolution cible (ex: 640x480)",
    )
    parser.add_argument(
        "--crop",
        type=str,
        help="Zone de crop (ex: 100,100,640,480)",
    )

    args = parser.parse_args()

    # Parser résolution
    w, h = map(int, args.resolution.split("x"))
    resolution = (w, h)

    # Parser crop si fourni
    crop = None
    if args.crop:
        x, y, w_crop, h_crop = map(int, args.crop.split(","))
        crop = (x, y, w_crop, h_crop)

    scale_calibration(
        calibration_file=Path(args.calibration),
        output_file=Path(args.output),
        resolution=resolution,
        crop=crop,
    )


if __name__ == "__main__":
    main()
