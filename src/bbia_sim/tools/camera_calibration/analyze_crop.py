#!/usr/bin/env python3
"""Analyse facteurs de crop pour résolutions multiples - Reachy Mini.

Usage:
    python -m bbia_sim.tools.camera_calibration.analyze_crop \
        --calibration ./camera_calibration.json \
        --resolutions 640x480,1280x720,1920x1080
"""

import argparse
import json
from pathlib import Path

import numpy as np

from .scale_calibration import scale_intrinsics


def analyze_crop(calibration_file: Path, resolutions: list[tuple[int, int]]) -> None:
    """Analyse les facteurs de crop pour différentes résolutions.

    Args:
        calibration_file: Fichier calibration
        resolutions: Liste de résolutions (width, height)
    """
    # Charger calibration
    with open(calibration_file) as f:
        calib_data = json.load(f)

    camera_matrix = np.array(calib_data["camera_matrix"])
    orig_res = (calib_data["resolution"]["width"], calib_data["resolution"]["height"])

    print(f"📊 Analyse crop pour {len(resolutions)} résolutions")
    print(f"📐 Résolution originale: {orig_res[0]}x{orig_res[1]}\n")

    for res in resolutions:
        w, h = res
        scale_x = w / orig_res[0]
        scale_y = h / orig_res[1]

        # Calculer intrinsics mises à l'échelle
        scaled_matrix = scale_intrinsics(camera_matrix, orig_res, res, None)

        fx_orig = camera_matrix[0, 0]
        fy_orig = camera_matrix[1, 1]
        fx_scaled = scaled_matrix[0, 0]
        fy_scaled = scaled_matrix[1, 1]

        print(f"📐 Résolution: {w}x{h}")
        print(f"   Scale: {scale_x:.3f}x (width), {scale_y:.3f}x (height)")
        print(f"   Focal: {fx_orig:.1f} → {fx_scaled:.1f} (fx), {fy_orig:.1f} → {fy_scaled:.1f} (fy)")
        print(f"   Principal point: ({scaled_matrix[0,2]:.1f}, {scaled_matrix[1,2]:.1f})")
        print()


def main() -> None:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(description="Analyse facteurs de crop")
    parser.add_argument("--calibration", type=str, required=True, help="Fichier calibration")
    parser.add_argument(
        "--resolutions", type=str, required=True, help="Résolutions (ex: 640x480,1280x720)"
    )

    args = parser.parse_args()

    resolutions = [tuple(map(int, r.split("x"))) for r in args.resolutions.split(",")]

    analyze_crop(calibration_file=Path(args.calibration), resolutions=resolutions)


if __name__ == "__main__":
    main()
