#!/usr/bin/env python3
"""Analyse des facteurs de crop pour différentes résolutions - Reachy Mini.

Usage:
    python -m bbia_sim.tools.camera_calibration.analyze_crop \
        --calibration ./camera_calibration.json \
        --resolutions 640x480,1280x720,1920x1080
"""

import argparse
import json
from pathlib import Path

import numpy as np


def analyze_crop_factors(
    calibration_file: Path,
    resolutions: list[tuple[int, int]],
) -> None:
    """Analyse les facteurs de crop pour différentes résolutions.

    Args:
        calibration_file: Fichier de calibration
        resolutions: Liste de résolutions (width, height)
    """
    # Charger calibration
    with open(calibration_file) as f:
        calib_data = json.load(f)

    orig_matrix = np.array(calib_data["camera_matrix"], dtype=np.float32)
    orig_res = (
        calib_data["resolution"]["width"],
        calib_data["resolution"]["height"],
    )

    fx, fy = orig_matrix[0][0], orig_matrix[1][1]
    cx, cy = orig_matrix[0][2], orig_matrix[1][2]

    print("📊 Analyse des facteurs de crop")
    print(f"   Résolution originale: {orig_res[0]}x{orig_res[1]}")
    print(f"   Centre optique: ({cx:.1f}, {cy:.1f})")
    print()

    for target_w, target_h in resolutions:
        # Facteurs d'échelle
        scale_x = target_w / orig_res[0]
        scale_y = target_h / orig_res[1]

        # Nouveaux centres optiques
        cx_scaled = cx * scale_x
        cy_scaled = cy * scale_y

        # Nouveaux focaux
        fx_scaled = fx * scale_x
        fy_scaled = fy * scale_y

        print(f"📐 Résolution: {target_w}x{target_h}")
        print(f"   Facteurs: {scale_x:.3f}x, {scale_y:.3f}y")
        print(f"   Focaux: fx={fx_scaled:.1f}, fy={fy_scaled:.1f}")
        print(f"   Centre: ({cx_scaled:.1f}, {cy_scaled:.1f})")
        print()

    # Recommandations
    print("💡 Recommandations:")
    print(
        "   - Utiliser scale_calibration.py pour générer calibrations mises à l'échelle"
    )
    print(
        "   - Fermer la caméra avant de changer de résolution (nécessaire pour WebRTC)"
    )
    print("   - Utiliser crop pour zoomer sur une zone spécifique")


def main() -> None:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description="Analyse des facteurs de crop pour différentes résolutions"
    )
    parser.add_argument(
        "--calibration",
        type=str,
        required=True,
        help="Fichier de calibration",
    )
    parser.add_argument(
        "--resolutions",
        type=str,
        required=True,
        help="Résolutions séparées par virgule (ex: 640x480,1280x720)",
    )

    args = parser.parse_args()

    # Parser résolutions
    resolutions = []
    for res_str in args.resolutions.split(","):
        w, h = map(int, res_str.strip().split("x"))
        resolutions.append((w, h))

    analyze_crop_factors(
        calibration_file=Path(args.calibration),
        resolutions=resolutions,
    )


if __name__ == "__main__":
    main()
