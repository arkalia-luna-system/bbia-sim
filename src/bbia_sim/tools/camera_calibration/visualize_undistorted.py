#!/usr/bin/env python3
"""Visualisation images corrigées (undistorted) - Reachy Mini.

Usage:
    python -m bbia_sim.tools.camera_calibration.visualize_undistorted \
        --calibration ./camera_calibration.json \
        --image ./test_image.jpg
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def visualize_undistorted(calibration_file: Path, image_file: Path) -> None:
    """Visualise une image corrigée avec la calibration.

    Args:
        calibration_file: Fichier calibration JSON
        image_file: Image à corriger
    """
    import json

    # Charger calibration
    with open(calibration_file) as f:
        calib_data = json.load(f)

    camera_matrix = np.array(calib_data["camera_matrix"])
    dist_coeffs = np.array(calib_data["distortion_coefficients"])

    # Charger image
    img = cv2.imread(str(image_file))
    if img is None:
        print(f"❌ Impossible de lire {image_file}")
        sys.exit(1)

    # Corriger distorsion
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)

    # Crop ROI si nécessaire
    x, y, w_roi, h_roi = roi
    if w_roi > 0 and h_roi > 0:
        undistorted = undistorted[y : y + h_roi, x : x + w_roi]

    # Afficher côte à côte
    comparison = np.hstack([img, undistorted])
    cv2.putText(comparison, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(
        comparison, "Undistorted", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
    )

    cv2.imshow("Original vs Undistorted - Appuyez sur 'q' pour quitter", comparison)
    print("💡 Appuyez sur 'q' pour quitter")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main() -> None:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(description="Visualisation images corrigées")
    parser.add_argument("--calibration", type=str, required=True, help="Fichier calibration")
    parser.add_argument("--image", type=str, required=True, help="Image à corriger")

    args = parser.parse_args()

    visualize_undistorted(calibration_file=Path(args.calibration), image_file=Path(args.image))


if __name__ == "__main__":
    main()
