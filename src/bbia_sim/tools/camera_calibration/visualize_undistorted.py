#!/usr/bin/env python3
"""Visualisation images corrigées (undistorted) - Reachy Mini.

Usage:
    python -m bbia_sim.tools.camera_calibration.visualize_undistorted \
        --calibration ./camera_calibration.json \
        --image ./test_image.jpg
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np


def visualize_undistorted(
    calibration_file: Path,
    image_file: Path,
    output_file: Path | None = None,
) -> None:
    """Visualise une image corrigée (undistorted).

    Args:
        calibration_file: Fichier de calibration
        image_file: Image à corriger
        output_file: Fichier de sortie (optionnel)
    """
    # Charger calibration
    with open(calibration_file) as f:
        calib_data = json.load(f)

    camera_matrix = np.array(calib_data["camera_matrix"], dtype=np.float32)
    dist_coeffs = np.array(calib_data["distortion_coefficients"], dtype=np.float32)

    # Charger image
    img = cv2.imread(str(image_file))
    if img is None:
        print(f"❌ Impossible de lire {image_file}")
        sys.exit(1)

    # Corriger distorsion
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    undistorted = cv2.undistort(
        img, camera_matrix, dist_coeffs, None, new_camera_matrix
    )

    # Crop ROI si nécessaire
    x, y, w_roi, h_roi = roi
    if w_roi > 0 and h_roi > 0:
        undistorted = undistorted[y : y + h_roi, x : x + w_roi]

    # Afficher comparaison
    comparison = np.hstack([img, undistorted])
    cv2.putText(
        comparison,
        "Original",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
    )
    cv2.putText(
        comparison,
        "Undistorted",
        (w + 10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
    )

    cv2.imshow("Original vs Undistorted", comparison)
    print("⌨️  Appuyez sur 's' pour sauvegarder, 'q' pour quitter")

    key = cv2.waitKey(0) & 0xFF
    if key == ord("s") and output_file:
        cv2.imwrite(str(output_file), undistorted)
        print(f"✅ Image sauvegardée: {output_file}")
    elif key == ord("s") and not output_file:
        output_file = image_file.parent / f"{image_file.stem}_undistorted.jpg"
        cv2.imwrite(str(output_file), undistorted)
        print(f"✅ Image sauvegardée: {output_file}")

    cv2.destroyAllWindows()


def main() -> None:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description="Visualisation images corrigées (undistorted)"
    )
    parser.add_argument(
        "--calibration",
        type=str,
        required=True,
        help="Fichier de calibration",
    )
    parser.add_argument(
        "--image",
        type=str,
        required=True,
        help="Image à corriger",
    )
    parser.add_argument(
        "--output",
        type=str,
        help="Fichier de sortie (optionnel)",
    )

    args = parser.parse_args()

    output_file = Path(args.output) if args.output else None

    visualize_undistorted(
        calibration_file=Path(args.calibration),
        image_file=Path(args.image),
        output_file=output_file,
    )


if __name__ == "__main__":
    main()
