#!/usr/bin/env python3
"""Calibration caméra Charuco - Reachy Mini.

Usage:
    python -m bbia_sim.tools.camera_calibration.calibrate \
        --images ./calibration_images \
        --output ./camera_calibration.json
"""

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from cv2 import aruco


def calibrate_camera(
    images_dir: Path,
    output_file: Path,
    board_size: tuple[int, int] = (7, 5),
    square_size: float = 20.0,
) -> None:
    """Calibre la caméra à partir d'images Charuco.

    Args:
        images_dir: Dossier contenant les images
        board_size: Taille du board (squares_x, squares_y)
        square_size: Taille d'un carré en mm
        output_file: Fichier de sortie JSON
    """
    # Créer le Charuco board
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
    board = aruco.CharucoBoard(
        board_size, squareLength=square_size, markerLength=square_size * 0.75, dictionary=dictionary
    )

    # Détecteur Charuco
    charuco_params = aruco.DetectorParameters()
    charuco_detector = aruco.CharucoDetector(board, charuco_params)

    # Collecter toutes les images
    image_files = sorted(images_dir.glob("*.jpg")) + sorted(images_dir.glob("*.png"))
    if not image_files:
        print(f"❌ Aucune image trouvée dans {images_dir}")
        sys.exit(1)

    print(f"📸 Traitement de {len(image_files)} images...")

    all_charuco_corners = []
    all_charuco_ids = []
    image_size = None

    for img_file in image_files:
        img = cv2.imread(str(img_file))
        if img is None:
            print(f"⚠️  Impossible de lire {img_file.name}")
            continue

        if image_size is None:
            image_size = (img.shape[1], img.shape[0])
        elif image_size != (img.shape[1], img.shape[0]):
            print(f"⚠️  Résolution incohérente dans {img_file.name}")
            continue

        # Détecter Charuco
        corners, ids, _ = charuco_detector.detectMarkers(img)
        if ids is None or len(ids) == 0:
            print(f"⚠️  Aucun marqueur détecté dans {img_file.name}")
            continue

        result = charuco_detector.interpolateCornersCharuco(corners, ids, img)
        charuco_corners, charuco_ids, _, _ = result

        if charuco_corners is None or len(charuco_corners) < 4:
            print(f"⚠️  Pas assez de coins Charuco dans {img_file.name}")
            continue

        all_charuco_corners.append(charuco_corners)
        all_charuco_ids.append(charuco_ids)
        print(f"✅ {img_file.name}: {len(charuco_corners)} coins détectés")

    if len(all_charuco_corners) < 5:
        print(f"❌ Pas assez d'images valides (minimum 5, trouvé {len(all_charuco_corners)})")
        sys.exit(1)

    print(f"\n🔧 Calibration avec {len(all_charuco_corners)} images...")

    # Préparer les points 3D
    obj_points = []
    img_points = []

    for charuco_corners, charuco_ids in zip(all_charuco_corners, all_charuco_ids, strict=False):
        # Points 3D du board
        obj_pts = []
        for corner_id in charuco_ids.flatten():
            row = corner_id // (board_size[0] - 1)
            col = corner_id % (board_size[0] - 1)
            obj_pts.append([col * square_size, row * square_size, 0.0])
        obj_points.append(np.array(obj_pts, dtype=np.float32))
        img_points.append(charuco_corners)

    # Calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None
    )

    if not ret:
        print("❌ Échec de la calibration")
        sys.exit(1)

    # Calculer erreur de reprojection
    total_error = 0
    total_points = 0
    for i in range(len(obj_points)):
        img_points_proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(img_points[i], img_points_proj, cv2.NORM_L2) / len(img_points_proj)
        total_error += error * len(img_points_proj)
        total_points += len(img_points_proj)

    mean_error = total_error / total_points if total_points > 0 else 0.0

    # Sauvegarder résultats
    calibration_data = {
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": dist_coeffs.flatten().tolist(),
        "resolution": {"width": image_size[0], "height": image_size[1]},
        "reprojection_error": float(mean_error),
        "board_size": {"squares_x": board_size[0], "squares_y": board_size[1]},
        "square_size_mm": square_size,
        "images_used": len(all_charuco_corners),
        "timestamp": datetime.now().isoformat(),
    }

    output_file.parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, "w") as f:
        json.dump(calibration_data, f, indent=2)

    print("\n✅ Calibration terminée !")
    print(f"   📁 Fichier: {output_file}")
    print(f"   📊 Erreur reprojection: {mean_error:.3f} pixels")
    print(f"   📐 Résolution: {image_size[0]}x{image_size[1]}")
    print(f"   📸 Images utilisées: {len(all_charuco_corners)}")


def main() -> None:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(description="Calibration caméra Charuco")
    parser.add_argument("--images", type=str, default="./calibration_images", help="Dossier images")
    parser.add_argument("--output", type=str, default="./camera_calibration.json", help="Fichier sortie")
    parser.add_argument("--board-size", type=str, default="7x5", help="Taille board (ex: 7x5)")
    parser.add_argument("--square-size", type=float, default=20.0, help="Taille carré en mm")

    args = parser.parse_args()

    board_size_tuple = tuple(map(int, args.board_size.split("x")))
    if len(board_size_tuple) != 2:
        print(f"❌ Taille board invalide: {args.board_size} (attendu format: 7x5)")
        sys.exit(1)
    board_size = (board_size_tuple[0], board_size_tuple[1])

    calibrate_camera(
        images_dir=Path(args.images),
        output_file=Path(args.output),
        board_size=board_size,
        square_size=args.square_size,
    )


if __name__ == "__main__":
    main()
