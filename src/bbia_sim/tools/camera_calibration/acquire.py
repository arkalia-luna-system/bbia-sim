#!/usr/bin/env python3
"""Acquisition d'images pour calibration caméra Charuco - Reachy Mini.

Usage:
    python -m bbia_sim.tools.camera_calibration.acquire \
        --output ./calibration_images \
        --count 20 \
        --delay 2.0
"""

import argparse
import sys
import time
from pathlib import Path

try:
    import cv2
    from cv2 import aruco
except ImportError:
    print(
        "❌ OpenCV non disponible. "
        "Installez: pip install opencv-python opencv-contrib-python"
    )
    sys.exit(1)

try:
    from reachy_mini import ReachyMini

    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    print("⚠️  SDK Reachy Mini non disponible. Utilisation webcam OpenCV.")


def acquire_images(
    output_dir: Path,
    count: int = 20,
    delay: float = 2.0,
    resolution: tuple[int, int] | None = None,
    crop: tuple[int, int, int, int] | None = None,
    use_sim: bool = False,
) -> None:
    """Acquiert des images pour calibration.

    Args:
        output_dir: Dossier de sortie
        count: Nombre d'images
        delay: Délai entre captures (secondes)
        resolution: Résolution (width, height) ou None pour auto
        crop: Zone de crop (x, y, width, height) ou None
        use_sim: Utiliser simulation si True
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    # Initialiser caméra
    camera = None
    if SDK_AVAILABLE:
        try:
            robot = ReachyMini(use_sim=use_sim)
            robot.__enter__()
            if hasattr(robot, "media") and robot.media:
                camera = robot.media.camera
                print(f"✅ Caméra SDK Reachy Mini initialisée (sim={use_sim})")
        except Exception as e:
            print(f"⚠️  Échec initialisation SDK: {e}")
            camera = None

    if camera is None:
        # Fallback webcam OpenCV
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ Impossible d'ouvrir la webcam")
            sys.exit(1)
        if resolution:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        print("✅ Webcam OpenCV initialisée")

    # Détecteur Charuco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
    charuco_params = aruco.DetectorParameters()
    board = aruco.CharucoBoard(
        (7, 5), squareLength=20.0, markerLength=15.0, dictionary=dictionary
    )
    charuco_detector = aruco.CharucoDetector(board, charuco_params)

    print(f"\n📸 Acquisition de {count} images...")
    print("💡 Déplacez le Charuco board à différents angles et distances")
    print("⌨️  Appuyez sur 's' pour sauvegarder, 'q' pour quitter\n")

    saved_count = 0
    frame_count = 0

    try:
        while saved_count < count:
            # Capturer frame
            if SDK_AVAILABLE and camera:
                frame = camera.get_frame()
                if frame is None:
                    time.sleep(0.1)
                    continue
                # Convertir RGB -> BGR si nécessaire
                if len(frame.shape) == 3 and frame.shape[2] == 3:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

            # Appliquer crop si nécessaire
            if crop:
                x, y, w, h = crop
                frame = frame[y : y + h, x : x + w]

            # Détecter Charuco
            corners, ids, _ = charuco_detector.detectMarkers(frame)
            if ids is not None and len(ids) > 0:
                result = charuco_detector.interpolateCornersCharuco(corners, ids, frame)
                charuco_corners, charuco_ids, _, _ = result
                if charuco_corners is not None and len(charuco_corners) > 0:
                    # Dessiner détection
                    frame = aruco.drawDetectedMarkers(frame, corners, ids)
                    frame = aruco.drawDetectedCornersCharuco(
                        frame, charuco_corners, charuco_ids
                    )

            # Afficher
            text = f"Saved: {saved_count}/{count} | Frame: {frame_count}"
            cv2.putText(
                frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )
            title = "Calibration - Appuyez sur 's' pour sauvegarder"
            cv2.imshow(title, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s") or (delay > 0 and frame_count % int(delay * 30) == 0):
                # Sauvegarder
                filename = output_dir / f"calibration_{saved_count:04d}.jpg"
                cv2.imwrite(str(filename), frame)
                saved_count += 1
                print(f"✅ Image {saved_count}/{count} sauvegardée: {filename.name}")

            frame_count += 1
            if delay > 0:
                time.sleep(1.0 / 30.0)  # ~30 FPS

    finally:
        cv2.destroyAllWindows()
        if SDK_AVAILABLE and camera:
            try:
                robot.__exit__(None, None, None)
            except Exception:
                pass
        elif not SDK_AVAILABLE:
            cap.release()

    print(f"\n✅ {saved_count} images sauvegardées dans {output_dir}")


def main() -> None:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description="Acquisition d'images pour calibration caméra"
    )
    parser.add_argument(
        "--output", type=str, default="./calibration_images", help="Dossier de sortie"
    )
    parser.add_argument("--count", type=int, default=20, help="Nombre d'images")
    parser.add_argument(
        "--delay", type=float, default=2.0, help="Délai entre captures (secondes)"
    )
    parser.add_argument("--resolution", type=str, help="Résolution (ex: 640x480)")
    parser.add_argument("--crop", type=str, help="Crop (ex: 100,100,640,480)")
    parser.add_argument("--sim", action="store_true", help="Utiliser simulation")

    args = parser.parse_args()

    resolution = None
    if args.resolution:
        w, h = map(int, args.resolution.split("x"))
        resolution = (w, h)

    crop = None
    if args.crop:
        x, y, w, h = map(int, args.crop.split(","))
        crop = (x, y, w, h)

    acquire_images(
        output_dir=Path(args.output),
        count=args.count,
        delay=args.delay,
        resolution=resolution,
        crop=crop,
        use_sim=args.sim,
    )


if __name__ == "__main__":
    main()
