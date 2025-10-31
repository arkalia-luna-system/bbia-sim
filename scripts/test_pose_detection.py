#!/usr/bin/env python3
"""
Test MediaPipe Pose - Détection postures/gestes

Usage:
    # Dans venv-vision-py310 (où MediaPipe est installé)
    source venv-vision-py310/bin/activate

    # Test avec webcam
    python scripts/test_pose_detection.py --webcam

    # Test avec image
    python scripts/test_pose_detection.py --image photo.jpg
"""

import argparse
import sys
from pathlib import Path

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

try:
    from bbia_sim.pose_detection import create_pose_detector

    MEDIAPIPE_POSE_AVAILABLE = True
except ImportError:
    print("❌ MediaPipe Pose non disponible")
    print("💡 MediaPipe est déjà installé dans venv-vision-py310")
    print("   Vérifier: pip install mediapipe")
    sys.exit(1)


def test_webcam(pose_detector, complexity: int = 1):
    """Test avec webcam en temps réel."""
    try:
        import cv2
    except ImportError:
        print("❌ OpenCV non disponible")
        return 1

    # Ouvrir webcam
    camera_index = int(__import__("os").environ.get("BBIA_CAMERA_INDEX", "0"))
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(f"❌ Impossible d'ouvrir la caméra (index {camera_index})")
        return 1

    print("📹 Webcam ouverte. Appuie sur 'q' pour quitter")
    print("💡 Affiche-toi devant la caméra pour tester la détection de posture")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Détecter la pose
        pose_result = pose_detector.detect_pose(frame)

        if pose_result:
            # Afficher les résultats
            posture = pose_result["posture"]
            gestures = pose_result["gestures"]
            landmarks_count = pose_result["num_landmarks"]

            # Afficher sur l'image
            cv2.putText(
                frame,
                f"Posture: {posture}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

            gesture_text = []
            if gestures.get("bras_levés"):
                gesture_text.append("Bras levés")
            if gestures.get("bras_gauche_levé"):
                gesture_text.append("Bras gauche levé")
            if gestures.get("bras_droit_levé"):
                gesture_text.append("Bras droit levé")
            if gestures.get("mains_sur_tete"):
                gesture_text.append("Mains sur tête")

            if gesture_text:
                cv2.putText(
                    frame,
                    ", ".join(gesture_text),
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 0),
                    2,
                )

            cv2.putText(
                frame,
                f"Landmarks: {landmarks_count}/33",
                (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
        else:
            cv2.putText(
                frame,
                "Aucune posture detectee",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )

        cv2.imshow("BBIA Pose Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


def test_image(pose_detector, image_path: str):
    """Test avec une image."""
    try:
        import cv2
    except ImportError:
        print("❌ OpenCV non disponible")
        return 1

    if not Path(image_path).exists():
        print(f"❌ Image introuvable: {image_path}")
        return 1

    # Charger l'image
    image = cv2.imread(image_path)
    if image is None:
        print(f"❌ Impossible de charger l'image: {image_path}")
        return 1

    print(f"🔍 Analyse de l'image: {image_path}")

    # Détecter la pose
    pose_result = pose_detector.detect_pose(image)

    if pose_result:
        print("\n✅ Posture détectée:")
        print(f"   • Posture: {pose_result['posture']}")
        print(f"   • Landmarks: {pose_result['num_landmarks']}/33")
        print(f"\n   Gestes détectés:")
        for gesture, detected in pose_result["gestures"].items():
            status = "✅" if detected else "❌"
            print(f"      {status} {gesture}")

        # Sauvegarder l'image avec résultats (optionnel)
        # cv2.imwrite("output_pose.jpg", image)
    else:
        print("❌ Aucune posture détectée dans l'image")
        print("💡 Assure-toi que l'image contient une personne visible")

    return 0


def main():
    parser = argparse.ArgumentParser(description="Test MediaPipe Pose Detection BBIA")
    parser.add_argument(
        "--webcam",
        action="store_true",
        help="Test avec webcam en temps réel",
    )
    parser.add_argument(
        "--image",
        type=str,
        help="Test avec une image (chemin vers fichier)",
    )
    parser.add_argument(
        "--complexity",
        type=int,
        default=1,
        choices=[0, 1, 2],
        help="Complexité du modèle (0=rapide, 1=équilibré, 2=précis)",
    )

    args = parser.parse_args()

    # Créer le détecteur
    pose_detector = create_pose_detector(model_complexity=args.complexity)
    if not pose_detector or not pose_detector.is_initialized:
        print("❌ Impossible de créer le détecteur MediaPipe Pose")
        return 1

    print(f"✅ Détecteur Pose créé (complexité: {args.complexity})")

    # Test webcam
    if args.webcam:
        return test_webcam(pose_detector, complexity=args.complexity)

    # Test image
    if args.image:
        return test_image(pose_detector, args.image)

    # Aucune action, afficher aide
    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
