#!/usr/bin/env python3
"""
Test Vision BBIA avec webcam USB - Détection objets et visages en temps réel

Usage:
    source venv-vision-py310/bin/activate
    export BBIA_CAMERA_INDEX=0  # optionnel
    python scripts/test_vision_webcam.py
"""

import sys
from pathlib import Path

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

try:
    import cv2
except ImportError as e:
    print(f"❌ Import error: {e}")
    print("💡 Active le venv-vision-py310: source venv-vision-py310/bin/activate")
    sys.exit(1)

from bbia_sim.bbia_vision import BBIAVision


def draw_detections(frame, objects, faces):
    """Dessine les détections sur la frame."""
    # Détection objets (YOLO)
    for obj in objects:
        bbox = obj.get("bbox", {})
        if isinstance(bbox, dict):
            x = bbox.get("x", 0)
            y = bbox.get("y", 0)
            w = bbox.get("width", 0)
            h = bbox.get("height", 0)
        elif isinstance(bbox, list) and len(bbox) == 4:
            # Format YOLO [x1, y1, x2, y2]
            x1, y1, x2, y2 = bbox
            x, y, w, h = x1, y1, x2 - x1, y2 - y1
        else:
            continue

        name = obj.get("name", "objet")
        conf = obj.get("confidence", 0.0)

        # Filtrer détections avec faible confiance
        if conf < 0.3:
            continue

        # Rectangle (vert pour objets)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Label avec fond pour meilleure lisibilité
        label = f"{name} {conf:.2f}"
        (text_width, text_height), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
        )
        cv2.rectangle(
            frame,
            (x, y - text_height - 10),
            (x + text_width, y),
            (0, 255, 0),
            -1,
        )
        cv2.putText(
            frame,
            label,
            (x, y - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            2,
        )

    # Détection visages (MediaPipe)
    for face in faces:
        bbox = face.get("bbox", {})
        if isinstance(bbox, dict):
            x = bbox.get("x", 0)
            y = bbox.get("y", 0)
            w = bbox.get("width", 0)
            h = bbox.get("height", 0)
        else:
            continue

        conf = face.get("confidence", 0.0)
        emotion = face.get("emotion", "neutral")

        # Rectangle
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Label
        label = f"Face {conf:.2f} ({emotion})"
        cv2.putText(
            frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2
        )


def main():
    print("🔍 Initialisation BBIA Vision...")

    # Initialiser vision (pas de robot_api, donc utilise webcam OpenCV)
    vision = BBIAVision(robot_api=None)

    # Vérifier disponibilité webcam
    if not vision._opencv_camera_available:
        print("❌ Webcam OpenCV non disponible")
        print("💡 Vérifie:")
        print("   - La webcam est branchée")
        print(
            "   - Les permissions macOS (Réglages Système > Confidentialité > Caméra)"
        )
        print("   - L'index est correct (BBIA_CAMERA_INDEX=0)")
        return 1

    print("✅ BBIA Vision initialisé")
    print(f"   • YOLO: {'✅' if vision.yolo_detector else '❌'}")
    print(f"   • MediaPipe: {'✅' if vision.face_detector else '❌'}")
    print("   • Webcam: ✅")

    print("\n🎥 Démarrage capture...")
    print("   • Appuie sur 'q' pour quitter")
    print("   • Appuie sur 's' pour sauvegarder une capture")

    frame_count = 0
    detection_count = 0

    try:
        while True:
            # Capture image
            image = vision._capture_image_from_camera()

            if image is None:
                print("⚠️ Impossible de capturer une image")
                break

            frame_count += 1

            # Détection (toutes les 3 frames pour meilleure réactivité)
            if frame_count % 3 == 0:
                scan_result = vision.scan_environment()
                objects = scan_result.get("objects", [])
                faces = scan_result.get("faces", [])

                if objects or faces:
                    detection_count += 1
                    print(
                        f"🔍 Frame {frame_count}: {len(objects)} objets, {len(faces)} visages"
                    )

                # Dessiner détections
                draw_detections(image, objects, faces)

            # Info frame
            info_text = (
                f"Frame {frame_count} | "
                f"Objects: {len(vision.objects_detected)} | "
                f"Faces: {len(vision.faces_detected)}"
            )
            cv2.putText(
                image,
                info_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # Afficher
            cv2.imshow("BBIA Vision Test - Webcam USB", image)

            # Gestion touches
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("\n👋 Arrêt demandé")
                break
            elif key == ord("s"):
                from datetime import datetime

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"vision_capture_{timestamp}_frame{frame_count}.jpg"
                success = cv2.imwrite(filename, image)
                if success:
                    import os

                    abs_path = os.path.abspath(filename)
                    print(f"\n📸 ✅ Capture sauvegardée: {filename}")
                    print(f"   📁 Chemin complet: {abs_path}")
                else:
                    print(f"\n❌ Erreur: Impossible de sauvegarder {filename}")

    except KeyboardInterrupt:
        print("\n👋 Interruption utilisateur")
    finally:
        # Nettoyer
        if vision._opencv_camera:
            vision._opencv_camera.release()
        cv2.destroyAllWindows()

        print("\n📊 Statistiques:")
        print(f"   • Frames capturées: {frame_count}")
        print(f"   • Détections: {detection_count}")
        print("✅ Test terminé")

    return 0


if __name__ == "__main__":
    sys.exit(main())
