#!/usr/bin/env python3
"""
Test simple webcam USB - Preview en temps réel

Usage:
    source venv-vision-py310/bin/activate
    export BBIA_CAMERA_INDEX=0  # optionnel, défaut: 0
    python scripts/test_webcam_simple.py
"""

import os
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


def main():
    # Lire index depuis variable d'environnement
    camera_index_str = os.environ.get("BBIA_CAMERA_INDEX", "0")
    camera_device = os.environ.get("BBIA_CAMERA_DEVICE")

    try:
        if camera_device:
            camera_index = camera_device
            print(f"🔌 Ouverture webcam: {camera_device}")
        else:
            camera_index = int(camera_index_str)
            print(f"🔌 Ouverture webcam (index {camera_index})")
    except ValueError:
        print(f"⚠️ BBIA_CAMERA_INDEX invalide: {camera_index_str}, utilisation index 0")
        camera_index = 0

    # Ouvrir la webcam
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(f"❌ Impossible d'ouvrir la webcam (index {camera_index})")
        print("💡 Vérifie:")
        print("   - La webcam est branchée")
        print(
            "   - Les permissions macOS (Réglages Système > Confidentialité > Caméra)"
        )
        print("   - L'index est correct (essaie 0, 1, 2...)")
        return 1

    # Configurer résolution (640x480 pour performance)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("✅ Webcam ouverte !")
    print("   • Appuie sur 'q' pour quitter")
    print("   • Appuie sur 's' pour sauvegarder une capture")

    frame_count = 0

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("⚠️ Impossible de lire la frame")
                break

            frame_count += 1

            # Ajouter info sur la frame
            info_text = (
                f"Frame {frame_count} | Resolution: {frame.shape[1]}x{frame.shape[0]}"
            )
            cv2.putText(
                frame,
                info_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

            # Afficher la frame
            cv2.imshow("BBIA Webcam Test - Logitech MX Brio", frame)

            # Gestion touches
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("\n👋 Arrêt demandé")
                break
            elif key == ord("s"):
                from datetime import datetime

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"webcam_capture_{timestamp}_frame{frame_count}.jpg"
                success = cv2.imwrite(filename, frame)
                if success:
                    abs_path = os.path.abspath(filename)
                    print(f"\n📸 ✅ Capture sauvegardée: {filename}")
                    print(f"   📁 Chemin complet: {abs_path}")
                else:
                    print(f"\n❌ Erreur: Impossible de sauvegarder {filename}")

    except KeyboardInterrupt:
        print("\n👋 Interruption utilisateur")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("✅ Webcam libérée")

    return 0


if __name__ == "__main__":
    sys.exit(main())
