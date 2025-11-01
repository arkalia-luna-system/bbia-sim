#!/usr/bin/env python3
"""Demo look_at_image pour Reachy Mini - BBIA-SIM.

Quand vous cliquez sur l'image, Reachy Mini regarde vers ce point.
Utilise la vision BBIA (YOLO) ou OpenCV pour capturer la vidéo.

Adapté du repo officiel pour BBIA-SIM.

Usage:
    python examples/reachy_mini/look_at_image.py
    python examples/reachy_mini/look_at_image.py --backend cv2
"""

import argparse
import time

import cv2

# Essayer d'utiliser le SDK officiel si disponible
try:
    from reachy_mini import ReachyMini

    USE_SDK = True
except ImportError:
    USE_SDK = False
    print("⚠️  SDK officiel non disponible, utilisation du backend BBIA")

if not USE_SDK:
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


def click(event, x, y, flags, param):
    """Gérer les clics souris pour obtenir les coordonnées."""
    if event == cv2.EVENT_LBUTTONDOWN:
        param["just_clicked"] = True
        param["x"] = x
        param["y"] = y


def get_frame_from_backend(backend, vision_source="cv2"):
    """Récupérer une frame depuis le backend (BBIA vision ou caméra SDK)."""
    if USE_SDK:
        # Utiliser caméra SDK si disponible
        if hasattr(backend, "media") and backend.media:
            frame = backend.media.get_frame()
            return frame
        return None
    else:
        # Utiliser vision BBIA ou OpenCV
        if vision_source == "cv2":
            # Utiliser webcam OpenCV directement
            cap = getattr(click, "_cap", None)
            if cap is None:
                cap = cv2.VideoCapture(0)
                click._cap = cap
            ret, frame = cap.read()
            if ret:
                return frame
        elif vision_source == "bbia":
            # Utiliser vision BBIA (YOLO)
            try:
                # from bbia_sim.bbia_vision import BBIAVision
                # vision = BBIAVision(robot_api=backend)
                # Obtenir frame depuis vision BBIA
                # Note: adaptation nécessaire selon API vision BBIA
                # TODO: Implémenter capture frame depuis BBIAVision
                return None  # À adapter
            except ImportError:
                pass
        return None


def main(vision_source: str = "cv2") -> None:
    """Afficher le feed caméra et faire regarder Reachy Mini vers les points cliqués."""
    state = {"x": 0, "y": 0, "just_clicked": False}

    cv2.namedWindow("Reachy Mini Camera - BBIA")
    cv2.setMouseCallback("Reachy Mini Camera - BBIA", click, param=state)

    print("🤖 Demo look_at_image - Reachy Mini")
    print("=" * 50)
    print("📸 Cliquez sur l'image pour faire regarder Reachy Mini vers ce point")
    print("⌨️  Appuyez sur 'q' pour quitter")

    if USE_SDK:
        with ReachyMini(
            media_backend="default" if vision_source != "cv2" else "no_media",
            use_sim=True,
        ) as reachy_mini:
            try:
                while True:
                    if vision_source != "cv2":
                        frame = (
                            reachy_mini.media.get_frame() if reachy_mini.media else None
                        )
                    else:
                        cap = cv2.VideoCapture(0)
                        ret, frame = cap.read()
                        if not ret:
                            print("⚠️  Échec capture frame")
                            time.sleep(0.1)
                            continue

                    if frame is None:
                        print("⚠️  Frame vide")
                        time.sleep(0.1)
                        continue

                    cv2.imshow("Reachy Mini Camera - BBIA", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        print("⏹️  Sortie...")
                        break

                    if state["just_clicked"]:
                        print(f"👁️  Regard vers ({state['x']}, {state['y']})")
                        reachy_mini.look_at_image(state["x"], state["y"], duration=0.3)
                        state["just_clicked"] = False

            except KeyboardInterrupt:
                print("\n⏹️  Interrompu")
            finally:
                cv2.destroyAllWindows()
                if vision_source == "cv2":
                    cap.release()

    else:
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        try:
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("❌ Impossible d'ouvrir la caméra")
                return

            print("✅ Caméra ouverte")
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("⚠️  Échec capture frame")
                    break

                cv2.imshow("Reachy Mini Camera - BBIA", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("⏹️  Sortie...")
                    break

                if state["just_clicked"]:
                    print(f"👁️  Regard vers ({state['x']}, {state['y']})")
                    backend.look_at_image(state["x"], state["y"], duration=0.3)
                    state["just_clicked"] = False

        except KeyboardInterrupt:
            print("\n⏹️  Interrompu")
        finally:
            cv2.destroyAllWindows()
            cap.release()
            backend.disconnect()

    print("✅ Demo terminée")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Afficher le feed caméra et faire regarder Reachy Mini vers les points cliqués."
    )
    parser.add_argument(
        "--vision",
        type=str,
        choices=["cv2", "bbia"],
        default="cv2",
        help="Source vidéo (cv2=webcam OpenCV, bbia=vision BBIA)",
    )

    args = parser.parse_args()
    main(vision_source=args.vision)
