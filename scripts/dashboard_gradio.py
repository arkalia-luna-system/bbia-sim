#!/usr/bin/env python3
"""
Dashboard Gradio pour BBIA - Interface no-code simple
Upload images, chat, test DeepFace, détection objets
"""

import logging
import sys
from pathlib import Path

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

try:
    import gradio as gr

    GRADIO_AVAILABLE = True
except ImportError:
    GRADIO_AVAILABLE = False
    print("❌ Gradio non disponible")
    print("💡 Installer avec: pip install gradio")
    sys.exit(1)

from bbia_sim.bbia_huggingface import BBIAHuggingFace
from bbia_sim.bbia_vision import BBIAVision

logger = logging.getLogger(__name__)

# Initialiser modules BBIA
try:
    vision = BBIAVision()
    hf = BBIAHuggingFace()
except Exception as e:
    logger.error(f"Erreur initialisation BBIA: {e}")
    vision = None
    hf = None


def scan_image(image):
    """Scanne une image uploadée."""
    if vision is None:
        return "❌ Module vision non disponible"

    try:
        import cv2
        import numpy as np

        # Convertir image Gradio (PIL/numpy) en format OpenCV
        if isinstance(image, np.ndarray):
            if len(image.shape) == 3:
                # RGB -> BGR pour OpenCV
                image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            else:
                image_bgr = image
        else:
            # PIL Image
            import numpy as np

            image_np = np.array(image)
            if len(image_np.shape) == 3:
                image_bgr = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
            else:
                image_bgr = image_np

        # Scanner l'environnement depuis l'image uploadée
        result = vision.scan_environment_from_image(image_bgr)

        # Formater résultat
        output = []
        output.append("🔍 **Résultats de scan**\n")

        if result.get("objects"):
            output.append(f"### 📦 Objets détectés: {len(result['objects'])}")
            for obj in result["objects"][:5]:  # Limiter à 5
                output.append(
                    f"- {obj.get('name', 'objet')} "
                    f"(confiance: {obj.get('confidence', 0):.2f})"
                )

        if result.get("faces"):
            output.append(f"\n### 👤 Visages détectés: {len(result['faces'])}")
            for face in result["faces"][:5]:  # Limiter à 5
                name = face.get("name", "humain")
                emotion = face.get("emotion", "neutral")
                output.append(
                    f"- {name} "
                    f"(émotion: {emotion}, "
                    f"confiance: {face.get('emotion_confidence', 0):.2f})"
                )

        if result.get("poses"):
            output.append(f"\n### 🧍 Postures détectées: {len(result['poses'])}")
            for pose in result["poses"][:5]:  # Limiter à 5
                posture = pose.get("posture", "inconnu")
                gestures = pose.get("gestures", {})
                gesture_list = [k for k, v in gestures.items() if v]
                output.append(
                    f"- {posture} "
                    f"(gestes: {', '.join(gesture_list) if gesture_list else 'aucun'})"
                )

        if (
            not result.get("objects")
            and not result.get("faces")
            and not result.get("poses")
        ):
            output.append("❌ Aucune détection dans l'image")

        return "\n".join(output)

    except Exception as e:
        logger.error(f"Erreur scan image: {e}")
        return f"❌ Erreur: {e}"


def chat_with_bbia(message, history):
    """Chat avec BBIA."""
    if hf is None:
        return history + [[message, "❌ Module chat non disponible"]], ""

    try:
        response = hf.chat(message)
        history.append([message, response])
        return history, ""
    except Exception as e:
        logger.error(f"Erreur chat: {e}")
        return history + [[message, f"❌ Erreur: {e}"]], ""


def register_face(image, name):
    """Enregistre une personne dans DeepFace."""
    if vision is None or vision.face_recognition is None:
        return "❌ DeepFace non disponible. Installer avec: pip install deepface"

    try:
        import tempfile

        import cv2
        import numpy as np

        # Convertir image Gradio
        if isinstance(image, np.ndarray):
            if len(image.shape) == 3:
                image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            else:
                image_bgr = image
        else:
            image_np = np.array(image)
            if len(image_np.shape) == 3:
                image_bgr = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
            else:
                image_bgr = image_np

        # Sauvegarder temporairement
        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
            tmp_path = tmp.name
            cv2.imwrite(tmp_path, image_bgr)

        # Enregistrer personne
        success = vision.face_recognition.register_person(tmp_path, name)

        # Supprimer fichier temp
        Path(tmp_path).unlink()

        if success:
            return f"✅ Personne '{name}' enregistrée avec succès !"
        else:
            return f"❌ Erreur lors de l'enregistrement de '{name}'"

    except Exception as e:
        logger.error(f"Erreur enregistrement visage: {e}")
        return f"❌ Erreur: {e}"


# Interface Gradio
with gr.Blocks(title="BBIA Dashboard", theme=gr.themes.Soft()) as demo:
    gr.Markdown(
        """
    # 🤖 Dashboard BBIA - Interface No-Code

    Interface simple pour tester les capacités de BBIA :
    - 📷 Upload images → détection objets/visages/postures
    - 💬 Chat avec BBIA
    - 👤 Enregistrer personnes (DeepFace)
    """
    )

    with gr.Tabs():
        with gr.Tab("📷 Vision"):
            gr.Markdown("### Upload une image pour détection")
            image_input = gr.Image(label="Image à scanner", type="numpy")
            scan_button = gr.Button("🔍 Scanner", variant="primary")
            scan_output = gr.Markdown(label="Résultats")

            scan_button.click(fn=scan_image, inputs=image_input, outputs=scan_output)

        with gr.Tab("💬 Chat"):
            gr.Markdown("### Chat avec BBIA")
            chatbot = gr.Chatbot(label="Conversation", height=400)
            msg = gr.Textbox(
                label="Message", placeholder="Tapez votre message...", lines=2
            )
            chat_button = gr.Button("Envoyer", variant="primary")

            def chat_wrapper(message, history):
                if not message.strip():
                    return "", history
                if hf is None:
                    return "", history + [[message, "❌ Module chat non disponible"]]
                try:
                    response = hf.chat(message)
                    return "", history + [[message, response]]
                except Exception as e:
                    logger.error(f"Erreur chat: {e}")
                    return "", history + [[message, f"❌ Erreur: {e}"]]

            chat_button.click(
                fn=chat_wrapper,
                inputs=[msg, chatbot],
                outputs=[msg, chatbot],
                show_progress=True,
            )
            msg.submit(
                fn=chat_wrapper,
                inputs=[msg, chatbot],
                outputs=[msg, chatbot],
                show_progress=True,
            )

        with gr.Tab("👤 DeepFace"):
            gr.Markdown("### Enregistrer une personne")
            gr.Markdown(
                "Upload une photo et donnez un nom pour que BBIA reconnaisse cette personne."
            )
            face_image = gr.Image(label="Photo de la personne", type="numpy")
            face_name = gr.Textbox(label="Nom", placeholder="Alice")
            register_button = gr.Button("✅ Enregistrer", variant="primary")
            register_output = gr.Markdown(label="Résultat")

            register_button.click(
                fn=register_face,
                inputs=[face_image, face_name],
                outputs=register_output,
            )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Dashboard Gradio BBIA")
    parser.add_argument("--host", default="127.0.0.1", help="Adresse d'écoute")
    parser.add_argument("--port", type=int, default=7860, help="Port d'écoute")
    parser.add_argument("--share", action="store_true", help="Créer lien public")

    args = parser.parse_args()

    print("🚀 Démarrage Dashboard Gradio BBIA...")
    print(f"🌐 URL: http://{args.host}:{args.port}")
    if args.share:
        print("🔗 Lien public sera généré automatiquement")

    demo.launch(server_name=args.host, server_port=args.port, share=args.share)
