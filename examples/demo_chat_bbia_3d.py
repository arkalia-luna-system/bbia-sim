#!/usr/bin/env python3
"""
💬 DÉMO CHAT BBIA EN 3D - Visualisation MuJoCo avec Chat Interactif
Combine chat intelligent BBIA + robot 3D MuJoCo

⚠️ IMPORTANT SUR macOS:
   Pour voir le viewer 3D, lancez avec:
   mjpython examples/demo_chat_bbia_3d.py

   Sinon, la démo fonctionne mais sans viewer (limitation MuJoCo sur macOS)
"""

import math
import sys
import time
from pathlib import Path

import mujoco
from mujoco import viewer as mujoco_viewer

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

try:
    from bbia_sim.bbia_huggingface import BBIAHuggingFace

    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False
    print("⚠️ Hugging Face non disponible, utilisation du mode fallback")


class MockHuggingFace:
    """Mock pour démontrer le chat sans HF installé."""

    def __init__(self):
        self.bbia_personality = "friendly_robot"
        self.conversation_history = []
        self.context = {}

    def chat(self, user_message: str) -> str:
        """Chat simple sans HF."""
        message_lower = user_message.lower()

        # Salutations
        if any(word in message_lower for word in ["bonjour", "salut", "hello"]):
            response = (
                "🤖 Bonjour ! Comment allez-vous ? Je suis BBIA, votre robot compagnon."
            )
        # Au revoir
        elif any(word in message_lower for word in ["au revoir", "bye", "à bientôt"]):
            response = "🤖 Au revoir ! Ce fut un plaisir. À bientôt !"
        # Positif
        elif any(
            word in message_lower for word in ["content", "heureux", "cool", "super"]
        ):
            response = "🤖 C'est super ! Je suis content pour vous. Continuez !"
        # Question
        elif message_lower.count("?") > 0:
            response = "🤖 C'est une excellente question ! Je réfléchis..."
        # Par défaut
        else:
            response = f"🤖 Je comprends: {user_message}. C'est intéressant, dites-moi en plus !"

        # Sauvegarder
        from datetime import datetime

        self.conversation_history.append(
            {
                "user": user_message,
                "bbia": response,
                "sentiment": {"sentiment": "NEUTRAL", "score": 0.5},
                "timestamp": datetime.now().isoformat(),
            }
        )

        return response


def get_joint_id(model, joint_name):
    """Obtenir l'ID d'un joint."""
    try:
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    except Exception:
        return None


def animate_with_viewer(
    model, data, viewer, message: str, response: str, duration: float = 2.0
):
    """Animer le robot avec viewer pendant duration secondes - NOUVEAU.

    ⚠️ IMPORTANT EXPERT: Cette fonction utilise data.qpos[stewart_*] directement
    uniquement pour la simulation MuJoCo de bas niveau (visualisation viewer).
    Pour le robot physique ou avec le SDK, utiliser goto_target() ou
    set_target_head_pose() avec create_head_pose() (cinématique inverse requise).

    NOTE: L'utilisation de stewart_1/stewart_2 ici est une approximation simplifiée
    pour MuJoCo direct (stewart_1 ≈ pitch, stewart_2 ≈ yaw). Le robot réel nécessite IK.
    """
    import math

    message_lower = message.lower()
    response_lower = response.lower()

    # Détecter type de mouvement
    is_greeting = any(word in message_lower for word in ["bonjour", "salut", "hello"])
    is_positive = any(
        word in response_lower for word in ["super", "content", "excité", "heureux"]
    )
    is_question = "?" in message_lower

    frames = int(duration * 60)

    for frame in range(frames):
        progress = frame / frames if frames > 0 else 0

        # Réinitialiser positions
        for joint_id in range(model.njnt):
            data.qpos[joint_id] = 0

        if is_greeting:
            # SALUTATION : Basé sur SDK happy (pitch=0.1 rad max)
            # NOTE: Utilisation stewart_1 uniquement pour MuJoCo direct (viewer)
            stewart_1_id = get_joint_id(model, "stewart_1")
            if stewart_1_id is not None:
                # Hochement safe conforme SDK (pitch 0.1 max)
                pos = 0.08 * math.sin(progress * math.pi * 2)
                data.qpos[stewart_1_id] = pos

        elif is_positive:
            # POSITIF : Basé sur SDK excited (pitch=0.2, yaw=0.1)
            # Corps rotation joyeuse
            body_id = get_joint_id(model, "yaw_body")
            if body_id is not None:
                rotation = 0.12 * math.sin(progress * math.pi * 2)
                data.qpos[body_id] = rotation

            # Tête excited (pitch 0.12 max - sous limite 0.2 SDK)
            # NOTE: Utilisation stewart_1 uniquement pour MuJoCo direct (viewer)
            stewart_1_id = get_joint_id(model, "stewart_1")
            if stewart_1_id is not None:
                pos = 0.12 * math.sin(progress * math.pi * 2)
                data.qpos[stewart_1_id] = pos

        elif is_question:
            # CURIEUX : Basé sur SDK curious (pitch=0.05, yaw=0.2)
            # NOTE: Utilisation stewart_2 uniquement pour MuJoCo direct (viewer)
            stewart_2_id = get_joint_id(model, "stewart_2")
            if stewart_2_id is not None:
                pos = 0.06 * math.sin(progress * math.pi)
                data.qpos[stewart_2_id] = pos

        # CRITIQUE: Simulation avec mise à jour physique
        mujoco.mj_forward(model, data)
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(1 / 60)


def animate_robot_from_chat(model, data, message: str, response: str):
    """Animer le robot selon le message et réponse de BBIA avec mouvements réalistes.

    ⚠️ IMPORTANT EXPERT: Cette fonction utilise data.qpos[stewart_*] directement
    uniquement pour la simulation MuJoCo de bas niveau (visualisation viewer).
    Pour le robot physique ou avec le SDK, utiliser goto_target() ou
    set_target_head_pose() avec create_head_pose() (cinématique inverse requise).

    NOTE: L'utilisation de stewart_1/stewart_2 ici est une approximation simplifiée
    pour MuJoCo direct (stewart_1 ≈ pitch, stewart_2 ≈ yaw). Le robot réel nécessite IK.
    """

    message_lower = message.lower()
    response_lower = response.lower()

    # Utiliser SEULEMENT les joints mobiles : stewart_1-6 et yaw_body
    # Les antennes sont bloquées (range = 0.000) dans le modèle officiel
    # NOTE: Utilisation stewart_* uniquement pour MuJoCo direct (viewer)

    # Salutations -> Hochement conforme SDK happy (pitch=0.1 max)
    if any(word in message_lower for word in ["bonjour", "salut", "hello"]):
        stewart_1_id = get_joint_id(model, "stewart_1")
        if stewart_1_id is not None:
            data.qpos[stewart_1_id] = 0.08  # Pitch conforme SDK happy (MuJoCo direct)

    # Positif/Joyeux -> excited (pitch=0.2, yaw=0.1 max)
    if any(word in response_lower for word in ["super", "content", "excité"]):
        body_id = get_joint_id(model, "yaw_body")
        if body_id is not None:
            data.qpos[body_id] = 0.12  # Yaw conforme SDK excited

        stewart_1_id = get_joint_id(model, "stewart_1")
        if stewart_1_id is not None:
            data.qpos[stewart_1_id] = 0.12  # Pitch conforme SDK excited (MuJoCo direct)

    # Question/Curieux -> curious (pitch=0.05, yaw=0.2 max)
    if any(word in message_lower for word in ["quoi", "comment", "pourquoi", "?"]):
        stewart_2_id = get_joint_id(model, "stewart_2")
        if stewart_2_id is not None:
            data.qpos[stewart_2_id] = 0.08  # Yaw conforme SDK curious (MuJoCo direct)

        body_id = get_joint_id(model, "yaw_body")
        if body_id is not None:
            data.qpos[body_id] = 0.1  # Légère rotation

    # Triste/Emphatique -> sad (pitch=-0.1)
    if any(
        word in response_lower for word in ["triste", "comprends", "pardon", "désolé"]
    ):
        stewart_1_id = get_joint_id(model, "stewart_1")
        if stewart_1_id is not None:
            data.qpos[stewart_1_id] = -0.08  # Pitch conforme SDK sad (MuJoCo direct)

        body_id = get_joint_id(model, "yaw_body")
        if body_id is not None:
            data.qpos[body_id] = -0.08  # Rotation légère


def demo_chat_bbia_3d():
    """Démo chat BBIA en 3D avec MuJoCo."""
    print("💬🤖 DÉMO CHAT BBIA EN 3D")
    print("=" * 60)
    print("Combine chat intelligent + visualisation robot 3D")
    print("=" * 60)

    # Initialiser BBIA
    if HF_AVAILABLE:
        try:
            bbia = BBIAHuggingFace()
            print(f"\n🤖 BBIA initialisé avec personnalité: {bbia.bbia_personality}")
        except Exception:
            bbia = MockHuggingFace()
            print("\n🤖 BBIA (mode fallback) initialisé")
    else:
        bbia = MockHuggingFace()
        print("\n🤖 BBIA (mode fallback) initialisé")

    # Charger le modèle MuJoCo
    model_path = Path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
    if not model_path.exists():
        print(f"❌ Modèle MuJoCo non trouvé: {model_path}")
        return

    print("\n🔧 Chargement modèle MuJoCo...")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    print(f"✅ Modèle chargé: {model.nq} joints")

    # Messages de test
    messages = [
        "Bonjour BBIA",
        "Comment allez-vous ?",
        "Je suis très content de te rencontrer",
        "C'est un super projet !",
    ]

    print("\n🎬 LANCEMENT DE LA DÉMO 3D...")
    print("=" * 60)
    print("Le viewer MuJoCo va s'ouvrir")
    print("Le robot va bouger selon les conversations")
    print("❌ Fermez le viewer pour arrêter")
    print("=" * 60)

    # Lancer le viewer MuJoCo
    # Note: Sur macOS, utilisez mjpython pour voir le viewer 3D
    print("\n✅ Ouverture du viewer MuJoCo...")
    print("💬 Conversation avec BBIA en cours...\n")

    with mujoco_viewer.launch_passive(model, data) as viewer:
        print("\n✅ Viewer MuJoCo ouvert !")

        for msg in messages:
            # Chat
            print(f"Vous: {msg}")
            response = bbia.chat(msg)
            print(f"BBIA: {response}\n")

            # Animer le robot avec viewer pendant 2 secondes
            animate_with_viewer(model, data, viewer, msg, response, duration=2.0)

            # Pause entre messages
            print()

        # Animation finale joyeuse (conforme SDK)
        print("\n🎊 Animation finale joyeuse...")
        for i in range(240):  # 4 secondes à 60 FPS
            progress = i / 240

            # Réinitialiser positions
            for joint_id in range(model.njnt):
                data.qpos[joint_id] = 0

            # Rotation corps joyeuse conforme SDK (happy excited)
            body_id = get_joint_id(model, "yaw_body")
            if body_id is not None:
                # Rotation safe selon SDK (excited yaw=0.1 max)
                pos = 0.12 * math.sin(progress * math.pi * 2)
                data.qpos[body_id] = pos

            # Mouvement tête expressif conforme SDK (happy pitch=0.1)
            # NOTE: Utilisation stewart_1 uniquement pour MuJoCo direct (viewer)
            stewart_1_id = get_joint_id(model, "stewart_1")
            if stewart_1_id is not None:
                # Hochements conformes SDK create_head_pose(happy pitch=0.1)
                # Approximation MuJoCo direct (robot réel nécessite IK)
                pos = 0.08 * math.sin(progress * math.pi * 4)
                data.qpos[stewart_1_id] = pos

            # CRITIQUE: Mettre à jour la physique ET avancer simulation
            mujoco.mj_forward(model, data)
            mujoco.mj_step(model, data)
            viewer.sync()  # Synchroniser le viewer
            time.sleep(1 / 60)

        print("\n✅ Démo terminée !")
        print(f"📊 Messages échangés: {len(bbia.conversation_history)}")
        print("💬 Merci d'avoir chaté avec BBIA !")


if __name__ == "__main__":
    try:
        demo_chat_bbia_3d()
    except KeyboardInterrupt:
        print("\n👋 Démo interrompue")
    except Exception as e:
        print(f"\n❌ Erreur: {e}")
        import traceback

        traceback.print_exc()
