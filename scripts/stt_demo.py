#!/usr/bin/env python3
"""
stt_demo.py - Démo Speech-to-Text avec Whisper
Script CLI pour tester les commandes vocales → actions RobotAPI
"""

import argparse
import logging
import sys
import time
from pathlib import Path

# Ajouter le chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_api import RobotFactory
from bbia_sim.voice_whisper import WHISPER_AVAILABLE, VoiceCommandMapper, WhisperSTT

logger = logging.getLogger(__name__)


def test_voice_command(text: str, backend: str = "mujoco") -> bool:
    """
    Teste une commande vocale avec le robot.

    Args:
        text: Commande vocale à tester
        backend: Backend robot ("mujoco" ou "reachy")

    Returns:
        True si succès, False sinon
    """
    try:
        # Initialisation
        mapper = VoiceCommandMapper()
        robot = RobotFactory.create_backend(backend)

        # Connexion robot
        robot.connect()

        # Mapping commande
        action_data = mapper.map_command(text)
        if not action_data:
            logger.error(f"❌ Commande non reconnue: '{text}'")
            return False

        action = action_data["action"]
        confidence = action_data["confidence"]

        logger.info(f"🎯 Exécution action: {action} (confiance: {confidence})")

        # Exécution action
        if action == "greet":
            robot.set_emotion("happy", intensity=0.8)
            time.sleep(2)
            robot.set_emotion("neutral", intensity=0.5)

        elif action == "look_at":
            # Mouvement de tête simple
            robot.set_joint_pos("yaw_body", 0.2)
            time.sleep(1)
            robot.set_joint_pos("yaw_body", -0.2)
            time.sleep(1)
            robot.set_joint_pos("yaw_body", 0.0)

        elif action == "happy":
            robot.set_emotion("happy", intensity=0.9)
            time.sleep(3)

        elif action == "sad":
            robot.set_emotion("sad", intensity=0.8)
            time.sleep(3)

        elif action == "excited":
            robot.set_emotion("excited", intensity=0.9)
            time.sleep(3)

        elif action == "wake_up":
            robot.set_emotion("excited", intensity=0.7)
            time.sleep(1)
            robot.set_joint_pos("yaw_body", 0.1)
            time.sleep(0.5)
            robot.set_joint_pos("yaw_body", -0.1)
            time.sleep(0.5)
            robot.set_joint_pos("yaw_body", 0.0)
            robot.set_emotion("happy", intensity=0.8)
            time.sleep(2)

        # Déconnexion
        robot.disconnect()

        logger.info(f"✅ Action '{action}' exécutée avec succès")
        return True

    except Exception as e:
        logger.error(f"❌ Erreur exécution action: {e}")
        return False


def test_whisper_transcription(audio_file: str) -> bool:
    """
    Teste la transcription Whisper sur un fichier audio.

    Args:
        audio_file: Chemin vers le fichier audio

    Returns:
        True si succès, False sinon
    """
    if not WHISPER_AVAILABLE:
        logger.error("❌ Whisper non disponible")
        return False

    try:
        # Initialisation Whisper
        stt = WhisperSTT(model_size="tiny", language="fr")

        # Test transcription
        start_time = time.time()
        text = stt.transcribe_audio(audio_file)
        transcription_time = time.time() - start_time

        if text:
            logger.info(
                f"✅ Transcription réussie en {transcription_time:.1f}s: '{text}'"
            )

            # Test mapping
            mapper = VoiceCommandMapper()
            action_data = mapper.map_command(text)

            if action_data:
                logger.info(f"🎯 Action mappée: {action_data}")
                return True
            else:
                logger.warning(f"⚠️ Commande non reconnue: '{text}'")
                return False
        else:
            logger.error("❌ Transcription échouée")
            return False

    except Exception as e:
        logger.error(f"❌ Erreur test Whisper: {e}")
        return False


def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(description="Démo Speech-to-Text BBIA")
    parser.add_argument(
        "--backend",
        choices=["mujoco", "reachy"],
        default="mujoco",
        help="Backend robot à utiliser",
    )
    parser.add_argument(
        "--lang", choices=["fr", "en"], default="fr", help="Langue des commandes"
    )
    parser.add_argument(
        "--command",
        type=str,
        help="Commande vocale à tester (ex: 'salue', 'regarde-moi')",
    )
    parser.add_argument("--audio-file", type=str, help="Fichier audio à transcrire")
    parser.add_argument(
        "--test-microphone", action="store_true", help="Test avec microphone (3s)"
    )
    parser.add_argument(
        "--list-commands",
        action="store_true",
        help="Lister toutes les commandes disponibles",
    )

    args = parser.parse_args()

    # Configuration logging
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )

    print("🎤 Démo Speech-to-Text BBIA")
    print("=" * 40)

    # Test disponibilité Whisper
    print(f"Whisper disponible: {WHISPER_AVAILABLE}")

    if args.list_commands:
        print("\n📋 Commandes disponibles:")
        mapper = VoiceCommandMapper()
        for cmd, action in mapper.commands.items():
            print(f"  '{cmd}' → {action}")
        return

    if args.test_microphone:
        if not WHISPER_AVAILABLE:
            print("❌ Whisper requis pour le test microphone")
            return

        print("🎤 Test microphone (3s)...")
        stt = WhisperSTT(model_size="tiny", language=args.lang)
        text = stt.transcribe_microphone(duration=3.0)

        if text:
            print(f"✅ Transcription: '{text}'")
            # Test action
            success = test_voice_command(text, args.backend)
            print(f"🎯 Action exécutée: {'✅' if success else '❌'}")
        else:
            print("❌ Transcription échouée")

    elif args.audio_file:
        print(f"🎵 Test fichier audio: {args.audio_file}")
        success = test_whisper_transcription(args.audio_file)
        print(f"🎯 Test terminé: {'✅' if success else '❌'}")

    elif args.command:
        print(f"🗣️ Test commande: '{args.command}'")
        success = test_voice_command(args.command, args.backend)
        print(f"🎯 Action exécutée: {'✅' if success else '❌'}")

    else:
        print("❓ Aucune action spécifiée. Utilisez --help pour voir les options.")
        print("\nExemples:")
        print("  python scripts/stt_demo.py --command 'salue' --backend mujoco")
        print("  python scripts/stt_demo.py --test-microphone --lang fr")
        print("  python scripts/stt_demo.py --list-commands")


if __name__ == "__main__":
    main()
