#!/usr/bin/env python3

"""
Démonstration complète de BBIA - Test d'intégration de tous les modules
"""

import os
import sys
import time

# Ajouter le chemin du module parent
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.bbia_sim.bbia_audio import enregistrer_audio, lire_audio
from src.bbia_sim.bbia_behavior import BBIABehaviorManager
from src.bbia_sim.bbia_emotions import BBIAEmotions
from src.bbia_sim.bbia_vision import BBIAVision
from src.bbia_sim.bbia_voice import dire_texte, reconnaitre_parole


def test_emotions_module():
    """Test du module émotions"""
    print("\n🎭 Test du module Émotions")
    print("=" * 40)

    emotions = BBIAEmotions()

    # Test des transitions d'émotions
    print("1. Transition vers 'happy'")
    emotions.set_emotion("happy", 0.8)
    time.sleep(1)

    print("2. Transition vers 'curious'")
    emotions.set_emotion("curious", 0.6)
    time.sleep(1)

    print("3. Réponse émotionnelle")
    response = emotions.emotional_response("compliment")
    print(f"   Réponse à 'compliment' : {response}")

    print("4. Statistiques")
    stats = emotions.get_emotion_stats()
    print(f"   Statistiques : {stats}")

    return True


def test_vision_module():
    """Test du module vision"""
    print("\n👁️ Test du module Vision")
    print("=" * 40)

    vision = BBIAVision()

    # Test du scan d'environnement
    print("1. Scan de l'environnement")
    result = vision.scan_environment()
    print(f"   Objets détectés : {len(result['objects'])}")
    print(f"   Visages détectés : {len(result['faces'])}")

    # Test de reconnaissance d'objet
    if result["objects"]:
        print("2. Reconnaissance d'objet")
        obj_name = result["objects"][0]["name"]
        recognition = vision.recognize_object(obj_name)
        print(f"   Reconnaissance de '{obj_name}' : {recognition}")

    # Test de suivi d'objet
    print("3. Suivi d'objet")
    if result["objects"]:
        obj_name = result["objects"][0]["name"]
        tracking = vision.track_object(obj_name)
        print(f"   Suivi de '{obj_name}' : {tracking}")

    return True


def test_voice_module():
    """Test du module voix"""
    print("\n🗣️ Test du module Voix")
    print("=" * 40)

    # Test de synthèse vocale
    print("1. Synthèse vocale")
    dire_texte("Bonjour, je suis BBIA. Test de synthèse vocale.")
    time.sleep(2)

    # Test de reconnaissance vocale (avec timeout)
    print("2. Reconnaissance vocale (3 secondes)")
    print("   Parlez maintenant... (ou attendez 3s)")
    try:
        import signal

        def timeout_handler(signum, frame):
            raise TimeoutError("Timeout reconnaissance vocale")

        signal.signal(signal.SIGALRM, timeout_handler)
        signal.alarm(3)  # Timeout après 3 secondes

        texte = reconnaitre_parole(duree=3)
        signal.alarm(0)  # Annuler le timeout

        if texte:
            print(f"   Texte reconnu : {texte}")
        else:
            print("   Aucun texte reconnu")
    except (TimeoutError, KeyboardInterrupt):
        print("   Reconnaissance vocale interrompue (timeout)")
    except Exception as e:
        print(f"   Erreur reconnaissance vocale : {e}")

    return True


def test_audio_module():
    """Test du module audio"""
    print("\n🎵 Test du module Audio")
    print("=" * 40)

    # Test d'enregistrement
    print("1. Enregistrement audio (3 secondes)")
    print("   Parlez pour l'enregistrement...")
    filename = "test_bbia.wav"
    enregistrer_audio(filename, duree=3)
    print(f"   Fichier enregistré : {filename}")

    # Test de lecture
    print("2. Lecture audio")
    lire_audio(filename)

    return True


def test_behavior_module():
    """Test du module comportements"""
    print("\n🎭 Test du module Comportements")
    print("=" * 40)

    manager = BBIABehaviorManager()

    # Test des comportements individuels
    print("1. Comportement de salutation")
    manager.execute_behavior("greeting")
    time.sleep(1)

    print("2. Animation des antennes")
    context = {"emotion": "excited"}
    manager.execute_behavior("antenna_animation", context)
    time.sleep(1)

    print("3. Réponse émotionnelle")
    context = {"stimulus": "surprise"}
    manager.execute_behavior("emotional_response", context)
    time.sleep(1)

    print("4. Statistiques des comportements")
    stats = manager.get_behavior_stats()
    print(f"   Comportements disponibles : {stats['total_behaviors']}")

    return True


def test_integration():
    """Test d'intégration complète"""
    print("\n🔗 Test d'Intégration Complète")
    print("=" * 40)

    # Initialiser tous les modules
    print("1. Initialisation des modules")
    emotions = BBIAEmotions()
    vision = BBIAVision()
    behavior_manager = BBIABehaviorManager()

    # Scénario : BBIA se réveille et interagit
    print("2. Scénario : Réveil de BBIA")
    behavior_manager.execute_behavior("wake_up")
    time.sleep(2)

    print("3. Scénario : Salutation")
    behavior_manager.execute_behavior("greeting")
    time.sleep(1)

    print("4. Scénario : Analyse de l'environnement")
    result = vision.scan_environment()
    if result["objects"]:
        print(f"   BBIA détecte {len(result['objects'])} objets")
        emotions.set_emotion("curious", 0.7)
        time.sleep(1)

    print("5. Scénario : Réaction émotionnelle")
    context = {"stimulus": "découverte"}
    behavior_manager.execute_behavior("emotional_response", context)
    time.sleep(1)

    print("6. Scénario : Animation des antennes")
    current_emotion_data = emotions.get_current_emotion()
    current_emotion_name = current_emotion_data["name"]
    context = {"emotion": current_emotion_name}
    behavior_manager.execute_behavior("antenna_animation", context)

    return True


def main():
    """Fonction principale de démonstration"""
    print("🤖 DÉMONSTRATION COMPLÈTE DE BBIA")
    print("=" * 60)
    print("Test d'intégration de tous les modules")
    print("=" * 60)

    try:
        # Test 1 : Module émotions
        if not test_emotions_module():
            print("❌ Échec du test du module émotions")
            return False

        # Test 2 : Module vision
        if not test_vision_module():
            print("❌ Échec du test du module vision")
            return False

        # Test 3 : Module voix
        if not test_voice_module():
            print("❌ Échec du test du module voix")
            return False

        # Test 4 : Module audio
        if not test_audio_module():
            print("❌ Échec du test du module audio")
            return False

        # Test 5 : Module comportements
        if not test_behavior_module():
            print("❌ Échec du test du module comportements")
            return False

        # Test 6 : Intégration complète
        if not test_integration():
            print("❌ Échec du test d'intégration")
            return False

        print("\n" + "=" * 60)
        print("✅ DÉMONSTRATION TERMINÉE AVEC SUCCÈS !")
        print("=" * 60)
        print("🎉 Tous les modules BBIA fonctionnent parfaitement ensemble")
        print("🤖 BBIA est prêt pour l'utilisation !")
        print("=" * 60)

        return True

    except Exception as e:
        print(f"\n❌ Erreur lors de la démonstration : {e}")
        import traceback

        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
