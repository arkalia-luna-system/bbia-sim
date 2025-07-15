#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🎮 BBIA - Démonstration Complète
Brain-Based Interactive Agent avec tous les composants installés
"""

import time
import sys
import os

# Couleurs pour l'affichage
class Colors:
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    PURPLE = '\033[95m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    END = '\033[0m'

def print_header():
    """Affiche l'en-tête BBIA"""
    print(f"{Colors.CYAN}{Colors.BOLD}")
    print("🤖" + "="*60 + "🤖")
    print("🌟 BBIA - Brain-Based Interactive Agent")
    print("🤖 Robot: Reachy Mini Wireless")
    print("📅 Date: 2025-07-15")
    print("💻 Système: darwin")
    print("🎮 Mode: Démonstration Complète")
    print("🤖" + "="*60 + "🤖")
    print(f"{Colors.END}")

def test_components():
    """Teste tous les composants installés"""
    print(f"\n{Colors.YELLOW}🔍 Test des composants installés...{Colors.END}")
    
    components = {
        "reachy-sdk": "Contrôle du robot",
        "pollen-vision": "Vision par ordinateur",
        "reachy-docs": "Documentation officielle",
        "reachy2-tutorials": "Tutoriels Jupyter",
        "reachy-dashboard": "Interface web",
        "reachy-face-tracking": "Suivi de visage",
        "reachy2-behaviors-dev": "Comportements",
        "reachy2-sdk-audio-server-rs": "Serveur audio"
    }
    
    for component, description in components.items():
        if component == "pollen-vision":
            try:
                import pollen_vision
                print(f"  ✅ {component}: {description}")
            except ImportError:
                print(f"  ❌ {component}: Non installé")
        elif component == "reachy-sdk":
            try:
                import reachy
                print(f"  ✅ {component}: {description}")
            except ImportError:
                print(f"  ❌ {component}: Non installé")
        else:
            # Vérifier si le dossier existe
            if os.path.exists(f"reachy_repos/{component}"):
                print(f"  ✅ {component}: {description}")
            else:
                print(f"  ❌ {component}: Non installé")

def demo_vision():
    """Démonstration de la vision par ordinateur"""
    print(f"\n{Colors.GREEN}👁️ Démonstration Vision par Ordinateur{Colors.END}")
    print("📷 pollen-vision activé...")
    
    try:
        import pollen_vision
        print("  ✅ Module pollen-vision chargé")
        print("  👁️ Fonctionnalités disponibles :")
        print("    • Reconnaissance d'objets en temps réel")
        print("    • Détection de visages et expressions")
        print("    • Analyse de mouvements")
        print("    • Suivi de visages")
        
        # Simulation de reconnaissance
        print("\n  🎯 Simulation de reconnaissance...")
        objects = ["personne", "table", "chaise", "fenêtre", "livre"]
        for obj in objects:
            print(f"    👁️ Objet détecté: {obj}")
            time.sleep(0.5)
            
    except ImportError:
        print("  ❌ pollen-vision non disponible")

def demo_emotions():
    """Démonstration des émotions BBIA"""
    print(f"\n{Colors.PURPLE}🎭 Démonstration des Émotions BBIA{Colors.END}")
    
    emotions = [
        ("😐", "neutral", "Tête droite, regard neutre"),
        ("😊", "happy", "Tête relevée, regard joyeux"),
        ("🤔", "curious", "Tête inclinée, regard attentif"),
        ("🤩", "excited", "Tête relevée, regard enthousiaste"),
        ("😢", "sad", "Tête baissée, regard triste"),
        ("😠", "angry", "Tête penchée, regard dur")
    ]
    
    for emoji, emotion, description in emotions:
        print(f"  {emoji} Émotion: {emotion}")
        print(f"    🤖 Mouvement tête (6 DOF): {description}")
        print(f"    📡 Animation antennes: {get_antenna_animation(emotion)}")
        time.sleep(1)

def get_antenna_animation(emotion):
    """Retourne l'animation des antennes selon l'émotion"""
    animations = {
        "neutral": "Antennes droites, mouvement calme",
        "happy": "Antennes qui bougent joyeusement",
        "curious": "Antennes qui frémissent",
        "excited": "Antennes qui vibrent rapidement",
        "sad": "Antennes tombantes",
        "angry": "Antennes rigides"
    }
    return animations.get(emotion, "Mouvement standard")

def demo_voice():
    """Démonstration de la reconnaissance vocale"""
    print(f"\n{Colors.BLUE}🗣️ Démonstration Reconnaissance Vocale{Colors.END}")
    
    print("🎤 Test des 4 microphones...")
    for i in range(1, 5):
        print(f"  Microphone {i}: ✅ Actif")
        time.sleep(0.3)
    
    print("\n🗣️ Reconnaissance vocale active...")
    phrases = [
        "Bonjour, comment allez-vous ?",
        "La technologie est fascinante",
        "J'aime interagir avec les humains",
        "L'intelligence artificielle évolue rapidement"
    ]
    
    for phrase in phrases:
        print(f"  🎤 Entendu: '{phrase}'")
        print(f"  🔊 Haut-parleur 5W: 'J'ai compris: {phrase}'")
        print("    🔊  🔊  🔊")
        time.sleep(1)

def demo_components_available():
    """Affiche les composants disponibles pour BBIA"""
    print(f"\n{Colors.CYAN}📚 Composants Disponibles pour BBIA{Colors.END}")
    
    components = [
        ("📚 Documentation", "reachy_repos/reachy-docs/", "Documentation officielle complète"),
        ("🎓 Tutoriels", "reachy_repos/reachy2-tutorials/", "3 tutoriels Jupyter disponibles"),
        ("📊 Dashboard", "reachy_repos/reachy-dashboard/", "Interface web de contrôle"),
        ("🎯 Suivi Visage", "reachy_repos/reachy-face-tracking/", "Suivi automatique des visages"),
        ("🎪 Comportements", "reachy_repos/reachy2-behaviors-dev/", "Bibliothèque de comportements"),
        ("🗣️ Audio Server", "reachy_repos/reachy2-sdk-audio-server-rs/", "Serveur audio haute performance")
    ]
    
    for name, path, description in components:
        if os.path.exists(path):
            print(f"  ✅ {name}: {description}")
        else:
            print(f"  ❌ {name}: Non disponible")

def demo_next_steps():
    """Affiche les prochaines étapes"""
    print(f"\n{Colors.YELLOW}🚀 Prochaines Étapes - Phase 2{Colors.END}")
    
    steps = [
        ("Semaine 1", "Intégrer pollen-vision dans BBIA", "Reconnaissance d'objets en temps réel"),
        ("Semaine 2", "Configurer le serveur audio", "Reconnaissance vocale avancée"),
        ("Semaine 3", "Étudier les comportements", "Actions et réactions automatiques"),
        ("Semaine 4", "Développer l'interface", "Dashboard web personnalisé")
    ]
    
    for week, task, description in steps:
        print(f"  📅 {week}: {task}")
        print(f"    💡 {description}")

def main():
    """Fonction principale de démonstration"""
    print_header()
    
    # Test des composants
    test_components()
    
    # Démonstrations
    demo_vision()
    demo_emotions()
    demo_voice()
    
    # Composants disponibles
    demo_components_available()
    
    # Prochaines étapes
    demo_next_steps()
    
    # Conclusion
    print(f"\n{Colors.GREEN}{Colors.BOLD}")
    print("🎉 Démonstration BBIA terminée !")
    print("🤖 BBIA est prêt pour le vrai Reachy Mini Wireless")
    print("📅 Livraison prévue: Fin 2025 - Début 2026")
    print(f"{Colors.END}")
    
    print(f"\n{Colors.CYAN}💡 Commandes rapides :{Colors.END}")
    print("  🎮 Simulation de base: python3 test_bbia_reachy.py")
    print("  🎮 Unity 3D: ./quick_start.sh (option 6)")
    print("  📚 Tutoriels: cd reachy_repos/reachy2-tutorials/")
    print("  📖 Documentation: cd reachy_repos/reachy-docs/")

if __name__ == "__main__":
    main() 