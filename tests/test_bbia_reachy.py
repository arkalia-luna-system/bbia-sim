#!/usr/bin/env python3
"""
Test BBIA pour Reachy Mini Wireless
Simulation des fonctionnalités du robot avant livraison
"""

import time
import random
import sys
from datetime import datetime

class ReachyMiniWirelessSimulator:
    """Simulateur du Reachy Mini Wireless pour BBIA"""
    
    def __init__(self):
        self.name = "BBIA"
        self.robot_model = "Reachy Mini Wireless"
        self.specs = {
            'height': '28cm (23cm veille)',
            'width': '16cm',
            'weight': '1.5kg',
            'processor': 'Raspberry Pi 5',
            'microphones': 4,
            'speaker_power': '5W',
            'head_dof': 6,
            'camera': 'Grand angle',
            'battery': 'Intégrée + USB-C',
            'connectivity': 'Wi-Fi intégré'
        }
        self.current_emotion = 'neutral'
        self.battery_level = 100
        self.is_awake = True
        
    def print_banner(self):
        """Affiche la bannière de démarrage"""
        print("🤖" + "="*60 + "🤖")
        print("🌟 BBIA - Brain-Based Interactive Agent")
        print(f"🤖 Robot: {self.robot_model}")
        print(f"📅 Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"💻 Système: {sys.platform}")
        print("🤖" + "="*60 + "🤖")
        print()
        
    def show_specs(self):
        """Affiche les spécifications du robot"""
        print("📋 Spécifications Reachy Mini Wireless:")
        print("-" * 40)
        for key, value in self.specs.items():
            print(f"  {key.replace('_', ' ').title()}: {value}")
        print()
        
    def simulate_microphones(self):
        """Simule l'utilisation des 4 microphones"""
        print("🎤 Test des 4 microphones...")
        for i in range(4):
            print(f"  Microphone {i+1}: ✅ Actif")
            time.sleep(0.2)
        print("🎵 Reconnaissance vocale: Prête")
        print()
        
    def simulate_speaker(self, message):
        """Simule le haut-parleur 5W"""
        print(f"🔊 Haut-parleur 5W: '{message}'")
        # Simulation d'effets sonores
        for _ in range(3):
            print("  🔊", end="", flush=True)
            time.sleep(0.1)
        print()
        
    def simulate_head_movement(self, emotion):
        """Simule les 6 degrés de liberté de la tête"""
        movements = {
            'neutral': "Tête droite, regard neutre",
            'happy': "Tête légèrement relevée, regard joyeux",
            'sad': "Tête baissée, regard triste",
            'angry': "Tête penchée, regard dur",
            'curious': "Tête inclinée, regard attentif",
            'excited': "Tête relevée, regard enthousiaste"
        }
        print(f"🤖 Mouvement tête (6 DOF): {movements.get(emotion, 'Mouvement par défaut')}")
        
    def simulate_antenna_animation(self, emotion):
        """Simule l'animation des 2 antennes"""
        patterns = {
            'neutral': "Antennes droites, mouvement calme",
            'happy': "Antennes qui bougent joyeusement",
            'sad': "Antennes tombantes",
            'angry': "Antennes rigides",
            'curious': "Antennes qui frémissent",
            'excited': "Antennes qui vibrent rapidement"
        }
        print(f"📡 Animation antennes: {patterns.get(emotion, 'Mouvement par défaut')}")
        
    def simulate_camera(self):
        """Simule la caméra grand angle"""
        print("📷 Caméra grand angle: Active")
        print("  👁️ Reconnaissance d'objets: En cours...")
        objects = ["personne", "table", "chaise", "fenêtre", "livre"]
        detected = random.sample(objects, random.randint(1, 3))
        print(f"  🎯 Objets détectés: {', '.join(detected)}")
        
    def simulate_battery(self):
        """Simule la batterie intégrée"""
        self.battery_level = max(0, self.battery_level - random.randint(0, 2))
        print(f"🔋 Batterie: {self.battery_level}%")
        if self.battery_level < 20:
            print("  ⚠️ Batterie faible - Recharge recommandée")
            
    def change_emotion(self, emotion):
        """Change l'émotion de BBIA"""
        emotions = {
            'neutral': '😐',
            'happy': '😊',
            'sad': '😢',
            'angry': '😠',
            'curious': '🤔',
            'excited': '🤩'
        }
        
        self.current_emotion = emotion
        emoji = emotions.get(emotion, '😐')
        print(f"{emoji} Émotion changée: {emotion}")
        
        # Simuler les réactions physiques
        self.simulate_head_movement(emotion)
        self.simulate_antenna_animation(emotion)
        
    def simulate_voice_interaction(self):
        """Simule l'interaction vocale"""
        print("🗣️ Reconnaissance vocale active...")
        phrases = [
            "Bonjour, comment allez-vous ?",
            "Je suis BBIA, votre assistant IA",
            "Que puis-je faire pour vous ?",
            "J'aime interagir avec les humains",
            "La technologie est fascinante"
        ]
        
        for phrase in random.sample(phrases, 2):
            print(f"  🎤 Entendu: '{phrase}'")
            time.sleep(0.5)
            response = f"J'ai compris: {phrase}"
            self.simulate_speaker(response)
            time.sleep(1)
            
    def run_demo(self):
        """Lance la démonstration complète"""
        self.print_banner()
        self.show_specs()
        
        print("🚀 Démarrage de la simulation BBIA...")
        print()
        
        # Test des composants
        self.simulate_microphones()
        self.simulate_camera()
        
        # Test des émotions
        print("🎭 Test des émotions:")
        emotions = ['neutral', 'happy', 'curious', 'excited', 'sad', 'angry']
        
        for emotion in emotions:
            self.change_emotion(emotion)
            time.sleep(1)
            print()
            
        # Test d'interaction vocale
        print("🗣️ Test d'interaction vocale:")
        self.simulate_voice_interaction()
        print()
        
        # Test de la batterie
        print("🔋 Test de la batterie:")
        for _ in range(5):
            self.simulate_battery()
            time.sleep(0.5)
        print()
        
        # Message final
        print("🎉 Simulation terminée !")
        print("🤖 BBIA est prêt pour le vrai Reachy Mini Wireless")
        print("📅 Livraison prévue: Fin 2025 - Début 2026")
        print()
        print("💡 Conseils pour la suite:")
        print("  1. Étudier la documentation officielle")
        print("  2. Rejoindre la communauté Discord")
        print("  3. Tester en simulation Unity")
        print("  4. Préparer l'architecture BBIA")
        print()
        print("🌟 Merci d'avoir testé BBIA !")

def main():
    """Fonction principale"""
    try:
        simulator = ReachyMiniWirelessSimulator()
        simulator.run_demo()
    except KeyboardInterrupt:
        print("\n👋 Simulation interrompue par l'utilisateur")
    except Exception as e:
        print(f"❌ Erreur lors de la simulation: {e}")

if __name__ == "__main__":
    main() 