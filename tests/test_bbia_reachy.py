#!/usr/bin/env python3
"""Test BBIA pour Reachy Mini Wireless
Simulation des fonctionnalités du robot avant livraison.
"""

import random
import time


class ReachyMiniWirelessSimulator:
    """Simulateur du Reachy Mini Wireless pour BBIA."""

    def __init__(self):
        self.name = "BBIA"
        self.robot_model = "Reachy Mini Wireless"
        self.specs = {
            "height": "28cm (23cm veille)",
            "width": "16cm",
            "weight": "1.5kg",
            "processor": "Raspberry Pi 5",
            "microphones": 4,
            "speaker_power": "5W",
            "head_dof": 6,
            "camera": "Grand angle",
            "battery": "Intégrée + USB-C",
            "connectivity": "Wi-Fi intégré",
        }
        self.current_emotion = "neutral"
        self.battery_level = 100
        self.is_awake = True

    def print_banner(self):
        """Affiche la bannière de démarrage."""

    def show_specs(self):
        """Affiche les spécifications du robot."""
        for _key, _value in self.specs.items():
            pass

    def simulate_microphones(self):
        """Simule l'utilisation des 4 microphones."""
        for _i in range(4):
            time.sleep(0.2)

    def simulate_speaker(self, message):
        """Simule le haut-parleur 5W."""
        # Simulation d'effets sonores
        for _ in range(3):
            time.sleep(0.1)

    def simulate_head_movement(self, emotion):
        """Simule les 6 degrés de liberté de la tête."""

    def simulate_antenna_animation(self, emotion):
        """Simule l'animation des 2 antennes."""

    def simulate_camera(self):
        """Simule la caméra grand angle."""
        objects = ["personne", "table", "chaise", "fenêtre", "livre"]
        random.sample(objects, random.randint(1, 3))

    def simulate_battery(self):
        """Simule la batterie intégrée."""
        self.battery_level = max(0, self.battery_level - random.randint(0, 2))
        if self.battery_level < 20:
            pass

    def change_emotion(self, emotion):
        """Change l'émotion de BBIA."""
        emotions = {
            "neutral": "😐",
            "happy": "😊",
            "sad": "😢",
            "angry": "😠",
            "curious": "🤔",
            "excited": "🤩",
        }

        self.current_emotion = emotion
        emotions.get(emotion, "😐")

        # Simuler les réactions physiques
        self.simulate_head_movement(emotion)
        self.simulate_antenna_animation(emotion)

    def simulate_voice_interaction(self):
        """Simule l'interaction vocale."""
        phrases = [
            "Bonjour, comment allez-vous ?",
            "Je suis BBIA, votre assistant IA",
            "Que puis-je faire pour vous ?",
            "J'aime interagir avec les humains",
            "La technologie est fascinante",
        ]

        for phrase in random.sample(phrases, 2):
            time.sleep(0.5)
            response = f"J'ai compris: {phrase}"
            self.simulate_speaker(response)
            time.sleep(1)

    def run_demo(self):
        """Lance la démonstration complète."""
        self.print_banner()
        self.show_specs()

        # Test des composants
        self.simulate_microphones()
        self.simulate_camera()

        # Test des émotions
        emotions = ["neutral", "happy", "curious", "excited", "sad", "angry"]

        for emotion in emotions:
            self.change_emotion(emotion)
            time.sleep(1)

        # Test d'interaction vocale
        self.simulate_voice_interaction()

        # Test de la batterie
        for _ in range(5):
            self.simulate_battery()
            time.sleep(0.5)

        # Message final


def main():
    """Fonction principale."""
    try:
        simulator = ReachyMiniWirelessSimulator()
        simulator.run_demo()
    except KeyboardInterrupt:
        pass
    except Exception:
        pass


if __name__ == "__main__":
    main()
