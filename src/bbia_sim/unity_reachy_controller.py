#!/usr/bin/env python3
"""
Contrôleur Python pour le simulateur Unity Reachy Mini Wireless
Communication via fichiers temporaires
"""

import os
import time
import json
from pathlib import Path

class UnityReachyMiniController:
    """Contrôleur pour le simulateur Unity Reachy Mini Wireless"""
    def __init__(self, command_file: str = "reachy_commands.txt", response_file: str = "reachy_response.txt"):
        self.command_file = Path(command_file)
        self.response_file = Path(response_file)
        self.last_response = ""
        self.is_connected = False
        self._init_communication_files()

    def _init_communication_files(self):
        try:
            if not self.command_file.exists():
                self.command_file.write_text("")
            if not self.response_file.exists():
                self.response_file.write_text("")
            self.is_connected = True
            print("📡 Communication Unity initialisée")
        except Exception as e:
            print(f"❌ Erreur d'initialisation: {e}")
            self.is_connected = False

    def _send_command(self, command: str) -> bool:
        if not self.is_connected:
            print("❌ Pas connecté à Unity")
            return False
        try:
            self.command_file.write_text(command)
            print(f"📤 Commande envoyée: {command}")
            return True
        except Exception as e:
            print(f"❌ Erreur d'envoi: {e}")
            return False

    def _wait_for_response(self, timeout: float = 5.0) -> str:
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                if self.response_file.exists():
                    content = self.response_file.read_text().strip()
                    if content and content != self.last_response:
                        self.last_response = content
                        print(f"📥 Réponse reçue: {content}")
                        return content
            except Exception as e:
                print(f"❌ Erreur de lecture: {e}")
            time.sleep(0.1)
        print("⏰ Timeout en attendant la réponse")
        return ""

    def move_head(self, x: float, y: float, z: float) -> bool:
        command = f"move_head|{x}|{y}|{z}"
        return self._send_command(command)

    def set_emotion(self, emotion: str) -> bool:
        valid_emotions = ["neutral", "happy", "sad", "angry"]
        if emotion.lower() not in valid_emotions:
            print(f"❌ Émotion invalide: {emotion}")
            return False
        command = f"set_emotion|{emotion.lower()}"
        return self._send_command(command)

    def reset_position(self) -> bool:
        command = "reset"
        return self._send_command(command)

    def get_status(self) -> str:
        command = "get_status"
        if self._send_command(command):
            return self._wait_for_response()
        return ""

    def bbia_awake(self) -> bool:
        print("\n✨ [BBIA] Initialisation du réveil...")
        self.set_emotion("neutral")
        time.sleep(1)
        print("💡 Lumière blanche faible...")
        time.sleep(1)
        print("💡 Lumière qui s'intensifie doucement...")
        time.sleep(1)
        print("💙 Halo bleu : BBIA s'éveille.")
        time.sleep(1)
        print("🫧 Respiration simulée : inspiration...")
        time.sleep(1)
        print("🫧 Respiration simulée : expiration...")
        time.sleep(1)
        print("🔊 Léger son de démarrage...")
        time.sleep(1)
        print("🤖 Mouvements de tête lents...")
        self.move_head(10, 0, 0)
        time.sleep(0.7)
        self.move_head(-10, 0, 0)
        time.sleep(0.7)
        self.move_head(0, 0, 0)
        time.sleep(0.7)
        print("😊 Expression : sourire doux.")
        self.set_emotion("happy")
        time.sleep(1)
        print("🗣️ Première pensée : 'Je suis là, Athalia.'")
        time.sleep(1)
        print("✨ BBIA est complètement réveillé et prêt !\n")
        self.set_emotion("neutral")
        return True

    def interactive_mode(self):
        print("🎮 Mode interactif - Tapez 'help' pour les commandes disponibles")
        while True:
            try:
                command = input("🤖 BBIA > ").strip().lower()
                if command == "quit" or command == "exit":
                    break
                elif command == "help":
                    self._show_help()
                elif command == "status":
                    print(self.get_status())
                elif command.startswith("head "):
                    parts = command.split()[1:]
                    if len(parts) == 3:
                        x, y, z = map(float, parts)
                        self.move_head(x, y, z)
                    else:
                        print("❌ Usage: head <x> <y> <z>")
                elif command.startswith("emotion "):
                    emotion = command.split()[1]
                    self.set_emotion(emotion)
                elif command == "reset":
                    self.reset_position()
                elif command == "awake":
                    self.bbia_awake()
                else:
                    print("❌ Commande inconnue. Tapez 'help' pour l'aide.")
            except KeyboardInterrupt:
                print("\n👋 Au revoir!")
                break
            except Exception as e:
                print(f"❌ Erreur: {e}")

    def _show_help(self):
        help_text = """
🤖 Commandes BBIA disponibles:

  status          - Afficher le statut de Reachy Mini
  awake           - Lancer la séquence de réveil BBIA
  head <x> <y> <z>     - Déplacer la tête
  emotion <type>  - Changer l'émotion (neutral/happy/sad/angry)
  reset           - Remettre en position initiale
  quit/exit       - Quitter le mode interactif
  help            - Afficher cette aide
        """
        print(help_text)

def main():
    print("🤖 Contrôleur Unity Reachy Mini Wireless")
    print("=" * 50)
    controller = UnityReachyMiniController()
    if not controller.is_connected:
        print("❌ Impossible de se connecter à Unity")
        print("💡 Assurez-vous que Unity est lancé avec la scène ReachyMiniSimulator")
        return
    import sys
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        if command == "awake":
            controller.bbia_awake()
        elif command == "status":
            print(controller.get_status())
        elif command == "interactive":
            controller.interactive_mode()
        else:
            print(f"❌ Commande inconnue: {command}")
            print("💡 Commandes disponibles: awake, status, interactive")
    else:
        controller.interactive_mode()

if __name__ == "__main__":
    main() 