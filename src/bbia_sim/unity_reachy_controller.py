#!/usr/bin/env python3
"""Contrôleur Python pour le simulateur Unity Reachy Mini Wireless
Communication via fichiers temporaires.
"""

import time
from pathlib import Path


class UnityReachyMiniController:
    """Contrôleur pour le simulateur Unity Reachy Mini Wireless."""

    def __init__(
        self,
        command_file: str = "log/reachy_commands.txt",
        response_file: str = "log/reachy_response.txt",
    ) -> None:
        self.command_file = Path(command_file)
        self.response_file = Path(response_file)
        self.last_response = ""
        self.is_connected = False
        self._init_communication_files()

    def _init_communication_files(self) -> None:
        try:
            # S'assurer que les répertoires existent (ex: 'log/')
            if self.command_file.parent and not self.command_file.parent.exists():
                self.command_file.parent.mkdir(parents=True, exist_ok=True)
            if self.response_file.parent and not self.response_file.parent.exists():
                self.response_file.parent.mkdir(parents=True, exist_ok=True)
            if not self.command_file.exists():
                self.command_file.write_text("")
            if not self.response_file.exists():
                self.response_file.write_text("")
            self.is_connected = True
        except Exception:
            self.is_connected = False

    def _send_command(self, command: str) -> bool:
        if not self.is_connected:
            return False
        try:
            self.command_file.write_text(command)
            return True
        except Exception:
            return False

    def _wait_for_response(self, timeout: float = 1.0) -> str:
        """Attend une réponse avec timeout réduit pour les tests."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                if self.response_file.exists():
                    content = self.response_file.read_text().strip()
                    if content and content != self.last_response:
                        self.last_response = content
                        return content
            except OSError:  # nosec B110
                # Erreur de lecture de fichier - ignorer et continuer
                pass
            except Exception:  # nosec B110
                # Autres erreurs inattendues - ignorer et continuer
                pass
            time.sleep(0.01)  # Réduit de 0.1s à 0.01s pour les tests
        return ""

    def move_head(self, x: float, y: float, z: float) -> bool:
        command = f"move_head|{x}|{y}|{z}"
        return self._send_command(command)

    def set_emotion(self, emotion: str) -> bool:
        valid_emotions = ["neutral", "happy", "sad", "angry"]
        if emotion.lower() not in valid_emotions:
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
        """Séquence de réveil BBIA optimisée."""
        self.set_emotion("neutral")
        time.sleep(0.1)  # Réduit de 7s à 0.1s pour les tests
        self.move_head(10, 0, 0)
        time.sleep(0.1)  # Réduit de 0.7s à 0.1s
        self.move_head(-10, 0, 0)
        time.sleep(0.1)  # Réduit de 0.7s à 0.1s
        self.move_head(0, 0, 0)
        time.sleep(0.1)  # Réduit de 0.7s à 0.1s
        self.set_emotion("happy")
        time.sleep(0.1)  # Réduit de 2s à 0.1s
        self.set_emotion("neutral")
        return True

    def interactive_mode(self, max_iterations: int = 1000) -> None:
        """Mode interactif avec limite d'itérations pour éviter les boucles infinies."""
        iteration_count = 0
        while iteration_count < max_iterations:
            try:
                command = input("🤖 BBIA > ").strip().lower()
                if command in {"quit", "exit"}:
                    break
                elif command == "help":
                    self._show_help()
                elif command == "status":
                    print(
                        "Status: Connected"
                        if self.is_connected
                        else "Status: Disconnected"
                    )
                elif command.startswith("head "):
                    parts = command.split()[1:]
                    if len(parts) == 3:
                        try:
                            x, y, z = map(float, parts)
                            self.move_head(x, y, z)
                        except ValueError:
                            print(
                                "❌ Valeurs invalides pour head. Utilisez: head x y z"
                            )
                    else:
                        print("❌ Commande head invalide. Utilisez: head x y z")
                elif command.startswith("emotion "):
                    emotion = command.split()[1]
                    if self.set_emotion(emotion):
                        print(f"✅ Émotion '{emotion}' définie")
                    else:
                        print(f"❌ Émotion '{emotion}' invalide")
                elif command == "reset":
                    if self.reset_position():
                        print("✅ Position réinitialisée")
                    else:
                        print("❌ Erreur lors de la réinitialisation")
                elif command == "awake":
                    print("🤖 BBIA se réveille...")
                    self.bbia_awake()
                    print("✅ BBIA est réveillé!")
                else:
                    print("❌ Commande inconnue. Tapez 'help' pour l'aide.")
                iteration_count += 1
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ Erreur: {e}")
                iteration_count += 1

        if iteration_count >= max_iterations:
            print("⚠️ Limite d'itérations atteinte, arrêt du mode interactif")

    def _show_help(self) -> None:
        help_text = """
🤖 Commandes BBIA disponibles:

• status     - Afficher le statut de connexion
• awake      - Réveiller BBIA (séquence complète)
• head x y z - Déplacer la tête (x, y, z en degrés)
• emotion <nom> - Définir une émotion (neutral, happy, sad, angry)
• reset      - Réinitialiser la position
• help       - Afficher cette aide
• quit/exit - Quitter le mode interactif

Exemples:
  head 10 0 0    - Tourner la tête à droite
  emotion happy - Rendre BBIA heureux
  awake         - Séquence de réveil complète
"""
        print(help_text)


def main() -> None:
    controller = UnityReachyMiniController()
    if not controller.is_connected:
        return
    import sys

    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        if command == "awake":
            controller.bbia_awake()
        elif command == "status":
            pass
        elif command == "interactive":
            controller.interactive_mode()
        else:
            pass
    else:
        controller.interactive_mode()


if __name__ == "__main__":
    main()
