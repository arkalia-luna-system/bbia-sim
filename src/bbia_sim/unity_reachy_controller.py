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
        command_file: str = "reachy_commands.txt",
        response_file: str = "reachy_response.txt",
    ):
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

    def _wait_for_response(self, timeout: float = 5.0) -> str:
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                if self.response_file.exists():
                    content = self.response_file.read_text().strip()
                    if content and content != self.last_response:
                        self.last_response = content
                        return content
            except Exception:
                pass
            time.sleep(0.1)
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
        self.set_emotion("neutral")
        time.sleep(1)
        time.sleep(1)
        time.sleep(1)
        time.sleep(1)
        time.sleep(1)
        time.sleep(1)
        time.sleep(1)
        self.move_head(10, 0, 0)
        time.sleep(0.7)
        self.move_head(-10, 0, 0)
        time.sleep(0.7)
        self.move_head(0, 0, 0)
        time.sleep(0.7)
        self.set_emotion("happy")
        time.sleep(1)
        time.sleep(1)
        self.set_emotion("neutral")
        return True

    def interactive_mode(self):
        while True:
            try:
                command = input("🤖 BBIA > ").strip().lower()
                if command in {"quit", "exit"}:
                    break
                elif command == "help":
                    self._show_help()
                elif command == "status":
                    pass
                elif command.startswith("head "):
                    parts = command.split()[1:]
                    if len(parts) == 3:
                        x, y, z = map(float, parts)
                        self.move_head(x, y, z)
                    else:
                        pass
                elif command.startswith("emotion "):
                    emotion = command.split()[1]
                    self.set_emotion(emotion)
                elif command == "reset":
                    self.reset_position()
                elif command == "awake":
                    self.bbia_awake()
                else:
                    pass
            except KeyboardInterrupt:
                break
            except Exception:
                pass

    def _show_help(self):
        pass


def main():
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
