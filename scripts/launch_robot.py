#!/usr/bin/env python3
"""
Script simple pour lancer le robot Reachy Mini complet
Usage: python scripts/launch_robot.py [graphical|headless|test]
"""

import subprocess
import sys
from pathlib import Path


def main():
    # Chemin vers le lanceur principal
    script_dir = Path(__file__).parent
    launcher = script_dir / "launch_complete_robot.py"

    if not launcher.exists():
        print("❌ Lanceur non trouvé:", launcher)
        sys.exit(1)

    # Mode par défaut
    mode = sys.argv[1] if len(sys.argv) > 1 else "graphical"

    # Construction de la commande
    cmd = ["python3", str(launcher)]

    if mode == "graphical":
        print("🎮 Lancement mode graphique...")
        print("💡 Sur macOS, utilisez 'mjpython' pour la fenêtre 3D")
    elif mode == "headless":
        print("🔄 Lancement mode headless...")
        cmd.append("--headless")
    elif mode == "test":
        print("🧪 Test rapide (2s)...")
        cmd.extend(["--headless", "--duration", "2"])
    else:
        print("❌ Mode inconnu:", mode)
        print("💡 Modes disponibles: graphical, headless, test")
        sys.exit(1)

    # Exécution
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"❌ Erreur: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé par l'utilisateur")
        sys.exit(0)


if __name__ == "__main__":
    main()
