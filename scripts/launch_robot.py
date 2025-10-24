#!/usr/bin/env python3
"""Script simple pour lancer le robot Reachy Mini complet
Usage: python scripts/launch_robot.py [graphical|headless|test].
"""

import subprocess
import sys
from pathlib import Path


def main():
    # Chemin vers le lanceur principal
    script_dir = Path(__file__).parent
    launcher = script_dir / "launch_complete_robot.py"

    if not launcher.exists():
        sys.exit(1)

    # Mode par défaut
    mode = sys.argv[1] if len(sys.argv) > 1 else "graphical"

    # Construction de la commande
    cmd = ["python3", str(launcher)]

    if mode == "graphical":
        pass
    elif mode == "headless":
        cmd.append("--headless")
    elif mode == "test":
        cmd.extend(["--headless", "--duration", "2"])
    else:
        sys.exit(1)

    # Exécution
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError:
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
