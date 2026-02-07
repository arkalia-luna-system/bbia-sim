#!/usr/bin/env python3
"""Wrapper équivalent à Pollen « scan_motors.py » – appelle scan_motors_baudrate.py.

Dans la doc Pollen / Discord on parle de « scan_motors.py » pour diagnostiquer.
Dans bbia-reachy-sim le script équivalent est scan_motors_baudrate.py.

Usage (identique à scan_motors_baudrate.py):
    python examples/reachy_mini/scan_motors.py
    python examples/reachy_mini/scan_motors.py --serialport /dev/ttyAMA3
    python examples/reachy_mini/scan_motors.py --auto-fix

Voir aussi: examples/reachy_mini/MOTEURS_DIAGNOSTIC_ET_RECONFIG.md
"""

import runpy
import sys
from pathlib import Path

# Lancer scan_motors_baudrate.py dans le même répertoire
script_dir = Path(__file__).resolve().parent
scan_script = script_dir / "scan_motors_baudrate.py"
if not scan_script.exists():
    print("❌ scan_motors_baudrate.py introuvable dans", script_dir)
    sys.exit(1)
sys.argv[0] = str(scan_script)
runpy.run_path(str(scan_script), run_name="__main__")
