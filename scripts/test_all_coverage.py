#!/usr/bin/env python3
"""Script pour tester tous les tests et voir la coverage globale."""

import subprocess
import sys
import time


def run_all_tests():
    """Exécute tous les tests disponibles."""
    start_time = time.time()

    try:
        # Exécuter tous les tests avec coverage
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "pytest",
                "tests/",
                "-v",
                "--tb=short",
                "--cov-fail-under=0",  # Pas de limite pour voir la vraie coverage
                "--cov-report=term-missing",
                "--cov-report=html",
            ],
            capture_output=True,
            text=True,
            timeout=300,  # 5 minutes max
        )

        time.time() - start_time

        if result.returncode == 0:
            pass
        else:
            pass

        # Afficher la sortie

        if result.stderr:
            pass

        return result.returncode == 0

    except subprocess.TimeoutExpired:
        return False
    except Exception:
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
