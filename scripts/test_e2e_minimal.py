#!/usr/bin/env python3
"""Script de test E2E rapide pour BBIA-SIM
Exécute tous les tests E2E minimaux en < 15s.
"""

import subprocess
import sys
import time


def run_e2e_tests():
    """Exécute tous les tests E2E."""
    # Tests E2E à exécuter
    test_files = [
        "tests/e2e/test_api_simu_roundtrip.py",
        "tests/e2e/test_websocket_telemetry_e2e.py",
        "tests/e2e/test_bbia_modules_e2e.py",
    ]

    total_start = time.time()
    results = []

    for test_file in test_files:
        start_time = time.time()

        try:
            # Exécuter le test avec timeout de 15s
            result = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pytest",
                    test_file,
                    "-v",
                    "--tb=short",
                    "--cov-fail-under=0",  # Pas de limite de coverage pour les tests E2E
                ],
                capture_output=True,
                text=True,
                timeout=15,  # Timeout global de 15s
            )

            duration = time.time() - start_time

            if result.returncode == 0:
                results.append((test_file, "PASS", duration))
            else:
                results.append((test_file, "FAIL", duration))

        except subprocess.TimeoutExpired:
            results.append((test_file, "TIMEOUT", 15.0))
        except Exception:
            results.append((test_file, "ERROR", 0.0))

    time.time() - total_start

    # Résumé

    passed = sum(1 for _, status, _ in results if status == "PASS")
    # failed = sum(1 for _, status, _ in results if status in ["FAIL", "TIMEOUT", "ERROR"])

    for _test_file, _status, _duration in results:
        pass

    return passed == len(results)


if __name__ == "__main__":
    success = run_e2e_tests()
    sys.exit(0 if success else 1)
