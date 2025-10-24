#!/usr/bin/env python3
"""
Script de test E2E rapide pour BBIA-SIM
Exécute tous les tests E2E minimaux en < 15s
"""

import subprocess
import sys
import time


def run_e2e_tests():
    """Exécute tous les tests E2E"""
    print("🧪 EXÉCUTION DES TESTS E2E MINIMAUX")
    print("=" * 50)

    # Tests E2E à exécuter
    test_files = [
        "tests/e2e/test_api_simu_roundtrip.py",
        "tests/e2e/test_websocket_telemetry_e2e.py",
        "tests/e2e/test_bbia_modules_e2e.py",
    ]

    total_start = time.time()
    results = []

    for test_file in test_files:
        print(f"\n📋 Exécution de {test_file}...")
        start_time = time.time()

        try:
            # Exécuter le test avec timeout de 15s
            result = subprocess.run(
                [
                    sys.executable, "-m", "pytest",
                    test_file,
                    "-v",
                    "--tb=short",
                    "--no-cov"  # Pas de coverage pour les tests E2E
                ],
                capture_output=True,
                text=True,
                timeout=15  # Timeout global de 15s
            )

            duration = time.time() - start_time

            if result.returncode == 0:
                print(f"✅ {test_file} - PASSÉ ({duration:.2f}s)")
                results.append((test_file, "PASS", duration))
            else:
                print(f"❌ {test_file} - ÉCHEC ({duration:.2f}s)")
                print(f"   Erreur: {result.stderr}")
                results.append((test_file, "FAIL", duration))

        except subprocess.TimeoutExpired:
            print(f"⏰ {test_file} - TIMEOUT (>15s)")
            results.append((test_file, "TIMEOUT", 15.0))
        except Exception as e:
            print(f"💥 {test_file} - ERREUR: {e}")
            results.append((test_file, "ERROR", 0.0))

    total_duration = time.time() - total_start

    # Résumé
    print("\n" + "=" * 50)
    print("📊 RÉSUMÉ DES TESTS E2E")
    print("=" * 50)

    passed = sum(1 for _, status, _ in results if status == "PASS")
    # failed = sum(1 for _, status, _ in results if status in ["FAIL", "TIMEOUT", "ERROR"])

    for test_file, status, duration in results:
        status_icon = "✅" if status == "PASS" else "❌"
        print(f"{status_icon} {test_file}: {status} ({duration:.2f}s)")

    print(f"\n🎯 Résultat: {passed}/{len(results)} tests passés")
    print(f"⏱️  Durée totale: {total_duration:.2f}s")

    if passed == len(results):
        print("🎉 TOUS LES TESTS E2E SONT PASSÉS !")
        return True
    else:
        print("⚠️  CERTAINS TESTS E2E ONT ÉCHOUÉ")
        return False


if __name__ == "__main__":
    success = run_e2e_tests()
    sys.exit(0 if success else 1)
