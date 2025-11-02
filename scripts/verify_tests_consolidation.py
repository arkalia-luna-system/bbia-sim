#!/usr/bin/env python3
"""
Script de vérification pour la consolidation des tests.
Vérifie qu'aucun test n'est perdu lors de la consolidation.

Usage:
    python scripts/verify_tests_consolidation.py
"""

import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

# Fichiers de tests Reachy Mini à analyser
REACHY_MINI_TEST_FILES = [
    "tests/test_reachy_mini_backend.py",
    "tests/test_reachy_mini_full_conformity_official.py",
    "tests/test_reachy_mini_strict_conformity.py",
    "tests/test_reachy_mini_complete_conformity.py",
    "tests/test_reachy_mini_backend_extended.py",
    "tests/test_reachy_mini_backend_rapid.py",
    "tests/test_reachy_mini_conformity.py",
    "tests/test_reachy_mini_advanced_conformity.py",
]


def collect_tests_from_file(filepath: Path) -> list[str]:
    """Collecter tous les noms de tests d'un fichier."""
    if not filepath.exists():
        return []

    tests = []
    try:
        # Utiliser python -m pytest pour lister les tests
        result = subprocess.run(
            ["python", "-m", "pytest", str(filepath), "--collect-only", "-q"],
            capture_output=True,
            text=True,
            timeout=30,
            cwd=filepath.parent.parent,  # S'assurer d'être dans le bon répertoire
        )

        if result.returncode == 0:
            # Parser la sortie pour extraire les noms de tests
            for line in result.stdout.split("\n"):
                line = line.strip()
                # Chercher les lignes avec des tests (format: <Function test_...> ou <Method test_...>)
                if ("test_" in line and ("Function" in line or "Method" in line)) or (
                    "::" in line and "test_" in line
                ):
                    # Extraire le nom du test
                    if "::" in line:
                        parts = line.split("::")
                        if len(parts) >= 2:
                            test_name = parts[-1].strip()
                            if test_name.startswith("test_"):
                                tests.append(test_name)
                    else:
                        # Format <Function test_name> ou <Method test_name>
                        match = re.search(r"test_\w+", line)
                        if match:
                            tests.append(match.group(0))
    except Exception as e:
        print(f"⚠️  Erreur lors de la collecte de {filepath}: {e}")

    return tests


def collect_all_tests() -> dict[str, list[str]]:
    """Collecter tous les tests de tous les fichiers."""
    project_root = Path(__file__).parent.parent
    all_tests = {}

    for test_file in REACHY_MINI_TEST_FILES:
        filepath = project_root / test_file
        tests = collect_tests_from_file(filepath)
        if tests:
            all_tests[test_file] = tests
            print(f"✅ {test_file}: {len(tests)} tests")
        else:
            print(f"⚠️  {test_file}: Aucun test collecté (fichier absent ou erreur)")

    return all_tests


def find_duplicate_tests(all_tests: dict[str, list[str]]) -> dict[str, list[str]]:
    """Trouver les tests dupliqués entre fichiers."""
    # Inverser: test_name -> fichiers qui le contiennent
    test_to_files = defaultdict(list)

    for filepath, tests in all_tests.items():
        for test_name in tests:
            test_to_files[test_name].append(filepath)

    # Trouver les doublons (test présent dans plusieurs fichiers)
    duplicates = {
        test_name: files for test_name, files in test_to_files.items() if len(files) > 1
    }

    return duplicates


def find_unique_tests(all_tests: dict[str, list[str]]) -> dict[str, list[str]]:
    """Trouver les tests uniques par fichier."""
    # Inverser: test_name -> fichiers qui le contiennent
    test_to_files = defaultdict(list)

    for filepath, tests in all_tests.items():
        for test_name in tests:
            test_to_files[test_name].append(filepath)

    # Trouver les tests uniques (présents dans un seul fichier)
    unique_by_file = defaultdict(list)

    for test_name, files in test_to_files.items():
        if len(files) == 1:
            filepath = files[0]
            unique_by_file[filepath].append(test_name)

    return unique_by_file


def generate_consolidation_report(
    all_tests: dict, duplicates: dict, unique_by_file: dict
):
    """Générer un rapport de consolidation."""
    print("\n" + "=" * 70)
    print("📊 RAPPORT DE CONSOLIDATION DES TESTS")
    print("=" * 70)

    # Statistiques globales
    total_tests = sum(len(tests) for tests in all_tests.values())
    total_unique_tests = sum(len(tests) for tests in unique_by_file.values())
    total_duplicates = len(duplicates)

    print("\n📈 Statistiques:")
    print(f"   Total de tests: {total_tests}")
    print(f"   Tests uniques: {total_unique_tests}")
    print(f"   Tests dupliqués: {total_duplicates}")

    # Tests par fichier
    print("\n📁 Tests par fichier:")
    for filepath, tests in sorted(all_tests.items()):
        unique_count = len(unique_by_file.get(filepath, []))
        duplicate_count = len(tests) - unique_count
        print(f"   {filepath}:")
        print(f"      Total: {len(tests)} tests")
        print(f"      Uniques: {unique_count}")
        print(f"      Dupliqués: {duplicate_count}")

    # Tests dupliqués
    if duplicates:
        print("\n🔄 Tests dupliqués (présents dans plusieurs fichiers):")
        for test_name, files in sorted(duplicates.items())[:20]:  # Limiter à 20
            print(f"   {test_name}:")
            for filepath in files:
                print(f"      - {filepath}")
        if len(duplicates) > 20:
            print(f"   ... et {len(duplicates) - 20} autres")

    # Tests uniques par fichier
    print("\n⭐ Tests uniques par fichier (à préserver lors consolidation):")
    for filepath, tests in sorted(unique_by_file.items()):
        if tests:
            print(f"   {filepath}: {len(tests)} tests uniques")
            for test_name in sorted(tests)[:10]:  # Limiter affichage
                print(f"      - {test_name}")
            if len(tests) > 10:
                print(f"      ... et {len(tests) - 10} autres")

    # Recommandations
    print("\n💡 Recommandations:")

    # Identifier fichiers avec beaucoup de tests uniques
    files_with_unique = {f: len(t) for f, t in unique_by_file.items() if len(t) > 0}

    if files_with_unique:
        print("   Fichiers avec tests uniques à PRÉSERVER:")
        for filepath, count in sorted(files_with_unique.items(), key=lambda x: -x[1]):
            print(f"      ✅ {filepath}: {count} tests uniques")

    # Identifier fichiers potentiellement redondants
    files_with_few_unique = {
        f: len(t)
        for f, t in unique_by_file.items()
        if len(t) == 0 and len(all_tests.get(f, [])) > 0
    }

    if files_with_few_unique:
        print("   Fichiers potentiellement REDONDANTS (peu de tests uniques):")
        for filepath, count in sorted(
            files_with_few_unique.items(), key=lambda x: -len(all_tests[x[0]])
        ):
            total = len(all_tests[filepath])
            print(f"      ⚠️  {filepath}: {total} tests (aucun unique)")

    print("\n" + "=" * 70)


def main():
    """Fonction principale."""
    print("🔍 Collecte des tests...")
    print("-" * 70)

    all_tests = collect_all_tests()

    if not all_tests:
        print(
            "❌ Aucun test collecté. Vérifiez que pytest est installé et que les fichiers existent."
        )
        sys.exit(1)

    print(f"\n✅ {len(all_tests)} fichiers analysés")

    # Analyser les doublons et tests uniques
    duplicates = find_duplicate_tests(all_tests)
    unique_by_file = find_unique_tests(all_tests)

    # Générer le rapport
    generate_consolidation_report(all_tests, duplicates, unique_by_file)

    # Vérification finale
    total_before = sum(len(tests) for tests in all_tests.values())
    total_unique = sum(len(tests) for tests in unique_by_file.values())

    print("\n✅ Vérification terminée!")
    print(f"   Total tests: {total_before}")
    print(f"   Tests uniques: {total_unique}")
    print(f"   Doublons: {len(duplicates)}")

    if total_unique == total_before:
        print(
            "   ✅ Aucun doublon détecté (ou tous les tests sont dans un seul fichier)"
        )
    else:
        print(f"   ⚠️  {len(duplicates)} tests sont dupliqués")


if __name__ == "__main__":
    main()
