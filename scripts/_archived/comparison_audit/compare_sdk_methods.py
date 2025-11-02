#!/usr/bin/env python3
"""Compare les méthodes du SDK officiel avec celles de BBIA."""

import re
from pathlib import Path


def extract_methods_from_file(file_path: Path) -> set[str]:
    """Extrait les noms des méthodes publiques d'un fichier Python."""
    methods = set()

    try:
        content = file_path.read_text()

        # Utiliser regex pour extraire les méthodes (plus simple qu'AST)
        # Chercher def method_name( ou async def method_name(
        pattern = r"^\s+(?:async\s+)?def\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*\("
        matches = re.findall(pattern, content, re.MULTILINE)

        # Filtrer les méthodes privées et spéciales
        for match in matches:
            if not match.startswith("_") or match in [
                "__init__",
                "__enter__",
                "__exit__",
                "__del__",
            ]:
                methods.add(match)

    except Exception as e:
        print(f"Erreur lecture {file_path}: {e}")

    return methods


def main():
    """Compare les méthodes."""
    official_file = Path("/Volumes/T7/reachy_mini/src/reachy_mini/reachy_mini.py")
    bbia_file = Path(
        "/Volumes/T7/bbia-reachy-sim/src/bbia_sim/backends/reachy_mini_backend.py",
    )

    official_methods = extract_methods_from_file(official_file)
    bbia_methods = extract_methods_from_file(bbia_file)

    print("=" * 80)
    print("COMPARAISON MÉTHODES SDK OFFICIEL vs BBIA")
    print("=" * 80)
    print("\n📊 Statistiques:")
    print(f"  Méthodes SDK officiel: {len(official_methods)}")
    print(f"  Méthodes BBIA backend: {len(bbia_methods)}")

    missing_in_bbia = official_methods - bbia_methods
    extra_in_bbia = bbia_methods - official_methods

    print(
        f"\n✅ Méthodes présentes dans les deux: {len(official_methods & bbia_methods)}",
    )
    print(f"⚠️  Méthodes manquantes dans BBIA: {len(missing_in_bbia)}")
    print(f"ℹ️  Méthodes supplémentaires dans BBIA: {len(extra_in_bbia)}")

    if missing_in_bbia:
        print("\n🔴 MÉTHODES MANQUANTES DANS BBIA:")
        for method in sorted(missing_in_bbia):
            print(f"  - {method}")

    if extra_in_bbia:
        print("\n🟢 MÉTHODES SUPPLÉMENTAIRES DANS BBIA (extensions légitimes):")
        for method in sorted(extra_in_bbia):
            print(f"  - {method}")

    # Méthodes communes
    common = official_methods & bbia_methods
    if common:
        print(f"\n✅ MÉTHODES COMMUNES ({len(common)}):")
        for method in sorted(common):
            print(f"  - {method}")


if __name__ == "__main__":
    main()
