#!/usr/bin/env python3
"""
Audit exhaustif des méthodes Backend - Compare BackendAdapter avec Backend officiel.
"""

import ast
import re
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).parent.parent
OFFICIAL_REPO = PROJECT_ROOT.parent / "reachy_mini"
BBIA_BACKEND_ADAPTER = (
    PROJECT_ROOT / "src" / "bbia_sim" / "daemon" / "app" / "backend_adapter.py"
)
OFFICIAL_BACKEND = (
    OFFICIAL_REPO / "src" / "reachy_mini" / "daemon" / "backend" / "abstract.py"
)


def extract_methods_from_file(file_path: Path) -> dict[str, dict[str, Any]]:
    """Extrait toutes les méthodes d'un fichier Python."""
    methods = {}

    if not file_path.exists():
        return methods

    try:
        with open(file_path, encoding="utf-8") as f:
            content = f.read()
            tree = ast.parse(content)

        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef):
                is_async = isinstance(node, ast.AsyncFunctionDef)
                is_property = any(
                    isinstance(decorator, ast.Name) and decorator.id == "property"
                    for decorator in node.decorator_list
                )

                # Extraire signature
                args = []
                for arg in node.args.args:
                    arg_name = arg.arg
                    if arg_name == "self":
                        continue
                    args.append(arg_name)

                methods[node.name] = {
                    "async": is_async,
                    "property": is_property,
                    "args": args,
                    "line": node.lineno,
                }
    except Exception as e:
        print(f"Erreur parsing {file_path}: {e}")

    return methods


def extract_methods_from_file_regex(file_path: Path) -> dict[str, dict[str, Any]]:
    """Extrait méthodes avec regex (fallback)."""
    methods = {}

    if not file_path.exists():
        return methods

    try:
        with open(file_path, encoding="utf-8") as f:
            lines = f.readlines()

        for i, line in enumerate(lines, 1):
            # Méthodes async
            match = re.match(r"^\s+async def (\w+)\(", line)
            if match:
                methods[match.group(1)] = {
                    "async": True,
                    "property": False,
                    "line": i,
                }
            # Méthodes sync
            match = re.match(r"^\s+def (\w+)\(", line)
            if match:
                method_name = match.group(1)
                if method_name not in methods:
                    methods[method_name] = {
                        "async": False,
                        "property": False,
                        "line": i,
                    }
            # Properties
            match = re.match(r"^\s+@property", line)
            if match and i + 1 < len(lines):
                prop_match = re.match(r"^\s+def (\w+)\(", lines[i])
                if prop_match:
                    methods[prop_match.group(1)] = {
                        "async": False,
                        "property": True,
                        "line": i + 1,
                    }
    except Exception as e:
        print(f"Erreur regex {file_path}: {e}")

    return methods


def main():
    """Point d'entrée principal."""
    print("🔍 Audit méthodes BackendAdapter vs Backend officiel\n")

    # Extraire méthodes
    official_methods = extract_methods_from_file_regex(OFFICIAL_BACKEND)
    bbia_methods = extract_methods_from_file_regex(BBIA_BACKEND_ADAPTER)

    print(f"Méthodes Backend officiel: {len(official_methods)}")
    print(f"Méthodes BackendAdapter BBIA: {len(bbia_methods)}\n")

    # Trouver méthodes manquantes
    missing = []
    for method_name, info in official_methods.items():
        if method_name.startswith("_"):
            continue  # Ignorer méthodes privées

        if method_name not in bbia_methods:
            missing.append((method_name, info))

    print(f"❌ Méthodes MANQUANTES dans BackendAdapter: {len(missing)}\n")
    for method_name, info in sorted(missing):
        async_mark = "async " if info["async"] else ""
        prop_mark = "@property " if info["property"] else ""
        print(f"  - {prop_mark}{async_mark}{method_name} (ligne {info['line']})")

    # Méthodes supplémentaires BBIA
    extra = []
    for method_name, info in bbia_methods.items():
        if method_name.startswith("_"):
            continue
        if method_name not in official_methods:
            extra.append((method_name, info))

    if extra:
        print(f"\nℹ️  Méthodes SUPPLÉMENTAIRES dans BackendAdapter: {len(extra)}")
        for method_name, info in sorted(extra):
            print(f"  - {method_name} (ligne {info['line']})")

    # Méthodes avec signatures différentes
    print("\n⚠️  Vérification signatures...")
    common = set(official_methods.keys()) & set(bbia_methods.keys())
    signature_diff = []
    for method_name in common:
        if method_name.startswith("_"):
            continue
        off_info = official_methods[method_name]
        bbia_info = bbia_methods[method_name]
        if off_info["async"] != bbia_info["async"]:
            signature_diff.append((method_name, "async mismatch", off_info, bbia_info))

    if signature_diff:
        print(f"\n⚠️  Méthodes avec signatures différentes: {len(signature_diff)}")
        for method_name, reason, _off_info, _bbia_info in signature_diff:
            print(f"  - {method_name}: {reason}")
    else:
        print("✅ Toutes les signatures communes sont compatibles")

    print("\n📊 Résumé:")
    print(
        f"  - Méthodes officielles: {len([m for m in official_methods if not m.startswith('_')])}"
    )
    print(
        f"  - Méthodes BBIA: {len([m for m in bbia_methods if not m.startswith('_')])}"
    )
    print(f"  - Manquantes: {len(missing)}")
    print(f"  - Supplémentaires: {len(extra)}")
    print(f"  - Signatures différentes: {len(signature_diff)}")


if __name__ == "__main__":
    main()
