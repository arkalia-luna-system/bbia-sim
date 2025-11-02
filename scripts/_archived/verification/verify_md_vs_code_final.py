#!/usr/bin/env python3
"""
Script pour vérifier tous les MD et identifier :
1. Tâches mentionnées comme "à faire" qui sont déjà faites → corriger MD
2. Tâches vraiment pas encore faites → lister
"""

import re
from pathlib import Path
from typing import Any

# Patterns pour tâches "à faire"
TODO_PATTERNS = [
    r"⏳|🔄.*[àa]\s*faire|⚠️.*[àa]\s*faire|❌.*pas.*encore|non.*implémenté|non.*fait|TODO|FIXME",
    r"à\s+faire|à\s+implémenter|à\s+corriger|à\s+ajouter|à\s+vérifier|à\s+mettre",
    r"en\s+attente|en\s+cours|pas\s+encore|manquant|reste\s+à",
]

# Fonctionnalités à vérifier dans le code
FUNCTIONALITIES_TO_CHECK = {
    "uptime": {
        "code_path": "src/bbia_sim/daemon/app/routers/ecosystem.py",
        "patterns": ["get_app_start_time", "format_uptime", "uptime_formatted"],
    },
    "SmolVLM2": {
        "code_path": "src/bbia_sim/bbia_huggingface.py",
        "patterns": ["SmolVLM", "smolvlm", "moondream"],
    },
    "VAD": {
        "code_path": "src/bbia_sim/voice_whisper.py",
        "patterns": ["detect_speech_activity", "silero/vad", "VAD"],
    },
    "NER": {
        "code_path": "src/bbia_sim/bbia_huggingface.py",
        "patterns": ["_extract_angle", "_extract_intensity"],
    },
    "WebSocket connections": {
        "code_path": "src/bbia_sim/daemon/app/routers/ecosystem.py",
        "patterns": ["get_active_connections", "active_websocket"],
    },
    "demo start": {
        "code_path": "src/bbia_sim/daemon/app/routers/ecosystem.py",
        "patterns": ["start_demo_mode"],
        "check_todo": True,  # Vérifier s'il y a encore TODO
    },
}


def check_functionality_implemented(
    name: str, info: dict[str, Any], root_dir: Path
) -> bool:
    """Vérifie si une fonctionnalité est implémentée dans le code."""
    code_path = root_dir / info["code_path"]
    if not code_path.exists():
        return False

    content = code_path.read_text(encoding="utf-8")

    # Vérifier si tous les patterns sont présents
    patterns_found = all(pattern in content for pattern in info["patterns"])

    # Si check_todo, vérifier qu'il n'y a plus de TODO
    if info.get("check_todo", False):
        todo_in_content = "TODO" in content.upper() and any(
            p in content for p in info["patterns"]
        )
        if todo_in_content:
            # Chercher TODO près des patterns
            lines = content.split("\n")
            for i, line in enumerate(lines):
                if any(p in line for p in info["patterns"]):
                    # Vérifier lignes proches
                    nearby = " ".join(
                        lines[max(0, i - 2) : min(len(lines), i + 3)]
                    ).upper()
                    if "TODO" in nearby:
                        return False  # TODO encore présent

    return patterns_found


def find_todos_in_md(file_path: Path) -> list[dict[str, Any]]:
    """Trouve les mentions de TODOs ou tâches à faire dans un MD."""
    todos = []
    try:
        content = file_path.read_text(encoding="utf-8")
        lines = content.split("\n")

        for i, line in enumerate(lines, 1):
            # Chercher patterns TODO
            for pattern in TODO_PATTERNS:
                matches = re.finditer(pattern, line, re.IGNORECASE)
                for match in matches:
                    # Chercher contexte (lignes autour)
                    context_start = max(0, i - 2)
                    context_end = min(len(lines), i + 2)
                    context = "\n".join(lines[context_start:context_end])

                    todos.append(
                        {
                            "line": i,
                            "match": match.group(),
                            "content": line.strip(),
                            "context": context,
                        }
                    )
    except Exception:
        pass

    return todos


def main():
    """Fonction principale."""
    root_dir = Path(__file__).parent.parent

    print("🔍 Vérification MD vs Code - Tâches à faire\n")

    # Vérifier fonctionnalités dans le code
    print("✅ Vérification fonctionnalités implémentées:\n")
    implemented = {}
    for name, info in FUNCTIONALITIES_TO_CHECK.items():
        is_impl = check_functionality_implemented(name, info, root_dir)
        implemented[name] = is_impl
        status = "✅ IMPLÉMENTÉ" if is_impl else "❌ NON IMPLÉMENTÉ"
        print(f"  {status}: {name}")

    print("\n" + "=" * 60 + "\n")

    # Trouver tous les MD avec TODOs
    print("📋 Recherche TODOs dans les MD:\n")
    md_files = []
    for path in root_dir.rglob("*.md"):
        if any(part.startswith(".") or part == "venv" for part in path.parts):
            continue
        if path.name.startswith("._"):
            continue
        md_files.append(path)

    md_with_todos = []
    for md_file in md_files:
        todos = find_todos_in_md(md_file)
        if todos:
            md_with_todos.append((md_file, todos))

    print(f"{len(md_with_todos)} fichiers MD avec mentions 'à faire'\n")

    # Analyser chaque fichier
    outdated_docs = []
    real_todos = []

    for md_file, todos in md_with_todos[:20]:  # Limiter pour lisibilité
        rel_path = md_file.relative_to(root_dir)
        print(f"📝 {rel_path}")

        for todo in todos[:2]:  # 2 premiers par fichier
            line_content = todo["content"][:60]

            # Vérifier si c'est une fonctionnalité déjà implémentée
            is_outdated = False
            for func_name, is_impl in implemented.items():
                if func_name.lower() in line_content.lower() and is_impl:
                    is_outdated = True
                    outdated_docs.append((md_file, todo, func_name))
                    print(
                        f"  ⚠️  OUTDATÉ (ligne {todo['line']}): {func_name} est implémenté"
                    )
                    break

            if not is_outdated:
                real_todos.append((md_file, todo))
                print(f"  ⏳ RÉEL TODO (ligne {todo['line']}): {line_content}...")
        print()

    # Résumé
    print("=" * 60)
    print("\n📊 RÉSUMÉ\n")
    print(
        f"✅ Fonctionnalités implémentées: {sum(implemented.values())}/{len(implemented)}"
    )
    print(f"⚠️  MD à corriger (outdatés): {len(outdated_docs)}")
    print(f"⏳ Vrais TODOs restants: {len(real_todos)}\n")

    # Liste des vraies tâches à faire
    if real_todos:
        print("🎯 VRAIES TÂCHES À FAIRE:\n")
        for md_file, todo in real_todos[:15]:
            print(f"  - {md_file.relative_to(root_dir)}:L{todo['line']}")
            print(f"    {todo['content'][:80]}")
            print()


if __name__ == "__main__":
    main()
