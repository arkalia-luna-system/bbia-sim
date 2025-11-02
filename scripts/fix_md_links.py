#!/usr/bin/env python3
"""Script pour corriger automatiquement les liens Markdown cassés les plus courants."""

import glob
import re
from pathlib import Path

ROOT = Path(__file__).parent.parent
DOCS_DIR = ROOT / "docs"

# Mapping des corrections de liens
LINK_FIXES = [
    # Corrections INDEX
    (r"\[([^\]]+)\]\(docs/INDEX\.md\)", r"[\1](../INDEX_FINAL.md)"),
    (r"\[([^\]]+)\]\(INDEX\.md\)", r"[\1](INDEX_FINAL.md)"),
    (r"\[([^\]]+)\]\(INDEX_FINAL\.md\)", r"[\1](INDEX_FINAL.md)"),
    # Corrections GUIDE_DEBUTANT
    (
        r"\[([^\]]+)\]\(\.\./guides/GUIDE_DEBUTANT\.md\)",
        r"[\1](../guides/GUIDE_DEBUTANT.md)",
    ),
    (
        r"\[([^\]]+)\]\(docs/guides/GUIDE_DEBUTANT\.md\)",
        r"[\1](../guides/GUIDE_DEBUTANT.md)",
    ),
    (r"\[([^\]]+)\]\(GUIDE_DEBUTANT\.md\)", r"[\1](../guides/GUIDE_DEBUTANT.md)"),
    # Corrections STATUT_PROJET
    (
        r"\[([^\]]+)\]\(docs/STATUT_PROJET\.md\)",
        r"[\1](../references/STATUT_PROJET.md)",
    ),
    (r"\[([^\]]+)\]\(STATUT_PROJET\.md\)", r"[\1](../references/STATUT_PROJET.md)"),
    # Corrections TESTING_GUIDE
    (r"\[([^\]]+)\]\(docs/TESTING_GUIDE\.md\)", r"[\1](../GUIDE_SYSTEME_TESTS.md)"),
    (r"\[([^\]]+)\]\(TESTING_GUIDE\.md\)", r"[\1](../GUIDE_SYSTEME_TESTS.md)"),
    # Corrections ARCHITECTURE
    (
        r"\[([^\]]+)\]\(docs/ARCHITECTURE\.md\)",
        r"[\1](../architecture/ARCHITECTURE.md)",
    ),
    (r"\[([^\]]+)\]\(ARCHITECTURE\.md\)", r"[\1](../architecture/ARCHITECTURE.md)"),
    # Corrections corrections/
    (
        r"\[([^\]]+)\]\(corrections/TOUTES_DEMOS_CORRIGEES\.md\)",
        r"[\1](../archives/corrections_terminees/TOUTES_DEMOS_CORRIGEES.md)",
    ),
    (
        r"\[([^\]]+)\]\(corrections/DEMO_3D_CORRIGEE\.md\)",
        r"[\1](../corrections/DEMO_3D_CORRIGEE.md)",
    ),
    (
        r"\[([^\]]+)\]\(corrections/CORRECTIONS_DEMOS_REACHY\.md\)",
        r"[\1](../corrections/CORRECTIONS_DEMOS_REACHY.md)",
    ),
    # Corrections conformite/
    (
        r"\[([^\]]+)\]\(conformite/CONFORMITE_REACHY_MINI_COMPLETE\.md\)",
        r"[\1](../conformite/CONFORMITE_REACHY_MINI_COMPLETE.md)",
    ),
    (
        r"\[([^\]]+)\]\(conformite/ANALYSE_CONFORMITE_REACHY_MINI\.md\)",
        r"[\1](../conformite/ANALYSE_CONFORMITE_REACHY_MINI.md)",
    ),
    # Corrections guides/
    (
        r"\[([^\]]+)\]\(guides/GUIDE_DEBUTANT\.md\)",
        r"[\1](../guides/GUIDE_DEBUTANT.md)",
    ),
]


def fix_links_in_content(content: str) -> tuple[str, int]:
    """Corrige les liens dans le contenu."""
    fixes_count = 0
    for pattern, replacement in LINK_FIXES:
        new_content = re.sub(pattern, replacement, content)
        if new_content != content:
            fixes_count += content.count(pattern) if pattern in content else 0
            content = new_content
    return content, fixes_count


def cleanup_metadata_files(file_path: Path) -> None:
    """Supprime les fichiers de métadonnées macOS créés automatiquement."""
    parent_dir = file_path.parent
    base_name = file_path.name

    # Supprimer fichier ._* standard
    metadata_file = parent_dir / f"._{base_name}"
    if metadata_file.exists():
        try:
            metadata_file.unlink()
        except Exception:
            pass

    # Supprimer fichiers .!*!._* (format avec numéro)
    # Pattern: .!XXXXX!._FILENAME
    pattern = str(parent_dir / f".!*._{base_name}")
    for metadata_file_path in glob.glob(pattern):
        try:
            Path(metadata_file_path).unlink()
        except Exception:
            pass


def fix_file(file_path: Path) -> int:
    """Corrige les liens dans un fichier Markdown."""
    try:
        content = file_path.read_text(encoding="utf-8")
        fixed_content, fixes_count = fix_links_in_content(content)

        if fixes_count > 0:
            file_path.write_text(fixed_content, encoding="utf-8")
            # Nettoyer les métadonnées macOS créées automatiquement
            cleanup_metadata_files(file_path)
            return fixes_count
    except Exception as e:
        print(f"⚠️  Erreur sur {file_path}: {e}")
    return 0


def main():
    """Point d'entrée principal."""
    print("🔧 Correction automatique des liens Markdown...\n")

    # Récupérer tous les fichiers MD
    md_files = list(ROOT.rglob("*.md"))
    ignored_dirs = {".git", "venv", "__pycache__", ".pytest_cache", "htmlcov"}
    md_files = [
        f
        for f in md_files
        if not any(part in ignored_dirs for part in f.parts)
        and not f.name.startswith("._")
    ]

    print(f"📄 Analyse de {len(md_files)} fichiers Markdown...\n")

    total_fixes = 0
    fixed_files = []

    for md_file in sorted(md_files):
        fixes = fix_file(md_file)
        if fixes > 0:
            total_fixes += fixes
            fixed_files.append((md_file, fixes))
            print(f"✅ {md_file.relative_to(ROOT)}: {fixes} lien(s) corrigé(s)")

    print(
        f"\n📊 Résumé: {len(fixed_files)} fichier(s) modifié(s), {total_fixes} lien(s) corrigé(s)",
    )

    return 0


if __name__ == "__main__":
    exit(main())
