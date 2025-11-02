#!/usr/bin/env python3
"""Script pour vérifier les liens internes dans les fichiers Markdown."""

import re
from pathlib import Path
from urllib.parse import urlparse

# Racine du projet
ROOT = Path(__file__).parent.parent
DOCS_DIR = ROOT / "docs"


def extract_links(content: str, file_path: Path) -> list[tuple[str, str]]:
    """Extrait tous les liens depuis un contenu Markdown.

    Returns:
        Liste de tuples (texte_du_lien, url_ou_chemin)

    """
    # Pattern pour liens Markdown [texte](url)
    link_pattern = r"\[([^\]]+)\]\(([^)]+)\)"
    links = re.findall(link_pattern, content)

    # Pattern pour références [texte][ref]
    ref_pattern = r"\[([^\]]+)\]\[([^\]]+)\]"
    refs = re.findall(ref_pattern, content)

    # Pattern pour définitions de références [ref]: url
    ref_def_pattern = r"^\[([^\]]+)\]:\s*(.+)$"
    ref_defs = {}
    for line in content.split("\n"):
        match = re.match(ref_def_pattern, line.strip())
        if match:
            ref_defs[match.group(1)] = match.group(2).strip()

    # Résoudre les références
    resolved_refs = []
    for text, ref in refs:
        if ref in ref_defs:
            resolved_refs.append((text, ref_defs[ref]))

    return links + resolved_refs


def is_external_url(link: str) -> bool:
    """Vérifie si un lien est externe (URL HTTP/HTTPS)."""
    parsed = urlparse(link)
    return parsed.scheme in ("http", "https", "ftp", "mailto")


def resolve_link(link: str, base_path: Path) -> Path | None:
    """Résout un lien relatif vers un chemin absolu.

    Args:
        link: Lien à résoudre (peut contenir #ancre)
        base_path: Chemin du fichier source

    Returns:
        Chemin résolu ou None si externe/invalide

    """
    # Enlever l'ancre si présente
    link_without_anchor = link.split("#")[0]

    if not link_without_anchor or is_external_url(link):
        return None

    # Résoudre chemin relatif
    if link_without_anchor.startswith("/"):
        # Chemin absolu depuis racine du projet
        resolved = ROOT / link_without_anchor.lstrip("/")
    else:
        # Chemin relatif depuis fichier source
        resolved = (base_path.parent / link_without_anchor).resolve()

    return resolved if resolved.exists() else None


def check_file(file_path: Path) -> list[dict]:
    """Vérifie tous les liens dans un fichier Markdown."""
    issues = []

    try:
        content = file_path.read_text(encoding="utf-8")
        links = extract_links(content, file_path)

        for text, link in links:
            if is_external_url(link):
                # Liens externes: pas de vérification
                continue

            # Vérifier si fichier existe
            resolved = resolve_link(link, file_path)
            if resolved is None:
                # Vérifier si c'est une ancre dans le même fichier
                if "#" in link:
                    anchor = link.split("#")[1]
                    # Vérifier si l'ancre existe dans le fichier
                    if anchor.lower().replace(" ", "-") not in content.lower():
                        issues.append(
                            {
                                "type": "broken_anchor",
                                "text": text,
                                "link": link,
                                "file": str(file_path.relative_to(ROOT)),
                                "line": (
                                    content[: content.find(f"[{text}]({link}")].count(
                                        "\n",
                                    )
                                    + 1
                                ),
                            },
                        )
                else:
                    issues.append(
                        {
                            "type": "broken_file",
                            "text": text,
                            "link": link,
                            "file": str(file_path.relative_to(ROOT)),
                            "line": (
                                content[: content.find(f"[{text}]({link}")].count("\n")
                                + 1
                            ),
                        },
                    )

    except Exception as e:
        issues.append(
            {
                "type": "error",
                "message": str(e),
                "file": str(file_path.relative_to(ROOT)),
            },
        )

    return issues


def main():
    """Point d'entrée principal."""
    print("🔍 Vérification des liens Markdown...\n")

    all_issues = []
    md_files = list(ROOT.rglob("*.md"))
    md_files.extend(ROOT.rglob("*.MD"))

    # Filtrer fichiers ignorés
    ignored_dirs = {
        ".git",
        "venv",
        "__pycache__",
        ".pytest_cache",
        "htmlcov",
        "venv-voice",
        "venv-vision",
    }
    md_files = [
        f
        for f in md_files
        if not any(part in ignored_dirs for part in f.parts)
        and not f.name.startswith("._")  # Ignorer fichiers macOS metadata
    ]

    print(f"📄 Vérification de {len(md_files)} fichiers Markdown...\n")

    for md_file in sorted(md_files):
        issues = check_file(md_file)
        if issues:
            all_issues.extend(issues)

    # Afficher résultats
    if not all_issues:
        print("✅ Tous les liens sont valides !\n")
        return 0

    print(f"⚠️  {len(all_issues)} problème(s) détecté(s):\n")

    # Grouper par type
    broken_files = [i for i in all_issues if i["type"] == "broken_file"]
    broken_anchors = [i for i in all_issues if i["type"] == "broken_anchor"]
    errors = [i for i in all_issues if i["type"] == "error"]

    if broken_files:
        print("📄 Fichiers introuvables:")
        for issue in broken_files:
            print(
                f"  - {issue['file']}:{issue['line']} - [{issue['text']}]({issue['link']})",
            )
        print()

    if broken_anchors:
        print("🔗 Ancres introuvables:")
        for issue in broken_anchors:
            print(
                f"  - {issue['file']}:{issue['line']} - [{issue['text']}]({issue['link']})",
            )
        print()

    if errors:
        print("❌ Erreurs:")
        for issue in errors:
            print(f"  - {issue['file']}: {issue['message']}")
        print()

    return len(all_issues)


if __name__ == "__main__":
    exit(main())
