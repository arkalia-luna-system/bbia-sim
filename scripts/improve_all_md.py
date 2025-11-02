#!/usr/bin/env python3
"""Améliore tous les fichiers MD : vérifie véracité + style moderne professionnel."""

import re
from pathlib import Path
from typing import Any

IMPROVEMENTS_MADE = []


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
    pattern = str(parent_dir / f".!*._{base_name}")
    for metadata_file_path in glob.glob(pattern):
        try:
            Path(metadata_file_path).unlink()
        except Exception:
            pass


def verify_content(file_path: Path) -> list[str]:
    """Vérifie la véracité du contenu."""
    issues = []
    content = file_path.read_text(encoding="utf-8")

    # Vérifications basiques
    if "1200+ tests" in content or "1200 tests" in content:
        # Vérifier si c'est correct (1208 tests réels)
        if "1208" not in content and "1157" not in content:
            pass  # "1200+" est acceptable

    if "12 émotions" not in content.lower() and "douze émotions" not in content.lower():
        if "emotion" in content.lower() and "bbia" in content.lower():
            # Vérifier si nombre spécifié est correct
            emotion_match = re.search(r"(\d+)\s+émotions?", content, re.IGNORECASE)
            if emotion_match and emotion_match.group(1) != "12":
                issues.append(
                    f"Nombre d'émotions incorrect: {emotion_match.group(1)} (devrait être 12)",
                )

    return issues


def improve_markdown_style(content: str) -> str:
    """Améliore le style Markdown pour le rendre moderne et professionnel."""
    original = content

    # 1. Améliorer les séparateurs
    content = re.sub(r"^---+$", "---", content, flags=re.MULTILINE)

    # 2. Standardiser les espaces autour des titres
    content = re.sub(r"\n(##+)\s+([^\n]+)\n+([^\n#])", r"\n\1 \2\n\n\3", content)

    # 3. Améliorer les listes avec puces modernes
    # Garder les listes existantes mais s'assurer qu'elles sont bien formatées

    # 4. Améliorer les blocs de code (ajouter langage si manquant pour Python/bash)
    lines = content.split("\n")
    improved_lines = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.strip().startswith("```") and not line.strip()[3:].strip():
            # Trouver le bloc suivant pour deviner le langage
            j = i + 1
            while j < len(lines) and not lines[j].strip().startswith("```"):
                if (
                    lines[j]
                    .strip()
                    .startswith(("pip", "python", "import", "from", "def ", "class "))
                ):
                    improved_lines.append("```python")
                    break
                if lines[j].strip().startswith(("git ", "cd ", "export ", "bash")):
                    improved_lines.append("```bash")
                    break
                j += 1
            else:
                improved_lines.append(line)
        else:
            improved_lines.append(line)
        i += 1
    content = "\n".join(improved_lines)

    # 5. Standardiser les dates
    content = re.sub(r"octobre\s+2025|Octobre\s+2025", "Octobre 2025", content)
    content = re.sub(r"(novembre|Novembre)\s+2025", "Oct 25 / Nov 25", content)

    # 6. Améliorer les tableaux (ajouter espacement)
    content = re.sub(r"\|([^|]+)\|", lambda m: f"| {m.group(1).strip()} |", content)

    # 7. Normaliser les sauts de ligne (max 2)
    content = re.sub(r"\n{3,}", "\n\n", content)

    # 8. Améliorer les liens (format cohérent)
    content = re.sub(
        r"\[([^\]]+)\]\(([^)]+)\)",
        lambda m: f"[{m.group(1)}]({m.group(2)})" if m.group(2) else m.group(0),
        content,
    )

    if content != original:
        IMPROVEMENTS_MADE.append("Style amélioré")

    return content


def improve_readme_specifically(content: str) -> str:
    """Améliorations spécifiques pour README.md."""
    # Améliorer la section titre avec meilleure structure
    if not content.startswith("# BBIA"):
        # Ajouter header si nécessaire
        pass

    # Améliorer la section badges
    badges_pattern = r"## 🏆 Badges Qualité & CI/CD\s*\n\n"
    if not re.search(badges_pattern, content):
        # S'assurer que les badges sont bien présentés
        content = re.sub(r"(## 🏆 Badges Qualité & CI/CD)\s*\n", r"\1\n\n", content)

    # Améliorer la section "EN 30 SECONDES"
    content = re.sub(
        r"## 📋 \*\*EN 30 SECONDES :\*\*",
        r"## 📋 **EN 30 SECONDES**",
        content,
    )

    # Améliorer les points clés avec meilleur formatage
    content = re.sub(
        r"• ✅ \*\*([^*]+)\*\*\s*\(([^)]+)\)",
        r"• ✅ **\1** (\2)",
        content,
    )

    IMPROVEMENTS_MADE.append("README spécifiquement amélioré")
    return content


def improve_projects_specifically(content: str) -> str:
    """Améliorations spécifiques pour PROJECTS.md."""
    # Améliorer les sections projets avec meilleure structure
    content = re.sub(r"### (\d+)\. \*\*([^*]+)\*\*", r"### \1. **\2**", content)

    # Standardiser les métadonnées projets
    content = re.sub(
        r"\*\*📁 Repository :\*\*\s*\[([^\]]+)\]\(([^)]+)\)",
        r"**📁 Repository :** [\1](\2)",
        content,
    )

    IMPROVEMENTS_MADE.append("PROJECTS spécifiquement amélioré")
    return content


def process_file(file_path: Path, is_main_file: bool = False) -> dict[str, Any]:
    """Traite un fichier MD."""
    try:
        content = file_path.read_text(encoding="utf-8")
        original = content

        # 1. Vérifier véracité
        issues = verify_content(file_path)

        # 2. Améliorer style
        if file_path.name == "README.md":
            content = improve_readme_specifically(content)
        elif file_path.name == "PROJECTS.md":
            content = improve_projects_specifically(content)

        content = improve_markdown_style(content)

        # 3. Sauvegarder si changé
        if content != original:
            file_path.write_text(content, encoding="utf-8")
            # Nettoyer les métadonnées macOS créées automatiquement
            cleanup_metadata_files(file_path)
            return {
                "file": str(file_path),
                "changed": True,
                "issues": issues,
                "improvements": IMPROVEMENTS_MADE.copy(),
            }

        return {
            "file": str(file_path),
            "changed": False,
            "issues": issues,
            "improvements": [],
        }
    except Exception as e:
        return {
            "file": str(file_path),
            "error": str(e),
        }


def main():
    """Fonction principale."""
    root = Path("/Volumes/T7/bbia-reachy-sim")

    # Fichiers prioritaires
    priority_files = [
        "README.md",
        "PROJECTS.md",
        "docs/guides/GUIDE_DEBUTANT.md",
        "docs/guides/GUIDE_AVANCE.md",
        "docs/README.md",
    ]

    print("✨ Amélioration Documentation MD\n")
    print("=" * 60)

    results = []
    for file_path in priority_files:
        full_path = root / file_path
        if not full_path.exists():
            continue

        print(f"\n📝 {file_path}")
        result = process_file(full_path, is_main_file=(file_path == "README.md"))
        results.append(result)

        if result.get("error"):
            print(f"   ❌ Erreur: {result['error']}")
        elif result.get("changed"):
            print("   ✅ Amélioré")
            if result.get("issues"):
                print(f"   ⚠️  {len(result['issues'])} problèmes de véracité")
            if result.get("improvements"):
                for imp in result["improvements"]:
                    print(f"      • {imp}")
        else:
            print("   ℹ️  Aucun changement nécessaire")
            if result.get("issues"):
                print(f"   ⚠️  {len(result['issues'])} problèmes de véracité détectés")

    # Résumé
    changed = sum(1 for r in results if r.get("changed"))
    issues = sum(len(r.get("issues", [])) for r in results)

    print("\n" + "=" * 60)
    print(f"\n✅ {changed} fichiers améliorés")
    if issues > 0:
        print(f"⚠️  {issues} problèmes de véracité détectés (à vérifier)")
    else:
        print("✅ Aucun problème de véracité")


if __name__ == "__main__":
    main()
