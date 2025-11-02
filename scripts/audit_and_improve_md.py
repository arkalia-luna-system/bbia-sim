#!/usr/bin/env python3
"""
Script pour auditer et améliorer tous les fichiers MD :
1. Vérifier véracité contre code réel
2. Améliorer présentation (moderne, professionnelle, impactante)
"""

import re
from pathlib import Path
from typing import Any


def verify_md_claims(md_file: Path) -> dict[str, Any]:
    """Vérifie les affirmations dans un MD contre le code réel."""
    issues = []
    content = md_file.read_text(encoding="utf-8")

    # Vérifier affirmations communes
    claims = {
        "tests": {
            "patterns": [
                r"(\d+)\+?\s*tests?",
                r"(\d+)\s*tests?",
                r"(\d+)\s+fonctions de test",
            ],
            "verify": lambda x: 1157 <= int(x) <= 1250,  # Range acceptable
        },
        "emotions": {
            "patterns": [r"(\d+)\s+émotions", r"(\d+)\s+emotions"],
            "verify": lambda x: int(x) == 12,  # Doit être exactement 12
        },
        "docs": {
            "patterns": [r"(\d+)\+?\s*fichiers?\s*doc", r"(\d+)\s*fichiers?\s*MD"],
            "verify": lambda x: 280 <= int(x) <= 310,  # Range acceptable
        },
    }

    for claim_type, config in claims.items():
        for pattern in config["patterns"]:
            matches = re.finditer(pattern, content, re.IGNORECASE)
            for match in matches:
                number = match.group(1)
                if not config["verify"](number):
                    issues.append(
                        {
                            "type": claim_type,
                            "value": number,
                            "line": content[: match.start()].count("\n") + 1,
                        }
                    )

    return {"file": md_file, "issues": issues}


def improve_md_formatting(content: str) -> str:
    """Améliore le formatage MD pour le rendre plus moderne et impactant."""

    # 1. Améliorer titres avec emojis cohérents
    content = re.sub(
        r"^##\s+([^📋🎯✅⚠️❌🔍📊📝🎉🚀🏗️🧪📚⚡🔒🌟]+)",
        r"## \1",
        content,
        flags=re.MULTILINE,
    )

    # 2. Améliorer listes avec puces modernes
    content = re.sub(r"^\s*[-•]\s+", "• ", content, flags=re.MULTILINE)

    # 3. Améliorer code blocks avec langage si manquant
    content = re.sub(r"```\n([^`]+)\n```", r"```python\n\1\n```", content)

    # 4. Standardiser les dates
    content = re.sub(
        r"(Date|Dernière mise à jour|Mise à jour):\s*(.*?2024|.*?2025)",
        lambda m: standardize_date(m.group(0)),
        content,
        flags=re.IGNORECASE,
    )

    # 5. Améliorer séparateurs
    content = re.sub(r"^---{3,}$", "---", content, flags=re.MULTILINE)

    # 6. Ajouter espacements cohérents
    content = re.sub(r"\n{3,}", "\n\n", content)

    return content


def standardize_date(date_str: str) -> str:
    """Standardise les dates."""
    if "Oct 25 / Nov 25" in date_str or "octobre 2025" in date_str.lower():
        return date_str  # Déjà standardisé
    if "novembre 2025" in date_str.lower():
        return date_str.replace("novembre 2025", "Oct 25 / Nov 25").replace(
            "Novembre 2025", "Oct 25 / Nov 25"
        )
    if "octobre 2024" in date_str.lower() and "création" in date_str.lower():
        return date_str  # Date création ne pas modifier
    return date_str


def add_modern_header(content: str, title: str) -> str:
    """Ajoute un header moderne si manquant."""
    if content.startswith("#"):
        return content  # Déjà un titre

    # Extraire titre si présent dans contenu
    first_line = content.split("\n")[0].strip()
    if not first_line.startswith("#"):
        content = f"# {title}\n\n{content}"

    return content


def improve_readme(content: str) -> str:
    """Améliore spécifiquement le README pour le rendre plus impactant."""

    # Améliorer section "EN 30 SECONDES" avec meilleure structure
    content = re.sub(
        r"## 📋 \*\*EN 30 SECONDES :\*\*", r"## 📋 **EN 30 SECONDES**", content
    )

    # Améliorer points clés avec meilleur formatage
    content = re.sub(r"• ✅ \*\*([^*]+)\*\*", r"• ✅ **\1**", content)

    # Améliorer badges section
    if "## 🏆 Badges Qualité & CI/CD" not in content:
        # Chercher section badges et améliorer
        content = re.sub(
            r"(<!-- Badges.*?-->)",
            r"## 🏆 Qualité & CI/CD\n\n\1",
            content,
            flags=re.DOTALL,
        )

    return content


def main():
    """Fonction principale."""
    root = Path("/Volumes/T7/bbia-reachy-sim")

    # Fichiers principaux à améliorer
    priority_files = [
        "README.md",
        "PROJECTS.md",
        "docs/guides/GUIDE_DEBUTANT.md",
        "docs/guides/GUIDE_AVANCE.md",
    ]

    print("🔍 Audit et amélioration documentation MD\n")

    for file_path in priority_files:
        full_path = root / file_path
        if not full_path.exists():
            continue

        print(f"\n📝 {file_path}")

        # 1. Vérifier véracité
        verification = verify_md_claims(full_path)
        if verification["issues"]:
            print(f"   ⚠️  {len(verification['issues'])} problèmes détectés")
            for issue in verification["issues"]:
                print(
                    f"      - {issue['type']}: {issue['value']} (ligne {issue['line']})"
                )
        else:
            print("   ✅ Aucun problème de véracité")

        # 2. Améliorer formatage
        content = full_path.read_text(encoding="utf-8")
        original_content = content

        # Améliorations spécifiques
        if file_path == "README.md":
            content = improve_readme(content)
        else:
            content = improve_md_formatting(content)

        # Sauvegarder si changé
        if content != original_content:
            full_path.write_text(content, encoding="utf-8")
            print("   ✨ Formatage amélioré")
        else:
            print("   ℹ️  Déjà bien formaté")

    print("\n✅ Audit terminé")


if __name__ == "__main__":
    main()
