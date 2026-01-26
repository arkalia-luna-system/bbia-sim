#!/usr/bin/env python3
"""
Audit complet des fichiers Markdown - 26 Janvier 2026
Vérifie:
1. Dates obsolètes (doivent être mises à jour à 26 Janvier 2026 si pertinentes)
2. Versions cohérentes (1.4.0)
3. Redondances entre fichiers
4. Fichiers obsolètes à archiver
5. Liens brisés
6. Cohérence des informations
"""

import re
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Set, Tuple
from datetime import datetime

# Configuration
CURRENT_DATE = "26 Janvier 2026"
CURRENT_VERSION = "1.4.0"
ROOT_DIR = Path(__file__).parent.parent

# Patterns de dates à vérifier
DATE_PATTERNS = [
    r"Dernière mise à jour.*?(\d{1,2}\s+\w+\s+\d{4})",
    r"dernière mise à jour.*?(\d{1,2}\s+\w+\s+\d{4})",
    r"mise à jour.*?(\d{1,2}\s+\w+\s+\d{4})",
    r"Mise à jour.*?(\d{1,2}\s+\w+\s+\d{4})",
]

# Dates qui doivent être mises à jour (avant janvier 2026)
OBSOLETE_DATES = [
    "8 Décembre 2025",
    "15 Décembre 2025",
    "22 Décembre 2025",
    "Décembre 2025",
    "Novembre 2025",
    "Octobre 2025",
]

# Dates qui sont correctes (historiques ou spécifiques)
CORRECT_DATES = [
    "18 Décembre 2025",  # Réception robot
    "20 Décembre 2025",  # Montage robot
    "17 Janvier 2026",   # Réception moteurs
    "21 Janvier 2026",   # Vérification QC
    "26 Janvier 2026",   # Date actuelle
    "20 Janvier 2026",   # Analyse repo
]


def find_all_md_files(root_dir: Path) -> List[Path]:
    """Trouve tous les fichiers MD."""
    md_files = []
    for path in root_dir.rglob("*.md"):
        # Ignorer venv, .git, fichiers cachés, archives
        parts = path.parts
        if any(
            part.startswith(".")
            or part == "venv"
            or part == "__pycache__"
            or "_archive" in part
            or "_archived" in part
            for part in parts
        ):
            continue
        if path.name.startswith("._"):
            continue
        md_files.append(path)
    return sorted(md_files)


def extract_dates_from_file(file_path: Path) -> List[Tuple[str, int]]:
    """Extrait toutes les dates d'un fichier."""
    dates = []
    try:
        content = file_path.read_text(encoding="utf-8")
        for pattern in DATE_PATTERNS:
            for match in re.finditer(pattern, content, re.IGNORECASE):
                date_str = match.group(1)
                line_num = content[: match.start()].count("\n") + 1
                dates.append((date_str, line_num))
    except Exception as e:
        print(f"⚠️  Erreur lecture {file_path}: {e}")
    return dates


def check_version_consistency(file_path: Path) -> List[Tuple[str, int]]:
    """Vérifie la cohérence des versions."""
    issues = []
    try:
        content = file_path.read_text(encoding="utf-8")
        lines = content.split("\n")
        
        # Chercher toutes les mentions de version
        version_pattern = r"version\s*[:=]\s*([0-9]+\.[0-9]+\.[0-9]+)"
        for i, line in enumerate(lines, 1):
            matches = re.finditer(version_pattern, line, re.IGNORECASE)
            for match in matches:
                version = match.group(1)
                if version != CURRENT_VERSION and version not in ["1.3.0", "1.3.1", "1.3.2"]:
                    # Accepter les versions historiques dans CHANGELOG/RELEASE_NOTES
                    if "CHANGELOG" not in file_path.name and "RELEASE_NOTES" not in file_path.name:
                        issues.append((f"Version {version} trouvée (attendu {CURRENT_VERSION})", i))
    except Exception as e:
        print(f"⚠️  Erreur vérification version {file_path}: {e}")
    return issues


def find_duplicate_content(files: List[Path]) -> Dict[str, List[Path]]:
    """Trouve les contenus dupliqués entre fichiers."""
    content_hash = defaultdict(list)
    
    for file_path in files:
        try:
            content = file_path.read_text(encoding="utf-8")
            # Normaliser le contenu (supprimer espaces, dates variables)
            normalized = re.sub(r"\d{1,2}\s+\w+\s+\d{4}", "DATE", content)
            normalized = re.sub(r"\s+", " ", normalized)
            # Prendre les 500 premiers caractères comme signature
            signature = normalized[:500]
            if len(signature) > 100:  # Ignorer les fichiers trop courts
                content_hash[signature].append(file_path)
        except Exception:
            pass
    
    # Retourner seulement les duplications
    return {sig: paths for sig, paths in content_hash.items() if len(paths) > 1}


def check_links(file_path: Path) -> List[Tuple[str, int]]:
    """Vérifie les liens internes."""
    broken_links = []
    try:
        content = file_path.read_text(encoding="utf-8")
        lines = content.split("\n")
        
        # Pattern pour liens markdown
        link_pattern = r"\[([^\]]+)\]\(([^)]+)\)"
        
        for i, line in enumerate(lines, 1):
            for match in re.finditer(link_pattern, line):
                link_text = match.group(1)
                link_path = match.group(2)
                
                # Ignorer les liens externes
                if link_path.startswith("http"):
                    continue
                
                # Ignorer les ancres
                if link_path.startswith("#"):
                    continue
                
                # Résoudre le chemin relatif
                if link_path.startswith("../") or link_path.startswith("./"):
                    target = (file_path.parent / link_path).resolve()
                else:
                    target = (file_path.parent / link_path).resolve()
                
                # Vérifier si le fichier existe
                if not target.exists():
                    broken_links.append((f"Lien brisé: {link_path}", i))
    except Exception as e:
        print(f"⚠️  Erreur vérification liens {file_path}: {e}")
    return broken_links


def audit_file(file_path: Path) -> Dict[str, Any]:
    """Audit complet d'un fichier."""
    issues = {
        "obsolete_dates": [],
        "version_issues": [],
        "broken_links": [],
    }
    
    # Vérifier les dates
    dates = extract_dates_from_file(file_path)
    for date_str, line_num in dates:
        # Vérifier si la date est obsolète
        is_obsolete = False
        for obsolete in OBSOLETE_DATES:
            if obsolete.lower() in date_str.lower():
                is_obsolete = True
                break
        
        if is_obsolete:
            # Vérifier si c'est une date historique correcte
            is_correct_historical = False
            for correct in CORRECT_DATES:
                if correct.lower() in date_str.lower():
                    is_correct_historical = True
                    break
            
            if not is_correct_historical:
                issues["obsolete_dates"].append((date_str, line_num))
    
    # Vérifier les versions
    issues["version_issues"] = check_version_consistency(file_path)
    
    # Vérifier les liens
    issues["broken_links"] = check_links(file_path)
    
    return issues


def main():
    """Fonction principale."""
    print("🔍 Audit complet des fichiers Markdown - 26 Janvier 2026\n")
    
    md_files = find_all_md_files(ROOT_DIR)
    print(f"📁 {len(md_files)} fichiers MD trouvés\n")
    
    all_issues = defaultdict(list)
    files_with_issues = []
    
    # Auditer chaque fichier
    for file_path in md_files:
        relative_path = file_path.relative_to(ROOT_DIR)
        issues = audit_file(file_path)
        
        total_issues = (
            len(issues["obsolete_dates"])
            + len(issues["version_issues"])
            + len(issues["broken_links"])
        )
        
        if total_issues > 0:
            files_with_issues.append((relative_path, issues))
            all_issues["obsolete_dates"].extend(
                [(relative_path, date, line) for date, line in issues["obsolete_dates"]]
            )
            all_issues["version_issues"].extend(
                [(relative_path, msg, line) for msg, line in issues["version_issues"]]
            )
            all_issues["broken_links"].extend(
                [(relative_path, msg, line) for msg, line in issues["broken_links"]]
            )
    
    # Afficher le résumé
    print("=" * 80)
    print("📊 RÉSUMÉ DE L'AUDIT")
    print("=" * 80)
    print(f"\n✅ Fichiers audités : {len(md_files)}")
    print(f"⚠️  Fichiers avec problèmes : {len(files_with_issues)}")
    print(f"\n📅 Dates obsolètes : {len(all_issues['obsolete_dates'])}")
    print(f"🔢 Problèmes de version : {len(all_issues['version_issues'])}")
    print(f"🔗 Liens brisés : {len(all_issues['broken_links'])}")
    
    # Afficher les détails
    if all_issues["obsolete_dates"]:
        print("\n" + "=" * 80)
        print("📅 DATES OBSOLÈTES À METTRE À JOUR")
        print("=" * 80)
        for file_path, date, line in all_issues["obsolete_dates"][:20]:  # Limiter à 20
            print(f"  {file_path}:{line} - Date: {date}")
        if len(all_issues["obsolete_dates"]) > 20:
            print(f"  ... et {len(all_issues['obsolete_dates']) - 20} autres")
    
    if all_issues["version_issues"]:
        print("\n" + "=" * 80)
        print("🔢 PROBLÈMES DE VERSION")
        print("=" * 80)
        for file_path, msg, line in all_issues["version_issues"][:10]:
            print(f"  {file_path}:{line} - {msg}")
    
    if all_issues["broken_links"]:
        print("\n" + "=" * 80)
        print("🔗 LIENS BRISÉS")
        print("=" * 80)
        for file_path, msg, line in all_issues["broken_links"][:10]:
            print(f"  {file_path}:{line} - {msg}")
    
    # Chercher les duplications
    print("\n" + "=" * 80)
    print("🔄 RECHERCHE DE CONTENUS DUPLIQUÉS")
    print("=" * 80)
    duplicates = find_duplicate_content(md_files)
    if duplicates:
        print(f"⚠️  {len(duplicates)} groupes de fichiers avec contenu similaire trouvés")
        for sig, paths in list(duplicates.items())[:5]:  # Limiter à 5
            print(f"\n  Fichiers similaires ({len(paths)}):")
            for path in paths:
                print(f"    - {path.relative_to(ROOT_DIR)}")
    else:
        print("✅ Aucune duplication majeure détectée")
    
    print("\n" + "=" * 80)
    print("✅ Audit terminé")
    print("=" * 80)
    
    # Sauvegarder le rapport
    report_path = ROOT_DIR / "artifacts" / "audit_md_janvier2026.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    
    import json
    report = {
        "date_audit": CURRENT_DATE,
        "total_files": len(md_files),
        "files_with_issues": len(files_with_issues),
        "issues": {
            "obsolete_dates": len(all_issues["obsolete_dates"]),
            "version_issues": len(all_issues["version_issues"]),
            "broken_links": len(all_issues["broken_links"]),
        },
        "details": {
            "obsolete_dates": [
                {"file": str(f), "date": d, "line": l}
                for f, d, l in all_issues["obsolete_dates"]
            ],
            "version_issues": [
                {"file": str(f), "message": m, "line": l}
                for f, m, l in all_issues["version_issues"]
            ],
            "broken_links": [
                {"file": str(f), "message": m, "line": l}
                for f, m, l in all_issues["broken_links"]
            ],
        },
    }
    
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"\n📄 Rapport sauvegardé : {report_path}")


if __name__ == "__main__":
    main()
