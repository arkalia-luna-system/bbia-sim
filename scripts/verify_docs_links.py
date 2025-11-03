#!/usr/bin/env python3
"""Script pour vérifier tous les liens dans les fichiers MD de la documentation.

Vérifie que tous les liens internes pointent vers des fichiers existants.
"""

import re
import subprocess
from pathlib import Path
from typing import List, Tuple

DOCS_ROOT = Path(__file__).parent.parent / "docs"

def extract_links(content: str, file_path: Path) -> List[Tuple[str, str]]:
    """Extrait tous les liens markdown d'un fichier."""
    links = []
    
    # Pattern pour liens markdown [texte](chemin)
    pattern = r'\[([^\]]+)\]\(([^)]+)\)'
    
    for match in re.finditer(pattern, content):
        link_text = match.group(1)
        link_path = match.group(2)
        
        # Ignorer les liens externes (http, https, mailto)
        if not link_path.startswith(('http://', 'https://', 'mailto:', '#', '..')):
            links.append((link_text, link_path))
    
    return links

def check_link_exists(link_path: str, from_file: Path) -> Tuple[bool, str]:
    """Vérifie si un lien pointe vers un fichier existant."""
    # Résoudre le chemin relatif
    if link_path.startswith('/'):
        target = DOCS_ROOT / link_path.lstrip('/')
    else:
        target = (from_file.parent / link_path).resolve()
    
    # Vérifier si le fichier existe
    if target.exists() and target.is_file():
        return True, ""
    
    # Si c'est un lien vers un dossier, chercher README.md ou index.md
    if target.is_dir():
        for readme in ['README.md', 'index.md', 'INDEX.md']:
            readme_path = target / readme
            if readme_path.exists():
                return True, f"(dossier avec {readme})"
        return False, "dossier sans README"
    
    return False, "fichier introuvable"

def verify_docs_links() -> int:
    """Vérifie tous les liens dans la documentation."""
    print("🔍 Vérification des liens dans la documentation...\n")
    
    errors = []
    warnings = []
    total_links = 0
    
    # Parcourir tous les fichiers MD
    for md_file in DOCS_ROOT.rglob("*.md"):
        if '.git' in str(md_file) or 'venv' in str(md_file):
            continue
        
        content = md_file.read_text(encoding='utf-8', errors='ignore')
        links = extract_links(content, md_file)
        
        for link_text, link_path in links:
            total_links += 1
            exists, reason = check_link_exists(link_path, md_file)
            
            if not exists:
                rel_path = md_file.relative_to(DOCS_ROOT)
                errors.append((rel_path, link_text, link_path, reason))
            elif reason:
                rel_path = md_file.relative_to(DOCS_ROOT)
                warnings.append((rel_path, link_text, link_path, reason))
    
    # Afficher les résultats
    print(f"📊 Statistiques:")
    print(f"   Total liens vérifiés: {total_links}")
    print(f"   ✅ Liens valides: {total_links - len(errors) - len(warnings)}")
    print(f"   ⚠️  Avertissements: {len(warnings)}")
    print(f"   ❌ Erreurs: {len(errors)}\n")
    
    if warnings:
        print("⚠️  Avertissements (liens vers dossiers sans README):")
        for file_path, link_text, link_path, reason in warnings[:10]:
            print(f"   {file_path}: [{link_text}]({link_path}) - {reason}")
        if len(warnings) > 10:
            print(f"   ... et {len(warnings) - 10} autres avertissements")
        print()
    
    if errors:
        print("❌ Erreurs (liens cassés):")
        for file_path, link_text, link_path, reason in errors[:20]:
            print(f"   {file_path}: [{link_text}]({link_path}) - {reason}")
        if len(errors) > 20:
            print(f"   ... et {len(errors) - 20} autres erreurs")
        print()
    else:
        print("✅ Aucune erreur trouvée !\n")
    
    return len(errors)

if __name__ == "__main__":
    exit(verify_docs_links())

