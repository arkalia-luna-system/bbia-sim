#!/usr/bin/env python3
"""Script pour corriger les liens MD cassés dans les archives."""

import re
from pathlib import Path

# Chemins relatifs depuis docs/archives vers docs/
def fix_link(match: re.Match) -> str:
    """Corrige un lien markdown."""
    full_link = match.group(0)
    link_text = match.group(1)
    link_path = match.group(2)
    
    # Ignorer les liens HTTP/HTTPS
    if link_path.startswith(('http://', 'https://')):
        return full_link
    
    # Ignorer les liens vers fichiers externes
    if link_path.startswith('/') or '://' in link_path:
        return full_link
    
    # Ignorer les ancres (#)
    if link_path.startswith('#'):
        return full_link
    
    # Liens relatifs depuis archives vers docs/
    if not link_path.startswith('docs/') and not link_path.startswith('../'):
        # Si c'est un lien .md ou .html dans docs/, ajouter ../../
        if link_path.endswith(('.md', '.html')):
            if not link_path.startswith('../'):
                # Ajouter ../../
                new_path = '../../' + link_path
                return f"[{link_text}]({new_path})"
    
    return full_link

def fix_file(file_path: Path) -> int:
    """Corrige les liens dans un fichier."""
    try:
        content = file_path.read_text(encoding='utf-8')
        original_content = content
        
        # Pattern pour [text](link)
        pattern = r'\[([^\]]+)\]\(([^)]+)\)'
        
        # Corriger les liens
        content = re.sub(pattern, fix_link, content)
        
        # Corriger les liens cassés vers docs/ (sans ../)
        # Ex: [Guide](docs/guides/GUIDE.md) -> [Guide](../../guides/GUIDE.md)
        content = re.sub(
            r'\]\(docs/([^)]+)\)',
            r'](../../\1)',
            content
        )
        
        if content != original_content:
            file_path.write_text(content, encoding='utf-8')
            return 1
        return 0
    except Exception as e:
        print(f"Erreur {file_path}: {e}")
        return 0

def main():
    """Corrige tous les fichiers MD dans docs/archives."""
    archives_dir = Path(__file__).parent.parent / "docs" / "archives"
    
    if not archives_dir.exists():
        print(f"❌ Répertoire introuvable: {archives_dir}")
        return
    
    md_files = list(archives_dir.rglob("*.md"))
    print(f"📁 Trouvé {len(md_files)} fichiers MD dans archives")
    
    fixed_count = 0
    for md_file in md_files:
        if fix_file(md_file):
            fixed_count += 1
            print(f"✅ Corrigé: {md_file.relative_to(archives_dir)}")
    
    print(f"\n✅ {fixed_count} fichiers corrigés sur {len(md_files)}")

if __name__ == "__main__":
    main()

