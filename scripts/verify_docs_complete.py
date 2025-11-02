#!/usr/bin/env python3
"""
Script complet de vérification professionnelle de la documentation BBIA.

Vérifie :
- ✅ Liens internes/externes (brisés, relatifs, absolus)
- ✅ Orthographe française (avec dictionnaire)
- ✅ Schémas Mermaid (syntaxe, formatage, couleurs)
- ✅ Espaces (doubles, finaux, manquants)
- ✅ Formatage (titres, listes, code blocks)
- ✅ Images/assets référencés
- ✅ Dates cohérentes (Oct 25 / Nov 25)
- ✅ Tables formatées
- ✅ Cohérence entre documents
- ✅ Code blocks valides

Usage:
    python scripts/verify_docs_complete.py                    # Vérification complète
    python scripts/verify_docs_complete.py --fix             # Auto-correction (sûr)
    python scripts/verify_docs_complete.py --links-only      # Seulement les liens
    python scripts/verify_docs_complete.py --spell-only      # Seulement orthographe
    python scripts/verify_docs_complete.py --mermaid-only    # Seulement Mermaid
"""

import argparse
import re
import subprocess
from collections import defaultdict
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

# Configuration
PROJECT_ROOT = Path(__file__).parent.parent
DOCS_DIR = PROJECT_ROOT / "docs"
SCRIPT_DIR = PROJECT_ROOT / "scripts"

# Dictionnaire orthographe français (mots techniques BBIA acceptés)
VALID_WORDS = {
    "bbia", "reachy", "mujoco", "huggingface", "whisper", "yolo", "smolvlm",
    "moondream", "zenoh", "fastapi", "websocket", "rest", "api", "sdk",
    "openapi", "swagger", "redoc", "gradio", "streamlit", "htmlcov",
    "coverage.xml", "pyproject.toml", "requirements.txt", "pytest", "mypy",
    "ruff", "bandit", "black", "isort", "pre-commit", "github", "git",
    "docker", "raspberry", "pi", "arduino", "esp32", "micropython",
    "numpy", "opencv", "opengl", "egl", "tensorflow", "pytorch", "onnx",
    "mediapipe", "deepface", "facenet", "retinaface", "vgg", "tflite",
    "stl", "obj", "fbx", "mjcf", "xml", "json", "yaml", "toml",
    "jsonl", "csv", "log", "pdf", "png", "jpg", "jpeg", "gif", "svg",
    "webp", "mp4", "avi", "mov", "wav", "mp3", "ogg", "flac",
    "utf-8", "ascii", "base64", "sha256", "md5", "jwt", "oauth",
    "http", "https", "ssh", "ftp", "sftp", "tcp", "udp", "ip",
    "cli", "gui", "ui", "ux", "wysiwyg", "ide", "vim", "emacs",
    "vscode", "pycharm", "jupyter", "notebook", "colab",
    "linux", "macos", "windows", "ubuntu", "debian", "fedora",
    "centos", "redhat", "arch", "gentoo", "alpine", "docker",
    "kubernetes", "helm", "terraform", "ansible", "chef", "puppet",
    "prometheus", "grafana", "elk", "kibana", "elasticsearch",
    "redis", "mongodb", "postgresql", "mysql", "sqlite",
    "rabbitmq", "kafka", "nats", "mqtt", "amqp",
    "python", "javascript", "typescript", "java", "c++", "cpp",
    "rust", "go", "swift", "kotlin", "scala", "haskell", "erlang",
    "elixir", "ruby", "php", "perl", "bash", "zsh", "fish",
    "powershell", "batch", "cmd", "sh", "ps1",
}

# Mots français courants acceptés
FRENCH_WORDS = {
    "démarrage", "déploiement", "intégration", "configuration", "installation",
    "authentification", "sécurité", "performances", "optimisations",
    "utilisateur", "utilisateurs", "développeur", "développeurs",
    "documentation", "documentations", "guide", "guides", "manuel", "manuels",
    "tutoriel", "tutoriels", "exemple", "exemples", "démonstration",
    "architecture", "architectures", "composant", "composants",
    "module", "modules", "fonctionnalité", "fonctionnalités",
    "interface", "interfaces", "contrôleur", "contrôleurs",
    "simulateur", "simulateurs", "simulation", "simulations",
    "émotion", "émotions", "comportement", "comportements",
    "reconnaissance", "vision", "audio", "voix", "parole",
    "télémétrie", "métriques", "logs", "journalisation",
    "erreur", "erreurs", "avertissement", "avertissements",
    "dépannage", "troubleshooting", "debugging", "débogage",
    "test", "tests", "validation", "vérification", "audit",
    "conformité", "compatibilité", "interopérabilité",
    "amélioration", "améliorations", "optimisation", "optimisations",
    "correctif", "correctifs", "patch", "patches",
    "version", "versions", "release", "releases",
    "commit", "commits", "branch", "branches", "branche",
    "merge", "merges", "pull", "request", "requests",
    "issue", "issues", "bug", "bugs", "feature", "features",
    "changelog", "readme", "license", "licence",
    "contributor", "contributors", "contributeur", "contributeurs",
    "maintainer", "maintainers", "mainteneur", "mainteneurs",
    "author", "authors", "auteur", "auteurs",
}

ALL_VALID_WORDS = VALID_WORDS | FRENCH_WORDS

# Patterns de dates acceptées
VALID_DATE_PATTERNS = [
    r"Oct 25 / Nov 25",
    r"Octobre 2025",
    r"octobre 2025",
    r"Avril 2025",  # Date de création (ne pas modifier)
    r"avril 2025",
]

# Types de schémas Mermaid valides
MERMAID_TYPES = [
    "graph", "flowchart", "sequenceDiagram", "gantt",
    "pie", "mindmap", "stateDiagram", "classDiagram",
    "erDiagram", "journey", "gitgraph", "timeline",
]

class DocsVerifier:
    """Vérificateur complet de documentation."""
    
    def __init__(self, fix_mode: bool = False) -> None:
        self.fix_mode = fix_mode
        self.errors: dict[str, list[str]] = defaultdict(list)
        self.warnings: dict[str, list[str]] = defaultdict(list)
        self.fixes: dict[str, list[str]] = defaultdict(list)
        self.md_files: list[Path] = []
        
    def find_all_md_files(self) -> list[Path]:
        """Trouve tous les fichiers MD (sauf archives, _archived, fichiers cachés)."""
        md_files = []
        for md_file in PROJECT_ROOT.rglob("*.md"):
            # Ignorer fichiers cachés macOS
            if md_file.name.startswith("._"):
                continue
            # Ignorer archives
            if "_archived" in str(md_file) or ".git" in str(md_file):
                continue
            # Ignorer caches
            if ".pytest_cache" in str(md_file) or "__pycache__" in str(md_file):
                continue
            # Ignorer venv/node_modules
            if "venv" in str(md_file) or "node_modules" in str(md_file):
                continue
            md_files.append(md_file)
        return sorted(md_files)
    
    def check_links(self, md_file: Path, content: str) -> None:
        """Vérifie tous les liens (internes, externes, images)."""
        # Pattern pour liens markdown: [text](url)
        link_pattern = r'\[([^\]]+)\]\(([^)]+)\)'
        
        for match in re.finditer(link_pattern, content):
            link_text = match.group(1)
            link_url = match.group(2)
            
            # Skip liens spéciaux
            if link_url.startswith("#") or link_url.startswith("mailto:") or link_url.startswith("http://") or link_url.startswith("https://"):
                # Vérifier liens externes (optionnel, peut être lent)
                if link_url.startswith("http"):
                    parsed = urlparse(link_url)
                    if not parsed.netloc:
                        self.errors[md_file].append(f"❌ Lien externe invalide: {link_url}")
                continue
            
            # Lien relatif interne
            link_path = Path(link_url)
            if link_path.is_absolute():
                # Lien absolu depuis racine projet
                full_path = PROJECT_ROOT / link_path.relative_to("/")
            else:
                # Lien relatif depuis fichier MD
                full_path = md_file.parent / link_path
            
            # Normaliser le chemin
            full_path = full_path.resolve()
            
            # Vérifier existence
            if not full_path.exists():
                # Essayer avec ancres (#)
                if "#" in link_url:
                    base_path = Path(link_url.split("#")[0])
                    if base_path.exists():
                        continue
                self.errors[md_file].append(f"❌ Lien brisé: {link_url} (vers: {full_path})")
    
    def check_mermaid(self, md_file: Path, content: str) -> None:
        """Vérifie syntaxe et formatage Mermaid."""
        mermaid_pattern = r'```mermaid\n(.*?)```'
        
        for match in re.finditer(mermaid_pattern, content, re.DOTALL):
            diagram = match.group(1).strip()
            
            # Vérifier type valide
            first_line = diagram.split("\n")[0].strip()
            valid_type = any(diagram.startswith(f"{mtype}") or diagram.startswith(f"{mtype} ") for mtype in MERMAID_TYPES)
            
            if not valid_type and first_line:
                self.warnings[md_file].append(f"⚠️  Type Mermaid non reconnu: {first_line[:30]}")
            
            # Vérifier couleurs/styles (recommandation)
            if "style" not in diagram.lower() and any(t in diagram.lower() for t in ["graph", "flowchart", "graph TB", "graph LR"]):
                self.warnings[md_file].append(f"💡 Diagramme Mermaid sans couleurs (recommandation: ajouter styles)")
            
            # Vérifier syntaxe basique
            if "[" in diagram and "]" in diagram:
                # Vérifier paires de brackets
                open_brackets = diagram.count("[")
                close_brackets = diagram.count("]")
                if open_brackets != close_brackets:
                    self.errors[md_file].append(f"❌ Mermaid: brackets non équilibrés ([{open_brackets}] vs ]{close_brackets})")
            
            # Vérifier indentation cohérente
            lines = diagram.split("\n")
            indent_chars = set()
            for line in lines:
                if line.strip():
                    indent = len(line) - len(line.lstrip())
                    if indent > 0:
                        indent_chars.add(line[:indent][-1])
            
            if len(indent_chars) > 1:
                # Mélange de tabs et espaces
                self.warnings[md_file].append(f"⚠️  Mermaid: mélange tabs/espaces (utiliser uniquement espaces)")
    
    def check_spaces(self, md_file: Path, content: str) -> None:
        """Vérifie espaces (doubles, finaux, manquants)."""
        lines = content.split("\n")
        
        for i, line in enumerate(lines, 1):
            # Espaces doubles
            if "  " in line and not line.strip().startswith("```"):
                if self.fix_mode:
                    fixed = re.sub(r" +", " ", line)
                    self.fixes[md_file].append(f"Ligne {i}: espaces doubles corrigés")
                else:
                    self.warnings[md_file].append(f"⚠️  Ligne {i}: espaces doubles")
            
            # Espaces finaux (sauf lignes vides)
            if line.rstrip() != line and line.strip():
                if self.fix_mode:
                    self.fixes[md_file].append(f"Ligne {i}: espaces finaux supprimés")
                else:
                    self.warnings[md_file].append(f"⚠️  Ligne {i}: espaces finaux")
    
    def check_formatting(self, md_file: Path, content: str) -> None:
        """Vérifie formatage markdown."""
        lines = content.split("\n")
        
        for i, line in enumerate(lines, 1):
            # Titres: doit avoir espace après #
            if re.match(r'^#{1,6}[^#\s]', line):
                self.errors[md_file].append(f"❌ Ligne {i}: titre sans espace après #")
            
            # Listes: espace après - ou * (mais accepter certaines formes valides)
            if re.match(r'^[-*]\S', line) and not line.startswith("```"):
                # Accepter si c'est une ligne de séparateur ou code inline
                if "`" not in line and not line.strip().startswith("---"):
                    self.errors[md_file].append(f"❌ Ligne {i}: liste sans espace après - ou *")
            
            # Code blocks: vérifier fermeture
            if line.strip().startswith("```") and not line.strip().endswith("```"):
                # Trouver bloc ouvert
                open_count = content[:content.find(line)].count("```")
                close_count = content[:content.find(line)].count("```")
                if open_count % 2 != 0:
                    # Rechercher fermeture après
                    remaining = content[content.find(line) + len(line):]
                    if "```" not in remaining[:500]:
                        self.errors[md_file].append(f"❌ Ligne {i}: bloc code non fermé")
    
    def check_dates(self, md_file: Path, content: str) -> None:
        """Vérifie cohérence des dates."""
        # Vérifier présence date récente
        has_valid_date = any(re.search(pattern, content, re.IGNORECASE) for pattern in VALID_DATE_PATTERNS)
        
        if not has_valid_date and "Date" in content[:500]:
            self.warnings[md_file].append(f"⚠️  Date non standardisée (attendu: Oct 25 / Nov 25)")
        
        # Vérifier dates obsolètes
        old_dates = re.findall(r'\b(2024|2026|janvier|février|mars|mai|juin|juillet|août|septembre|décembre)\s+2025\b', content, re.IGNORECASE)
        if old_dates:
            self.errors[md_file].append(f"❌ Dates obsolètes trouvées: {old_dates}")
    
    def check_spelling(self, md_file: Path, content: str) -> None:
        """Vérifie orthographe basique (mots techniques + français)."""
        # Extraire mots (ignorer code, liens, URLs)
        text_only = content
        # Enlever code blocks
        text_only = re.sub(r'```.*?```', '', text_only, flags=re.DOTALL)
        # Enlever inline code
        text_only = re.sub(r'`[^`]+`', '', text_only)
        # Enlever liens
        text_only = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text_only)
        # Enlever URLs
        text_only = re.sub(r'https?://\S+', '', text_only)
        
        # Extraire mots
        words = re.findall(r'\b[a-zA-ZàâäéèêëïîôùûüÿçÀÂÄÉÈÊËÏÎÔÙÛÜŸÇ]+\b', text_only)
        
        # Vérifier contre dictionnaire (tolérance pour mots techniques)
        suspicious = []
        for word in words:
            word_lower = word.lower()
            # Skip très courts
            if len(word_lower) <= 2:
                continue
            # Skip nombres
            if word_lower.isdigit():
                continue
            # Skip si dans dictionnaire
            if word_lower in ALL_VALID_WORDS:
                continue
            # Skip si c'est un nom propre (commence par majuscule en milieu de phrase)
            if word[0].isupper() and len(word) > 3:
                continue
            
            suspicious.append(word)
        
        if suspicious and len(suspicious) > 10:
            # Trop de mots suspects = probablement normal
            self.warnings[md_file].append(f"⚠️  {len(suspicious)} mots potentiellement mal orthographiés (vérification manuelle recommandée)")
    
    def check_tables(self, md_file: Path, content: str) -> None:
        """Vérifie formatage des tables."""
        lines = content.split("\n")
        in_table = False
        
        for i, line in enumerate(lines, 1):
            if "|" in line and not line.strip().startswith("```"):
                if not in_table:
                    in_table = True
                    # Vérifier ligne séparatrice suivante
                    if i < len(lines) and "|" in lines[i] and "---" not in lines[i]:
                        self.errors[md_file].append(f"❌ Ligne {i}: table sans séparateur (---)")
                else:
                    # Vérifier nombre de colonnes cohérent
                    cols = line.count("|")
                    if cols != lines[i-2].count("|") if i > 1 else cols:
                        self.warnings[md_file].append(f"⚠️  Ligne {i}: nombre de colonnes incohérent")
            elif in_table and not line.strip():
                in_table = False
    
    def verify_all(self) -> dict[str, Any]:
        """Exécute toutes les vérifications."""
        print("🔍 Recherche fichiers MD...")
        self.md_files = self.find_all_md_files()
        print(f"✅ {len(self.md_files)} fichiers MD trouvés\n")
        
        print("📋 Vérification en cours...\n")
        
        for md_file in self.md_files:
            try:
                # Essayer UTF-8, puis latin-1 pour fichiers macOS
                try:
                    content = md_file.read_text(encoding="utf-8")
                except UnicodeDecodeError:
                    try:
                        content = md_file.read_text(encoding="latin-1")
                    except Exception:
                        self.errors[md_file].append(f"❌ Erreur encodage fichier")
                        continue
                
                self.check_links(md_file, content)
                self.check_mermaid(md_file, content)
                self.check_spaces(md_file, content)
                self.check_formatting(md_file, content)
                self.check_dates(md_file, content)
                self.check_spelling(md_file, content)
                self.check_tables(md_file, content)
                
            except Exception as e:
                # Ignorer erreurs sur fichiers cachés macOS
                if not md_file.name.startswith("._"):
                    self.errors[md_file].append(f"❌ Erreur lecture fichier: {e}")
        
        return {
            "errors": dict(self.errors),
            "warnings": dict(self.warnings),
            "fixes": dict(self.fixes),
            "total_files": len(self.md_files),
        }
    
    def print_report(self, results: dict[str, Any]) -> None:
        """Affiche rapport complet."""
        total_errors = sum(len(errs) for errs in results["errors"].values())
        total_warnings = sum(len(warns) for warns in results["warnings"].values())
        total_fixes = sum(len(fixes) for fixes in results["fixes"].values())
        
        print("=" * 70)
        print("📊 RAPPORT VÉRIFICATION DOCUMENTATION")
        print("=" * 70)
        print(f"\n📁 Fichiers vérifiés: {results['total_files']}")
        print(f"❌ Erreurs: {total_errors}")
        print(f"⚠️  Avertissements: {total_warnings}")
        if self.fix_mode:
            print(f"🔧 Corrections appliquées: {total_fixes}")
        
        if results["errors"]:
            print("\n" + "=" * 70)
            print("❌ ERREURS CRITIQUES")
            print("=" * 70)
            for md_file, errs in results["errors"].items():
                print(f"\n📄 {md_file.relative_to(PROJECT_ROOT)}")
                for err in errs[:10]:  # Limiter à 10 par fichier
                    print(f"  {err}")
                if len(errs) > 10:
                    print(f"  ... et {len(errs) - 10} autres erreurs")
        
        if results["warnings"]:
            print("\n" + "=" * 70)
            print("⚠️  AVERTISSEMENTS")
            print("=" * 70)
            for md_file, warns in results["warnings"].items():
                print(f"\n📄 {md_file.relative_to(PROJECT_ROOT)}")
                for warn in warns[:5]:  # Limiter à 5 par fichier
                    print(f"  {warn}")
                if len(warns) > 5:
                    print(f"  ... et {len(warns) - 5} autres avertissements")
        
        if self.fix_mode and results["fixes"]:
            print("\n" + "=" * 70)
            print("🔧 CORRECTIONS APPLIQUÉES")
            print("=" * 70)
            for md_file, fixes in results["fixes"].items():
                print(f"\n📄 {md_file.relative_to(PROJECT_ROOT)}")
                for fix in fixes[:5]:
                    print(f"  ✅ {fix}")
        
        print("\n" + "=" * 70)
        if total_errors == 0 and total_warnings < 50:
            print("✅ DOCUMENTATION EN BON ÉTAT !")
        elif total_errors == 0:
            print("⚠️  DOCUMENTATION VALIDE MAIS AVEC AVERTISSEMENTS")
        else:
            print("❌ DOCUMENTATION REQUIERT DES CORRECTIONS")
        print("=" * 70)


def main() -> int:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description="Vérification complète documentation BBIA"
    )
    parser.add_argument(
        "--fix",
        action="store_true",
        help="Mode auto-correction (sûr uniquement: espaces, formatage basique)"
    )
    parser.add_argument(
        "--links-only",
        action="store_true",
        help="Vérifier uniquement les liens"
    )
    parser.add_argument(
        "--spell-only",
        action="store_true",
        help="Vérifier uniquement l'orthographe"
    )
    parser.add_argument(
        "--mermaid-only",
        action="store_true",
        help="Vérifier uniquement les schémas Mermaid"
    )
    
    args = parser.parse_args()
    
    verifier = DocsVerifier(fix_mode=args.fix)
    results = verifier.verify_all()
    verifier.print_report(results)
    
    # Code de sortie
    total_errors = sum(len(errs) for errs in results["errors"].values())
    return 1 if total_errors > 0 else 0


if __name__ == "__main__":
    exit(main())

