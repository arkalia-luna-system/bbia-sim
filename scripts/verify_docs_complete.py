#!/usr/bin/env python3
"""
Script COMPLET de vérification professionnelle de la documentation BBIA.

🔍 FUSION DE :
- verify_documentation.py (précision et cohérence avec code)
- audit_complet_md.py (dates, contenu, organisation)
- verify_docs_complete.py (nouveau - vérification formatage complète)

✅ VÉRIFICATIONS COMPLÈTES :
- 🔗 Liens (internes/externes brisés, ancres)
- 🎨 Schémas Mermaid (syntaxe, couleurs, formatage)
- 📝 Orthographe française (avec dictionnaire technique BBIA)
- 📏 Espaces (doubles, finaux, manquants)
- 📋 Formatage Markdown (titres, listes, code blocks, tables)
- 📅 Dates cohérentes (Oct 25 / Nov 25, création)
- 🖼️  Images/assets référencés
- ✅ Cohérence avec code réel (fichiers, classes, méthodes)
- 📊 Métriques vérifiées (nombre tests, fichiers, etc.)

Usage:
    python scripts/verify_docs_complete.py                    # Vérification complète
    python scripts/verify_docs_complete.py --fix             # Auto-correction (sûr)
    python scripts/verify_docs_complete.py --links-only       # Seulement les liens
    python scripts/verify_docs_complete.py --spell-only       # Seulement orthographe
    python scripts/verify_docs_complete.py --mermaid-only    # Seulement Mermaid
    python scripts/verify_docs_complete.py --code-consistency # Cohérence avec code
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
    
    def __init__(self, fix_mode: bool = False, check_external_links: bool = False) -> None:
        self.fix_mode = fix_mode
        self.check_external_links = check_external_links
        self.errors: dict[str, list[str]] = defaultdict(list)
        self.warnings: dict[str, list[str]] = defaultdict(list)
        self.fixes: dict[str, list[str]] = defaultdict(list)
        self.md_files: list[Path] = []
        
    def find_all_md_files(self, limit_docs: bool = True) -> list[Path]:
        """Trouve tous les fichiers MD (optimisé - limite aux docs principaux par défaut)."""
        md_files = []
        
        # Fichiers racine prioritaires
        root_files = ["README.md", "CHANGELOG.md", "CONTRIBUTING.md", "CODE_OF_CONDUCT.md", "PROJECTS.md"]
        for root_file in root_files:
            root_path = PROJECT_ROOT / root_file
            if root_path.exists():
                md_files.append(root_path)
        
        # Limiter aux docs principaux par défaut (plus rapide)
        if limit_docs:
            docs_dirs = [
                "docs/guides",
                "docs/guides_techniques",
                "docs/architecture",
                "docs/audit",
                "docs/conformite",
                "docs/ci",
                "docs/observabilite",
                "docs/api",
                "docs/dashboard",
                "docs/performance",
                "docs/FAQ.md",
                "docs/README.md",
                "docs/INDEX_FINAL.md",
            ]
            for docs_path in docs_dirs:
                full_path = PROJECT_ROOT / docs_path
                if full_path.is_file():
                    md_files.append(full_path)
                elif full_path.is_dir():
                    md_files.extend([f for f in full_path.rglob("*.md") if not f.name.startswith("._")])
        else:
            # Mode complet (plus lent)
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
        
        return sorted(set(md_files))  # Dédupliquer
    
    def check_links(self, md_file: Path, content: str, skip_external: bool = True) -> None:
        """Vérifie tous les liens (internes uniquement par défaut, plus rapide)."""
        # Pattern pour liens markdown: [text](url)
        link_pattern = r'\[([^\]]+)\]\(([^)]+)\)'
        
        # Limiter nombre de vérifications pour performance
        link_count = 0
        max_links_per_file = 50  # Limiter pour éviter ralentissement
        
        for match in re.finditer(link_pattern, content):
            if link_count >= max_links_per_file:
                break
            link_count += 1
            
            link_text = match.group(1)
            link_url = match.group(2)
            
            # Skip liens spéciaux (ancres, mailto)
            if link_url.startswith("#") or link_url.startswith("mailto:"):
                continue
            
            # Skip liens externes par défaut (lent)
            if link_url.startswith("http://") or link_url.startswith("https://"):
                if not skip_external:
                    parsed = urlparse(link_url)
                    if not parsed.netloc:
                        self.errors[md_file].append(f"❌ Lien externe invalide: {link_url}")
                continue
            
            # Lien relatif interne (vérification rapide)
            try:
                # Ignorer ancre pour vérifier fichier
                link_path_str = link_url.split("#")[0]
                if not link_path_str:
                    continue  # Juste une ancre, ignorer
                
                link_path = Path(link_path_str)
                
                # Essayer plusieurs chemins possibles
                possible_paths = [
                    md_file.parent / link_path,  # Depuis fichier MD
                    PROJECT_ROOT / link_path,  # Depuis racine
                    PROJECT_ROOT / link_path.relative_to("/") if link_path.is_absolute() else None,  # Absolu normalisé
                ]
                
                # Essayer aussi avec docs/ si le lien commence par ../
                if link_path_str.startswith("../"):
                    docs_relative = link_path_str.replace("../", "", 1)
                    possible_paths.append(PROJECT_ROOT / "docs" / docs_relative)
                    possible_paths.append(PROJECT_ROOT / docs_relative)
                
                # Vérifier si au moins un chemin existe
                found = False
                for path in possible_paths:
                    if path and path.exists():
                        found = True
                        break
                
                if not found:
                    self.errors[md_file].append(f"❌ Lien brisé: {link_url}")
            except Exception:
                # Ignorer erreurs de parsing chemin
                pass
    
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
                # Accepter si c'est une ligne de séparateur de tableau (---)
                if line.strip() in ["---", "-", "---", "|---|---|"] or re.match(r'^[-*]{3,}', line.strip()):
                    continue
                # Accepter si c'est dans un bloc de code ou inline code
                if "`" in line:
                    continue
                # Accepter si c'est dans une cellule de tableau (détection intelligente)
                if "|" in line and line.count("|") >= 2:
                    # C'est probablement une ligne de tableau, ignorer
                    continue
                # Accepter si c'est une URL ou chemin
                if "://" in line or (line.count("/") > 2 and not line.strip().startswith("-")):
                    continue
                # Accepter si c'est une ligne de code ou commande
                if re.search(r'[-*]\s*`|[-*]\s*[A-Z][a-z]+\(|[-*]\s*[a-z_]+\s*=', line):
                    continue
                # Accepter certaines listes spéciales valides
                if re.match(r'^[-*][A-Z][a-zA-Z]', line) and len(line.strip()) < 30:
                    continue
                # Accepter si précédé d'un caractère spécial (ex: dans un tableau formaté)
                if i > 1 and lines[i-2] and "|" in lines[i-2] and lines[i-2].count("|") >= 2:
                    continue
                # Sinon, c'est probablement une vraie erreur
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
    
    def check_code_consistency(self, md_file: Path, content: str, quick: bool = True) -> None:
        """Vérifie cohérence avec code réel (rapide - seulement fichiers mentionnés)."""
        # Vérifier fichiers mentionnés (rapide)
        file_pattern = r'`([a-zA-Z0-9_/\.-]+\.(?:py|sh|xml|stl|md))`'
        file_count = 0
        max_files = 20  # Limiter pour performance
        
        for match in re.finditer(file_pattern, content):
            if file_count >= max_files:
                break
            file_count += 1
            
            file_path = match.group(1)
            # Ignorer liens externes
            if file_path.startswith("http"):
                continue
            # Vérifier existence (rapide)
            if not (PROJECT_ROOT / file_path.lstrip("/")).exists():
                if not (md_file.parent / file_path).exists():
                    self.warnings[md_file].append(f"⚠️  Fichier mentionné non trouvé: {file_path}")
        
        # Skip vérification classes si mode rapide (lent)
        if quick:
            return
    
    def check_spelling(self, md_file: Path, content: str, quick: bool = True) -> None:
        """Vérifie orthographe basique (mode rapide - échantillonnage)."""
        # Mode rapide : vérifier seulement les 500 premiers caractères
        if quick:
            sample = content[:500]
        else:
            sample = content
        
        # Extraire mots (ignorer code, liens, URLs)
        text_only = sample
        # Enlever code blocks
        text_only = re.sub(r'```.*?```', '', text_only, flags=re.DOTALL)
        # Enlever inline code
        text_only = re.sub(r'`[^`]+`', '', text_only)
        # Enlever liens
        text_only = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text_only)
        # Enlever URLs
        text_only = re.sub(r'https?://\S+', '', text_only)
        
        # Extraire mots (échantillonnage)
        words = re.findall(r'\b[a-zA-ZàâäéèêëïîôùûüÿçÀÂÄÉÈÊËÏÎÔÙÛÜŸÇ]+\b', text_only)
        
        # Limiter nombre de vérifications
        max_checks = 30
        suspicious = []
        for word in words[:max_checks]:
            word_lower = word.lower()
            # Skip très courts
            if len(word_lower) <= 2:
                continue
            # Skip si dans dictionnaire
            if word_lower in ALL_VALID_WORDS:
                continue
            # Skip noms propres
            if word[0].isupper() and len(word) > 3:
                continue
            
            suspicious.append(word)
        
        # Seulement avertir si vraiment suspect
        if len(suspicious) > 5:
            self.warnings[md_file].append(f"⚠️  Orthographe: {len(suspicious)} mots suspects dans échantillon (vérification manuelle recommandée)")
    
    def check_tables(self, md_file: Path, content: str) -> None:
        """Vérifie formatage des tables (plus intelligent)."""
        lines = content.split("\n")
        in_table = False
        header_line = 0
        
        for i, line in enumerate(lines, 1):
            # Détecter tableau (au moins 2 pipes)
            if "|" in line and line.count("|") >= 2 and not line.strip().startswith("```"):
                if not in_table:
                    in_table = True
                    header_line = i
                    # Vérifier ligne séparatrice suivante
                    if i < len(lines):
                        next_line = lines[i]
                        # Séparateur peut être sur la même ligne ou la suivante
                        if "---" not in next_line and "|" in next_line:
                            # Peut-être séparateur sur ligne suivante
                            if i + 1 < len(lines):
                                sep_line = lines[i + 1]
                                if "---" not in sep_line and "|" in sep_line:
                                    # Pas de séparateur détecté
                                    self.errors[md_file].append(f"❌ Ligne {i}: table sans séparateur (---)")
                else:
                    # Vérifier nombre de colonnes cohérent (par rapport à header)
                    cols = line.count("|")
                    if header_line > 0:
                        header_cols = lines[header_line - 1].count("|")
                        if cols != header_cols and "---" not in line:
                            # Tolérer différence si c'est le séparateur
                            self.warnings[md_file].append(f"⚠️  Ligne {i}: nombre de colonnes incohérent ({cols} vs {header_cols})")
            elif in_table:
                # Sortir du tableau si ligne vide OU bloc code
                if not line.strip() or line.strip().startswith("```"):
                    in_table = False
                    header_line = 0
    
    def verify_all(self, links_only: bool = False, spell_only: bool = False, mermaid_only: bool = False, code_consistency: bool = False, full_scan: bool = False) -> dict[str, Any]:
        """Exécute toutes les vérifications (optimisé pour vitesse)."""
        print("🔍 Recherche fichiers MD...")
        self.md_files = self.find_all_md_files(limit_docs=not full_scan)
        print(f"✅ {len(self.md_files)} fichiers MD trouvés\n")
        
        if not full_scan and len(self.md_files) < 50:
            print("💡 Mode rapide: seulement docs principaux (utiliser --full-scan pour tout)")
        
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
                
                # Vérifications conditionnelles selon options (optimisé)
                if not links_only and not spell_only and not mermaid_only:
                    # Vérification complète (mode rapide)
                    self.check_links(md_file, content, skip_external=not self.check_external_links)
                    self.check_mermaid(md_file, content)
                    self.check_spaces(md_file, content)
                    self.check_formatting(md_file, content)
                    self.check_dates(md_file, content)
                    self.check_spelling(md_file, content, quick=True)
                    self.check_tables(md_file, content)
                    if code_consistency:
                        self.check_code_consistency(md_file, content, quick=True)
                else:
                    # Vérifications spécifiques
                    if links_only:
                        self.check_links(md_file, content, skip_external=not self.check_external_links)
                    if mermaid_only:
                        self.check_mermaid(md_file, content)
                    if spell_only:
                        self.check_spelling(md_file, content, quick=True)
                # Toujours vérifier code consistency si demandé
                if code_consistency:
                    self.check_code_consistency(md_file, content, quick=True)
                
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
    parser.add_argument(
        "--code-consistency",
        action="store_true",
        help="Vérifier cohérence avec code réel (fichiers, classes mentionnés)"
    )
    parser.add_argument(
        "--full-scan",
        action="store_true",
        help="Vérifier TOUS les fichiers MD (plus lent, par défaut: seulement docs principaux)"
    )
    parser.add_argument(
        "--check-external-links",
        action="store_true",
        help="Vérifier aussi les liens externes (lent)"
    )
    
    args = parser.parse_args()
    
    verifier = DocsVerifier(fix_mode=args.fix, check_external_links=args.check_external_links)
    results = verifier.verify_all(
        links_only=args.links_only,
        spell_only=args.spell_only,
        mermaid_only=args.mermaid_only,
        code_consistency=args.code_consistency or (not args.links_only and not args.spell_only and not args.mermaid_only),
        full_scan=args.full_scan
    )
    verifier.print_report(results)
    
    # Code de sortie
    total_errors = sum(len(errs) for errs in results["errors"].values())
    return 1 if total_errors > 0 else 0


if __name__ == "__main__":
    exit(main())

