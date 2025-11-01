#!/usr/bin/env python3
"""
Script exhaustif de comparaison BBIA-SIM vs Repo Officiel reachy_mini

Compare TOUS les aspects:
- API REST endpoints
- Classes et modules Python
- Modèles MuJoCo
- Assets STL
- Tests
- Documentation
- Scripts
- Helpers

Usage:
    python scripts/compare_with_official_exhaustive.py
"""

import json
import logging
import subprocess
from collections import defaultdict
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


@dataclass
class Difference:
    """Une différence détectée entre BBIA et le repo officiel."""
    category: str  # API, Class, Model, Test, Doc, Script, Helper, Config
    file_path: str
    line: int | None
    endpoint: str | None
    description: str
    bbia_value: Any | None
    official_value: Any | None
    severity: str  # CRITICAL, HIGH, MEDIUM, LOW, INFO
    status: str  # pending, corrected, skipped
    correction: str | None
    test_file: str | None


@dataclass
class ComparisonResult:
    """Résultat d'une comparaison complète."""
    differences: list[Difference]
    summary: dict[str, int]
    checklist: dict[str, list[Difference]]


class OfficialRepoComparator:
    """Comparateur exhaustif BBIA vs repo officiel."""
    
    def __init__(self, bbia_root: Path, official_root: Path):
        self.bbia_root = Path(bbia_root)
        self.official_root = Path(official_root)
        self.differences: list[Difference] = []
        
        # Vérifier que les repos existent
        if not self.bbia_root.exists():
            raise ValueError(f"BBIA root non trouvé: {bbia_root}")
        if not self.official_root.exists():
            raise ValueError(f"Repo officiel non trouvé: {official_root}")
        
        logger.info(f"BBIA root: {self.bbia_root}")
        logger.info(f"Official root: {self.official_root}")
    
    def compare_directory_structure(self) -> list[Difference]:
        """Compare la structure des dossiers."""
        logger.info("📁 Comparaison structure des dossiers...")
        diffs = []
        
        # Dossiers à comparer
        dirs_to_check = [
            "src",
            "tests",
            "docs",
            "scripts",
            "examples",
        ]
        
        for dir_name in dirs_to_check:
            bbia_dir = self.bbia_root / dir_name
            official_dir = self.official_root / dir_name
            
            if not official_dir.exists():
                diffs.append(Difference(
                    category="Structure",
                    file_path=f"{dir_name}/",
                    line=None,
                    endpoint=None,
                    description=f"Dossier {dir_name} absent dans repo officiel (OK si normal)",
                    bbia_value=str(bbia_dir.exists()),
                    official_value=None,
                    severity="INFO",
                    status="pending",
                    correction=None,
                    test_file=None,
                ))
                continue
            
            # Lister fichiers dans chaque
            bbia_files = set(f.name for f in bbia_dir.rglob("*.py") if f.is_file()) if bbia_dir.exists() else set()
            official_files = set(f.name for f in official_dir.rglob("*.py") if f.is_file())
            
            missing_in_bbia = official_files - bbia_files
            extra_in_bbia = bbia_files - official_files
            
            for missing in missing_in_bbia:
                if missing != "__init__.py":  # Ignorer __init__.py
                    diffs.append(Difference(
                        category="Structure",
                        file_path=f"{dir_name}/{missing}",
                        line=None,
                        endpoint=None,
                        description=f"Fichier présent dans repo officiel mais absent dans BBIA",
                        bbia_value=None,
                        official_value=missing,
                        severity="MEDIUM",
                        status="pending",
                        correction=f"Vérifier si nécessaire d'ajouter {missing}",
                        test_file=None,
                    ))
        
        return diffs
    
    def compare_rest_api_endpoints(self) -> list[Difference]:
        """Compare les endpoints REST API."""
        logger.info("🌐 Comparaison endpoints REST API...")
        diffs = []
        
        # Chercher les routers FastAPI dans BBIA
        bbia_routers = list(self.bbia_root.glob("src/bbia_sim/daemon/app/routers/*.py"))
        
        # Chercher les routers dans le repo officiel
        official_routers = list(self.official_root.glob("**/routers/*.py"))
        official_routers.extend(self.official_root.glob("**/*router*.py"))
        official_routers.extend(self.official_root.glob("**/*api*.py"))
        
        # Extraire endpoints de BBIA
        bbia_endpoints = set()
        for router_file in bbia_routers:
            try:
                content = router_file.read_text()
                # Chercher @router.get, @router.post, etc.
                import re
                matches = re.findall(r'@router\.(get|post|put|delete|patch)\(["\']([^"\']+)["\']', content)
                for method, path in matches:
                    bbia_endpoints.add(f"{method.upper()} {path}")
            except Exception as e:
                logger.warning(f"Erreur lecture {router_file}: {e}")
        
        # Extraire endpoints du repo officiel
        official_endpoints = set()
        for router_file in official_routers:
            try:
                content = router_file.read_text()
                import re
                matches = re.findall(r'@(app|router)\.(get|post|put|delete|patch)\(["\']([^"\']+)["\']', content)
                for _, method, path in matches:
                    official_endpoints.add(f"{method.upper()} {path}")
            except Exception as e:
                logger.debug(f"Erreur lecture {router_file}: {e}")
        
        # Comparer
        missing_in_bbia = official_endpoints - bbia_endpoints
        extra_in_bbia = bbia_endpoints - official_endpoints
        
        for endpoint in missing_in_bbia:
            diffs.append(Difference(
                category="API",
                file_path="src/bbia_sim/daemon/app/routers/",
                line=None,
                endpoint=endpoint,
                description=f"Endpoint présent dans repo officiel mais absent dans BBIA",
                bbia_value=None,
                official_value=endpoint,
                severity="HIGH",
                status="pending",
                correction=f"Ajouter endpoint {endpoint} si nécessaire",
                test_file=None,
            ))
        
        for endpoint in extra_in_bbia:
            diffs.append(Difference(
                category="API",
                file_path="src/bbia_sim/daemon/app/routers/",
                line=None,
                endpoint=endpoint,
                description=f"Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)",
                bbia_value=endpoint,
                official_value=None,
                severity="INFO",
                status="pending",
                correction=None,
                test_file=None,
            ))
        
        logger.info(f"  Endpoints BBIA: {len(bbia_endpoints)}")
        logger.info(f"  Endpoints officiels: {len(official_endpoints)}")
        logger.info(f"  Différences détectées: {len(missing_in_bbia)} manquants, {len(extra_in_bbia)} supplémentaires")
        
        return diffs
    
    def compare_python_classes(self) -> list[Difference]:
        """Compare les classes Python principales."""
        logger.info("🐍 Comparaison classes Python...")
        diffs = []
        
        # Chercher classe ReachyMini dans repo officiel
        official_reachy_mini = self.official_root / "src" / "reachy_mini" / "__init__.py"
        official_main = self.official_root / "src" / "reachy_mini" / "reachy_mini.py"
        
        # Chercher backend BBIA
        bbia_backend = self.bbia_root / "src" / "bbia_sim" / "backends" / "reachy_mini_backend.py"
        
        if official_main.exists():
            try:
                official_content = official_main.read_text()
                
                # Extraire méthodes de la classe ReachyMini
                import ast
                import re
                
                # Méthodes extraites par regex (plus simple)
                method_pattern = r'def\s+(\w+)\s*\('
                official_methods = set(re.findall(method_pattern, official_content))
                
                if bbia_backend.exists():
                    bbia_content = bbia_backend.read_text()
                    bbia_methods = set(re.findall(method_pattern, bbia_content))
                    
                    missing_methods = official_methods - bbia_methods
                    for method in missing_methods:
                        if not method.startswith("_"):  # Ignorer méthodes privées
                            diffs.append(Difference(
                                category="Class",
                                file_path=str(bbia_backend.relative_to(self.bbia_root)),
                                line=None,
                                endpoint=None,
                                description=f"Méthode {method} présente dans SDK officiel mais absente dans BBIA backend",
                                bbia_value=None,
                                official_value=method,
                                severity="HIGH",
                                status="pending",
                                correction=f"Vérifier si {method} doit être implémentée",
                                test_file=None,
                            ))
                
            except Exception as e:
                logger.error(f"Erreur analyse classes: {e}")
        
        return diffs
    
    def compare_mujoco_models(self) -> list[Difference]:
        """Compare les modèles MuJoCo."""
        logger.info("🎮 Comparaison modèles MuJoCo...")
        diffs = []
        
        # Chercher modèles officiels
        official_models = list(self.official_root.glob("**/*.xml"))
        official_models = [m for m in official_models if "mjcf" in str(m).lower() or "reachy" in m.name.lower()]
        
        # Chercher modèles BBIA
        bbia_models = list(self.bbia_root.glob("src/bbia_sim/sim/models/*.xml"))
        
        if official_models and bbia_models:
            # Comparer le modèle principal
            official_main = None
            for m in official_models:
                if "reachy_mini.xml" in m.name or "main" in m.name.lower():
                    official_main = m
                    break
            
            bbia_main = self.bbia_root / "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
            
            if official_main and bbia_main.exists():
                try:
                    official_content = official_main.read_text()
                    bbia_content = bbia_main.read_text()
                    
                    # Comparer joints
                    import re
                    official_joints = set(re.findall(r'<joint name="([^"]+)"', official_content))
                    bbia_joints = set(re.findall(r'<joint name="([^"]+)"', bbia_content))
                    
                    missing_joints = official_joints - bbia_joints
                    extra_joints = bbia_joints - official_joints
                    
                    for joint in missing_joints:
                        diffs.append(Difference(
                            category="Model",
                            file_path=str(bbia_main.relative_to(self.bbia_root)),
                            line=None,
                            endpoint=None,
                            description=f"Joint {joint} présent dans modèle officiel mais absent dans BBIA",
                            bbia_value=None,
                            official_value=joint,
                            severity="HIGH",
                            status="pending",
                            correction=f"Vérifier si joint {joint} doit être ajouté",
                            test_file=None,
                        ))
                    
                except Exception as e:
                    logger.error(f"Erreur comparaison modèles: {e}")
        
        return diffs
    
    def compare_tests(self) -> list[Difference]:
        """Compare les tests."""
        logger.info("🧪 Comparaison tests...")
        diffs = []
        
        # Chercher tests officiels
        official_tests = list(self.official_root.glob("tests/**/*.py"))
        official_test_names = {f.stem for f in official_tests if f.is_file()}
        
        # Chercher tests BBIA
        bbia_tests = list(self.bbia_root.glob("tests/**/*.py"))
        bbia_test_names = {f.stem for f in bbia_tests if f.is_file()}
        
        # Tests manquants dans BBIA
        missing_tests = official_test_names - bbia_test_names
        
        for test_name in missing_tests:
            # Vérifier si c'est un test pertinent (pas juste setup/config)
            if "test" in test_name.lower() or "conform" in test_name.lower():
                diffs.append(Difference(
                    category="Test",
                    file_path=f"tests/{test_name}.py",
                    line=None,
                    endpoint=None,
                    description=f"Test {test_name} présent dans repo officiel mais absent dans BBIA",
                    bbia_value=None,
                    official_value=test_name,
                    severity="MEDIUM",
                    status="pending",
                    correction=f"Vérifier si test {test_name} doit être ajouté",
                    test_file=None,
                ))
        
        logger.info(f"  Tests officiels: {len(official_test_names)}")
        logger.info(f"  Tests BBIA: {len(bbia_test_names)}")
        logger.info(f"  Tests manquants: {len(missing_tests)}")
        
        return diffs
    
    def compare_documentation(self) -> list[Difference]:
        """Compare la documentation."""
        logger.info("📚 Comparaison documentation...")
        diffs = []
        
        # Chercher README officiel
        official_readme = self.official_root / "README.md"
        bbia_readme = self.bbia_root / "README.md"
        
        if official_readme.exists() and bbia_readme.exists():
            official_content = official_readme.read_text()
            bbia_content = bbia_readme.read_text()
            
            # Chercher sections importantes
            sections_to_check = [
                "Installation",
                "Usage",
                "API",
                "Examples",
                "Testing",
            ]
            
            for section in sections_to_check:
                if section.lower() in official_content.lower() and section.lower() not in bbia_content.lower():
                    diffs.append(Difference(
                        category="Doc",
                        file_path="README.md",
                        line=None,
                        endpoint=None,
                        description=f"Section '{section}' présente dans README officiel mais absente dans BBIA",
                        bbia_value=None,
                        official_value=section,
                        severity="LOW",
                        status="pending",
                        correction=f"Vérifier si section '{section}' doit être ajoutée",
                        test_file=None,
                    ))
        
        return diffs
    
    def run_full_comparison(self) -> ComparisonResult:
        """Exécute la comparaison complète."""
        logger.info("🚀 Démarrage comparaison exhaustive BBIA vs Repo Officiel")
        logger.info("=" * 80)
        
        all_diffs = []
        
        # 1. Structure
        all_diffs.extend(self.compare_directory_structure())
        
        # 2. API REST
        all_diffs.extend(self.compare_rest_api_endpoints())
        
        # 3. Classes Python
        all_diffs.extend(self.compare_python_classes())
        
        # 4. Modèles MuJoCo
        all_diffs.extend(self.compare_mujoco_models())
        
        # 5. Tests
        all_diffs.extend(self.compare_tests())
        
        # 6. Documentation
        all_diffs.extend(self.compare_documentation())
        
        # Résumé
        summary = {
            "total": len(all_diffs),
            "critical": len([d for d in all_diffs if d.severity == "CRITICAL"]),
            "high": len([d for d in all_diffs if d.severity == "HIGH"]),
            "medium": len([d for d in all_diffs if d.severity == "MEDIUM"]),
            "low": len([d for d in all_diffs if d.severity == "LOW"]),
            "info": len([d for d in all_diffs if d.severity == "INFO"]),
        }
        
        # Checklist par catégorie
        checklist = defaultdict(list)
        for diff in all_diffs:
            checklist[diff.category].append(diff)
        
        return ComparisonResult(
            differences=all_diffs,
            summary=summary,
            checklist=dict(checklist),
        )
    
    def save_results(self, result: ComparisonResult, output_file: Path):
        """Sauvegarde les résultats dans un fichier JSON."""
        output_file.parent.mkdir(parents=True, exist_ok=True)
        
        data = {
            "summary": result.summary,
            "differences": [asdict(d) for d in result.differences],
            "checklist": {
                cat: [asdict(d) for d in diffs]
                for cat, diffs in result.checklist.items()
            },
        }
        
        with open(output_file, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False, default=str)
        
        logger.info(f"✅ Résultats sauvegardés: {output_file}")
    
    def generate_markdown_report(self, result: ComparisonResult, output_file: Path):
        """Génère un rapport Markdown."""
        output_file.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_file, "w", encoding="utf-8") as f:
            f.write("# 🔍 Rapport de Comparaison Exhaustive - BBIA vs Repo Officiel\n\n")
            f.write(f"**Date**: {Path(__file__).stat().st_mtime}\n\n")
            f.write("## 📊 Résumé Exécutif\n\n")
            f.write(f"- **Total différences**: {result.summary['total']}\n")
            f.write(f"- **🔴 CRITICAL**: {result.summary['critical']}\n")
            f.write(f"- **🟠 HIGH**: {result.summary['high']}\n")
            f.write(f"- **🟡 MEDIUM**: {result.summary['medium']}\n")
            f.write(f"- **🟢 LOW**: {result.summary['low']}\n")
            f.write(f"- **ℹ️ INFO**: {result.summary['info']}\n\n")
            
            f.write("## 📋 Checklist par Catégorie\n\n")
            for category, diffs in sorted(result.checklist.items()):
                f.write(f"### {category} ({len(diffs)} différences)\n\n")
                for i, diff in enumerate(diffs, 1):
                    f.write(f"{i}. **{diff.severity}** - {diff.description}\n")
                    f.write(f"   - Fichier: `{diff.file_path}`\n")
                    if diff.endpoint:
                        f.write(f"   - Endpoint: `{diff.endpoint}`\n")
                    if diff.line:
                        f.write(f"   - Ligne: {diff.line}\n")
                    if diff.correction:
                        f.write(f"   - Correction: {diff.correction}\n")
                    f.write(f"   - Statut: {diff.status}\n\n")
            
            f.write("## ✅ Actions Recommandées\n\n")
            f.write("### Priorité CRITICAL\n")
            critical_diffs = [d for d in result.differences if d.severity == "CRITICAL"]
            for diff in critical_diffs:
                f.write(f"- [ ] {diff.description}\n")
            
            f.write("\n### Priorité HIGH\n")
            high_diffs = [d for d in result.differences if d.severity == "HIGH"]
            for diff in high_diffs[:10]:  # Limiter à 10
                f.write(f"- [ ] {diff.description}\n")
            if len(high_diffs) > 10:
                f.write(f"- ... et {len(high_diffs) - 10} autres\n")
        
        logger.info(f"✅ Rapport Markdown généré: {output_file}")


def main():
    """Point d'entrée principal."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Comparaison exhaustive BBIA vs Repo Officiel")
    parser.add_argument(
        "--bbia-root",
        type=Path,
        default=Path(__file__).parent.parent,
        help="Racine du projet BBIA",
    )
    parser.add_argument(
        "--official-root",
        type=Path,
        default=Path("/Volumes/T7/reachy_mini"),
        help="Racine du repo officiel",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("logs"),
        help="Répertoire de sortie",
    )
    
    args = parser.parse_args()
    
    # Créer le comparateur
    comparator = OfficialRepoComparator(args.bbia_root, args.official_root)
    
    # Exécuter la comparaison
    result = comparator.run_full_comparison()
    
    # Sauvegarder les résultats
    output_dir = args.output_dir
    comparator.save_results(result, output_dir / "comparison_official_results.json")
    comparator.generate_markdown_report(result, output_dir / "comparison_official_report.md")
    
    # Afficher le résumé
    logger.info("\n" + "=" * 80)
    logger.info("📊 RÉSUMÉ DE LA COMPARAISON")
    logger.info("=" * 80)
    logger.info(f"Total différences: {result.summary['total']}")
    logger.info(f"  🔴 CRITICAL: {result.summary['critical']}")
    logger.info(f"  🟠 HIGH: {result.summary['high']}")
    logger.info(f"  🟡 MEDIUM: {result.summary['medium']}")
    logger.info(f"  🟢 LOW: {result.summary['low']}")
    logger.info(f"  ℹ️ INFO: {result.summary['info']}")
    
    logger.info(f"\n📁 Rapports générés dans: {output_dir}")
    logger.info(f"  - JSON: {output_dir / 'comparison_official_results.json'}")
    logger.info(f"  - Markdown: {output_dir / 'comparison_official_report.md'}")


if __name__ == "__main__":
    main()

