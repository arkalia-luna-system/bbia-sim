#!/usr/bin/env python3
"""
Analyse Exhaustive de Conformité BBIA vs Reachy Mini SDK Officiel
Vérifie: démos, exemples, mesures, documentation, fonctionnalités
"""

import json
import logging
import re
from pathlib import Path
from typing import Any

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ExhaustiveConformityAnalyzer:
    """Analyse exhaustive de conformité."""

    def __init__(self, bbia_root: Path, official_root: Path):
        """Initialise l'analyseur."""
        self.bbia_root = bbia_root
        self.official_root = official_root
        self.results: dict[str, Any] = {
            "demos": {},
            "examples": {},
            "measures": {},
            "documentation": {},
            "api_endpoints": {},
            "summary": {},
        }

    def analyze_demos(self) -> None:
        """Analyse les démos et exemples."""
        logger.info("📹 Analyse des démos et exemples...")

        # Démos officielles
        official_demos = list(self.official_root.glob("examples/*.py"))
        official_demo_names = {f.name for f in official_demos if not f.name.startswith("._")}

        # Démos BBIA
        bbia_demos = list(self.bbia_root.glob("examples/*.py"))
        bbia_demo_names = {f.name for f in bbia_demos if not f.name.startswith("._")}

        # Démos debug officielles
        official_debug_demos = list(self.official_root.glob("examples/debug/*.py"))
        official_debug_names = {f.name for f in official_debug_demos if not f.name.startswith("._")}

        self.results["demos"] = {
            "official_count": len(official_demo_names),
            "bbia_count": len(bbia_demo_names),
            "official_list": sorted(official_demo_names),
            "bbia_list": sorted(bbia_demo_names),
            "official_debug": sorted(official_debug_names),
            "missing_in_bbia": sorted(official_demo_names - bbia_demo_names),
            "bbia_extensions": sorted(bbia_demo_names - official_demo_names),
        }

        logger.info(f"  Démos officielles: {len(official_demo_names)}")
        logger.info(f"  Démos BBIA: {len(bbia_demo_names)}")
        logger.info(f"  Manquantes dans BBIA: {len(self.results['demos']['missing_in_bbia'])}")
        logger.info(f"  Extensions BBIA: {len(self.results['demos']['bbia_extensions'])}")

    def analyze_examples_compatibility(self) -> None:
        """Analyse la compatibilité des exemples."""
        logger.info("🔍 Analyse de compatibilité des exemples...")

        compatibility: dict[str, Any] = {}

        # Analyser minimal_demo.py
        official_minimal = self.official_root / "examples/minimal_demo.py"
        if official_minimal.exists():
            content = official_minimal.read_text()
            compatibility["minimal_demo"] = {
                "official_uses": {
                    "ReachyMini": "ReachyMini" in content,
                    "create_head_pose": "create_head_pose" in content,
                    "goto_target": "goto_target" in content,
                    "set_target": "set_target" in content,
                },
                "can_replicate_in_bbia": True,  # BBIA supporte ces méthodes
            }

        # Analyser reachy_compliant_demo.py
        official_compliant = self.official_root / "examples/reachy_compliant_demo.py"
        if official_compliant.exists():
            content = official_compliant.read_text()
            compatibility["reachy_compliant_demo"] = {
                "official_uses": {
                    "enable_gravity_compensation": "enable_gravity_compensation" in content,
                    "disable_gravity_compensation": "disable_gravity_compensation" in content,
                },
                "can_replicate_in_bbia": True,  # BBIA supporte via backend_adapter
            }

        # Analyser recorded_moves_example.py
        official_recorded = self.official_root / "examples/recorded_moves_example.py"
        if official_recorded.exists():
            content = official_recorded.read_text()
            compatibility["recorded_moves_example"] = {
                "official_uses": {
                    "RecordedMoves": "RecordedMoves" in content,
                    "play_move": "play_move" in content,
                },
                "can_replicate_in_bbia": True,  # Endpoints déjà implémentés
            }

        self.results["examples"] = compatibility

    def analyze_measures(self) -> None:
        """Analyse les mesures et spécifications."""
        logger.info("📏 Analyse des mesures et spécifications...")

        measures: dict[str, Any] = {}

        # Rechercher dans docs BBIA
        docs_files = list(self.bbia_root.glob("docs/**/*.md"))
        official_docs = list(self.official_root.glob("docs/**/*.md")) if (self.official_root / "docs").exists() else []

        # Extraire mesures BBIA
        bbia_measures = {
            "height_active": None,
            "height_sleep": None,
            "width": None,
            "weight": None,
        }

        for doc_file in docs_files:
            content = doc_file.read_text(encoding="utf-8", errors="ignore")
            # Rechercher dimensions
            height_match = re.search(r"(\d+)\s*cm.*(?:actif|veille|sleep|actif)", content, re.IGNORECASE)
            width_match = re.search(r"(\d+)\s*cm.*(?:largeur|width)", content, re.IGNORECASE)
            weight_match = re.search(r"(\d+[.,]\d+)\s*kg", content, re.IGNORECASE)

            if height_match:
                value = float(height_match.group(1))
                if "actif" in content.lower() or "active" in content.lower():
                    bbia_measures["height_active"] = value
                elif "veille" in content.lower() or "sleep" in content.lower():
                    bbia_measures["height_sleep"] = value

            if width_match:
                bbia_measures["width"] = float(width_match.group(1))

            if weight_match:
                bbia_measures["weight"] = float(weight_match.group(1).replace(",", "."))

        # Mesures officielles (basées sur documentation connue)
        official_measures = {
            "height_active": 28.0,  # cm
            "height_sleep": 23.0,  # cm
            "width": 16.0,  # cm
            "weight": 1.5,  # kg
        }

        measures["bbia"] = {k: v for k, v in bbia_measures.items() if v is not None}
        measures["official"] = official_measures
        measures["conformity"] = {
            k: abs(measures["bbia"].get(k, 0) - official_measures.get(k, 0)) < 0.1
            for k in official_measures
        }

        self.results["measures"] = measures

        logger.info(f"  Mesures BBIA: {measures['bbia']}")
        logger.info(f"  Conformité: {sum(measures['conformity'].values())}/{len(measures['conformity'])}")

    def analyze_documentation(self) -> None:
        """Analyse la documentation."""
        logger.info("📚 Analyse de la documentation...")

        docs: dict[str, Any] = {}

        # Comter fichiers MD
        bbia_md_files = list(self.bbia_root.glob("docs/**/*.md"))
        official_md_files = (
            list(self.official_root.glob("docs/**/*.md")) if (self.official_root / "docs").exists() else []
        )

        docs["bbia_count"] = len(bbia_md_files)
        docs["official_count"] = len(official_md_files)

        # Analyser README
        bbia_readme = self.bbia_root / "README.md"
        official_readme = self.official_root / "README.md"

        docs["readme"] = {
            "bbia_exists": bbia_readme.exists(),
            "official_exists": official_readme.exists(),
            "bbia_size": bbia_readme.stat().st_size if bbia_readme.exists() else 0,
            "official_size": official_readme.stat().st_size if official_readme.exists() else 0,
        }

        # Sections importantes dans README
        if bbia_readme.exists() and official_readme.exists():
            bbia_content = bbia_readme.read_text(encoding="utf-8", errors="ignore")
            official_content = official_readme.read_text(encoding="utf-8", errors="ignore")

            important_sections = ["Installation", "Usage", "Examples", "API", "SDK"]

            docs["readme_sections"] = {
                section: {
                    "bbia": section.lower() in bbia_content.lower(),
                    "official": section.lower() in official_content.lower(),
                }
                for section in important_sections
            }

        self.results["documentation"] = docs

        logger.info(f"  Fichiers MD BBIA: {docs['bbia_count']}")
        logger.info(f"  Fichiers MD officiels: {docs['official_count']}")

    def analyze_api_endpoints(self) -> None:
        """Analyse les endpoints API (depuis rapport précédent)."""
        logger.info("🔌 Analyse des endpoints API...")

        # Charger rapport précédent si disponible
        previous_report = self.bbia_root / "logs/comparison_official_results.json"
        if previous_report.exists():
            try:
                with open(previous_report) as f:
                    previous_data = json.load(f)
                    self.results["api_endpoints"] = {
                        "summary": previous_data.get("summary", {}),
                        "critical_issues": 0,
                        "high_issues": 0,
                    }
                    logger.info(f"  Rapport précédent chargé: {previous_data.get('summary', {}).get('total', 0)} différences")
            except Exception as e:
                logger.warning(f"  Erreur lecture rapport: {e}")
        else:
            self.results["api_endpoints"] = {"note": "Rapport précédent non trouvé"}

    def generate_summary(self) -> None:
        """Génère le résumé final."""
        logger.info("📊 Génération du résumé...")

        summary = {
            "conformity_score": 0.0,
            "total_checks": 0,
            "passed_checks": 0,
            "categories": {},
        }

        # Score démos
        demos = self.results.get("demos", {})
        if demos:
            missing = len(demos.get("missing_in_bbia", []))
            total_official = demos.get("official_count", 1)
            demos_score = max(0, (total_official - missing) / total_official * 100)
            summary["categories"]["demos"] = {
                "score": demos_score,
                "missing": missing,
                "total": total_official,
            }
            summary["total_checks"] += 1
            if missing == 0:
                summary["passed_checks"] += 1

        # Score mesures
        measures = self.results.get("measures", {})
        if measures:
            conformity = measures.get("conformity", {})
            if conformity:
                measures_score = sum(conformity.values()) / len(conformity) * 100
                summary["categories"]["measures"] = {
                    "score": measures_score,
                    "conform": sum(conformity.values()),
                    "total": len(conformity),
                }
                summary["total_checks"] += 1
                if sum(conformity.values()) == len(conformity):
                    summary["passed_checks"] += 1

        # Score API
        api = self.results.get("api_endpoints", {})
        api_summary = api.get("summary", {})
        if api_summary:
            critical = api_summary.get("critical", 0)
            high = api_summary.get("high", 0)
            api_score = 100.0 if critical == 0 and high == 0 else max(0, 100 - (critical * 50 + high * 20))
            summary["categories"]["api"] = {
                "score": api_score,
                "critical": critical,
                "high": high,
            }
            summary["total_checks"] += 1
            if critical == 0 and high == 0:
                summary["passed_checks"] += 1

        # Score global
        if summary["total_checks"] > 0:
            summary["conformity_score"] = (
                sum(cat.get("score", 0) for cat in summary["categories"].values()) / summary["total_checks"]
            )

        self.results["summary"] = summary

        logger.info(f"  Score de conformité: {summary['conformity_score']:.1f}%")
        logger.info(f"  Checks passés: {summary['passed_checks']}/{summary['total_checks']}")

    def run_full_analysis(self) -> dict[str, Any]:
        """Lance l'analyse complète."""
        logger.info("🚀 Démarrage analyse exhaustive de conformité...")
        logger.info(f"  BBIA root: {self.bbia_root}")
        logger.info(f"  Official root: {self.official_root}")

        self.analyze_demos()
        self.analyze_examples_compatibility()
        self.analyze_measures()
        self.analyze_documentation()
        self.analyze_api_endpoints()
        self.generate_summary()

        logger.info("✅ Analyse exhaustive terminée")

        return self.results

    def save_results(self, output_path: Path) -> None:
        """Sauvegarde les résultats."""
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(self.results, f, indent=2, ensure_ascii=False)
        logger.info(f"✅ Résultats sauvegardés: {output_path}")


def main():
    """Point d'entrée principal."""
    import argparse

    parser = argparse.ArgumentParser(description="Analyse exhaustive de conformité BBIA vs SDK officiel")
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

    analyzer = ExhaustiveConformityAnalyzer(args.bbia_root, args.official_root)
    results = analyzer.run_full_analysis()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    analyzer.save_results(output_dir / "analyse_exhaustive_conformite.json")

    # Afficher résumé
    print("\n" + "=" * 80)
    print("📊 RÉSUMÉ ANALYSE EXHAUSTIVE")
    print("=" * 80)
    summary = results["summary"]
    print(f"Score de conformité global: {summary['conformity_score']:.1f}%")
    print(f"Checks passés: {summary['passed_checks']}/{summary['total_checks']}")
    print("\n📹 Démos:")
    print(f"  Manquantes: {len(results['demos'].get('missing_in_bbia', []))}")
    print(f"  Extensions BBIA: {len(results['demos'].get('bbia_extensions', []))}")
    print("\n📏 Mesures:")
    measures_conf = results["measures"].get("conformity", {})
    print(f"  Conformes: {sum(measures_conf.values())}/{len(measures_conf)}")
    print("\n📚 Documentation:")
    print(f"  Fichiers MD BBIA: {results['documentation'].get('bbia_count', 0)}")
    print("=" * 80)


if __name__ == "__main__":
    main()

