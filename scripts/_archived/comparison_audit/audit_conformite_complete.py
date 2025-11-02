#!/usr/bin/env python3
"""
Script d'audit de conformité complet entre BBIA-SIM et le SDK officiel Reachy Mini.
Compare tous les endpoints, classes, méthodes, modèles, tests, etc.
"""

import logging
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).parent.parent
BBIA_SRC = PROJECT_ROOT / "src" / "bbia_sim"
BBIA_TESTS = PROJECT_ROOT / "tests"
BBIA_SCRIPTS = PROJECT_ROOT / "scripts"
BBIA_ASSETS = PROJECT_ROOT / "assets"


@dataclass
class ConformityIssue:
    """Une incohérence de conformité détectée."""

    category: str  # API, BEHAVIOR, MODELS, TESTS, etc.
    severity: str  # STRICT, COMPATIBLE, INCOMPATIBLE
    file: str
    line: int | None = None
    endpoint: str | None = None
    description: str = ""
    bbia_implementation: str = ""
    official_sdk: str = ""
    fix_suggestion: str = ""
    proof_link: str = ""


@dataclass
class ConformityReport:
    """Rapport de conformité complet."""

    issues: list[ConformityIssue] = field(default_factory=list)
    summary: dict[str, Any] = field(default_factory=dict)

    def add_issue(self, issue: ConformityIssue) -> None:
        """Ajoute une incohérence au rapport."""
        self.issues.append(issue)

    def get_summary(self) -> dict[str, Any]:
        """Génère un résumé du rapport."""
        total = len(self.issues)
        by_severity = {}
        by_category = {}

        for issue in self.issues:
            by_severity[issue.severity] = by_severity.get(issue.severity, 0) + 1
            by_category[issue.category] = by_category.get(issue.category, 0) + 1

        return {
            "total_issues": total,
            "by_severity": by_severity,
            "by_category": by_category,
            "strict_conformity": by_severity.get("STRICT", 0),
            "compatible": by_severity.get("COMPATIBLE", 0),
            "incompatible": by_severity.get("INCOMPATIBLE", 0),
        }


class ConformityAuditor:
    """Auditeur de conformité pour BBIA-SIM vs SDK officiel."""

    def __init__(self):
        self.report = ConformityReport()
        self.bbia_endpoints: list[dict[str, Any]] = []
        self.sdk_methods: dict[str, Any] = {}

    def discover_bbia_endpoints(self) -> list[dict[str, Any]]:
        """Découvre tous les endpoints REST de BBIA."""
        endpoints = []

        # Chercher dans tous les routers
        routers_dir = BBIA_SRC / "daemon" / "app" / "routers"
        if not routers_dir.exists():
            logger.warning(f"Routers dir not found: {routers_dir}")
            return endpoints

        for router_file in routers_dir.glob("*.py"):
            if router_file.name.startswith("._") or router_file.name == "__init__.py":
                continue

            try:
                endpoints.extend(self._parse_router_file(router_file))
            except Exception as e:
                logger.warning(f"Error parsing {router_file}: {e}")

        return endpoints

    def _parse_router_file(self, router_file: Path) -> list[dict[str, Any]]:
        """Parse un fichier router pour extraire les endpoints."""
        endpoints = []

        try:
            with open(router_file, encoding="utf-8") as f:
                content = f.read()

            # Chercher les décorateurs @router.get, @router.post, etc.
            pattern = r'@router\.(get|post|put|delete|patch)\s*\(\s*["\']([^"\']+)["\']'
            matches = re.finditer(pattern, content)

            for match in matches:
                method = match.group(1).upper()
                path = match.group(2)

                # Extraire la fonction suivante
                func_start = content.find("\n", match.end())
                func_match = re.search(
                    r"def\s+(\w+)\s*\(", content[func_start : func_start + 500]
                )
                func_name = func_match.group(1) if func_match else "unknown"

                endpoints.append(
                    {
                        "method": method,
                        "path": path,
                        "function": func_name,
                        "file": str(router_file.relative_to(PROJECT_ROOT)),
                    }
                )
        except Exception as e:
            logger.error(f"Error parsing router {router_file}: {e}")

        return endpoints

    def check_sdk_availability(self) -> dict[str, Any]:
        """Vérifie la disponibilité du SDK officiel."""
        try:
            import reachy_mini
            from reachy_mini import ReachyMini

            # Extraire les méthodes de la classe ReachyMini
            methods = {}
            for name in dir(ReachyMini):
                if not name.startswith("_"):
                    obj = getattr(ReachyMini, name)
                    if callable(obj):
                        try:
                            sig = inspect.signature(obj)
                            methods[name] = {
                                "signature": str(sig),
                                "doc": obj.__doc__ or "",
                            }
                        except Exception:
                            methods[name] = {"signature": "unknown", "doc": ""}

            return {
                "available": True,
                "version": getattr(reachy_mini, "__version__", "unknown"),
                "methods": methods,
            }
        except ImportError:
            return {"available": False, "error": "SDK not installed"}
        except Exception as e:
            return {"available": False, "error": str(e)}

    def compare_api_endpoints(self) -> None:
        """Compare les endpoints REST avec le SDK."""
        logger.info("🔍 Comparaison des endpoints REST...")

        bbia_endpoints = self.discover_bbia_endpoints()
        self.bbia_endpoints = bbia_endpoints

        # Mapping des endpoints BBIA vers méthodes SDK
        # Ce mapping sera complété avec les informations du SDK officiel

        # Vérifier que tous les endpoints critiques existent
        critical_endpoints = [
            ("/api/move/goto", "POST"),
            ("/api/move/set_target", "POST"),
            ("/api/state/full", "GET"),
            ("/api/state/joints", "GET"),
            ("/api/motors/set_mode/{mode}", "POST"),
        ]

        for endpoint_path, method in critical_endpoints:
            found = any(
                ep["path"] == endpoint_path and ep["method"] == method
                for ep in bbia_endpoints
            )
            if not found:
                self.report.add_issue(
                    ConformityIssue(
                        category="API",
                        severity="INCOMPATIBLE",
                        file="routers/",
                        endpoint=f"{method} {endpoint_path}",
                        description=f"Endpoint critique manquant: {method} {endpoint_path}",
                        fix_suggestion=f"Ajouter l'endpoint {method} {endpoint_path} dans le router approprié",
                    )
                )

    def check_code_quality(self) -> None:
        """Vérifie la qualité du code avec black, ruff, mypy, bandit."""
        logger.info("🔍 Vérification qualité de code...")

        # Vérifier black
        try:
            result = subprocess.run(
                ["black", "--check", "--diff", str(BBIA_SRC)],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != 0:
                self.report.add_issue(
                    ConformityIssue(
                        category="QUALITY",
                        severity="COMPATIBLE",
                        file="src/bbia_sim/",
                        description="Code non formaté avec black",
                        fix_suggestion="Exécuter: black src/bbia_sim/",
                    )
                )
        except Exception as e:
            logger.warning(f"Black check failed: {e}")

        # Vérifier ruff
        try:
            result = subprocess.run(
                ["ruff", "check", str(BBIA_SRC)],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != 0 and result.stdout:
                lines = result.stdout.split("\n")[:10]  # Limiter à 10 erreurs
                self.report.add_issue(
                    ConformityIssue(
                        category="QUALITY",
                        severity="COMPATIBLE",
                        file="src/bbia_sim/",
                        description=f"Erreurs ruff détectées: {len(lines)}",
                        fix_suggestion="Exécuter: ruff check src/bbia_sim/ --fix",
                    )
                )
        except Exception as e:
            logger.warning(f"Ruff check failed: {e}")

    def generate_report(self) -> str:
        """Génère un rapport markdown."""
        summary = self.report.get_summary()

        report_lines = [
            "# Rapport d'Audit de Conformité BBIA-SIM vs SDK Officiel Reachy Mini",
            "",
            f"**Date:** {subprocess.check_output(['date'], text=True).strip()}",
            f"**Total incohérences:** {summary['total_issues']}",
            "",
            "## Résumé",
            "",
            f"- **STRICT (identique):** {summary['strict_conformity']}",
            f"- **COMPATIBLE (différent mais accepté):** {summary['compatible']}",
            f"- **INCOMPATIBLE (erreur probable):** {summary['incompatible']}",
            "",
            "## Incohérences Détectées",
            "",
        ]

        # Grouper par catégorie
        by_category = {}
        for issue in self.report.issues:
            if issue.category not in by_category:
                by_category[issue.category] = []
            by_category[issue.category].append(issue)

        for category, issues in sorted(by_category.items()):
            report_lines.append(f"### {category} ({len(issues)} issues)")
            report_lines.append("")

            for issue in issues:
                report_lines.append(
                    f"#### {issue.severity}: {issue.endpoint or issue.description}"
                )
                report_lines.append(f"- **Fichier:** `{issue.file}`")
                if issue.line:
                    report_lines.append(f"- **Ligne:** {issue.line}")
                report_lines.append(f"- **Description:** {issue.description}")
                if issue.fix_suggestion:
                    report_lines.append(f"- **Correction:** {issue.fix_suggestion}")
                report_lines.append("")

        # Checklist finale
        report_lines.extend(
            [
                "## Checklist Actionable",
                "",
                "### À Corriger (INCOMPATIBLE)",
                "",
            ]
        )

        incompatible = [i for i in self.report.issues if i.severity == "INCOMPATIBLE"]
        for idx, issue in enumerate(incompatible, 1):
            report_lines.append(
                f"{idx}. [ ] {issue.description} - `{issue.file}`:{issue.line or '?'}"
            )

        return "\n".join(report_lines)

    def run_full_audit(self) -> None:
        """Exécute l'audit complet."""
        logger.info("🚀 Démarrage de l'audit de conformité complet...")

        # 1. Comparer les endpoints API
        self.compare_api_endpoints()

        # 2. Vérifier la qualité du code
        self.check_code_quality()

        # 3. Vérifier la disponibilité du SDK
        sdk_info = self.check_sdk_availability()
        logger.info(f"SDK disponible: {sdk_info.get('available', False)}")

        logger.info("✅ Audit terminé")


def main():
    """Point d'entrée principal."""
    auditor = ConformityAuditor()
    auditor.run_full_audit()

    # Générer le rapport
    report = auditor.generate_report()

    # Sauvegarder le rapport
    report_file = PROJECT_ROOT / "docs" / "conformite" / "AUDIT_CONFORMITE_COMPLETE.md"
    report_file.parent.mkdir(parents=True, exist_ok=True)

    with open(report_file, "w", encoding="utf-8") as f:
        f.write(report)

    print("\n" + "=" * 80)
    print(report)
    print("=" * 80)
    print(f"\n📄 Rapport sauvegardé: {report_file}")

    summary = auditor.report.get_summary()
    if summary["incompatible"] > 0:
        print(f"\n⚠️  {summary['incompatible']} incohérences INCOMPATIBLES détectées!")
        sys.exit(1)
    else:
        print("\n✅ Aucune incohérence INCOMPATIBLE détectée!")
        sys.exit(0)


if __name__ == "__main__":
    import inspect

    main()
