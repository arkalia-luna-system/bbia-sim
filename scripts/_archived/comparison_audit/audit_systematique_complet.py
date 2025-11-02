#!/usr/bin/env python3
"""
Audit systématique complet BBIA-SIM vs SDK officiel reachy_mini.
Compare TOUS les fichiers, endpoints, classes, méthodes, modèles, tests, etc.
"""

import ast
import logging
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).parent.parent
OFFICIAL_REPO = PROJECT_ROOT.parent / "reachy_mini"
BBIA_SRC = PROJECT_ROOT / "src" / "bbia_sim"
BBIA_TESTS = PROJECT_ROOT / "tests"
BBIA_SCRIPTS = PROJECT_ROOT / "scripts"
BBIA_DOCS = PROJECT_ROOT / "docs"
BBIA_ASSETS = PROJECT_ROOT / "assets"

if not OFFICIAL_REPO.exists():
    logger.error(f"Repo officiel non trouvé: {OFFICIAL_REPO}")
    sys.exit(1)


@dataclass
class ConformityItem:
    """Un élément à vérifier pour conformité."""

    category: str  # API, CLASS, MODEL, TEST, DOC, SCRIPT, etc.
    nature: str  # endpoint, method, class, file, etc.
    bbia_path: str
    official_path: str | None = None
    line: int | None = None
    status: str = "PENDING"  # OK, DIFF, MISSING, EXTRA
    description: str = ""
    fix_applied: bool = False
    test_result: str = "NOT_TESTED"  # PASSED, FAILED, NOT_TESTED, SKIPPED
    qa_ok: bool = False


@dataclass
class AuditReport:
    """Rapport d'audit complet."""

    items: list[ConformityItem] = field(default_factory=list)

    def add_item(self, item: ConformityItem) -> None:
        """Ajoute un élément au rapport."""
        self.items.append(item)

    def get_summary(self) -> dict[str, Any]:
        """Génère un résumé."""
        total = len(self.items)
        by_status = {}
        by_category = {}

        for item in self.items:
            by_status[item.status] = by_status.get(item.status, 0) + 1
            by_category[item.category] = by_category.get(item.category, 0) + 1

        return {
            "total": total,
            "by_status": by_status,
            "by_category": by_category,
            "fixed": sum(1 for i in self.items if i.fix_applied),
            "tested": sum(1 for i in self.items if i.test_result == "PASSED"),
        }


class SystematicAuditor:
    """Auditeur systématique comparant BBIA vs SDK officiel."""

    def __init__(self):
        self.report = AuditReport()
        self.official_src = OFFICIAL_REPO / "src" / "reachy_mini"
        self.official_tests = OFFICIAL_REPO / "tests"
        self.official_docs = OFFICIAL_REPO / "docs"

    def audit_api_endpoints(self) -> None:
        """Compare tous les endpoints REST."""
        logger.info("🔍 Audit endpoints REST...")

        # Extraire endpoints BBIA
        bbia_endpoints = self._extract_bbia_endpoints()

        # Pour chaque endpoint BBIA, vérifier s'il existe équivalent SDK
        for ep in bbia_endpoints:
            # Le SDK officiel n'expose pas d'API REST directement
            # Les endpoints BBIA sont une couche au-dessus du SDK
            item = ConformityItem(
                category="API",
                nature="endpoint",
                bbia_path=ep["file"],
                line=ep.get("line"),
                description=f"{ep['method']} {ep['path']}",
                status="OK" if self._validate_endpoint_uses_sdk(ep) else "DIFF",
            )
            self.report.add_item(item)

    def _extract_bbia_endpoints(self) -> list[dict[str, Any]]:
        """Extrait tous les endpoints BBIA."""
        endpoints = []
        routers_dir = BBIA_SRC / "daemon" / "app" / "routers"

        for router_file in routers_dir.glob("*.py"):
            if router_file.name.startswith("._") or router_file.name == "__init__.py":
                continue

            try:
                with open(router_file, encoding="utf-8") as f:
                    content = f.read()
                    lines = content.split("\n")

                pattern = (
                    r'@router\.(get|post|put|delete|patch)\s*\(\s*["\']([^"\']+)["\']'
                )
                for match in re.finditer(pattern, content):
                    method = match.group(1).upper()
                    path = match.group(2)
                    line_num = content[: match.start()].count("\n") + 1

                    endpoints.append(
                        {
                            "method": method,
                            "path": path,
                            "file": str(router_file.relative_to(PROJECT_ROOT)),
                            "line": line_num,
                        }
                    )
            except Exception as e:
                logger.warning(f"Error parsing {router_file}: {e}")

        return endpoints

    def _validate_endpoint_uses_sdk(self, endpoint: dict[str, Any]) -> bool:
        """Vérifie qu'un endpoint utilise le SDK correctement."""
        # Vérification basique: l'endpoint doit appeler des méthodes du backend
        # qui à leur tour utilisent le SDK
        router_file = PROJECT_ROOT / endpoint["file"]
        try:
            with open(router_file, encoding="utf-8") as f:
                content = f.read()

            # Vérifier présence de RobotFactory ou backend
            return "RobotFactory" in content or "backend" in content.lower()
        except Exception:
            return False

    def audit_sdk_methods(self) -> None:
        """Compare toutes les méthodes SDK."""
        logger.info("🔍 Audit méthodes SDK...")

        # Extraire méthodes SDK officiel
        sdk_methods = self._extract_sdk_methods()

        # Extraire méthodes BBIA backend
        bbia_backend_file = BBIA_SRC / "backends" / "reachy_mini_backend.py"
        bbia_methods = self._extract_backend_methods(bbia_backend_file)

        # Comparer
        for sdk_method in sdk_methods:
            bbia_has = any(m["name"] == sdk_method["name"] for m in bbia_methods)

            item = ConformityItem(
                category="CLASS",
                nature="method",
                bbia_path=str(bbia_backend_file.relative_to(PROJECT_ROOT)),
                official_path="reachy_mini/ReachyMini",
                description=f"Méthode SDK: {sdk_method['name']}",
                status="OK" if bbia_has else "MISSING",
            )
            self.report.add_item(item)

    def _extract_sdk_methods(self) -> list[dict[str, Any]]:
        """Extrait les méthodes du SDK officiel."""
        try:
            import inspect

            from reachy_mini import ReachyMini

            methods = []
            for name in dir(ReachyMini):
                if not name.startswith("_"):
                    obj = getattr(ReachyMini, name)
                    if callable(obj):
                        try:
                            sig = inspect.signature(obj)
                            methods.append(
                                {
                                    "name": name,
                                    "signature": str(sig),
                                }
                            )
                        except Exception:
                            methods.append({"name": name, "signature": "unknown"})

            return methods
        except Exception as e:
            logger.error(f"Erreur extraction méthodes SDK: {e}")
            return []

    def _extract_backend_methods(self, backend_file: Path) -> list[dict[str, Any]]:
        """Extrait les méthodes d'un fichier backend."""
        methods = []
        try:
            with open(backend_file, encoding="utf-8") as f:
                tree = ast.parse(f.read(), filename=str(backend_file))

            for node in ast.walk(tree):
                if isinstance(node, ast.FunctionDef) and not node.name.startswith("_"):
                    methods.append(
                        {
                            "name": node.name,
                            "line": node.lineno,
                        }
                    )
        except Exception as e:
            logger.error(f"Erreur parsing {backend_file}: {e}")

        return methods

    def audit_models_mujoco(self) -> None:
        """Compare les modèles MuJoCo."""
        logger.info("🔍 Audit modèles MuJoCo...")

        # Modèles officiels
        official_mjcf = self.official_src / "descriptions" / "reachy_mini" / "mjcf"
        bbia_mjcf = BBIA_SRC / "sim" / "assets" / "reachy_official"

        if official_mjcf.exists():
            official_xml = list(official_mjcf.glob("*.xml"))
            bbia_xml = list(bbia_mjcf.glob("*.xml"))

            for xml_file in official_xml:
                bbia_equivalent = bbia_mjcf / xml_file.name
                item = ConformityItem(
                    category="MODEL",
                    nature="mujoco_xml",
                    bbia_path=(
                        str(bbia_equivalent.relative_to(PROJECT_ROOT))
                        if bbia_equivalent.exists()
                        else "MISSING"
                    ),
                    official_path=str(xml_file.relative_to(OFFICIAL_REPO)),
                    description=f"Modèle MuJoCo: {xml_file.name}",
                    status="OK" if bbia_equivalent.exists() else "MISSING",
                )
                self.report.add_item(item)

    def audit_tests(self) -> None:
        """Compare les tests."""
        logger.info("🔍 Audit tests...")

        # Tests officiels
        if self.official_tests.exists():
            official_test_files = list(self.official_tests.glob("test_*.py"))

            for test_file in official_test_files:
                # Chercher équivalent BBIA
                bbia_equivalent = BBIA_TESTS / test_file.name

                item = ConformityItem(
                    category="TEST",
                    nature="test_file",
                    bbia_path=(
                        str(bbia_equivalent.relative_to(PROJECT_ROOT))
                        if bbia_equivalent.exists()
                        else "MISSING"
                    ),
                    official_path=str(test_file.relative_to(OFFICIAL_REPO)),
                    description=f"Test: {test_file.name}",
                    status="OK" if bbia_equivalent.exists() else "MISSING",
                )
                self.report.add_item(item)

    def run_full_audit(self) -> None:
        """Exécute l'audit complet."""
        logger.info("🚀 Démarrage audit systématique complet...")

        self.audit_api_endpoints()
        self.audit_sdk_methods()
        self.audit_models_mujoco()
        self.audit_tests()

        logger.info("✅ Audit terminé")

    def generate_checklist(self) -> str:
        """Génère une checklist finale actionable."""
        summary = self.report.get_summary()

        lines = [
            "# Checklist Finale - Audit Systématique BBIA-SIM vs SDK Officiel",
            "",
            f"**Date:** {subprocess.check_output(['date', '+%Y-%m-%d'], text=True).strip()}",
            f"**Total éléments vérifiés:** {summary['total']}",
            "",
            "## Résumé",
            "",
            f"- **OK:** {summary['by_status'].get('OK', 0)}",
            f"- **DIFF:** {summary['by_status'].get('DIFF', 0)}",
            f"- **MISSING:** {summary['by_status'].get('MISSING', 0)}",
            f"- **EXTRA:** {summary['by_status'].get('EXTRA', 0)}",
            f"- **Corrigés:** {summary['fixed']}",
            f"- **Testés:** {summary['tested']}",
            "",
            "## Checklist Détaillée",
            "",
        ]

        # Grouper par catégorie
        by_category = {}
        for item in self.report.items:
            if item.category not in by_category:
                by_category[item.category] = []
            by_category[item.category].append(item)

        for category in sorted(by_category.keys()):
            lines.append(f"### {category}")
            lines.append("")
            lines.append(
                "| Nature | Fichier | Ligne | Status | Fix | Test | Description |"
            )
            lines.append(
                "|--------|---------|-------|--------|-----|------|-------------|"
            )

            for item in sorted(by_category[category], key=lambda x: x.bbia_path):
                fix_mark = "✅" if item.fix_applied else "❌"
                test_mark = (
                    "✅"
                    if item.test_result == "PASSED"
                    else "⏸️" if item.test_result == "SKIPPED" else "❌"
                )
                status_mark = (
                    "✅"
                    if item.status == "OK"
                    else "⚠️" if item.status == "DIFF" else "❌"
                )

                lines.append(
                    f"| {item.nature} | `{item.bbia_path}` | {item.line or 'N/A'} | {status_mark} {item.status} | {fix_mark} | {test_mark} | {item.description[:60]} |"
                )
            lines.append("")

        return "\n".join(lines)


def main():
    """Point d'entrée principal."""
    auditor = SystematicAuditor()
    auditor.run_full_audit()

    checklist = auditor.generate_checklist()

    # Sauvegarder
    checklist_file = (
        PROJECT_ROOT / "docs" / "conformite" / "CHECKLIST_AUDIT_SYSTEMATIQUE.md"
    )
    checklist_file.parent.mkdir(parents=True, exist_ok=True)

    with open(checklist_file, "w", encoding="utf-8") as f:
        f.write(checklist)

    print("\n" + "=" * 80)
    print(checklist)
    print("=" * 80)
    print(f"\n📄 Checklist sauvegardée: {checklist_file}")

    summary = auditor.report.get_summary()
    if summary["by_status"].get("MISSING", 0) > 0:
        print(f"\n⚠️  {summary['by_status']['MISSING']} éléments manquants détectés!")
        sys.exit(1)
    else:
        print("\n✅ Audit terminé - aucune anomalie critique détectée!")
        sys.exit(0)


if __name__ == "__main__":
    main()
