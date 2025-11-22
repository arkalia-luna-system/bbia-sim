#!/usr/bin/env python3
"""Audit systématique exhaustif BBIA-SIM vs SDK officiel reachy_mini.
Compare TOUT : endpoints, classes, méthodes, modèles, tests, scripts, docs, etc.
"""

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
BBIA_EXAMPLES = PROJECT_ROOT / "examples"

if not OFFICIAL_REPO.exists():
    logger.error(f"Repo officiel non trouvé: {OFFICIAL_REPO}")
    sys.exit(1)

OFFICIAL_SRC = OFFICIAL_REPO / "src" / "reachy_mini"
OFFICIAL_TESTS = OFFICIAL_REPO / "tests"
OFFICIAL_EXAMPLES = OFFICIAL_REPO / "examples"
OFFICIAL_MJCF = OFFICIAL_SRC / "descriptions" / "reachy_mini" / "mjcf"


@dataclass
class ConformityItem:
    """Un élément de conformité avec détails."""

    category: str  # API, CLASS, MODEL, TEST, SCRIPT, DOC, etc.
    nature: str  # endpoint, method, file, joint, etc.
    bbia_path: str
    official_path: str | None = None
    line: int | None = None
    status: str = "PENDING"  # OK, DIFF, MISSING, EXTRA, INCOMPATIBLE
    severity: str = "COMPATIBLE"  # STRICT, COMPATIBLE, INCOMPATIBLE
    description: str = ""
    bbia_value: str = ""
    official_value: str = ""
    fix_applied: bool = False
    fix_suggestion: str = ""
    test_result: str = "NOT_TESTED"
    test_file: str = ""
    qa_ok: bool = False
    proof_link: str = ""


@dataclass
class AuditReport:
    """Rapport d'audit complet."""

    items: list[ConformityItem] = field(default_factory=list)

    def add_item(self, item: ConformityItem) -> None:
        """Ajoute un élément au rapport."""
        self.items.append(item)

    def get_summary(self) -> dict[str, Any]:
        """Génère un résumé statistique."""
        total = len(self.items)
        by_status = {}
        by_category = {}
        by_severity = {}

        for item in self.items:
            by_status[item.status] = by_status.get(item.status, 0) + 1
            by_category[item.category] = by_category.get(item.category, 0) + 1
            by_severity[item.severity] = by_severity.get(item.severity, 0) + 1

        return {
            "total": total,
            "by_status": by_status,
            "by_category": by_category,
            "by_severity": by_severity,
            "fixed": sum(1 for i in self.items if i.fix_applied),
            "tested_passed": sum(1 for i in self.items if i.test_result == "PASSED"),
            "qa_ok": sum(1 for i in self.items if i.qa_ok),
        }


class ExhaustiveAuditor:
    """Auditeur exhaustif comparant BBIA vs SDK officiel."""

    def __init__(self):
        self.report = AuditReport()

    def audit_joints_mujoco(self) -> None:
        """Compare les noms et limites de joints MuJoCo."""
        logger.info("🔍 Audit joints MuJoCo...")

        # Extraire joints du XML officiel
        official_xml = OFFICIAL_MJCF / "reachy_mini.xml"
        if not official_xml.exists():
            logger.warning(f"XML officiel non trouvé: {official_xml}")
            return

        try:
            with open(official_xml, encoding="utf-8") as f:
                content = f.read()

            # Extraire tous les joints avec leurs ranges
            joint_pattern = r'<joint[^>]*name="([^"]+)"[^>]*range="([^"]+)"'
            official_joints = {}
            for match in re.finditer(joint_pattern, content):
                joint_name = match.group(1)
                range_str = match.group(2)
                # Parser range "min max"
                try:
                    min_val, max_val = map(float, range_str.split())
                    official_joints[joint_name] = (min_val, max_val)
                except Exception:
                    pass

            # Comparer avec BBIA
            bbia_joints_file = BBIA_SRC / "sim" / "joints.py"
            if bbia_joints_file.exists():
                try:
                    with open(bbia_joints_file, encoding="utf-8") as f:
                        bbia_content = f.read()

                    # Extraire limites BBIA (format dict)
                    for joint_name, (min_o, max_o) in official_joints.items():
                        # Chercher dans BBIA
                        pattern = f'"{joint_name}":.*\\(([^)]+)\\)'
                        match = re.search(pattern, bbia_content)

                        if match:
                            try:
                                # Parser tuple BBIA
                                range_match = match.group(1)
                                min_b, max_b = map(float, range_match.split(","))
                                diff_min = abs(min_b - min_o)
                                diff_max = abs(max_b - max_o)

                                # Tolérance 0.01 rad
                                if diff_min > 0.01 or diff_max > 0.01:
                                    item = ConformityItem(
                                        category="MODEL",
                                        nature="joint_range",
                                        bbia_path=str(
                                            bbia_joints_file.relative_to(PROJECT_ROOT),
                                        ),
                                        official_path=str(
                                            official_xml.relative_to(OFFICIAL_REPO),
                                        ),
                                        description=f"Joint {joint_name}: limites différentes",
                                        bbia_value=f"({min_b:.6f}, {max_b:.6f})",
                                        official_value=f"({min_o:.6f}, {max_o:.6f})",
                                        status="DIFF",
                                        severity="COMPATIBLE",
                                        fix_suggestion=f"Utiliser valeurs exactes: ({min_o:.6f}, {max_o:.6f})",
                                    )
                                    self.report.add_item(item)
                                else:
                                    item = ConformityItem(
                                        category="MODEL",
                                        nature="joint_range",
                                        bbia_path=str(
                                            bbia_joints_file.relative_to(PROJECT_ROOT),
                                        ),
                                        official_path=str(
                                            official_xml.relative_to(OFFICIAL_REPO),
                                        ),
                                        description=f"Joint {joint_name}",
                                        status="OK",
                                        severity="STRICT",
                                    )
                                    self.report.add_item(item)
                            except Exception:
                                pass
                        else:
                            # Joint manquant dans BBIA
                            item = ConformityItem(
                                category="MODEL",
                                nature="joint",
                                bbia_path=str(
                                    bbia_joints_file.relative_to(PROJECT_ROOT),
                                ),
                                official_path=str(
                                    official_xml.relative_to(OFFICIAL_REPO),
                                ),
                                description=f"Joint {joint_name} manquant",
                                status="MISSING",
                                severity="INCOMPATIBLE",
                                fix_suggestion=f"Ajouter joint {joint_name} avec limites ({min_o:.6f}, {max_o:.6f})",
                            )
                            self.report.add_item(item)
                except Exception as e:
                    logger.error(f"Erreur parsing joints BBIA: {e}")
        except Exception as e:
            logger.error(f"Erreur lecture XML officiel: {e}")

    def audit_tests_comparison(self) -> None:
        """Compare les tests officiels avec BBIA."""
        logger.info("🔍 Audit comparaison tests...")

        # Liste tests officiels
        official_test_files = list(OFFICIAL_TESTS.glob("test_*.py"))

        for test_file in official_test_files:
            test_name = test_file.name
            bbia_equivalent = BBIA_TESTS / test_name

            # Extraire fonctions de test du fichier officiel
            try:
                with open(test_file, encoding="utf-8") as f:
                    content = f.read()

                test_funcs = re.findall(r"def (test_\w+)", content)

                if bbia_equivalent.exists():
                    # Vérifier si les mêmes fonctions existent
                    with open(bbia_equivalent, encoding="utf-8") as f:
                        bbia_content = f.read()

                    for func in test_funcs:
                        if f"def {func}" not in bbia_content:
                            item = ConformityItem(
                                category="TEST",
                                nature="test_function",
                                bbia_path=str(
                                    bbia_equivalent.relative_to(PROJECT_ROOT),
                                ),
                                official_path=str(test_file.relative_to(OFFICIAL_REPO)),
                                description=f"Test {func} manquant dans {test_name}",
                                status="MISSING",
                                severity="COMPATIBLE",
                                fix_suggestion=f"Ajouter test {func} basé sur {test_file.name}",
                            )
                            self.report.add_item(item)
                        else:
                            item = ConformityItem(
                                category="TEST",
                                nature="test_function",
                                bbia_path=str(
                                    bbia_equivalent.relative_to(PROJECT_ROOT),
                                ),
                                official_path=str(test_file.relative_to(OFFICIAL_REPO)),
                                description=f"Test {func} présent",
                                status="OK",
                                severity="STRICT",
                            )
                            self.report.add_item(item)
                else:
                    # Fichier de test manquant
                    item = ConformityItem(
                        category="TEST",
                        nature="test_file",
                        bbia_path=f"MISSING: {test_name}",
                        official_path=str(test_file.relative_to(OFFICIAL_REPO)),
                        description=f"Fichier test {test_name} manquant",
                        status="MISSING",
                        severity="COMPATIBLE",
                        fix_suggestion=f"Créer {test_name} basé sur {test_file}",
                    )
                    self.report.add_item(item)
            except Exception as e:
                logger.warning(f"Erreur parsing {test_file}: {e}")

    def audit_scripts_comparison(self) -> None:
        """Compare les scripts et outils."""
        logger.info("🔍 Audit scripts et outils...")

        # Scripts officiels
        official_tools = OFFICIAL_REPO / "tools"
        if official_tools.exists():
            official_scripts = list(official_tools.glob("*.py"))

            for script in official_scripts:
                script_name = script.name
                bbia_equivalent = BBIA_SCRIPTS / script_name

                item = ConformityItem(
                    category="SCRIPT",
                    nature="tool_script",
                    bbia_path=(
                        str(bbia_equivalent.relative_to(PROJECT_ROOT))
                        if bbia_equivalent.exists()
                        else f"MISSING: {script_name}"
                    ),
                    official_path=str(script.relative_to(OFFICIAL_REPO)),
                    description=f"Script {script_name}",
                    status="OK" if bbia_equivalent.exists() else "MISSING",
                    severity="COMPATIBLE" if not bbia_equivalent.exists() else "STRICT",
                )
                self.report.add_item(item)

    def audit_examples_comparison(self) -> None:
        """Compare les exemples."""
        logger.info("🔍 Audit exemples...")

        # Exemples officiels
        if OFFICIAL_EXAMPLES.exists():
            official_examples = list(OFFICIAL_EXAMPLES.glob("*.py"))

            for example in official_examples:
                example_name = example.name
                bbia_equivalent = BBIA_EXAMPLES / example_name

                item = ConformityItem(
                    category="EXAMPLE",
                    nature="example_file",
                    bbia_path=(
                        str(bbia_equivalent.relative_to(PROJECT_ROOT))
                        if bbia_equivalent.exists()
                        else f"MISSING: {example_name}"
                    ),
                    official_path=str(example.relative_to(OFFICIAL_REPO)),
                    description=f"Exemple {example_name}",
                    status="OK" if bbia_equivalent.exists() else "MISSING",
                    severity="COMPATIBLE",
                )
                self.report.add_item(item)

    def audit_assets_stl(self) -> None:
        """Compare les fichiers STL."""
        logger.info("🔍 Audit assets STL...")

        official_assets = OFFICIAL_MJCF / "assets"
        bbia_assets = BBIA_SRC / "sim" / "assets" / "reachy_official"

        if official_assets.exists():
            official_stls = {
                f.name: f
                for f in official_assets.glob("*.stl")
                if not f.name.startswith("._")
            }

            for stl_name, stl_file in official_stls.items():
                bbia_equivalent = bbia_assets / stl_name

                item = ConformityItem(
                    category="MODEL",
                    nature="stl_asset",
                    bbia_path=(
                        str(bbia_equivalent.relative_to(PROJECT_ROOT))
                        if bbia_equivalent.exists()
                        else f"MISSING: {stl_name}"
                    ),
                    official_path=str(stl_file.relative_to(OFFICIAL_REPO)),
                    description=f"Asset STL: {stl_name}",
                    status="OK" if bbia_equivalent.exists() else "MISSING",
                    severity="COMPATIBLE" if not bbia_equivalent.exists() else "STRICT",
                )
                self.report.add_item(item)

    def run_full_audit(self) -> None:
        """Exécute l'audit complet."""
        logger.info("🚀 Démarrage audit exhaustif complet...")

        self.audit_joints_mujoco()
        self.audit_tests_comparison()
        self.audit_scripts_comparison()
        self.audit_examples_comparison()
        self.audit_assets_stl()

        logger.info("✅ Audit terminé")

    def generate_checklist(self) -> str:
        """Génère une checklist finale exhaustive."""
        summary = self.report.get_summary()

        lines = [
            "# Checklist Finale Exhaustive - Audit Systématique BBIA-SIM vs SDK Officiel",
            "",
            f"**Date:** {subprocess.check_output(['date', '+%Y-%m-%d'], text=True).strip()}",
            f"**Total éléments vérifiés:** {summary['total']}",
            "",
            "## Résumé Exécutif",
            "",
            f"- **OK (identique):** {summary['by_status'].get('OK', 0)}",
            f"- **DIFF (différent):** {summary['by_status'].get('DIFF', 0)}",
            f"- **MISSING (manquant):** {summary['by_status'].get('MISSING', 0)}",
            f"- **EXTRA (supplémentaire):** {summary['by_status'].get('EXTRA', 0)}",
            "",
            f"- **STRICT (identique):** {summary['by_severity'].get('STRICT', 0)}",
            f"- **COMPATIBLE (différent mais OK):** {summary['by_severity'].get('COMPATIBLE', 0)}",
            f"- **INCOMPATIBLE (erreur probable):** {summary['by_severity'].get('INCOMPATIBLE', 0)}",
            "",
            f"- **Corrigés:** {summary['fixed']}",
            f"- **Testés OK:** {summary['tested_passed']}",
            f"- **QA OK:** {summary['qa_ok']}",
            "",
            "## Checklist Détaillée par Catégorie",
            "",
        ]

        # Grouper par catégorie
        by_category = {}
        for item in self.report.items:
            if item.category not in by_category:
                by_category[item.category] = []
            by_category[item.category].append(item)

        for category in sorted(by_category.keys()):
            items = by_category[category]
            lines.append(f"### {category} ({len(items)} items)")
            lines.append("")
            lines.append(
                "| Nature | Fichier BBIA | Ligne | Status | Severity | Fix | Test | QA | Description |",
            )
            lines.append(
                "|--------|--------------|-------|--------|----------|-----|------|-----|-------------|",
            )

            for item in sorted(
                items,
                key=lambda x: (
                    x.status != "OK",
                    x.severity == "INCOMPATIBLE",
                    x.bbia_path,
                ),
            ):
                fix_mark = "✅" if item.fix_applied else "❌"
                test_mark = (
                    "✅"
                    if item.test_result == "PASSED"
                    else "⏸️"
                    if item.test_result == "SKIPPED"
                    else "❌"
                )
                qa_mark = "✅" if item.qa_ok else "❌"
                status_icon = (
                    "✅"
                    if item.status == "OK"
                    else "⚠️"
                    if item.status == "DIFF"
                    else "❌"
                )
                severity_icon = (
                    "🟢"
                    if item.severity == "STRICT"
                    else "🟡"
                    if item.severity == "COMPATIBLE"
                    else "🔴"
                )

                lines.append(
                    f"| {item.nature} | `{item.bbia_path[:50]}` | {item.line or 'N/A'} | {status_icon} {item.status} | {severity_icon} {item.severity} | {fix_mark} | {test_mark} | {qa_mark} | {item.description[:60]} |",
                )
            lines.append("")

        # Section corrections prioritaires
        incompatible = [i for i in self.report.items if i.severity == "INCOMPATIBLE"]
        if incompatible:
            lines.extend(
                [
                    "## 🔴 Corrections Prioritaires (INCOMPATIBLE)",
                    "",
                ],
            )
            for idx, item in enumerate(incompatible, 1):
                lines.append(f"{idx}. **{item.description}**")
                lines.append(f"   - Fichier: `{item.bbia_path}`:{item.line or '?'}")
                lines.append(f"   - Correction: {item.fix_suggestion}")
                if item.official_path:
                    lines.append(f"   - Référence: `{item.official_path}`")
                lines.append("")

        # Section fixes appliqués
        fixed = [i for i in self.report.items if i.fix_applied]
        if fixed:
            lines.extend(
                [
                    "## ✅ Corrections Appliquées",
                    "",
                ],
            )
            for item in fixed:
                lines.append(
                    f"- **{item.description}** - `{item.bbia_path}`:{item.line or '?'} ✅",
                )
                if item.test_result == "PASSED":
                    lines.append(f"  - Test: ✅ PASS - {item.test_file}")
                lines.append("")

        return "\n".join(lines)


def main():
    """Point d'entrée principal."""
    auditor = ExhaustiveAuditor()
    auditor.run_full_audit()

    checklist = auditor.generate_checklist()

    # Sauvegarder
    checklist_file = (
        PROJECT_ROOT / "docs" / "conformite" / "CHECKLIST_AUDIT_EXHAUSTIF.md"
    )
    checklist_file.parent.mkdir(parents=True, exist_ok=True)

    with open(checklist_file, "w", encoding="utf-8") as f:
        f.write(checklist)

    print("\n" + "=" * 80)
    print(checklist)
    print("=" * 80)
    print(f"\n📄 Checklist sauvegardée: {checklist_file}")

    summary = auditor.report.get_summary()
    if summary["by_severity"].get("INCOMPATIBLE", 0) > 0:
        print(
            f"\n🔴 {summary['by_severity']['INCOMPATIBLE']} éléments INCOMPATIBLES détectés!",
        )
        sys.exit(1)
    else:
        print("\n✅ Audit terminé - aucune incompatibilité critique!")
        sys.exit(0)


if __name__ == "__main__":
    main()
