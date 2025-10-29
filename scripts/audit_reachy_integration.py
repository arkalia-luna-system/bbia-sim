#!/usr/bin/env python3
"""
Script d'audit complet BBIA → Reachy Integration
Génère JSONL par module + synthèse MD globale selon procédure stricte
"""

import json
import logging
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

# Configuration logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("log/audit_reachy.log"),
        logging.StreamHandler(sys.stdout),
    ],
)
logger = logging.getLogger(__name__)

# Référentiel officiel
REACHY_REF_PATH = Path("/tmp/reachy_ref")
REACHY_COMMIT = "84c40c3"  # Commit utilisé
PROJECT_ROOT = Path(__file__).parent.parent

# Modules critiques par priorité
CRITICAL_MODULES = [
    {
        "name": "motor_controllers",
        "files": [
            "src/bbia_sim/backends/reachy_mini_backend.py",
            "src/bbia_sim/robot_api.py",
            "src/bbia_sim/mapping_reachy.py",
        ],
        "ref_files": [
            "/tmp/reachy_ref/src/reachy_mini/daemon/app/routers/motors.py",
            "/tmp/reachy_ref/src/reachy_mini/daemon/backend/abstract.py",
        ],
    },
    {
        "name": "urdf_sdf_models",
        "files": [
            "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml",
            "src/bbia_sim/sim/models/reachy_mini.xml",
        ],
        "ref_files": [
            "/tmp/reachy_ref/src/reachy_mini/descriptions/reachy_mini/mjcf/reachy_mini.xml",
            "/tmp/reachy_ref/src/reachy_mini/descriptions/reachy_mini/urdf/robot.urdf",
        ],
    },
    {
        "name": "audio_tts",
        "files": [
            "src/bbia_sim/bbia_audio.py",
            "src/bbia_sim/bbia_voice.py",
            "src/bbia_sim/bbia_voice_advanced.py",
        ],
        "ref_files": [
            "/tmp/reachy_ref/src/reachy_mini/media/",
        ],
    },
    {
        "name": "emotion_inference",
        "files": [
            "src/bbia_sim/bbia_emotions.py",
            "src/bbia_sim/bbia_emotion_recognition.py",
        ],
        "ref_files": [],
    },
    {
        "name": "safety",
        "files": [
            "src/bbia_sim/global_config.py",
            "src/bbia_sim/mapping_reachy.py",
        ],
        "ref_files": [],
    },
]

MEDIUM_MODULES = [
    {
        "name": "behaviors",
        "files": [
            "src/bbia_sim/bbia_behavior.py",
            "src/bbia_sim/bbia_adaptive_behavior.py",
        ],
        "ref_files": [],
    },
    {
        "name": "sdk_wrappers",
        "files": [
            "src/bbia_sim/backends/reachy_mini_backend.py",
            "src/bbia_sim/robot_factory.py",
        ],
        "ref_files": [],
    },
]


def run_command(cmd: list[str], cwd: Path | None = None) -> tuple[int, str, str]:
    """Exécute une commande et retourne (exit_code, stdout, stderr)."""
    try:
        result = subprocess.run(
            cmd, cwd=cwd, capture_output=True, text=True, timeout=60
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        logger.error(f"Timeout command: {' '.join(cmd)}")
        return 1, "", "Timeout"
    except Exception as e:
        logger.error(f"Erreur command: {' '.join(cmd)} - {e}")
        return 1, "", str(e)


def check_black(file_path: Path) -> tuple[bool, str]:
    """Vérifie formatage black."""
    exit_code, stdout, stderr = run_command(
        ["black", "--check", "--diff", str(file_path)]
    )
    if exit_code == 0:
        return True, ""
    return False, stdout + stderr


def check_ruff(file_path: Path) -> tuple[list[dict], str]:
    """Vérifie ruff et retourne issues."""
    exit_code, stdout, stderr = run_command(
        ["ruff", "check", "--output-format=json", str(file_path)]
    )
    issues = []
    if stdout:
        try:
            issues = json.loads(stdout)
        except json.JSONDecodeError:
            pass
    return issues, stderr


def check_bandit(file_path: Path) -> tuple[list[dict], str]:
    """Vérifie bandit et retourne issues >= medium."""
    exit_code, stdout, stderr = run_command(
        [
            "bandit",
            "-r",
            str(file_path.parent),
            "-f",
            "json",
            "-ll",
            "-i",
            str(file_path),
        ]
    )
    issues = []
    if stdout:
        try:
            data = json.loads(stdout)
            issues = [
                issue
                for issue in data.get("results", [])
                if issue.get("issue_severity", "low") in ["medium", "high"]
            ]
        except json.JSONDecodeError:
            pass
    return issues, stderr


def run_pytest_targeted(test_path: str) -> tuple[bool, str]:
    """Exécute tests ciblés uniquement (unit/fast, pas e2e)."""
    exit_code, stdout, stderr = run_command(
        [
            "pytest",
            test_path,
            "-q",
            "-k",
            "unit or fast",
            "-m",
            "not e2e",
            "--tb=short",
        ],
        cwd=PROJECT_ROOT,
    )
    success = exit_code == 0
    output = stdout + stderr
    return success, output


def audit_module(module_config: dict[str, Any]) -> dict[str, Any]:
    """Audite un module selon procédure stricte."""
    module_name = module_config["name"]
    logger.info(f"🔍 Audit module: {module_name}")

    result = {
        "module": module_name,
        "files_checked": [],
        "reference_files": [],
        "issues": [],
        "patches": [],
        "tests_added": [],
        "score": {"conformity": 0, "safety_tests": 0, "performance": 0, "docs": 0},
        "recommendation": "",
    }

    # 1. Références
    result["reference_files"] = [
        f"{ref}@{REACHY_COMMIT}" for ref in module_config.get("ref_files", [])
    ]

    # 2-14. Pour chaque fichier
    for file_path_str in module_config["files"]:
        file_path = PROJECT_ROOT / file_path_str
        if not file_path.exists():
            logger.warning(f"⚠️  Fichier non trouvé: {file_path}")
            continue

        result["files_checked"].append(file_path_str)
        logger.info(f"  📄 Audit: {file_path_str}")

        # 4. Black
        black_ok, black_diff = check_black(file_path)
        if not black_ok and black_diff:
            result["issues"].append(
                {
                    "severity": "medium",
                    "type": "format",
                    "desc": "Black formatage non conforme",
                    "file": file_path_str,
                    "line": 0,
                }
            )
            result["patches"].append(
                {"file": file_path_str, "diff": black_diff[:500]}
            )

        # 5. Ruff
        ruff_issues, _ = check_ruff(file_path)
        for issue in ruff_issues[:10]:  # Limiter pour taille
            result["issues"].append(
                {
                    "severity": "medium" if issue.get("code", "").startswith("E") else "low",
                    "type": "lint",
                    "desc": issue.get("message", ""),
                    "file": file_path_str,
                    "line": issue.get("location", {}).get("row", 0),
                }
            )

        # 6. Bandit
        bandit_issues, _ = check_bandit(file_path)
        for issue in bandit_issues[:10]:
            result["issues"].append(
                {
                    "severity": issue.get("issue_severity", "medium"),
                    "type": "security",
                    "desc": issue.get("issue_text", ""),
                    "file": file_path_str,
                    "line": issue.get("line_number", 0),
                }
            )

        # 8. Tests (chercher tests correspondants)
        test_file = PROJECT_ROOT / "tests" / f"test_{module_name}.py"
        if not test_file.exists():
            # Chercher tests liés
            test_pattern = f"test_*{Path(file_path_str).stem}*.py"
            test_files = list(PROJECT_ROOT.glob(f"tests/{test_pattern}"))
            if test_files:
                test_file = test_files[0]

        if test_file.exists():
            test_success, test_output = run_pytest_targeted(str(test_file))
            if not test_success:
                result["issues"].append(
                    {
                        "severity": "high",
                        "type": "test",
                        "desc": f"Tests échouent: {test_output[:200]}",
                        "file": str(test_file),
                        "line": 0,
                    }
                )

    # Scoring (simplifié pour l'exemple)
    total_issues_high = sum(
        1 for i in result["issues"] if i.get("severity") == "high"
    )
    total_issues_medium = sum(
        1 for i in result["issues"] if i.get("severity") == "medium"
    )

    result["score"]["conformity"] = max(0, 10 - total_issues_high * 2 - total_issues_medium)
    result["score"]["safety_tests"] = 7 if not total_issues_high else max(0, 10 - total_issues_high * 3)
    result["score"]["performance"] = 8  # À calculer selon analyse
    result["score"]["docs"] = 6  # À calculer selon analyse docs

    result["recommendation"] = (
        f"{len(result['issues'])} issues détectées. "
        f"Priorité: corriger {total_issues_high} issues high, "
        f"{total_issues_medium} issues medium."
    )

    return result


def generate_jsonl_report(results: list[dict]) -> None:
    """Génère rapport JSONL."""
    output_file = PROJECT_ROOT / "artifacts" / "audit_reachy_modules.jsonl"
    output_file.parent.mkdir(parents=True, exist_ok=True)

    with open(output_file, "w", encoding="utf-8") as f:
        for result in results:
            f.write(json.dumps(result, ensure_ascii=False) + "\n")

    logger.info(f"✅ Rapport JSONL généré: {output_file}")


def generate_synthesis_md(results: list[dict]) -> None:
    """Génère synthèse MD globale."""
    output_file = PROJECT_ROOT / "artifacts" / "AUDIT_REACHY_SYNTHESE.md"
    output_file.parent.mkdir(parents=True, exist_ok=True)

    content = f"""# 🔍 AUDIT COMPLET BBIA → REACHY INTEGRATION

**Date**: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}  
**Référentiel Reachy**: pollen-robotics/reachy_mini@{REACHY_COMMIT}  
**Commit utilisé**: {REACHY_COMMIT}

---

## 📊 RÉSUMÉ GLOBAL

{len(results)} modules audités:
- 🔴 **Critiques**: {len([r for r in results if r['module'] in [m['name'] for m in CRITICAL_MODULES]])}
- 🟡 **Moyens**: {len([r for r in results if r['module'] in [m['name'] for m in MEDIUM_MODULES]])}

---

## 🎯 ORDRE D'INTERVENTION PRIORISÉ

"""

    # Trier par score conformité + issues high
    sorted_results = sorted(
        results,
        key=lambda x: (
            x["score"]["conformity"],
            -sum(1 for i in x["issues"] if i.get("severity") == "high"),
        ),
    )

    for idx, result in enumerate(sorted_results, 1):
        issues_high = sum(1 for i in result["issues"] if i.get("severity") == "high")
        issues_medium = sum(
            1 for i in result["issues"] if i.get("severity") == "medium"
        )
        content += f"""
### {idx}. {result['module']}

**Score**: Conformité {result['score']['conformity']}/10 | Sécurité {result['score']['safety_tests']}/10 | Performance {result['score']['performance']}/10 | Docs {result['score']['docs']}/10

**Issues**: 🔴 {issues_high} high | 🟡 {issues_medium} medium

**Estimation**: {max(2, issues_high * 2 + issues_medium)} heures

**Recommandation**: {result['recommendation']}

**Fichiers audités**:
{chr(10).join(f"- `{f}`" for f in result['files_checked'])}

"""

    content += """
---

## ✅ CHECKLIST D'ACCEPTATION GLOBALE

- [ ] Tous modules critiques audités (100%)
- [ ] Aucune issue bandit high non traitée
- [ ] Tests unitaires verts pour modules critiques
- [ ] Conformité aux specs Reachy citées
- [ ] Patches appliqués et validés
- [ ] Documentation mise à jour

---

## 📋 PR TEMPLATE

### Title
`fix(audit): [MODULE] - [DESCRIPTION]`

### Body
```markdown
## 🔍 Audit BBIA → Reachy Integration

**Module**: [nom]
**Référentiel**: reachy_mini@{REACHY_COMMIT}
**Fichiers**: [liste]

### Issues corrigées
- [ ] Issue #1: [description]
- [ ] Issue #2: [description]

### Tests ajoutés
- [ ] Test: [description]

### Validation
- [ ] black --check ✅
- [ ] ruff check ✅
- [ ] bandit (pas de high) ✅
- [ ] pytest unit/fast ✅
```

### Reviewers suggérés
@[expert-robotique]
"""

    with open(output_file, "w", encoding="utf-8") as f:
        f.write(content)

    logger.info(f"✅ Synthèse MD générée: {output_file}")


def main():
    """Point d'entrée principal."""
    logger.info("🚀 Démarrage audit BBIA → Reachy Integration")
    logger.info(f"📁 Référentiel: {REACHY_REF_PATH}@{REACHY_COMMIT}")

    results = []

    # Audit modules critiques
    for module_config in CRITICAL_MODULES:
        try:
            result = audit_module(module_config)
            results.append(result)
        except Exception as e:
            logger.error(f"❌ Erreur audit {module_config['name']}: {e}")

    # Audit modules moyens (échantillon si temps)
    for module_config in MEDIUM_MODULES[:2]:  # Limiter pour temps
        try:
            result = audit_module(module_config)
            results.append(result)
        except Exception as e:
            logger.error(f"❌ Erreur audit {module_config['name']}: {e}")

    # Générer livrables
    generate_jsonl_report(results)
    generate_synthesis_md(results)

    logger.info("✅ Audit terminé")


if __name__ == "__main__":
    main()

