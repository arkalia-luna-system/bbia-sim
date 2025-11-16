#!/usr/bin/env python3
"""Script unifié pour vérifier la documentation (précision et cohérence).

Fusion de :
- verify_doc_accuracy.py (vérifie précision - fichiers/test existent)
- verify_md_vs_code.py (vérifie cohérence - fonctionnalités implémentées)
- audit_and_improve_md.py (vérifie véracité affirmations MD et améliore formatage)

Usage:
    python scripts/verify_documentation.py           # Les deux modes
    python scripts/verify_documentation.py --accuracy   # Précision uniquement
    python scripts/verify_documentation.py --consistency # Cohérence uniquement
    python scripts/verify_documentation.py --improve     # Améliorer formatage MD
"""

import argparse
import contextlib
import re
import subprocess
from pathlib import Path
from typing import Any

# Fonctionnalités mentionnées dans les MD comme "faites"
FUNCTIONALITIES_TO_CHECK = {
    "VAD activation auto": {
        "code_patterns": [
            r"detect_speech_activity",
            r"silero/vad",
            r"transcribe_microphone_with_vad",
        ],
        "test_patterns": [r"test_vad_streaming"],
    },
    "Extraction paramètres NER": {
        "code_patterns": [r"_extract_angle", r"_extract_intensity"],
        "test_patterns": [r"test_bbia_nlp_detection"],
    },
    "Whisper streaming": {
        "code_patterns": [r"transcribe_streaming"],
        "test_patterns": [r"test_vad_streaming"],
    },
    "SmolVLM2 vision": {
        "code_patterns": [r"SmolVLM|moondream2", r"describe_image"],
        "test_patterns": [r"test_bbia_nlp_detection"],
    },
    "model_optimizer": {
        "code_patterns": [r"model_optimizer"],
        "files": ["src/bbia_sim/model_optimizer.py"],
    },
    "bbia_doctor": {
        "code_patterns": [r"bbia_doctor"],
        "files": ["scripts/bbia_doctor.py"],
    },
    "emergency_stop": {
        "code_patterns": [r"emergency_stop"],
        "test_patterns": [r"test_emergency_stop"],
    },
    "Tests sécurité JSON": {
        "test_patterns": [r"test_security_json"],
    },
    "Tests sécurité limites": {
        "test_patterns": [r"test_safety_limits"],
    },
    "Tests bbia_memory": {
        "test_patterns": [r"test_bbia_memory"],
    },
    "Tests bbia_emotions améliorés": {
        "test_patterns": [r"test_bbia_emotions_improved"],
    },
    "Benchmarks audio": {
        "test_patterns": [r"test_benchmark_audio"],
    },
}


def count_tests() -> int:
    """Compte le nombre réel de tests."""
    result = subprocess.run(
        [
            "find",
            "tests",
            "-name",
            "*.py",
            "-exec",
            "grep",
            "-h",
            "def test_",
            "{}",
            ";",
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd="/Volumes/T7/bbia-reachy-sim",
    )
    return len(
        [
            line
            for line in result.stdout.split("\n")
            if line.strip() and line.strip().startswith("def test_")
        ],
    )


def count_docs() -> int:
    """Compte le nombre réel de fichiers MD."""
    result = subprocess.run(
        ["find", "docs", "-name", "*.md", "-type", "f"],
        check=False,
        capture_output=True,
        text=True,
        cwd="/Volumes/T7/bbia-reachy-sim",
    )
    return len([f for f in result.stdout.split("\n") if f.strip()])


def count_emotions() -> int:
    """Compte le nombre réel d'émotions dans bbia_emotions.py."""
    emotions_file = Path("/Volumes/T7/bbia-reachy-sim/src/bbia_sim/bbia_emotions.py")
    if not emotions_file.exists():
        return 0

    content = emotions_file.read_text()
    emotions = set()
    in_emotions_dict = False
    brace_count = 0

    for line in content.split("\n"):
        if "self.emotions = {" in line:
            in_emotions_dict = True
            brace_count = line.count("{") - line.count("}")
            continue

        if in_emotions_dict:
            brace_count += line.count("{") - line.count("}")

            if '":' in line or '" :' in line:
                match = re.search(r'["\']([a-z_]+)["\']\s*:', line)
                if match:
                    emotions.add(match.group(1))

            if brace_count <= 0 and "}" in line:
                break

    return len(emotions)


def verify_ci_cd_tools() -> dict[str, bool]:
    """Vérifie que les outils CI/CD mentionnés existent vraiment."""
    ci_file = Path("/Volumes/T7/bbia-reachy-sim/.github/workflows/ci.yml")
    if not ci_file.exists():
        return {}

    content = ci_file.read_text()
    return {
        "black": "black" in content.lower(),
        "ruff": "ruff" in content.lower(),
        "mypy": "mypy" in content.lower(),
        "bandit": "bandit" in content.lower(),
        "pip-audit": "pip-audit" in content.lower() or "pip audit" in content.lower(),
    }


def verify_architecture() -> dict[str, bool]:
    """Vérifie que Factory et ABC sont bien présents."""
    factory_file = Path("/Volumes/T7/bbia-reachy-sim/src/bbia_sim/robot_factory.py")
    api_file = Path("/Volumes/T7/bbia-reachy-sim/src/bbia_sim/robot_api.py")

    factory_exists = (
        factory_file.exists() and "class RobotFactory" in factory_file.read_text()
    )
    abc_exists = api_file.exists() and (
        "ABC" in api_file.read_text() and "@abstractmethod" in api_file.read_text()
    )

    return {"factory": factory_exists, "abc": abc_exists}


def check_code_exists(pattern: str, file_path: Path) -> bool:
    """Vérifie si un pattern existe dans un fichier."""
    try:
        # Lire seulement les premières lignes pour performance
        content = file_path.read_text(encoding="utf-8")[:5000]
        return bool(re.search(pattern, content, re.IGNORECASE))
    except Exception:
        return False


def check_test_exists(test_pattern: str) -> bool:
    """Vérifie si un test existe."""
    tests_dir = Path("tests")
    if not tests_dir.exists():
        return False

    # Limiter la recherche pour éviter blocage
    try:
        for test_file in list(tests_dir.rglob("*.py"))[:100]:  # Limiter à 100 fichiers
            if re.search(test_pattern, test_file.name, re.IGNORECASE):
                return True
            try:
                # Lire seulement les premières lignes pour performance
                content = test_file.read_text(encoding="utf-8")[:2000]
                if re.search(test_pattern, content, re.IGNORECASE):
                    return True
            except Exception:
                continue
    except Exception:
        pass
    return False


def verify_functionality(name: str, info: dict[str, Any]) -> dict[str, Any]:
    """Vérifie si une fonctionnalité est vraiment implémentée."""
    results: dict[str, Any] = {"name": name, "code_ok": False, "tests_ok": False, "issues": []}

    # Vérifier dans le code
    code_ok = False
    if "files" in info:
        code_ok = all(Path(f).exists() for f in info["files"])
    elif "code_patterns" in info:
        src_dir = Path("src")
        for pattern in info["code_patterns"]:
            found = False
            try:
                # Limiter la recherche pour éviter blocage
                for py_file in list(src_dir.rglob("*.py"))[
                    :200
                ]:  # Limiter à 200 fichiers
                    if check_code_exists(pattern, py_file):
                        found = True
                        break
            except Exception:
                pass
            if found:
                code_ok = True
                break
            results["issues"].append(f"Pattern code non trouvé: {pattern}")

    results["code_ok"] = code_ok

    # Vérifier les tests
    if "test_patterns" in info:
        tests_ok = all(check_test_exists(pattern) for pattern in info["test_patterns"])
        results["tests_ok"] = tests_ok
        if not tests_ok:
            results["issues"].extend(
                [
                    f"Test non trouvé: {pattern}"
                    for pattern in info["test_patterns"]
                    if not check_test_exists(pattern)
                ],
            )

    return results


def verify_accuracy():
    """Mode 1 : Vérification précision (fichiers/test existent)."""
    print("🔍 Vérification Précision Documentation\n")

    # Tests
    real_tests = count_tests()
    print(f"✅ Tests réels: {real_tests}")
    print("   README dit: 1200+")
    if real_tests < 1200:
        print(f"   ⚠️  Différence: {1200 - real_tests}")

    # Docs
    real_docs = count_docs()
    print(f"\n✅ Docs réels: {real_docs}")
    print("   README dit: 280")
    if real_docs != 280:
        print(f"   ⚠️  Différence: {abs(real_docs - 280)}")

    # Émotions
    real_emotions = count_emotions()
    print(f"\n✅ Émotions réelles: {real_emotions}")
    print("   README dit: 12")
    if real_emotions != 12:
        print(f"   ❌ ERREUR: {real_emotions} émotions trouvées, pas 12!")
        print("   ⚠️  Correction nécessaire dans README.md")
        return False

    # CI/CD
    ci_tools = verify_ci_cd_tools()
    print("\n✅ Outils CI/CD:")
    for tool, exists in ci_tools.items():
        status = "✅" if exists else "❌"
        print(f"   {status} {tool}: {exists}")

    # Architecture
    arch = verify_architecture()
    print("\n✅ Architecture:")
    print(f"   {'✅' if arch['factory'] else '❌'} Factory: {arch['factory']}")
    print(f"   {'✅' if arch['abc'] else '❌'} ABC: {arch['abc']}")

    print("\n" + "=" * 50)
    print("✅ Documentation globalement cohérente")
    return True


def verify_consistency():
    """Mode 2 : Vérification cohérence (fonctionnalités implémentées)."""
    print("🔍 Vérification Cohérence MD vs Code\n")

    # Vérifier fonctionnalités mentionnées
    print("📋 Vérification fonctionnalités:")
    all_ok = True
    for name, info in FUNCTIONALITIES_TO_CHECK.items():
        result = verify_functionality(name, info)
        status = "✅" if result["code_ok"] and result["tests_ok"] else "❌"
        print(f"{status} {name}")
        if not result["code_ok"] or not result["tests_ok"]:
            all_ok = False
            for issue in result["issues"]:
                print(f"   ⚠️ {issue}")

    print("\n📄 Vérification MD d'audit:")
    audit_dir = Path("docs/audit")
    if audit_dir.exists():
        try:
            md_files = list(audit_dir.glob("*.md"))[:30]  # Limiter à 30 fichiers
            for md_file in md_files:
                # Chercher affirmations dans les premières lignes seulement
                try:
                    content = md_file.read_text(encoding="utf-8")[
                        :5000
                    ]  # Limiter lecture
                    if "✅" in content or "TERMINÉ" in content:
                        print(
                            f"   📄 {md_file.name}: Contient affirmations de complétion",
                        )
                except Exception:
                    continue
        except Exception:
            pass

    if all_ok:
        print("\n✅ Toutes les vérifications sont OK!")
    else:
        print("\n⚠️ Certaines vérifications ont échoué. Voir détails ci-dessus.")

    return all_ok


def cleanup_metadata_files(file_path: Path) -> None:
    """Supprime les fichiers de métadonnées macOS créés automatiquement (fusionné depuis audit_and_improve_md.py)."""
    parent_dir = file_path.parent
    base_name = file_path.name

    # Supprimer fichier ._* standard
    metadata_file = parent_dir / f"._{base_name}"
    if metadata_file.exists():
        with contextlib.suppress(Exception):
            metadata_file.unlink()

    # Supprimer fichiers .!*!._* (format avec numéro)
    pattern = parent_dir / f".!*._{base_name}"
    for metadata_file_path in pattern.parent.glob(pattern.name):
        with contextlib.suppress(Exception):
            metadata_file_path.unlink()


def verify_md_claims(md_file: Path) -> dict[str, Any]:
    """Vérifie les affirmations dans un MD contre le code réel (fusionné depuis audit_and_improve_md.py)."""
    issues = []
    content = md_file.read_text(encoding="utf-8")

    # Vérifier affirmations communes
    claims = {
        "tests": {
            "patterns": [
                r"(\d+)\+?\s*tests?",
                r"(\d+)\s*tests?",
                r"(\d+)\s+fonctions de test",
            ],
            "verify": lambda x: 1157 <= int(x) <= 1250,  # Range acceptable
        },
        "emotions": {
            "patterns": [r"(\d+)\s+émotions", r"(\d+)\s+emotions"],
            "verify": lambda x: int(x) == 12,  # Doit être exactement 12
        },
        "docs": {
            "patterns": [r"(\d+)\+?\s*fichiers?\s*doc", r"(\d+)\s*fichiers?\s*MD"],
            "verify": lambda x: 280 <= int(x) <= 310,  # Range acceptable
        },
    }

    for claim_type, config in claims.items():
        patterns_list = config.get("patterns", [])
        verify_func = config.get("verify")
        if not verify_func or not isinstance(patterns_list, list):
            continue
        for pattern in patterns_list:
            matches = re.finditer(pattern, content, re.IGNORECASE)
            for match in matches:
                number = match.group(1)
                if callable(verify_func) and not verify_func(number):
                    issues.append(
                        {
                            "type": claim_type,
                            "value": number,
                            "line": content[: match.start()].count("\n") + 1,
                        },
                    )

    return {"file": md_file, "issues": issues}


def improve_md_formatting(content: str) -> str:
    """Améliore le formatage MD pour le rendre plus moderne et impactant (fusionné depuis audit_and_improve_md.py)."""
    # 1. Améliorer titres avec emojis cohérents
    content = re.sub(
        r"^##\s+([^📋🎯✅⚠️❌🔍📊📝🎉🚀🏗️🧪📚⚡🔒🌟]+)",
        r"## \1",
        content,
        flags=re.MULTILINE,
    )

    # 2. Améliorer listes avec puces modernes
    content = re.sub(r"^\s*[-•]\s+", "• ", content, flags=re.MULTILINE)

    # 3. Améliorer code blocks avec langage si manquant
    content = re.sub(r"```\n([^`]+)\n```", r"```python\n\1\n```", content)

    # 4. Standardiser les dates
    content = re.sub(
        r"(Date|Dernière mise à jour|Mise à jour):\s*(.*?2024|.*?2025)",
        lambda m: standardize_date(m.group(0)),
        content,
        flags=re.IGNORECASE,
    )

    # 5. Améliorer séparateurs
    content = re.sub(r"^---{3,}$", "---", content, flags=re.MULTILINE)

    # 6. Ajouter espacements cohérents
    content = re.sub(r"\n{3,}", "\n\n", content)

    return content


def standardize_date(date_str: str) -> str:
    """Standardise les dates (fusionné depuis audit_and_improve_md.py)."""
    if "Oct 25 / Nov 25" in date_str or "octobre 2025" in date_str.lower():
        return date_str  # Déjà standardisé
    if "novembre 2025" in date_str.lower():
        return date_str.replace("novembre 2025", "Oct 25 / Nov 25").replace(
            "Novembre 2025",
            "Oct 25 / Nov 25",
        )
    if "octobre 2024" in date_str.lower() and "création" in date_str.lower():
        return date_str  # Date création ne pas modifier
    return date_str


def improve_md_files():
    """Mode 3 : Améliorer formatage MD (fusionné depuis audit_and_improve_md.py)."""
    print("✨ Amélioration Formatage Documentation MD\n")

    root = Path("/Volumes/T7/bbia-reachy-sim")

    # Fichiers principaux à améliorer
    priority_files = [
        "README.md",
        "docs/guides/GUIDE_DEBUTANT.md",
    ]

    for file_path in priority_files:
        full_path = root / file_path
        if not full_path.exists():
            continue

        print(f"\n📝 {file_path}")

        # 1. Vérifier véracité
        verification = verify_md_claims(full_path)
        if verification["issues"]:
            print(f"   ⚠️  {len(verification['issues'])} problèmes détectés")
            for issue in verification["issues"]:
                issue_type = issue["type"]
                issue_value = issue["value"]
                issue_line = issue["line"]
                print(f"      - {issue_type}: {issue_value} (ligne {issue_line})")
        else:
            print("   ✅ Aucun problème de véracité")

        # 2. Améliorer formatage
        content = full_path.read_text(encoding="utf-8")
        original_content = content

        content = improve_md_formatting(content)

        # Sauvegarder si changé
        if content != original_content:
            full_path.write_text(content, encoding="utf-8")
            # Nettoyer les métadonnées macOS créées automatiquement
            cleanup_metadata_files(full_path)
            print("   ✨ Formatage amélioré")
        else:
            print("   ℹ️  Déjà bien formaté")

    print("\n✅ Amélioration terminée")
    return True


def main():
    """Fonction principale."""
    parser = argparse.ArgumentParser(
        description="Vérification documentation (précision et cohérence)",
    )
    parser.add_argument(
        "--accuracy",
        action="store_true",
        help="Vérifier uniquement la précision (fichiers/test existent)",
    )
    parser.add_argument(
        "--consistency",
        action="store_true",
        help="Vérifier uniquement la cohérence (fonctionnalités implémentées)",
    )
    parser.add_argument(
        "--improve",
        action="store_true",
        help="Améliorer formatage MD uniquement",
    )
    args = parser.parse_args()

    if args.accuracy:
        success = verify_accuracy()
        exit(0 if success else 1)
    elif args.consistency:
        success = verify_consistency()
        exit(0 if success else 1)
    elif args.improve:
        success = improve_md_files()
        exit(0 if success else 1)
    else:
        # Les deux modes
        print("=" * 50)
        print("MODE 1: Vérification Précision")
        print("=" * 50)
        accuracy_ok = verify_accuracy()

        print("\n" + "=" * 50)
        print("MODE 2: Vérification Cohérence")
        print("=" * 50)
        consistency_ok = verify_consistency()

        exit(0 if (accuracy_ok and consistency_ok) else 1)


if __name__ == "__main__":
    main()
