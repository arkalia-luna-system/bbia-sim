#!/usr/bin/env python3
"""Vérifie la cohérence entre les MD d'audit/améliorations et le code réel."""

import re
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

# Tests mentionnés comme "TODO" ou "à faire"
TODO_PATTERNS = [
    r"TODO",
    r"⚠️.*test",
    r"⚠️.*créer",
    r"restant",
    r"à faire",
    r"manquant",
]


def check_code_exists(pattern: str, file_path: Path) -> bool:
    """Vérifie si un pattern existe dans un fichier."""
    try:
        content = file_path.read_text(encoding="utf-8")
        return bool(re.search(pattern, content, re.IGNORECASE))
    except Exception:
        return False


def check_test_exists(test_pattern: str) -> bool:
    """Vérifie si un test existe."""
    tests_dir = Path("tests")
    if not tests_dir.exists():
        return False

    for test_file in tests_dir.rglob("*.py"):
        if re.search(test_pattern, test_file.name, re.IGNORECASE):
            return True
        try:
            content = test_file.read_text(encoding="utf-8")
            if re.search(test_pattern, content, re.IGNORECASE):
                return True
        except Exception:
            continue
    return False


def verify_functionality(name: str, info: dict[str, Any]) -> dict[str, Any]:
    """Vérifie si une fonctionnalité est vraiment implémentée."""
    results = {"name": name, "code_ok": False, "tests_ok": False, "issues": []}

    # Vérifier dans le code
    code_ok = False
    if "files" in info:
        code_ok = all(Path(f).exists() for f in info["files"])
    elif "code_patterns" in info:
        src_dir = Path("src")
        for pattern in info["code_patterns"]:
            found = False
            for py_file in src_dir.rglob("*.py"):
                if check_code_exists(pattern, py_file):
                    found = True
                    break
            if found:
                code_ok = True
                break
            else:
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
                ]
            )

    return results


def find_md_claims(md_file: Path) -> list[dict[str, Any]]:
    """Trouve les affirmations dans un MD (✅ TERMINÉ, etc.)."""
    try:
        content = md_file.read_text(encoding="utf-8")
    except Exception:
        return []

    claims = []
    # Chercher les affirmations de complétion
    patterns = [
        (r"✅\s*([A-Z][^✅]+?)\s*TERMINÉ", "terminé"),
        (r"✅\s*([A-Z][^✅]+?)\s*FAIT", "fait"),
        (r"✅\s*([A-Z][^✅]+?)\s*IMPLÉMENTÉ", "implémenté"),
        (r"⚠️\s*([A-Z][^⚠️]+?)\s*TODO", "todo"),
        (r"⚠️\s*([A-Z][^⚠️]+?)\s*restant", "restant"),
    ]

    for pattern, status in patterns:
        matches = re.finditer(pattern, content, re.IGNORECASE | re.MULTILINE)
        for match in matches:
            claims.append(
                {
                    "text": match.group(1).strip(),
                    "status": status,
                    "line": content[: match.start()].count("\n") + 1,
                }
            )

    return claims


def main():
    """Vérification principale."""
    print("🔍 Vérification cohérence MD vs Code\n")

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
    md_files = list(audit_dir.glob("*.md")) if audit_dir.exists() else []

    for md_file in md_files:
        claims = find_md_claims(md_file)
        if claims:
            print(f"\n📄 {md_file.name}:")
            for claim in claims[:5]:  # Limiter l'affichage
                print(f"   {claim['status'].upper()}: {claim['text'][:60]}...")

    if all_ok:
        print("\n✅ Toutes les vérifications sont OK!")
    else:
        print("\n⚠️ Certaines vérifications ont échoué. Voir détails ci-dessus.")


if __name__ == "__main__":
    main()
