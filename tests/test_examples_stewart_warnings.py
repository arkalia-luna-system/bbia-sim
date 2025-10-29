#!/usr/bin/env python3
"""
🧪 TESTS - VÉRIFICATION AVERTISSEMENTS STEWART JOINTS DANS EXEMPLES
Vérifie que tous les exemples/demos qui utilisent joints stewart directement
ont des commentaires explicites expliquant que c'est uniquement pour MuJoCo direct.
"""

import re
import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent))


class TestExamplesStewartWarnings:
    """Tests pour vérifier les avertissements dans les exemples."""

    def test_all_stewart_usage_has_warnings(self):
        """Test: Tous les fichiers utilisant stewart_* doivent avoir des avertissements."""
        print("\n🧪 TEST: Vérification avertissements stewart joints")
        print("=" * 60)

        examples_dir = Path(__file__).parent.parent / "examples"
        if not examples_dir.exists():
            pytest.skip("Dossier examples introuvable")

        # Chercher tous les fichiers Python utilisant stewart
        stewart_pattern = re.compile(r"stewart_\d")
        warning_keywords = [
            "uniquement pour",
            "seulement pour",
            "MuJoCo direct",
            "simulation MuJoCo",
            "approximation",
            "IMPORTANT EXPERT",
            "NOTE.*stewart",
            "valide uniquement",
            "requis.*IK",
            "goto_target",
            "create_head_pose",
        ]

        files_with_stewart = []
        files_missing_warnings = []

        for py_file in examples_dir.glob("*.py"):
            content = py_file.read_text(encoding="utf-8")

            # Vérifier si fichier utilise stewart
            if stewart_pattern.search(content):
                files_with_stewart.append(py_file)
                content_lower = content.lower()

                # Vérifier présence d'avertissements
                has_warning = any(
                    keyword.lower() in content_lower
                    or re.search(keyword, content, re.IGNORECASE)
                    for keyword in warning_keywords
                )

                if not has_warning:
                    files_missing_warnings.append(py_file)
                    print(
                        f"⚠️  {py_file.name}: Utilise stewart_* sans avertissement explicite"
                    )

        if files_with_stewart:
            print(f"\n📊 Fichiers utilisant stewart_*: {len(files_with_stewart)}")
            for f in files_with_stewart:
                has_warn = f not in files_missing_warnings
                print(f"   {'✅' if has_warn else '❌'} {f.name}")

        assert len(files_missing_warnings) == 0, (
            f"EXPERT: {len(files_missing_warnings)} fichier(s) utilisent stewart_* sans avertissement: "
            f"{[f.name for f in files_missing_warnings]}. "
            f"Sans avertissement = risque confusion SDK réel vs simulation."
        )

        print("\n✅ Tous les fichiers avec stewart_* ont des avertissements explicites")

    def test_no_direct_stewart_in_main_modules(self):
        """Test: Les modules principaux (src/) ne doivent PAS utiliser stewart directement."""
        print("\n🧪 TEST: Vérification modules principaux")
        print("=" * 60)

        src_dir = Path(__file__).parent.parent / "src" / "bbia_sim"
        if not src_dir.exists():
            pytest.skip("Dossier src/bbia_sim introuvable")

        # Exclure les fichiers backends (peuvent avoir du code SDK)
        exclude_patterns = ["backends", "__pycache__", "__init__", "robot_factory"]

        # Pattern pour détecter les appels réels problématiques à set_joint_pos avec stewart
        # Exclure les définitions de dictionnaires (clés simples comme "stewart_1": valeur)
        forbidden_pattern = re.compile(r'set_joint_pos\s*\(\s*["\']stewart_\d["\']')

        files_with_direct_stewart = []

        for py_file in src_dir.rglob("*.py"):
            # Ignorer fichiers exclus
            if any(pattern in str(py_file) for pattern in exclude_patterns):
                continue

            try:
                content = py_file.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                # Ignorer les fichiers binaires ou avec encodage différent
                continue

            # Chercher usage direct stewart (hors commentaires)
            lines = content.split("\n")
            for i, line in enumerate(lines, 1):
                # Ignorer commentaires et docstrings
                stripped = line.strip()
                if (
                    stripped.startswith("#")
                    or stripped.startswith('"""')
                    or stripped.startswith("'''")
                ):
                    continue

                if forbidden_pattern.search(line):
                    files_with_direct_stewart.append((py_file, i, line.strip()))
                    print(f"⚠️  {py_file.name}:{i}: {line.strip()[:60]}...")

        if files_with_direct_stewart:
            print(
                f"\n⚠️  Modules utilisant stewart directement: {len(files_with_direct_stewart)}"
            )
            # Vérifier si c'est dans des backends ou fichiers acceptables
            problematic = [
                (f, line_num)
                for f, line_num, _ in files_with_direct_stewart
                if "backend" not in str(f) and "test" not in str(f)
            ]
            if problematic:
                print("❌ Fichiers problématiques (hors backends/tests):")
                for f, line_num in problematic:
                    print(f"   {f.name}:{line_num}")
            else:
                print("✅ Usage stewart seulement dans backends (acceptable)")
        else:
            print("✅ Aucun usage direct stewart dans modules principaux")

        # Ne pas bloquer si c'est dans backends (acceptables)
        backend_files = [
            f for f, _, _ in files_with_direct_stewart if "backend" in str(f)
        ]
        if len(files_with_direct_stewart) > len(backend_files):
            problematic_count = len(files_with_direct_stewart) - len(backend_files)
            assert problematic_count == 0, (
                f"EXPERT: {problematic_count} module(s) principal(aux) utilisent stewart directement. "
                f"Les modules principaux doivent utiliser goto_target avec IK (conforme SDK)."
            )


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
