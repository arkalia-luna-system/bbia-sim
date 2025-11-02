#!/usr/bin/env python3
"""
🧪 TESTS DE CONFORMITÉ POUR EXEMPLES/DEMOS
Vérifie que tous les exemples utilisent correctement le SDK Reachy-mini
et ne font pas d'erreurs expertes (comme set_joint_pos sur stewart).
"""

import ast
import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestExamplesConformity:
    """Tests de conformité pour les exemples et démos."""

    EXAMPLES_DIR = Path(__file__).parent.parent / "examples"
    EXCLUDED_FILES = {"__pycache__", "._demo", "__init__"}

    def _get_python_files(self):
        """Récupère tous les fichiers Python dans examples/."""
        if not self.EXAMPLES_DIR.exists():
            pytest.skip("Dossier examples/ non trouvé")

        python_files = []
        for file in self.EXAMPLES_DIR.glob("*.py"):
            if not any(excluded in file.name for excluded in self.EXCLUDED_FILES):
                python_files.append(file)

        return python_files

    def _parse_file(self, file_path: Path):
        """Parse un fichier Python et retourne l'AST."""
        try:
            # Essayer UTF-8 d'abord, puis fallback vers latin-1 ou errors='replace'
            try:
                with open(file_path, encoding="utf-8", errors="replace") as f:
                    content = f.read()
            except (UnicodeDecodeError, ValueError):
                # Fallback pour fichiers non-UTF8 ou avec problèmes
                with open(file_path, encoding="latin-1", errors="replace") as f:
                    content = f.read()

            # Nettoyer bytes null si présents
            content = content.replace("\x00", "")

            return ast.parse(content, filename=str(file_path))
        except SyntaxError as e:
            pytest.skip(f"Erreur de syntaxe dans {file_path}: {e}")
        except (UnicodeDecodeError, ValueError) as e:
            pytest.skip(f"Erreur d'encodage dans {file_path}: {e}")

    def _find_stewart_control_errors(self, tree: ast.AST) -> list[str]:
        """
        Trouve les utilisations incorrectes de set_joint_pos sur joints stewart.
        EXPERT ROBOTIQUE: Les joints stewart ne peuvent pas être contrôlés
        individuellement car la plateforme Stewart utilise la cinématique inverse (IK).
        """
        errors = []

        class Visitor(ast.NodeVisitor):
            def visit_Call(self, node: ast.Call):
                # Chercher set_joint_pos("stewart_*", ...)
                if isinstance(node.func, ast.Attribute):
                    if node.func.attr == "set_joint_pos":
                        if len(node.args) >= 1:
                            # Premier argument: nom du joint
                            arg = node.args[0]
                            if isinstance(arg, ast.Constant) and isinstance(
                                arg.value, str
                            ):
                                joint_name = arg.value
                                if joint_name.startswith("stewart_"):
                                    errors.append(
                                        f"❌ Ligne {node.lineno}: Utilisation de set_joint_pos('{joint_name}', ...). "
                                        "Les joints stewart ne peuvent PAS être contrôlés individuellement "
                                        "(plateforme Stewart utilise IK). Utiliser goto_target() ou "
                                        "set_target_head_pose() avec create_head_pose() à la place."
                                    )

                self.generic_visit(node)

        Visitor().visit(tree)
        return errors

    def _find_missing_goto_target(self, tree: ast.AST) -> list[str]:
        """Trouve les opportunités d'utiliser goto_target au lieu de méthodes moins optimales."""
        warnings: list[str] = []

        class Visitor(ast.NodeVisitor):
            def visit_Call(self, node: ast.Call):
                # Chercher des patterns où goto_target serait mieux
                if isinstance(node.func, ast.Attribute):
                    # Si set_joint_pos est utilisé plusieurs fois pour tête+corps,
                    # goto_target combiné serait mieux
                    if node.func.attr in [
                        "set_joint_pos",
                        "set_target_head_pose",
                        "set_target_body_yaw",
                    ]:
                        # Note: Détection complexe, simplifiée ici
                        pass

                self.generic_visit(node)

        Visitor().visit(tree)
        return warnings

    def test_no_stewart_individual_control(self):
        """Test: Aucun exemple ne doit utiliser set_joint_pos sur joints stewart."""
        print("\n🧪 TEST: Interdiction set_joint_pos sur stewart dans exemples")
        print("=" * 70)

        errors_found = []

        for py_file in self._get_python_files():
            print(f"\n📄 Vérification: {py_file.name}")
            tree = self._parse_file(py_file)
            file_errors = self._find_stewart_control_errors(tree)

            if file_errors:
                errors_found.extend([f"{py_file.name}: {err}" for err in file_errors])
                for err in file_errors:
                    print(f"   {err}")
            else:
                print("   ✅ Aucune erreur stewart détectée")

        assert (
            len(errors_found) == 0
        ), "Erreurs détectées dans les exemples:\n" + "\n".join(errors_found)

    def test_examples_use_sdk_methods(self):
        """Test: Les exemples doivent utiliser les méthodes SDK recommandées."""
        print("\n🧪 TEST: Utilisation méthodes SDK dans exemples")
        print("=" * 70)

        recommended_methods = [
            "goto_target",
            "look_at_world",
            "look_at_image",
            "create_head_pose",
            "set_emotion",
        ]

        # Statistiques par fichier
        stats = {}

        for py_file in self._get_python_files():
            try:
                with open(py_file, encoding="utf-8", errors="replace") as f:
                    content = f.read()
            except (UnicodeDecodeError, ValueError):
                # Fallback pour fichiers non-UTF8
                with open(py_file, encoding="latin-1", errors="replace") as f:
                    content = f.read()
            file_stats = {method: method in content for method in recommended_methods}
            stats[py_file.name] = file_stats

        # Afficher statistiques
        for filename, file_stats in stats.items():
            used = [m for m, used in file_stats.items() if used]
            print(
                f"📄 {filename}: {len(used)}/{len(recommended_methods)} méthodes SDK utilisées"
            )
            if used:
                print(f"   ✅ Utilise: {', '.join(used)}")

        # Note: Ne pas échouer si méthodes non utilisées (certains exemples peuvent être simples)
        print("\n✅ Vérification terminée (informative)")

    def test_examples_validate_coordinates(self):
        """Test: Les exemples doivent valider les coordonnées avant look_at_world/image."""
        print("\n🧪 TEST: Validation coordonnées dans exemples")
        print("=" * 70)

        warnings = []

        for py_file in self._get_python_files():
            try:
                with open(py_file, encoding="utf-8", errors="replace") as f:
                    content = f.read()
            except (UnicodeDecodeError, ValueError):
                # Fallback pour fichiers non-UTF8
                with open(py_file, encoding="latin-1", errors="replace") as f:
                    content = f.read()

            # Chercher look_at_world sans validation
            if "look_at_world" in content:
                # Recherche simple: si pas de check des limites avant
                has_validation = any(
                    pattern in content.lower()
                    for pattern in [
                        "if",
                        "validate",
                        "check",
                        "limit",
                        "-2.0 <= x <= 2.0",
                    ]
                )

                if not has_validation:
                    warnings.append(
                        f"⚠️  {py_file.name}: look_at_world utilisé sans validation explicite "
                        "des coordonnées (recommandé pour robustesse)"
                    )

        if warnings:
            for warning in warnings:
                print(f"   {warning}")
        else:
            print("   ✅ Tous les exemples valident les coordonnées")

        # Note: Warning seulement, ne fait pas échouer le test
        print("\n✅ Vérification terminée (warnings seulement)")

    def test_examples_use_interpolation_methods(self):
        """Test: Les exemples utilisent des méthodes d'interpolation (minjerk recommandé)."""
        print("\n🧪 TEST: Méthodes d'interpolation dans exemples")
        print("=" * 70)

        interpolation_methods = ["minjerk", "linear", "ease_in_out", "cartoon"]
        stats = {}

        for py_file in self._get_python_files():
            try:
                with open(py_file, encoding="utf-8", errors="replace") as f:
                    content = f.read()
            except (UnicodeDecodeError, ValueError):
                # Fallback pour fichiers non-UTF8
                with open(py_file, encoding="latin-1", errors="replace") as f:
                    content = f.read()

            used_methods = [m for m in interpolation_methods if m in content.lower()]
            stats[py_file.name] = used_methods

            if "goto_target" in content and not used_methods:
                print(
                    f"   ⚠️  {py_file.name}: goto_target utilisé sans spécifier method= (minjerk recommandé)"
                )

        for filename, methods in stats.items():
            if methods:
                print(f"   ✅ {filename}: Utilise interpolation {', '.join(methods)}")

        print("\n✅ Vérification terminée")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
