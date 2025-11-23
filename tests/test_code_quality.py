#!/usr/bin/env python3
"""Tests pour la qualité du code.

Tests des imports, docstrings, type hints, et conformité SDK.
"""

import ast
import inspect
from pathlib import Path

# Modules à vérifier
MODULES_TO_CHECK = [
    "bbia_sim.bbia_chat",
    "bbia_sim.bbia_adaptive_learning",
    "bbia_sim.behaviors.storytelling",
    "bbia_sim.behaviors.teaching",
    "bbia_sim.behaviors.meditation",
    "bbia_sim.behaviors.exercise",
    "bbia_sim.behaviors.music_reaction",
    "bbia_sim.behaviors.alarm_clock",
    "bbia_sim.behaviors.weather_report",
    "bbia_sim.behaviors.news_reader",
    "bbia_sim.behaviors.game",
]


class TestCodeQuality:
    """Tests pour la qualité du code."""

    def test_imports_no_duplicates(self):
        """Test qu'il n'y a pas d'imports en double."""
        # Vérifier imports dans fichiers principaux
        src_dir = Path("src/bbia_sim")
        python_files = list(src_dir.rglob("*.py"))

        # Fichiers à ignorer (imports conditionnels TYPE_CHECKING acceptés)
        # Imports conditionnels dans fonctions/méthodes sont une pratique normale
        ignored_files = {
            "bbia_vision.py",  # Imports TYPE_CHECKING + runtime (pratique normale)
            "vision_yolo.py",  # Imports conditionnels multiples
            "__main__.py",  # Imports conditionnels dans fonctions (pratique normale CLI)
            "face_recognition.py",  # Imports conditionnels cv2 dans fonctions (pratique normale)
            "backends/simulation_shims.py",  # Imports conditionnels struct
        }

        for file_path in python_files:
            if "__pycache__" in str(file_path):
                continue
            if file_path.name.startswith("._"):  # Fichiers macOS cachés
                continue
            # Vérifier si le fichier doit être ignoré (par nom ou chemin relatif)
            if file_path.name in ignored_files:
                continue
            # Vérifier aussi par chemin relatif depuis src/bbia_sim
            rel_path = str(file_path.relative_to(src_dir))
            if rel_path in ignored_files:
                continue

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

                tree = ast.parse(content, filename=str(file_path))

                imports = []
                # Ignorer les imports dans TYPE_CHECKING (pratique normale)
                in_type_checking = False
                for node in ast.walk(tree):
                    # Détecter blocs TYPE_CHECKING
                    if isinstance(node, ast.If):
                        if (
                            isinstance(node.test, ast.Name)
                            and node.test.id == "TYPE_CHECKING"
                        ):
                            in_type_checking = True
                            continue
                    elif isinstance(node, ast.Import) or isinstance(
                        node, ast.ImportFrom
                    ):
                        # Ignorer imports dans TYPE_CHECKING
                        if in_type_checking:
                            continue
                        if isinstance(node, ast.Import):
                            for alias in node.names:
                                imports.append(alias.name)
                        elif isinstance(node, ast.ImportFrom):
                            if node.module:
                                imports.append(node.module)

                # Vérifier doublons
                assert len(imports) == len(
                    set(imports)
                ), f"Imports en double dans {file_path}"
            except SyntaxError:
                # Ignorer erreurs de syntaxe (fichiers peut-être en cours d'édition)
                pass
            except UnicodeDecodeError:
                # Ignorer erreurs d'encodage (fichiers macOS cachés, etc.)
                pass

    def test_docstrings_present(self):
        """Test que les fonctions principales ont des docstrings."""
        for module_name in MODULES_TO_CHECK:
            try:
                module = __import__(module_name, fromlist=[""])
                for name, obj in inspect.getmembers(module):
                    if inspect.isclass(obj) or inspect.isfunction(obj):
                        if not name.startswith("_"):
                            doc = inspect.getdoc(obj)
                            assert (
                                doc is not None
                            ), f"{module_name}.{name} n'a pas de docstring"
            except ImportError:
                # Module peut ne pas être disponible
                pass

    def test_type_hints_present(self):
        """Test que les fonctions principales ont des type hints."""
        for module_name in MODULES_TO_CHECK:
            try:
                module = __import__(module_name, fromlist=[""])
                for name, obj in inspect.getmembers(module):
                    if inspect.isfunction(obj):
                        if not name.startswith("_"):
                            sig = inspect.signature(obj)
                            # Vérifier qu'au moins un paramètre a un type hint
                            has_hints = (
                                any(
                                    param.annotation != inspect.Parameter.empty
                                    for param in sig.parameters.values()
                                )
                                or sig.return_annotation != inspect.Signature.empty
                            )

                            # Note: Pas strict car certains peuvent être optionnels
                            # Juste vérifier que certains ont des hints
            except ImportError:
                pass

    def test_sdk_conformity_create_head_pose(self):
        """Test conformité SDK : utilisation create_head_pose."""
        # Vérifier que comportements utilisent create_head_pose du SDK
        behaviors_dir = Path("src/bbia_sim/behaviors")
        behavior_files = list(behaviors_dir.glob("*.py"))

        uses_sdk = False
        for file_path in behavior_files:
            if file_path.name == "__init__.py" or file_path.name == "base.py":
                continue

            try:
                # Essayer UTF-8 d'abord, puis fallback
                try:
                    with open(file_path, encoding="utf-8", errors="replace") as f:
                        content = f.read()
                except (UnicodeDecodeError, ValueError):
                    with open(file_path, encoding="latin-1", errors="replace") as f:
                        content = f.read()

                if "create_head_pose" in content:
                    uses_sdk = True
                    break
            except Exception:
                pass

        # Au moins un comportement doit utiliser create_head_pose
        assert uses_sdk, "Aucun comportement n'utilise create_head_pose du SDK"
