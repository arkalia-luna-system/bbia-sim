#!/usr/bin/env python3
"""Comparaison Profonde des Méthodes Backend - BBIA vs SDK Officiel
Compare signatures, types, paramètres, valeurs de retour, exceptions
"""

import ast
import logging
from pathlib import Path
from typing import Any

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class BackendMethodComparator:
    """Compare les méthodes backend en profondeur."""

    def __init__(self, official_path: Path, bbia_path: Path):
        """Initialise le comparateur."""
        self.official_path = official_path
        self.bbia_path = bbia_path
        self.results: dict[str, Any] = {
            "methods": {},
            "missing": [],
            "extra": [],
            "signature_diffs": [],
            "return_type_diffs": [],
            "summary": {},
        }

    def extract_method_signatures(
        self,
        file_path: Path,
        class_name: str,
    ) -> dict[str, dict]:
        """Extrait les signatures des méthodes d'une classe."""
        if not file_path.exists():
            return {}

        try:
            content = file_path.read_text(encoding="utf-8")
            tree = ast.parse(content)

            methods: dict[str, dict] = {}

            for node in ast.walk(tree):
                if isinstance(node, ast.ClassDef) and node.name == class_name:
                    for item in node.body:
                        if isinstance(item, ast.FunctionDef):
                            method_name = item.name
                            if method_name.startswith("_"):
                                continue  # Skip private

                            # Extraire signature
                            params = []
                            for arg in item.args.args:
                                if arg.arg == "self":
                                    continue
                                param_info = {"name": arg.arg}
                                if arg.annotation:
                                    param_info["type"] = ast.unparse(arg.annotation)
                                params.append(param_info)

                            # Type de retour
                            return_type = None
                            if item.returns:
                                return_type = ast.unparse(item.returns)

                            # Async
                            is_async = isinstance(item, ast.AsyncFunctionDef)

                            methods[method_name] = {
                                "params": params,
                                "return_type": return_type,
                                "async": is_async,
                                "docstring": ast.get_docstring(item),
                            }

            return methods

        except Exception as e:
            logger.warning(f"Erreur extraction {file_path}: {e}")
            return {}

    def compare_methods(self) -> None:
        """Compare les méthodes entre officiel et BBIA."""
        logger.info("🔍 Comparaison profonde des méthodes backend...")

        # Extraire méthodes officielles
        official_backend = (
            self.official_path / "src/reachy_mini/daemon/backend/abstract.py"
        )
        official_methods = self.extract_method_signatures(official_backend, "Backend")

        # Extraire méthodes BackendAdapter BBIA
        bbia_adapter = self.bbia_path / "src/bbia_sim/daemon/app/backend_adapter.py"
        adapter_methods = self.extract_method_signatures(bbia_adapter, "BackendAdapter")

        # Extraire méthodes ReachyMiniBackend BBIA
        bbia_backend = self.bbia_path / "src/bbia_sim/backends/reachy_mini_backend.py"
        backend_methods = self.extract_method_signatures(
            bbia_backend,
            "ReachyMiniBackend",
        )

        # Combiner méthodes BBIA (adapter + backend)
        all_bbia_methods = {**adapter_methods, **backend_methods}

        # Comparer
        missing = []
        extra = []
        signature_diffs = []
        return_type_diffs = []

        # Méthodes critiques du SDK officiel
        critical_methods = [
            "goto_target",
            "play_move",
            "set_target",
            "get_present_head_pose",
            "get_present_antenna_joint_positions",
            "get_present_body_yaw",
            "set_target_head_pose",
            "set_target_antenna_joint_positions",
            "set_target_body_yaw",
            "get_motor_control_mode",
            "set_motor_control_mode",
            "wake_up",
            "goto_sleep",
            "get_urdf",
        ]

        for method_name in critical_methods:
            official = official_methods.get(method_name)
            bbia = all_bbia_methods.get(method_name)

            if not official:
                continue  # Pas critique

            if not bbia:
                missing.append(method_name)
                continue

            # Comparer signatures
            official_params = {p["name"]: p.get("type") for p in official["params"]}
            bbia_params = {p["name"]: p.get("type") for p in bbia["params"]}

            if official_params != bbia_params:
                signature_diffs.append(
                    {
                        "method": method_name,
                        "official": official_params,
                        "bbia": bbia_params,
                    },
                )

            # Comparer types de retour
            if official.get("return_type") != bbia.get("return_type"):
                return_type_diffs.append(
                    {
                        "method": method_name,
                        "official": official.get("return_type"),
                        "bbia": bbia.get("return_type"),
                    },
                )

        # Méthodes extra dans BBIA
        for method_name in all_bbia_methods:
            if method_name not in official_methods and not method_name.startswith("_"):
                extra.append(method_name)

        self.results["methods"]["official"] = len(official_methods)
        self.results["methods"]["bbia"] = len(all_bbia_methods)
        self.results["missing"] = missing
        self.results["extra"] = sorted(extra)
        self.results["signature_diffs"] = signature_diffs
        self.results["return_type_diffs"] = return_type_diffs

        logger.info(f"  Méthodes officielles: {len(official_methods)}")
        logger.info(f"  Méthodes BBIA: {len(all_bbia_methods)}")
        logger.info(f"  Manquantes: {len(missing)}")
        logger.info(f"  Différences signature: {len(signature_diffs)}")
        logger.info(f"  Différences retour: {len(return_type_diffs)}")

    def compare_default_values(self) -> None:
        """Compare les valeurs par défaut des paramètres."""
        logger.info("🔍 Comparaison des valeurs par défaut...")

        # Lire fichiers directement pour comparer valeurs par défaut
        official_file = (
            self.official_path / "src/reachy_mini/daemon/backend/abstract.py"
        )
        bbia_adapter_file = (
            self.bbia_path / "src/bbia_sim/daemon/app/backend_adapter.py"
        )

        if not official_file.exists() or not bbia_adapter_file.exists():
            return

        # Méthodes critiques à comparer
        critical_methods = ["goto_target", "play_move", "set_target"]

        defaults_diffs = []

        for method_name in critical_methods:
            # Extraire depuis code source (méthode simple)
            official_content = official_file.read_text()
            bbia_content = bbia_adapter_file.read_text()

            # Recherche simple (à améliorer)
            official_sig = self._find_method_signature(official_content, method_name)
            bbia_sig = self._find_method_signature(bbia_content, method_name)

            if official_sig and bbia_sig:
                if official_sig != bbia_sig:
                    defaults_diffs.append(
                        {
                            "method": method_name,
                            "official": official_sig,
                            "bbia": bbia_sig,
                        },
                    )

        self.results["default_values_diffs"] = defaults_diffs

    def _find_method_signature(self, content: str, method_name: str) -> str | None:
        """Trouve la signature d'une méthode dans le contenu."""
        lines = content.split("\n")
        in_method = False
        signature_lines = []

        for line in lines:
            if f"def {method_name}" in line or f"async def {method_name}" in line:
                in_method = True
                signature_lines.append(line.strip())
                if ":" in line:
                    break
                continue
            if in_method:
                if line.strip().startswith("def ") or line.strip().startswith(
                    "async def",
                ):
                    break
                signature_lines.append(line.strip())
                if ":" in line:
                    break

        if signature_lines:
            return " ".join(signature_lines)
        return None

    def run_comparison(self) -> dict[str, Any]:
        """Lance la comparaison complète."""
        logger.info("🚀 Démarrage comparaison profonde...")

        self.compare_methods()
        self.compare_default_values()

        # Générer résumé
        self.results["summary"] = {
            "methods_official": self.results["methods"].get("official", 0),
            "methods_bbia": self.results["methods"].get("bbia", 0),
            "missing_critical": len(self.results["missing"]),
            "signature_diffs_count": len(self.results["signature_diffs"]),
            "return_type_diffs_count": len(self.results["return_type_diffs"]),
            "extra_methods": len(self.results["extra"]),
            "conformity": (
                100.0
                if len(self.results["missing"]) == 0
                and len(self.results["signature_diffs"]) == 0
                else max(
                    0,
                    100
                    - (
                        len(self.results["missing"]) * 10
                        + len(self.results["signature_diffs"]) * 5
                    ),
                )
            ),
        }

        logger.info("✅ Comparaison profonde terminée")
        return self.results

    def save_results(self, output_path: Path) -> None:
        """Sauvegarde les résultats."""
        import json

        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(self.results, f, indent=2, ensure_ascii=False)
        logger.info(f"✅ Résultats sauvegardés: {output_path}")


def main():
    """Point d'entrée principal."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Comparaison profonde méthodes backend",
    )
    parser.add_argument(
        "--official-root",
        type=Path,
        default=Path("/Volumes/T7/reachy_mini"),
        help="Racine repo officiel",
    )
    parser.add_argument(
        "--bbia-root",
        type=Path,
        default=Path(__file__).parent.parent,
        help="Racine projet BBIA",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("logs"),
        help="Répertoire de sortie",
    )

    args = parser.parse_args()

    comparator = BackendMethodComparator(args.official_root, args.bbia_root)
    results = comparator.run_comparison()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    comparator.save_results(output_dir / "comparaison_profonde_methodes.json")

    # Afficher résumé
    summary = results["summary"]
    print("\n" + "=" * 80)
    print("📊 RÉSUMÉ COMPARAISON PROFONDE")
    print("=" * 80)
    print(f"Méthodes officielles: {summary['methods_official']}")
    print(f"Méthodes BBIA: {summary['methods_bbia']}")
    print(f"Manquantes critiques: {summary['missing_critical']}")
    print(f"Différences signature: {summary['signature_diffs_count']}")
    print(f"Différences type retour: {summary['return_type_diffs_count']}")
    print(f"Score conformité: {summary['conformity']:.1f}%")
    print("=" * 80)


if __name__ == "__main__":
    main()
