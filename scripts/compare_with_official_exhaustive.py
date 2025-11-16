#!/usr/bin/env python3
"""Script exhaustif de comparaison BBIA-SIM vs Repo Officiel reachy_mini

Compare TOUS les aspects:
- API REST endpoints
- Classes et modules Python
- Modèles MuJoCo
- Assets STL
- Tests
- Documentation
- Scripts
- Helpers
- Installation SDK et méthodes (fusionné depuis audit_sdk_officiel_nov2025.py)
- Comparaison profonde signatures backend (fusionné depuis comparaison_profonde_methodes_backend.py)

Usage:
    python scripts/compare_with_official_exhaustive.py
"""

import ast
import inspect
import json
import logging
import subprocess
import sys
from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@dataclass
class Difference:
    """Une différence détectée entre BBIA et le repo officiel."""

    category: str  # API, Class, Model, Test, Doc, Script, Helper, Config
    file_path: str
    line: int | None
    endpoint: str | None
    description: str
    bbia_value: Any | None
    official_value: Any | None
    severity: str  # CRITICAL, HIGH, MEDIUM, LOW, INFO
    status: str  # pending, corrected, skipped
    correction: str | None
    test_file: str | None


@dataclass
class ComparisonResult:
    """Résultat d'une comparaison complète."""

    differences: list[Difference]
    summary: dict[str, int]
    checklist: dict[str, list[Difference]]


class OfficialRepoComparator:
    """Comparateur exhaustif BBIA vs repo officiel."""

    def __init__(self, bbia_root: Path, official_root: Path):
        self.bbia_root = Path(bbia_root)
        self.official_root = Path(official_root)
        self.differences: list[Difference] = []

        # Vérifier que les repos existent
        if not self.bbia_root.exists():
            raise ValueError(f"BBIA root non trouvé: {bbia_root}")
        if not self.official_root.exists():
            raise ValueError(f"Repo officiel non trouvé: {official_root}")

        logger.info(f"BBIA root: {self.bbia_root}")
        logger.info(f"Official root: {self.official_root}")

    def compare_directory_structure(self) -> list[Difference]:
        """Compare la structure des dossiers."""
        logger.info("📁 Comparaison structure des dossiers...")
        diffs = []

        # Dossiers à comparer
        dirs_to_check = [
            "src",
            "tests",
            "docs",
            "scripts",
            "examples",
        ]

        for dir_name in dirs_to_check:
            bbia_dir = self.bbia_root / dir_name
            official_dir = self.official_root / dir_name

            if not official_dir.exists():
                diffs.append(
                    Difference(
                        category="Structure",
                        file_path=f"{dir_name}/",
                        line=None,
                        endpoint=None,
                        description=f"Dossier {dir_name} absent dans repo officiel (OK si normal)",
                        bbia_value=str(bbia_dir.exists()),
                        official_value=None,
                        severity="INFO",
                        status="pending",
                        correction=None,
                        test_file=None,
                    ),
                )
                continue

            # Lister fichiers dans chaque
            bbia_files = (
                {f.name for f in bbia_dir.rglob("*.py") if f.is_file()}
                if bbia_dir.exists()
                else set()
            )
            official_files = {f.name for f in official_dir.rglob("*.py") if f.is_file()}

            missing_in_bbia = official_files - bbia_files

            for missing in missing_in_bbia:
                if missing != "__init__.py":  # Ignorer __init__.py
                    diffs.append(
                        Difference(
                            category="Structure",
                            file_path=f"{dir_name}/{missing}",
                            line=None,
                            endpoint=None,
                            description="Fichier présent dans repo officiel mais absent dans BBIA",
                            bbia_value=None,
                            official_value=missing,
                            severity="MEDIUM",
                            status="pending",
                            correction=f"Vérifier si nécessaire d'ajouter {missing}",
                            test_file=None,
                        ),
                    )

        return diffs

    def compare_rest_api_endpoints(self) -> list[Difference]:
        """Compare les endpoints REST API."""
        logger.info("🌐 Comparaison endpoints REST API...")
        diffs = []

        # Chercher les routers FastAPI dans BBIA
        bbia_routers = list(self.bbia_root.glob("src/bbia_sim/daemon/app/routers/*.py"))

        # Chercher les routers dans le repo officiel
        official_routers = list(self.official_root.glob("**/routers/*.py"))
        official_routers.extend(self.official_root.glob("**/*router*.py"))
        official_routers.extend(self.official_root.glob("**/*api*.py"))

        # Extraire endpoints de BBIA
        bbia_endpoints = set()
        for router_file in bbia_routers:
            try:
                content = router_file.read_text()
                # Chercher @router.get, @router.post, etc.
                import re

                matches = re.findall(
                    r'@router\.(get|post|put|delete|patch)\(["\']([^"\']+)["\']',
                    content,
                )
                for method, path in matches:
                    bbia_endpoints.add(f"{method.upper()} {path}")
            except Exception as e:
                logger.warning(f"Erreur lecture {router_file}: {e}")

        # Extraire endpoints du repo officiel
        official_endpoints = set()
        for router_file in official_routers:
            try:
                content = router_file.read_text()
                import re

                matches = re.findall(
                    r'@(app|router)\.(get|post|put|delete|patch)\(["\']([^"\']+)["\']',
                    content,
                )
                for _, method, path in matches:
                    official_endpoints.add(f"{method.upper()} {path}")
            except Exception as e:
                logger.debug(f"Erreur lecture {router_file}: {e}")

        # Comparer
        missing_in_bbia = official_endpoints - bbia_endpoints
        extra_in_bbia = bbia_endpoints - official_endpoints

        for endpoint in missing_in_bbia:
            diffs.append(
                Difference(
                    category="API",
                    file_path="src/bbia_sim/daemon/app/routers/",
                    line=None,
                    endpoint=endpoint,
                    description="Endpoint présent dans repo officiel mais absent dans BBIA",
                    bbia_value=None,
                    official_value=endpoint,
                    severity="HIGH",
                    status="pending",
                    correction=f"Ajouter endpoint {endpoint} si nécessaire",
                    test_file=None,
                ),
            )

        for endpoint in extra_in_bbia:
            diffs.append(
                Difference(
                    category="API",
                    file_path="src/bbia_sim/daemon/app/routers/",
                    line=None,
                    endpoint=endpoint,
                    description="Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)",
                    bbia_value=endpoint,
                    official_value=None,
                    severity="INFO",
                    status="pending",
                    correction=None,
                    test_file=None,
                ),
            )

        logger.info(f"  Endpoints BBIA: {len(bbia_endpoints)}")
        logger.info(f"  Endpoints officiels: {len(official_endpoints)}")
        logger.info(
            f"  Différences détectées: {len(missing_in_bbia)} manquants, {len(extra_in_bbia)} supplémentaires",
        )

        return diffs

    def compare_python_classes(self) -> list[Difference]:
        """Compare les classes Python principales (amélioré avec fonctionnalités fusionnées)."""
        logger.info("🐍 Comparaison classes Python...")
        diffs = []

        # Chercher classe ReachyMini dans repo officiel
        official_main = self.official_root / "src" / "reachy_mini" / "reachy_mini.py"

        # Chercher backend BBIA (ReachyMiniBackend)
        bbia_backend = (
            self.bbia_root / "src" / "bbia_sim" / "backends" / "reachy_mini_backend.py"
        )

        # Chercher BackendAdapter BBIA
        bbia_adapter = (
            self.bbia_root
            / "src"
            / "bbia_sim"
            / "daemon"
            / "app"
            / "backend_adapter.py"
        )

        # Chercher Backend officiel (abstract.py)
        official_backend_abstract = (
            self.official_root
            / "src"
            / "reachy_mini"
            / "daemon"
            / "backend"
            / "abstract.py"
        )

        if official_main.exists():
            try:
                import re

                def extract_methods(
                    file_path: Path,
                    include_private: bool = False,
                ) -> dict[str, dict[str, any]]:
                    """Extrait méthodes avec détails (async, property, ligne)."""
                    methods = {}
                    if not file_path.exists():
                        return methods

                    try:
                        content = file_path.read_text()
                        lines = content.split("\n")

                        for i, line in enumerate(lines, 1):
                            # Méthodes async
                            match = re.match(r"^\s+async def (\w+)\(", line)
                            if match:
                                method_name = match.group(1)
                                if include_private or not method_name.startswith("_"):
                                    methods[method_name] = {
                                        "async": True,
                                        "property": False,
                                        "line": i,
                                    }

                            # Méthodes sync
                            match = re.match(r"^\s+def (\w+)\(", line)
                            if match:
                                method_name = match.group(1)
                                if method_name not in methods and (
                                    include_private or not method_name.startswith("_")
                                ):
                                    methods[method_name] = {
                                        "async": False,
                                        "property": False,
                                        "line": i,
                                    }

                            # Properties
                            if re.match(r"^\s+@property", line) and i + 1 < len(lines):
                                prop_match = re.match(r"^\s+def (\w+)\(", lines[i])
                                if prop_match:
                                    methods[prop_match.group(1)] = {
                                        "async": False,
                                        "property": True,
                                        "line": i + 1,
                                    }
                    except Exception as e:
                        logger.warning(f"Erreur extraction méthodes {file_path}: {e}")

                    return methods

                # Extraire méthodes SDK officiel (ReachyMini)
                official_methods = extract_methods(official_main, include_private=False)

                # Extraire méthodes BBIA Backend
                bbia_backend_methods = (
                    extract_methods(bbia_backend, include_private=False)
                    if bbia_backend.exists()
                    else {}
                )

                # Comparer ReachyMini vs ReachyMiniBackend
                missing_methods = set(official_methods.keys()) - set(
                    bbia_backend_methods.keys(),
                )
                for method in missing_methods:
                    method_info = official_methods[method]
                    async_mark = "async " if method_info["async"] else ""
                    diffs.append(
                        Difference(
                            category="Class",
                            file_path=str(bbia_backend.relative_to(self.bbia_root)),
                            line=method_info["line"],
                            endpoint=None,
                            description=f"Méthode {async_mark}{method} présente dans SDK officiel mais absente dans BBIA backend",
                            bbia_value=None,
                            official_value=f"{method} (ligne {method_info['line']})",
                            severity="HIGH",
                            status="pending",
                            correction=f"Vérifier si {method} doit être implémentée",
                            test_file=None,
                        ),
                    )

                # Comparer BackendAdapter vs Backend abstract (si fichiers existent)
                if official_backend_abstract.exists() and bbia_adapter.exists():
                    official_backend_methods = extract_methods(
                        official_backend_abstract,
                        include_private=False,
                    )
                    bbia_adapter_methods = extract_methods(
                        bbia_adapter,
                        include_private=False,
                    )

                    missing_adapter = set(official_backend_methods.keys()) - set(
                        bbia_adapter_methods.keys(),
                    )
                    for method in missing_adapter:
                        if not method.startswith("_"):  # Ignorer méthodes privées
                            method_info = official_backend_methods[method]
                            async_mark = "async " if method_info["async"] else ""
                            diffs.append(
                                Difference(
                                    category="Class",
                                    file_path=str(
                                        bbia_adapter.relative_to(self.bbia_root),
                                    ),
                                    line=method_info["line"],
                                    endpoint=None,
                                    description=f"Méthode Backend {async_mark}{method} présente dans SDK officiel mais absente dans BackendAdapter",
                                    bbia_value=None,
                                    official_value=f"{method} (ligne {method_info['line']})",
                                    severity="HIGH",
                                    status="pending",
                                    correction=f"Vérifier si {method} doit être implémentée dans BackendAdapter",
                                    test_file=None,
                                ),
                            )

                    # Vérifier signatures async/sync compatibles
                    common_methods = set(official_backend_methods.keys()) & set(
                        bbia_adapter_methods.keys(),
                    )
                    for method in common_methods:
                        if not method.startswith("_"):
                            off_info = official_backend_methods[method]
                            bbia_info = bbia_adapter_methods[method]
                            if off_info["async"] != bbia_info["async"]:
                                diffs.append(
                                    Difference(
                                        category="Class",
                                        file_path=str(
                                            bbia_adapter.relative_to(self.bbia_root),
                                        ),
                                        line=bbia_info["line"],
                                        endpoint=None,
                                        description=f"Méthode {method} : signature async différente (officiel: {'async' if off_info['async'] else 'sync'}, BBIA: {'async' if bbia_info['async'] else 'sync'})",
                                        bbia_value=f"async={bbia_info['async']}",
                                        official_value=f"async={off_info['async']}",
                                        severity="MEDIUM",
                                        status="pending",
                                        correction=f"Aligner signature async de {method}",
                                        test_file=None,
                                    ),
                                )

                logger.info(f"  Méthodes SDK officiel: {len(official_methods)}")
                logger.info(f"  Méthodes BBIA backend: {len(bbia_backend_methods)}")
                logger.info(f"  Méthodes manquantes: {len(missing_methods)}")

            except Exception as e:
                logger.error(f"Erreur analyse classes: {e}")

        return diffs

    def compare_mujoco_models(self) -> list[Difference]:
        """Compare les modèles MuJoCo."""
        logger.info("🎮 Comparaison modèles MuJoCo...")
        diffs = []

        # Chercher modèles officiels
        official_models = list(self.official_root.glob("**/*.xml"))
        official_models = [
            m
            for m in official_models
            if "mjcf" in str(m).lower() or "reachy" in m.name.lower()
        ]

        # Chercher modèles BBIA
        bbia_models = list(self.bbia_root.glob("src/bbia_sim/sim/models/*.xml"))

        if official_models and bbia_models:
            # Comparer le modèle principal
            official_main = None
            for m in official_models:
                if "reachy_mini.xml" in m.name or "main" in m.name.lower():
                    official_main = m
                    break

            bbia_main = (
                self.bbia_root / "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
            )

            if official_main and bbia_main.exists():
                try:
                    official_content = official_main.read_text()
                    bbia_content = bbia_main.read_text()

                    # Comparer joints
                    import re

                    official_joints = set(
                        re.findall(r'<joint name="([^"]+)"', official_content),
                    )
                    bbia_joints = set(
                        re.findall(r'<joint name="([^"]+)"', bbia_content),
                    )

                    missing_joints = official_joints - bbia_joints

                    for joint in missing_joints:
                        diffs.append(
                            Difference(
                                category="Model",
                                file_path=str(bbia_main.relative_to(self.bbia_root)),
                                line=None,
                                endpoint=None,
                                description=f"Joint {joint} présent dans modèle officiel mais absent dans BBIA",
                                bbia_value=None,
                                official_value=joint,
                                severity="HIGH",
                                status="pending",
                                correction=f"Vérifier si joint {joint} doit être ajouté",
                                test_file=None,
                            ),
                        )

                except Exception as e:
                    logger.error(f"Erreur comparaison modèles: {e}")

        return diffs

    def compare_tests(self) -> list[Difference]:
        """Compare les tests."""
        logger.info("🧪 Comparaison tests...")
        diffs = []

        # Chercher tests officiels
        official_tests = list(self.official_root.glob("tests/**/*.py"))
        official_test_names = {f.stem for f in official_tests if f.is_file()}

        # Chercher tests BBIA
        bbia_tests = list(self.bbia_root.glob("tests/**/*.py"))
        bbia_test_names = {f.stem for f in bbia_tests if f.is_file()}

        # Tests manquants dans BBIA
        missing_tests = official_test_names - bbia_test_names

        for test_name in missing_tests:
            # Vérifier si c'est un test pertinent (pas juste setup/config)
            if "test" in test_name.lower() or "conform" in test_name.lower():
                diffs.append(
                    Difference(
                        category="Test",
                        file_path=f"tests/{test_name}.py",
                        line=None,
                        endpoint=None,
                        description=f"Test {test_name} présent dans repo officiel mais absent dans BBIA",
                        bbia_value=None,
                        official_value=test_name,
                        severity="MEDIUM",
                        status="pending",
                        correction=f"Vérifier si test {test_name} doit être ajouté",
                        test_file=None,
                    ),
                )

        logger.info(f"  Tests officiels: {len(official_test_names)}")
        logger.info(f"  Tests BBIA: {len(bbia_test_names)}")
        logger.info(f"  Tests manquants: {len(missing_tests)}")

        return diffs

    def compare_documentation(self) -> list[Difference]:
        """Compare la documentation."""
        logger.info("📚 Comparaison documentation...")
        diffs = []

        # Chercher README officiel
        official_readme = self.official_root / "README.md"
        bbia_readme = self.bbia_root / "README.md"

        if official_readme.exists() and bbia_readme.exists():
            official_content = official_readme.read_text()
            bbia_content = bbia_readme.read_text()

            # Chercher sections importantes
            sections_to_check = [
                "Installation",
                "Usage",
                "API",
                "Examples",
                "Testing",
            ]

            for section in sections_to_check:
                if (
                    section.lower() in official_content.lower()
                    and section.lower() not in bbia_content.lower()
                ):
                    diffs.append(
                        Difference(
                            category="Doc",
                            file_path="README.md",
                            line=None,
                            endpoint=None,
                            description=f"Section '{section}' présente dans README officiel mais absente dans BBIA",
                            bbia_value=None,
                            official_value=section,
                            severity="LOW",
                            status="pending",
                            correction=f"Vérifier si section '{section}' doit être ajoutée",
                            test_file=None,
                        ),
                    )

        return diffs

    def compare_assets_and_config(self) -> list[Difference]:
        """Compare assets (STL, WAV, JSON) et fichiers de configuration (amélioration)."""
        logger.info("📦 Comparaison assets et configuration...")
        diffs = []

        # Comparer fichiers audio (WAV)
        official_audio_dir = self.official_root / "src" / "reachy_mini" / "assets"
        bbia_audio_dir = self.bbia_root / "assets" / "voice"

        if official_audio_dir.exists():
            official_audio = {
                f.name
                for f in official_audio_dir.glob("*.wav")
                if not f.name.startswith("._")
            }
            bbia_audio = (
                {f.name for f in bbia_audio_dir.glob("*.wav")}
                if bbia_audio_dir.exists()
                else set()
            )

            missing_audio = official_audio - bbia_audio
            for audio_file in missing_audio:
                diffs.append(
                    Difference(
                        category="Assets",
                        file_path=f"assets/voice/{audio_file}",
                        line=None,
                        endpoint=None,
                        description=f"Fichier audio {audio_file} présent dans SDK officiel mais absent dans BBIA",
                        bbia_value=None,
                        official_value=audio_file,
                        severity="MEDIUM",
                        status="pending",
                        correction=f"Vérifier si {audio_file} doit être ajouté",
                        test_file=None,
                    ),
                )

        # Comparer constantes (constants.py)
        official_constants = (
            self.official_root / "src" / "reachy_mini" / "utils" / "constants.py"
        )
        if official_constants.exists():
            try:
                import re

                official_content = official_constants.read_text()
                # Chercher constantes importantes
                constants_patterns = [
                    r"URDF_ROOT_PATH",
                    r"ASSETS_ROOT_PATH",
                    r"MODELS_ROOT_PATH",
                ]
                found_constants = [
                    c for c in constants_patterns if re.search(c, official_content)
                ]

                if found_constants:
                    # Vérifier si ces constantes existent dans BBIA (chercher dans plusieurs fichiers)
                    bbia_config_files = [
                        self.bbia_root / "src" / "bbia_sim" / "global_config.py",
                        self.bbia_root / "src" / "bbia_sim" / "daemon" / "config.py",
                    ]

                    bbia_has_constants = False
                    for config_file in bbia_config_files:
                        if config_file.exists():
                            bbia_content = config_file.read_text()
                            if any(re.search(c, bbia_content) for c in found_constants):
                                bbia_has_constants = True
                                break

                    if not bbia_has_constants and found_constants:
                        diffs.append(
                            Difference(
                                category="Config",
                                file_path="src/bbia_sim/",
                                line=None,
                                endpoint=None,
                                description=f"Constantes {', '.join(found_constants)} présentes dans SDK officiel mais peut-être absentes dans BBIA",
                                bbia_value=None,
                                official_value="constants.py",
                                severity="MEDIUM",
                                status="pending",
                                correction="Vérifier si ces constantes doivent être ajoutées",
                                test_file=None,
                            ),
                        )
            except Exception as e:
                logger.debug(f"Erreur comparaison constantes: {e}")

        return diffs

    def check_sdk_installation(self) -> list[Difference]:
        """Vérifie l'installation SDK officiel (fusionné depuis audit_sdk_officiel_nov2025.py)."""
        logger.info("🔍 Vérification installation SDK officiel...")
        diffs = []

        try:
            result_pip = subprocess.run(
                ["pip", "show", "reachy-mini"],
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result_pip.returncode != 0:
                diffs.append(
                    Difference(
                        category="SDK",
                        file_path="",
                        line=None,
                        endpoint=None,
                        description="SDK reachy-mini non installé",
                        bbia_value=None,
                        official_value="reachy-mini",
                        severity="HIGH",
                        status="pending",
                        correction="Installer avec: pip install reachy-mini",
                        test_file=None,
                    ),
                )
        except Exception as e:
            logger.debug(f"Erreur vérification SDK: {e}")

        return diffs

    def check_sdk_methods_implementation(self) -> list[Difference]:
        """Vérifie que toutes les méthodes SDK officiel sont implémentées (fusionné depuis audit_sdk_officiel_nov2025.py)."""
        logger.info("🔍 Vérification méthodes SDK implémentées...")
        diffs = []

        # Méthodes officielles selon README
        official_methods = {
            "wake_up",
            "goto_sleep",
            "look_at_world",
            "look_at_image",
            "goto_target",
            "set_target",
            "get_current_joint_positions",
            "set_target_head_pose",
            "set_target_body_yaw",
            "get_current_head_pose",
            "get_present_antenna_joint_positions",
            "enable_motors",
            "disable_motors",
            "enable_gravity_compensation",
            "disable_gravity_compensation",
        }

        # Vérifier notre implémentation
        try:
            sys.path.insert(0, str(self.bbia_root / "src"))
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend_attrs = set(dir(ReachyMiniBackend))
            missing_methods = official_methods - backend_attrs

            for method in missing_methods:
                diffs.append(
                    Difference(
                        category="SDK",
                        file_path="src/bbia_sim/backends/reachy_mini_backend.py",
                        line=None,
                        endpoint=None,
                        description=f"Méthode SDK {method} manquante dans ReachyMiniBackend",
                        bbia_value=None,
                        official_value=method,
                        severity="HIGH",
                        status="pending",
                        correction=f"Implémenter méthode {method}",
                        test_file=None,
                    ),
                )
        except ImportError as e:
            logger.warning(f"Impossible d'importer ReachyMiniBackend: {e}")

        return diffs

    def check_create_head_pose(self) -> list[Difference]:
        """Vérifie create_head_pose (fusionné depuis audit_sdk_officiel_nov2025.py)."""
        logger.info("🔍 Vérification create_head_pose...")
        diffs = []

        try:
            sys.path.insert(0, str(self.bbia_root / "src"))
            try:
                from reachy_mini.utils import create_head_pose

                sig = inspect.signature(create_head_pose)
                params = list(sig.parameters.keys())

                # Selon README: create_head_pose(z=10, roll=15, degrees=True, mm=True)
                expected_params = {"z", "roll", "degrees", "mm", "pitch", "yaw"}
                if not set(params).intersection(expected_params):
                    diffs.append(
                        Difference(
                            category="SDK",
                            file_path="",
                            line=None,
                            endpoint=None,
                            description="create_head_pose signature peut différer",
                            bbia_value=str(params),
                            official_value="z, roll, degrees, mm, pitch, yaw",
                            severity="MEDIUM",
                            status="pending",
                            correction="Vérifier signature create_head_pose",
                            test_file=None,
                        ),
                    )
            except ImportError:
                diffs.append(
                    Difference(
                        category="SDK",
                        file_path="",
                        line=None,
                        endpoint=None,
                        description="create_head_pose non disponible",
                        bbia_value=None,
                        official_value="create_head_pose",
                        severity="HIGH",
                        status="pending",
                        correction="Installer SDK officiel: pip install reachy-mini",
                        test_file=None,
                    ),
                )
        except Exception as e:
            logger.debug(f"Erreur vérification create_head_pose: {e}")

        return diffs

    def check_python_versions(self) -> list[Difference]:
        """Vérifie versions Python supportées (fusionné depuis audit_sdk_officiel_nov2025.py)."""
        logger.info("🔍 Vérification versions Python...")
        diffs = []

        python_version = sys.version_info
        if not (3.10 <= python_version.minor <= 3.13):
            diffs.append(
                Difference(
                    category="SDK",
                    file_path="",
                    line=None,
                    endpoint=None,
                    description=f"Python {python_version.major}.{python_version.minor} - officiel supporte 3.10-3.13",
                    bbia_value=f"{python_version.major}.{python_version.minor}",
                    official_value="3.10-3.13",
                    severity="MEDIUM",
                    status="pending",
                    correction="Vérifier compatibilité avec version Python",
                    test_file=None,
                ),
            )

        return diffs

    def extract_method_signatures_ast(
        self,
        file_path: Path,
        class_name: str,
    ) -> dict[str, dict]:
        """Extrait les signatures des méthodes d'une classe avec AST (fusionné depuis comparaison_profonde_methodes_backend.py)."""
        if not file_path.exists():
            return {}

        try:
            content = file_path.read_text(encoding="utf-8")
            tree = ast.parse(content)

            methods: dict[str, dict] = {}

            for node in ast.walk(tree):
                if isinstance(node, ast.ClassDef) and node.name == class_name:
                    for item in node.body:
                        if isinstance(item, ast.FunctionDef) or isinstance(
                            item, ast.AsyncFunctionDef
                        ):
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
                                    try:
                                        param_info["type"] = ast.unparse(arg.annotation)
                                    except Exception:
                                        param_info["type"] = None
                                params.append(param_info)

                            # Type de retour
                            return_type = None
                            if item.returns:
                                try:
                                    return_type = ast.unparse(item.returns)
                                except Exception:
                                    return_type = None

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
            logger.warning(f"Erreur extraction AST {file_path}: {e}")
            return {}

    def compare_backend_methods_deep(self) -> list[Difference]:
        """Compare les méthodes backend en profondeur avec AST (fusionné depuis comparaison_profonde_methodes_backend.py)."""
        logger.info("🔍 Comparaison profonde méthodes backend (AST)...")
        diffs = []

        # Extraire méthodes officielles
        official_backend = (
            self.official_root / "src/reachy_mini/daemon/backend/abstract.py"
        )
        official_methods = self.extract_method_signatures_ast(official_backend, "Backend")

        # Extraire méthodes BackendAdapter BBIA
        bbia_adapter = self.bbia_root / "src/bbia_sim/daemon/app/backend_adapter.py"
        adapter_methods = self.extract_method_signatures_ast(bbia_adapter, "BackendAdapter")

        # Extraire méthodes ReachyMiniBackend BBIA
        bbia_backend = self.bbia_root / "src/bbia_sim/backends/reachy_mini_backend.py"
        backend_methods = self.extract_method_signatures_ast(
            bbia_backend,
            "ReachyMiniBackend",
        )

        # Combiner méthodes BBIA (adapter + backend)
        all_bbia_methods = {**adapter_methods, **backend_methods}

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
                diffs.append(
                    Difference(
                        category="Backend",
                        file_path="src/bbia_sim/backends/reachy_mini_backend.py",
                        line=None,
                        endpoint=None,
                        description=f"Méthode critique {method_name} manquante dans BBIA",
                        bbia_value=None,
                        official_value=method_name,
                        severity="HIGH",
                        status="pending",
                        correction=f"Implémenter méthode {method_name}",
                        test_file=None,
                    ),
                )
                continue

            # Comparer signatures
            official_params = {p["name"]: p.get("type") for p in official["params"]}
            bbia_params = {p["name"]: p.get("type") for p in bbia["params"]}

            if official_params != bbia_params:
                diffs.append(
                    Difference(
                        category="Backend",
                        file_path="src/bbia_sim/backends/reachy_mini_backend.py",
                        line=None,
                        endpoint=None,
                        description=f"Méthode {method_name}: différences de signature (paramètres)",
                        bbia_value=str(bbia_params),
                        official_value=str(official_params),
                        severity="MEDIUM",
                        status="pending",
                        correction=f"Aligner signature de {method_name}",
                        test_file=None,
                    ),
                )

            # Comparer types de retour
            if official.get("return_type") != bbia.get("return_type"):
                diffs.append(
                    Difference(
                        category="Backend",
                        file_path="src/bbia_sim/backends/reachy_mini_backend.py",
                        line=None,
                        endpoint=None,
                        description=f"Méthode {method_name}: différences de type de retour",
                        bbia_value=bbia.get("return_type"),
                        official_value=official.get("return_type"),
                        severity="MEDIUM",
                        status="pending",
                        correction=f"Aligner type de retour de {method_name}",
                        test_file=None,
                    ),
                )

        return diffs

    def run_full_comparison(self) -> ComparisonResult:
        """Exécute la comparaison complète."""
        logger.info("🚀 Démarrage comparaison exhaustive BBIA vs Repo Officiel")
        logger.info("=" * 80)

        all_diffs = []

        # 1. Structure
        all_diffs.extend(self.compare_directory_structure())

        # 2. API REST
        all_diffs.extend(self.compare_rest_api_endpoints())

        # 3. Classes Python
        all_diffs.extend(self.compare_python_classes())

        # 4. Modèles MuJoCo
        all_diffs.extend(self.compare_mujoco_models())

        # 5. Tests
        all_diffs.extend(self.compare_tests())

        # 6. Documentation
        all_diffs.extend(self.compare_documentation())

        # 7. Assets et fichiers de configuration (amélioration)
        all_diffs.extend(self.compare_assets_and_config())

        # 8. Vérification SDK installation et méthodes (fusionné depuis audit_sdk_officiel_nov2025.py)
        all_diffs.extend(self.check_sdk_installation())
        all_diffs.extend(self.check_sdk_methods_implementation())
        all_diffs.extend(self.check_create_head_pose())
        all_diffs.extend(self.check_python_versions())

        # 9. Comparaison profonde méthodes backend (fusionné depuis comparaison_profonde_methodes_backend.py)
        all_diffs.extend(self.compare_backend_methods_deep())

        # Résumé
        summary = {
            "total": len(all_diffs),
            "critical": len([d for d in all_diffs if d.severity == "CRITICAL"]),
            "high": len([d for d in all_diffs if d.severity == "HIGH"]),
            "medium": len([d for d in all_diffs if d.severity == "MEDIUM"]),
            "low": len([d for d in all_diffs if d.severity == "LOW"]),
            "info": len([d for d in all_diffs if d.severity == "INFO"]),
        }

        # Checklist par catégorie
        checklist = defaultdict(list)
        for diff in all_diffs:
            checklist[diff.category].append(diff)

        return ComparisonResult(
            differences=all_diffs,
            summary=summary,
            checklist=dict(checklist),
        )

    def save_results(self, result: ComparisonResult, output_file: Path):
        """Sauvegarde les résultats dans un fichier JSON."""
        output_file.parent.mkdir(parents=True, exist_ok=True)

        data = {
            "summary": result.summary,
            "differences": [asdict(d) for d in result.differences],
            "checklist": {
                cat: [asdict(d) for d in diffs]
                for cat, diffs in result.checklist.items()
            },
        }

        with open(output_file, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False, default=str)

        logger.info(f"✅ Résultats sauvegardés: {output_file}")

    def generate_markdown_report(self, result: ComparisonResult, output_file: Path):
        """Génère un rapport Markdown."""
        output_file.parent.mkdir(parents=True, exist_ok=True)

        with open(output_file, "w", encoding="utf-8") as f:
            f.write(
                "# 🔍 Rapport de Comparaison Exhaustive - BBIA vs Repo Officiel\n\n",
            )
            f.write(f"**Date**: {Path(__file__).stat().st_mtime}\n\n")
            f.write("## 📊 Résumé Exécutif\n\n")
            f.write(f"- **Total différences**: {result.summary['total']}\n")
            f.write(f"- **🔴 CRITICAL**: {result.summary['critical']}\n")
            f.write(f"- **🟠 HIGH**: {result.summary['high']}\n")
            f.write(f"- **🟡 MEDIUM**: {result.summary['medium']}\n")
            f.write(f"- **🟢 LOW**: {result.summary['low']}\n")
            f.write(f"- **ℹ️ INFO**: {result.summary['info']}\n\n")

            f.write("## 📋 Checklist par Catégorie\n\n")
            for category, diffs in sorted(result.checklist.items()):
                f.write(f"### {category} ({len(diffs)} différences)\n\n")
                for i, diff in enumerate(diffs, 1):
                    f.write(f"{i}. **{diff.severity}** - {diff.description}\n")
                    f.write(f"   - Fichier: `{diff.file_path}`\n")
                    if diff.endpoint:
                        f.write(f"   - Endpoint: `{diff.endpoint}`\n")
                    if diff.line:
                        f.write(f"   - Ligne: {diff.line}\n")
                    if diff.correction:
                        f.write(f"   - Correction: {diff.correction}\n")
                    f.write(f"   - Statut: {diff.status}\n\n")

            f.write("## ✅ Actions Recommandées\n\n")
            f.write("### Priorité CRITICAL\n")
            critical_diffs = [d for d in result.differences if d.severity == "CRITICAL"]
            for diff in critical_diffs:
                f.write(f"- [ ] {diff.description}\n")

            f.write("\n### Priorité HIGH\n")
            high_diffs = [d for d in result.differences if d.severity == "HIGH"]
            for diff in high_diffs[:10]:  # Limiter à 10
                f.write(f"- [ ] {diff.description}\n")
            if len(high_diffs) > 10:
                f.write(f"- ... et {len(high_diffs) - 10} autres\n")

        logger.info(f"✅ Rapport Markdown généré: {output_file}")


def main():
    """Point d'entrée principal."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Comparaison exhaustive BBIA vs Repo Officiel",
    )
    parser.add_argument(
        "--bbia-root",
        type=Path,
        default=Path(__file__).parent.parent,
        help="Racine du projet BBIA",
    )
    parser.add_argument(
        "--official-root",
        type=Path,
        default=Path("/Volumes/T7/reachy_mini"),
        help="Racine du repo officiel",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("logs"),
        help="Répertoire de sortie",
    )

    args = parser.parse_args()

    # Créer le comparateur
    comparator = OfficialRepoComparator(args.bbia_root, args.official_root)

    # Exécuter la comparaison
    result = comparator.run_full_comparison()

    # Sauvegarder les résultats
    output_dir = args.output_dir
    comparator.save_results(result, output_dir / "comparison_official_results.json")
    comparator.generate_markdown_report(
        result,
        output_dir / "comparison_official_report.md",
    )

    # Afficher le résumé
    logger.info("\n" + "=" * 80)
    logger.info("📊 RÉSUMÉ DE LA COMPARAISON")
    logger.info("=" * 80)
    logger.info(f"Total différences: {result.summary['total']}")
    logger.info(f"  🔴 CRITICAL: {result.summary['critical']}")
    logger.info(f"  🟠 HIGH: {result.summary['high']}")
    logger.info(f"  🟡 MEDIUM: {result.summary['medium']}")
    logger.info(f"  🟢 LOW: {result.summary['low']}")
    logger.info(f"  ℹ️ INFO: {result.summary['info']}")

    logger.info(f"\n📁 Rapports générés dans: {output_dir}")
    logger.info(f"  - JSON: {output_dir / 'comparison_official_results.json'}")
    logger.info(f"  - Markdown: {output_dir / 'comparison_official_report.md'}")


if __name__ == "__main__":
    main()
