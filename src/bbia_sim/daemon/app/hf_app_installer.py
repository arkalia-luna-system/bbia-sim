"""Module d'installation automatique d'apps depuis Hugging Face Spaces."""

import asyncio
import logging
import shutil
import subprocess
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# Répertoire pour stocker les apps installées
INSTALLED_APPS_DIR = Path("installed_apps")
INSTALLED_APPS_DIR.mkdir(parents=True, exist_ok=True)


class HFAppInstaller:
    """Gestionnaire d'installation d'apps depuis Hugging Face Spaces."""

    def __init__(self, apps_dir: Path | None = None) -> None:
        """Initialise l'installer.

        Args:
            apps_dir: Répertoire pour stocker les apps installées (défaut: installed_apps/)
        """
        self.apps_dir = apps_dir or INSTALLED_APPS_DIR
        self.apps_dir.mkdir(parents=True, exist_ok=True)

    async def install_app(
        self,
        hf_space_id: str,
        app_name: str | None = None,
    ) -> dict[str, Any]:
        """Installe une app depuis Hugging Face Spaces.

        Args:
            hf_space_id: ID du Space HF (format: "username/space-name")
            app_name: Nom de l'app (optionnel, utilise hf_space_id si None)

        Returns:
            Dictionnaire avec statut de l'installation

        Raises:
            ValueError: Si hf_space_id est invalide
            RuntimeError: Si l'installation échoue
        """
        if not hf_space_id or "/" not in hf_space_id:
            raise ValueError(f"HF Space ID invalide: {hf_space_id}")

        app_name = app_name or hf_space_id.split("/")[-1]
        app_path = self.apps_dir / app_name

        # Vérifier si déjà installé
        if app_path.exists():
            logger.info("App %s déjà installée dans %s", app_name, app_path)
            return {
                "status": "already_installed",
                "app_name": app_name,
                "app_path": str(app_path),
            }

        try:
            # Cloner le repo HF Space
            logger.info("Clonage de %s vers %s...", hf_space_id, app_path)
            repo_url = f"https://huggingface.co/spaces/{hf_space_id}"

            # Exécuter git clone dans un thread séparé
            result = await asyncio.to_thread(
                subprocess.run,
                ["git", "clone", repo_url, str(app_path)],
                capture_output=True,
                text=True,
                check=False,
            )

            if result.returncode != 0:
                # Nettoyer en cas d'erreur
                if app_path.exists():
                    shutil.rmtree(app_path, ignore_errors=True)
                error_msg = result.stderr or result.stdout or "Erreur inconnue"
                logger.error(
                    "❌ Échec clonage app %s depuis %s: %s",
                    app_name,
                    hf_space_id,
                    error_msg,
                )
                raise RuntimeError(
                    f"Échec clonage app '{app_name}' depuis Hugging Face Space '{hf_space_id}': {error_msg}. "
                    f"Vérifiez que le Space existe et que vous avez les permissions nécessaires."
                )

            logger.info("✅ App %s clonée avec succès", app_name)

            # Installer les dépendances si requirements.txt existe
            requirements_file = app_path / "requirements.txt"
            if requirements_file.exists():
                logger.info("Installation des dépendances depuis requirements.txt...")
                await self._install_dependencies(requirements_file)

            return {
                "status": "installed",
                "app_name": app_name,
                "app_path": str(app_path),
            }

        except Exception as e:
            # Nettoyer en cas d'erreur
            if app_path.exists():
                shutil.rmtree(app_path, ignore_errors=True)
            logger.exception(
                "❌ Erreur installation app %s (HF Space: %s): %s",
                app_name,
                hf_space_id,
                e,
            )
            error_type = type(e).__name__
            raise RuntimeError(
                f"Échec installation app '{app_name}' depuis Hugging Face Space '{hf_space_id}': "
                f"{error_type}: {str(e)}. "
                f"Vérifiez votre connexion réseau, les permissions du Space, et que le Space existe bien."
            ) from e

    async def _install_dependencies(self, requirements_file: Path) -> None:
        """Installe les dépendances depuis requirements.txt.

        Args:
            requirements_file: Chemin vers requirements.txt

        Raises:
            RuntimeError: Si l'installation des dépendances échoue
        """
        try:
            # Installer avec pip
            result = await asyncio.to_thread(
                subprocess.run,
                [
                    "pip",
                    "install",
                    "-r",
                    str(requirements_file),
                ],
                capture_output=True,
                text=True,
                check=False,
            )

            if result.returncode != 0:
                logger.warning(
                    "⚠️ Erreur installation dépendances: %s",
                    result.stderr or result.stdout,
                )
                # Ne pas faire échouer l'installation si dépendances échouent
                # (certaines apps peuvent fonctionner sans toutes les dépendances)
            else:
                logger.info("✅ Dépendances installées avec succès")

        except Exception as e:
            logger.warning("⚠️ Erreur installation dépendances: %s", e)
            # Ne pas faire échouer l'installation

    async def uninstall_app(self, app_name: str) -> dict[str, Any]:
        """Désinstalle une app.

        Args:
            app_name: Nom de l'app à désinstaller

        Returns:
            Dictionnaire avec statut de la désinstallation

        Raises:
            ValueError: Si l'app n'est pas installée
        """
        app_path = self.apps_dir / app_name

        if not app_path.exists():
            raise ValueError(f"App {app_name} non installée")

        try:
            shutil.rmtree(app_path)
            logger.info("✅ App %s désinstallée", app_name)
            return {
                "status": "uninstalled",
                "app_name": app_name,
            }
        except Exception as e:
            logger.exception("Erreur désinstallation app %s: %s", app_name, e)
            raise RuntimeError(f"Échec désinstallation: {e}") from e

    def list_installed_apps(self) -> list[dict[str, Any]]:
        """Liste les apps installées.

        Returns:
            Liste des apps installées avec leurs informations
        """
        installed: list[dict[str, Any]] = []
        if not self.apps_dir.exists():
            return installed

        for app_path in self.apps_dir.iterdir():
            if app_path.is_dir() and not app_path.name.startswith("."):
                installed.append(
                    {
                        "name": app_path.name,
                        "app_path": str(app_path),
                        "installed": True,
                    },
                )

        return installed

    def is_installed(self, app_name: str) -> bool:
        """Vérifie si une app est installée.

        FIX v1.2.13: Vérifie par nom space ET par nom entry point.
        Certaines apps peuvent avoir un nom d'entry point différent du nom du space.

        Args:
            app_name: Nom de l'app (peut être "username/space-name" ou juste "space-name")

        Returns:
            True si installée, False sinon
        """
        # Essayer d'abord avec le nom complet
        app_path = self.apps_dir / app_name
        if app_path.exists() and app_path.is_dir():
            return True

        # Essayer avec juste le nom du space (sans username/)
        if "/" in app_name:
            space_name = app_name.split("/")[-1]
            app_path = self.apps_dir / space_name
            if app_path.exists() and app_path.is_dir():
                return True

        # Vérifier aussi par entry point si un setup.py ou pyproject.toml existe
        for installed_dir in self.apps_dir.iterdir():
            if not installed_dir.is_dir():
                continue

            # Vérifier si le nom correspond à un entry point dans setup.py
            setup_py = installed_dir / "setup.py"
            if setup_py.exists():
                try:
                    with open(setup_py) as f:
                        content = f.read()
                        # Chercher le nom de l'app dans setup.py
                        if (
                            f'name="{app_name}"' in content
                            or f"name='{app_name}'" in content
                        ):
                            return True
                        # Chercher aussi par space name
                        if "/" in app_name:
                            space_name = app_name.split("/")[-1]
                            if (
                                f'name="{space_name}"' in content
                                or f"name='{space_name}'" in content
                            ):
                                return True
                except Exception:
                    pass

        return False
