"""Module d'installation automatique d'apps depuis Hugging Face Spaces."""

from __future__ import annotations

import asyncio
import logging
import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

_APP_NAME_RE = re.compile(r"^[A-Za-z0-9._-]+$")

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
        raw_dir = apps_dir or INSTALLED_APPS_DIR
        raw_dir.mkdir(parents=True, exist_ok=True)
        # Base figée (resolved) — toutes les ops path restent sous ce préfixe.
        self.apps_dir = Path(os.path.realpath(str(raw_dir)))

    @staticmethod
    def _normalize_space_name(name: str) -> str:
        """Normalise et valide un nom d'app local (allowlist, pas de séparateurs)."""
        normalized = (name or "").strip()
        if not normalized or not _APP_NAME_RE.fullmatch(normalized):
            raise ValueError(f"Nom d'application invalide: {name!r}")
        return normalized

    def _safe_app_path(self, app_name: str) -> Path:
        """Construit un chemin d'app confiné sous apps_dir (sanitizer CodeQL)."""
        safe_name = self._normalize_space_name(app_name)
        base = os.path.realpath(str(self.apps_dir))
        # Join uniquement le basename validé — jamais un chemin utilisateur brut.
        candidate = os.path.realpath(os.path.join(base, safe_name))
        if os.path.commonpath([base, candidate]) != base:
            raise ValueError(f"Chemin d'application invalide: {app_name!r}")
        return Path(candidate)

    def _ensure_under_apps_dir(self, path: Path) -> Path:
        """Refuse tout chemin qui sort de apps_dir après resolve."""
        base = os.path.realpath(str(self.apps_dir))
        candidate = os.path.realpath(str(path))
        if os.path.commonpath([base, candidate]) != base:
            raise ValueError(f"Chemin hors répertoire apps: {path!r}")
        return Path(candidate)

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

        local_name = self._normalize_space_name(
            app_name or hf_space_id.split("/")[-1]
        )
        app_path = self._safe_app_path(local_name)

        # Vérifier si déjà installé
        if app_path.exists():
            logger.info("App %s déjà installée dans %s", local_name, app_path)
            return {
                "status": "already_installed",
                "app_name": local_name,
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
                    "Échec clonage app %s depuis %s: %s",
                    local_name,
                    hf_space_id,
                    error_msg,
                )
                raise RuntimeError(
                    f"Échec clonage app '{local_name}' depuis Hugging Face Space "
                    f"'{hf_space_id}'. Vérifiez que le Space existe et les permissions."
                )

            logger.info("App %s clonée avec succès", local_name)

            # Installer les dépendances si requirements.txt existe
            requirements_file = self._ensure_under_apps_dir(
                app_path / "requirements.txt"
            )
            if requirements_file.exists():
                logger.info("Installation des dépendances depuis requirements.txt...")
                await self._install_dependencies(requirements_file)

            return {
                "status": "installed",
                "app_name": local_name,
                "app_path": str(app_path),
            }

        except Exception as e:
            # Nettoyer en cas d'erreur
            if app_path.exists():
                shutil.rmtree(app_path, ignore_errors=True)
            logger.exception(
                "Erreur installation app %s (HF Space: %s): %s",
                local_name,
                hf_space_id,
                e,
            )
            raise RuntimeError(
                f"Échec installation app '{local_name}' depuis Hugging Face Space "
                f"'{hf_space_id}'. Vérifiez réseau, permissions et existence du Space."
            ) from e

    async def _install_dependencies(self, requirements_file: Path) -> None:
        """Installe les dépendances depuis requirements.txt.

        Args:
            requirements_file: Chemin vers requirements.txt (doit être sous apps_dir)

        Raises:
            RuntimeError: Si l'installation des dépendances échoue
        """
        safe_req = self._ensure_under_apps_dir(requirements_file)
        try:
            result = await asyncio.to_thread(
                subprocess.run,
                [
                    "pip",
                    "install",
                    "-r",
                    str(safe_req),
                ],
                capture_output=True,
                text=True,
                check=False,
            )

            if result.returncode != 0:
                logger.warning(
                    "Erreur installation dépendances: %s",
                    result.stderr or result.stdout,
                )
            else:
                logger.info("Dépendances installées avec succès")

        except Exception as e:
            logger.warning("Erreur installation dépendances: %s", e)

    async def uninstall_app(self, app_name: str) -> dict[str, Any]:
        """Désinstalle une app.

        Args:
            app_name: Nom de l'app à désinstaller

        Returns:
            Dictionnaire avec statut de la désinstallation

        Raises:
            ValueError: Si l'app n'est pas installée
        """
        app_path = self._safe_app_path(app_name)

        if not app_path.exists():
            raise ValueError(f"App {app_name} non installée")

        try:
            shutil.rmtree(app_path)
            logger.info("App %s désinstallée", app_name)
            return {
                "status": "uninstalled",
                "app_name": app_name,
            }
        except Exception as e:
            logger.exception("Erreur désinstallation app %s: %s", app_name, e)
            raise RuntimeError("Échec désinstallation de l'application") from e

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
                safe_path = self._ensure_under_apps_dir(app_path)
                installed.append(
                    {
                        "name": safe_path.name,
                        "app_path": str(safe_path),
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
        normalized = (app_name or "").strip()
        if not normalized:
            return False

        # Noms avec slash : ne tester que le basename allowlisté.
        candidates: list[str] = []
        if "/" in normalized:
            candidates.append(normalized.split("/")[-1])
        else:
            candidates.append(normalized)

        for candidate_name in candidates:
            try:
                app_path = self._safe_app_path(candidate_name)
            except ValueError:
                continue
            if app_path.exists() and app_path.is_dir():
                return True

        # Vérifier aussi par entry point si un setup.py existe
        needle = candidates[0]
        for installed_dir in self.apps_dir.iterdir():
            if not installed_dir.is_dir():
                continue
            try:
                safe_dir = self._ensure_under_apps_dir(installed_dir)
            except ValueError:
                continue

            setup_py = self._ensure_under_apps_dir(safe_dir / "setup.py")
            if not setup_py.exists():
                continue
            try:
                with open(setup_py, encoding="utf-8") as f:
                    content = f.read()
                if f'name="{needle}"' in content or f"name='{needle}'" in content:
                    return True
            except OSError:
                continue

        return False
