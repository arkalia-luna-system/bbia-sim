#!/usr/bin/env python3
"""Wrapper Bandit qui nettoie automatiquement les fichiers macOS cachés avant l'exécution.

Ce script supprime automatiquement les fichiers `._*` (macOS) qui peuvent causer
des erreurs de parsing et bloquer l'exécution de Bandit.
"""

import subprocess
import sys
from pathlib import Path


def clean_macos_hidden_files(root_dir: Path) -> int:
    """Supprime les fichiers macOS cachés (._*) récursivement.

    Args:
        root_dir: Répertoire racine à nettoyer

    Returns:
        Nombre de fichiers supprimés
    """
    deleted_count = 0
    for file_path in root_dir.rglob("._*"):
        if file_path.is_file():
            try:
                file_path.unlink()
                deleted_count += 1
            except OSError:
                pass  # Ignorer erreurs (fichier déjà supprimé, permissions, etc.)
    return deleted_count


def main() -> int:
    """Point d'entrée principal."""
    repo_root = Path(__file__).parent.parent.resolve()

    # Nettoyer fichiers macOS cachés
    print("🧹 Nettoyage fichiers macOS cachés...", end=" ", flush=True)
    src_deleted = clean_macos_hidden_files(repo_root / "src")
    tests_deleted = clean_macos_hidden_files(repo_root / "tests")
    total_deleted = src_deleted + tests_deleted

    if total_deleted > 0:
        print(f"✅ {total_deleted} fichier(s) supprimé(s)")
    else:
        print("✅ Aucun fichier à nettoyer")

    # Construire commande Bandit
    bandit_cmd = [
        sys.executable,
        "-m",
        "bandit",
        "-r",
        str(repo_root / "src"),
        "-x",
        str(repo_root / "tests"),
        "-c",
        str(repo_root / ".bandit"),
    ]

    # Ajouter arguments supplémentaires depuis ligne de commande
    if len(sys.argv) > 1:
        bandit_cmd.extend(sys.argv[1:])

    # Exécuter Bandit
    try:
        result = subprocess.run(bandit_cmd, cwd=repo_root, check=False)
        return result.returncode
    except KeyboardInterrupt:
        print("\n⚠️  Interruption utilisateur")
        return 130
    except Exception as e:
        print(f"❌ Erreur exécution Bandit: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
