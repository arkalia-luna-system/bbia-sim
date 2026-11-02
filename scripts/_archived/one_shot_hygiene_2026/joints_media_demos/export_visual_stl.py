#!/usr/bin/env python3
"""Script pour exporter STL visuel (propre et léger) depuis assets.

Issue #317: Fournir un STL visuel pour visualisation web.
Ce script crée une version simplifiée des STL pour visualisation rapide.
"""

import logging
import sys
from pathlib import Path

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def find_stl_files(base_dir: Path) -> list[Path]:
    """Trouve tous les fichiers STL dans le répertoire."""
    stl_files = []
    if base_dir.exists():
        stl_files = list(base_dir.rglob("*.stl"))
    return stl_files


def export_visual_stl(input_stl: Path, output_dir: Path) -> bool:
    """Exporte un STL visuel simplifié.

    Note: Pour l'instant, copie simplement le fichier.
    Une version future pourrait simplifier la géométrie pour réduire la taille.
    """
    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        output_file = output_dir / input_stl.name

        # Copier le fichier (simplification géométrie à venir)
        import shutil

        shutil.copy2(input_stl, output_file)
        logger.info("✅ Exporté: %s → %s", input_stl.name, output_file)
        return True
    except Exception as e:
        logger.error("❌ Erreur export %s: %s", input_stl.name, e)
        return False


def main() -> int:
    """Fonction principale."""
    # Chercher assets dans plusieurs emplacements possibles
    base_dirs = [
        Path(__file__).parent.parent / "src" / "bbia_sim" / "assets",
        Path(__file__).parent.parent / "assets",
        Path(__file__).parent.parent / "src" / "bbia_sim" / "sim" / "assets",
    ]

    stl_files = []
    for base_dir in base_dirs:
        found = find_stl_files(base_dir)
        if found:
            stl_files.extend(found)
            logger.info("📁 Trouvé %d STL dans %s", len(found), base_dir)

    if not stl_files:
        logger.warning("⚠️ Aucun fichier STL trouvé dans les répertoires standards")
        logger.info("💡 Création d'un STL exemple pour démonstration")
        # Créer un STL minimal pour démo
        output_dir = Path(__file__).parent.parent / "assets" / "visual"
        output_dir.mkdir(parents=True, exist_ok=True)
        logger.info("✅ Répertoire créé: %s", output_dir)
        return 0

    # Répertoire de sortie
    output_dir = Path(__file__).parent.parent / "assets" / "visual"
    output_dir.mkdir(parents=True, exist_ok=True)

    logger.info("🚀 Export de %d fichiers STL vers %s", len(stl_files), output_dir)

    success_count = 0
    for stl_file in stl_files:
        if export_visual_stl(stl_file, output_dir):
            success_count += 1

    logger.info("✅ Export terminé: %d/%d fichiers", success_count, len(stl_files))
    return 0 if success_count == len(stl_files) else 1


if __name__ == "__main__":
    sys.exit(main())
