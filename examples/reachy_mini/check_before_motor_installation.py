#!/usr/bin/env python3
"""Script de vérification avant installation des nouveaux moteurs.

Ce script vérifie que tout est prêt avant d'installer les nouveaux moteurs :
1. Vérification de la version SDK
2. Vérification de l'état actuel des moteurs
3. Vérification de la documentation
4. Checklist de préparation

Usage:
    python examples/reachy_mini/check_before_motor_installation.py
"""

import subprocess
import sys
from pathlib import Path

# Ajouter le répertoire racine au path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))


def print_header(title: str) -> None:
    """Affiche un en-tête."""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70 + "\n")


def check_sdk_version() -> bool:
    """Vérifie la version du SDK."""
    print_header("1️⃣ VÉRIFICATION DE LA VERSION SDK")

    try:
        result = subprocess.run(
            ["pip", "show", "reachy-mini"],
            capture_output=True,
            text=True,
            check=True,
        )
        lines = result.stdout.split("\n")
        version = None
        for line in lines:
            if line.startswith("Version:"):
                version = line.split(":")[1].strip()
                break

        if version:
            print(f"✅ SDK installé: v{version}")

            # Vérifier si c'est une version récente
            major, minor, patch = map(int, version.split("."))
            if major == 1 and minor >= 2 and patch >= 4:
                print("✅ Version compatible (v1.2.4+ avec reflash automatique)")
                if patch < 11:
                    print(f"⚠️  Version v1.2.11 disponible (vous êtes sur v{version})")
                    print(
                        "   Recommandation: Mettre à jour après installation des moteurs"
                    )
                return True
            else:
                print(f"⚠️  Version ancienne (v{version})")
                print(
                    "   Recommandation: Mettre à jour vers v1.2.4+ avant installation"
                )
                return False
        else:
            print("⚠️  Impossible de déterminer la version")
            return False

    except subprocess.CalledProcessError:
        print("❌ SDK reachy-mini non installé")
        print("   Installez avec: pip install reachy-mini")
        return False
    except Exception as e:
        print(f"❌ Erreur: {e}")
        return False


def check_documentation() -> bool:
    """Vérifie que la documentation est disponible."""
    print_header("2️⃣ VÉRIFICATION DE LA DOCUMENTATION")

    docs_path = Path(__file__).parent.parent.parent / "docs" / "hardware"
    required_docs = [
        "GUIDE_PREVENTION_PROBLEMES_MOTEURS.md",
        "PROBLEME_MOTEURS_QC_BATCH_DEC2025.md",
        "SUIVI_COMMUNICATION_POLLEN.md",
    ]

    all_ok = True
    for doc in required_docs:
        doc_path = docs_path / doc
        if doc_path.exists():
            print(f"✅ {doc}")
        else:
            print(f"❌ {doc} - MANQUANT")
            all_ok = False

    if all_ok:
        print("\n✅ Toute la documentation est disponible")
        print("   Consultez GUIDE_PREVENTION_PROBLEMES_MOTEURS.md avant installation")

    return all_ok


def check_current_motors() -> bool:
    """Vérifie l'état actuel des moteurs (si le robot est disponible)."""
    print_header("3️⃣ VÉRIFICATION DE L'ÉTAT ACTUEL DES MOTEURS")

    try:
        from reachy_mini import ReachyMini

        print("Tentative de connexion au robot...")
        robot = ReachyMini(use_sim=False, timeout=3.0)

        print("✅ Robot connecté")
        print("\nÉtat actuel des joints:")

        # Vérifier les joints de la tête
        head_joints = [
            "stewart_1",
            "stewart_2",
            "stewart_3",
            "stewart_4",
            "stewart_5",
            "stewart_6",
        ]

        for joint_name in head_joints:
            try:
                joint = getattr(robot.head, joint_name)
                pos = joint.present_position
                print(f"   ✅ {joint_name}: position = {pos:.3f}")
            except Exception as e:
                print(f"   ❌ {joint_name}: ERREUR - {e}")

        robot.close()
        return True

    except Exception as e:
        print(f"⚠️  Robot non disponible: {e}")
        print("   C'est normal si le robot est éteint pour l'installation")
        return True  # Pas une erreur critique


def print_checklist() -> None:
    """Affiche la checklist de préparation."""
    print_header("4️⃣ CHECKLIST DE PRÉPARATION")

    checklist = [
        ("Nouveaux moteurs reçus", "Vérifier que vous avez bien reçu les 3 moteurs"),
        (
            "Numéros QC vérifiés",
            "Vérifier que les nouveaux moteurs ne sont PAS QC 2542/2543/2544",
        ),
        ("Test mécanique effectué", "Chaque moteur doit tourner smooth (pas raide)"),
        ("Outils préparés", "Tournevis, câbles, documentation"),
        ("Robot éteint", "Éteindre le robot avant de commencer"),
        ("Documentation lue", "Lire GUIDE_PREVENTION_PROBLEMES_MOTEURS.md"),
        ("Photos prises", "Prendre des photos du câblage actuel (référence)"),
    ]

    print("Avant de commencer l'installation, vérifiez:")
    print()
    for i, (item, description) in enumerate(checklist, 1):
        print(f"{i}. [ ] {item}")
        print(f"   → {description}")
        print()

    print("💡 Consultez GUIDE_PREVENTION_PROBLEMES_MOTEURS.md pour les détails")


def main() -> None:
    """Fonction principale."""
    print("\n" + "=" * 70)
    print("  VÉRIFICATION AVANT INSTALLATION DES MOTEURS")
    print("=" * 70)
    print("\nCe script vérifie que tout est prêt avant l'installation.\n")

    sdk_ok = check_sdk_version()
    docs_ok = check_documentation()
    motors_ok = check_current_motors()

    print_checklist()

    # Résumé
    print_header("📊 RÉSUMÉ")

    all_ok = sdk_ok and docs_ok and motors_ok

    if all_ok:
        print("✅ Tout est prêt pour l'installation!")
        print("\n💡 Prochaines étapes:")
        print("   1. Suivre la checklist ci-dessus")
        print("   2. Consulter GUIDE_PREVENTION_PROBLEMES_MOTEURS.md")
        print("   3. Installer les nouveaux moteurs")
        print("   4. Exécuter validate_motor_installation.py après installation")
    else:
        print("⚠️  Certaines vérifications ont échoué")
        print("\n💡 Actions recommandées:")
        if not sdk_ok:
            print("   - Mettre à jour le SDK: pip install --upgrade reachy-mini")
        if not docs_ok:
            print("   - Vérifier que la documentation est disponible")

    print("\n" + "=" * 70 + "\n")


if __name__ == "__main__":
    main()
