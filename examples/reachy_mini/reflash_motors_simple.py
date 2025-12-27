#!/usr/bin/env python3
"""Script simple pour utiliser le script de reflash officiel reachy-mini-reflash-motors.

Ce script appelle le script officiel du SDK reachy_mini pour reflasher tous les moteurs.
C'est la solution recommandée par robertodipizzamano sur Discord qui a résolu son problème.

PROCÉDURE OFFICIELLE:
1. Alimenter le robot SANS démarrer le daemon (interrupteur ON, pas de dashboard)
2. Mettre à jour: pip install --upgrade reachy-mini
3. Lancer: reachy-mini-reflash-motors
4. Le script détecte automatiquement les ports et reprogramme les moteurs

Usage:
    python examples/reachy_mini/reflash_motors_simple.py
    python examples/reachy_mini/reflash_motors_simple.py --serialport /dev/ttyAMA3
"""

import argparse
import subprocess
import sys


def main() -> None:
    """Fonction principale."""
    parser = argparse.ArgumentParser(
        description="Utilise le script officiel reachy-mini-reflash-motors pour corriger le bug décembre 2025",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ce script appelle le script officiel du SDK reachy_mini qui reflashe tous les moteurs.
C'est la solution qui a fonctionné pour robertodipizzamano sur Discord.

Le script officiel est disponible via:
  - reachy-mini-reflash-motors (si installé globalement)
  - python -m reachy_mini.tools.reflash_motors (via module)

Note: Nécessite le SDK reachy_mini installé (pip install reachy-mini)
        """,
    )
    parser.add_argument(
        "--serialport",
        type=str,
        default=None,
        help="Port série du robot (e.g. /dev/ttyAMA3). "
        "Si non spécifié, le script officiel tentera de le détecter automatiquement.",
    )

    args = parser.parse_args()

    print("🔄 REFLASH MOTEURS - Reachy Mini (Script Officiel)")
    print("=" * 60)
    print()
    print("⚠️  IMPORTANT - Procédure avant de lancer:")
    print("   1. Alimenter le robot (interrupteur ON)")
    print("   2. NE PAS démarrer le daemon (sudo systemctl stop reachy-mini-daemon)")
    print("   3. NE PAS ouvrir le dashboard")
    print("   4. Mettre à jour: pip install --upgrade reachy-mini")
    print()
    print("Ce script utilise le script officiel reachy-mini-reflash-motors")
    print("pour reflasher tous les moteurs et corriger le bug décembre 2025.")
    print()

    # Essayer différentes méthodes pour appeler le script officiel
    commands_to_try = []

    # Méthode 1: Commande globale (si installée)
    commands_to_try.append(["reachy-mini-reflash-motors"])

    # Méthode 2: Module Python
    commands_to_try.append(["python", "-m", "reachy_mini.tools.reflash_motors"])

    # Méthode 3: python -m reachy_mini.tools.reflash_motors (alternative)
    commands_to_try.append(["python3", "-m", "reachy_mini.tools.reflash_motors"])

    # Ajouter le port série si spécifié
    if args.serialport:
        for cmd in commands_to_try:
            cmd.extend(["--serialport", args.serialport])

    # Essayer chaque commande jusqu'à ce qu'une fonctionne
    for i, cmd in enumerate(commands_to_try, 1):
        print(f"Tentative {i}/{len(commands_to_try)}: {' '.join(cmd)}")
        try:
            result = subprocess.run(cmd, check=False)
            if result.returncode == 0:
                print("\n✅ Reflash réussi!")
                return
            elif (
                result.returncode == 2
            ):  # Code d'erreur argparse (commande non trouvée)
                continue  # Essayer la commande suivante
            else:
                print(f"\n⚠️  Le script a retourné le code {result.returncode}")
                print("   Vérifiez les messages d'erreur ci-dessus")
                sys.exit(result.returncode)
        except FileNotFoundError:
            continue  # Commande non trouvée, essayer la suivante
        except KeyboardInterrupt:
            print("\n⏹️  Arrêt par l'utilisateur")
            sys.exit(1)
        except Exception as e:
            print(f"❌ Erreur: {e}")
            continue

    # Si aucune commande n'a fonctionné
    print("\n❌ Aucune méthode n'a fonctionné pour appeler le script officiel")
    print()
    print("💡 Solutions:")
    print("   1. Installer le SDK: pip install reachy-mini")
    print("   2. Vérifier que reachy-mini-reflash-motors est dans le PATH")
    print("   3. Utiliser directement: python -m reachy_mini.tools.reflash_motors")
    print("   4. Utiliser le script fix_motor_config_december_bug.py à la place")
    sys.exit(1)


if __name__ == "__main__":
    main()
