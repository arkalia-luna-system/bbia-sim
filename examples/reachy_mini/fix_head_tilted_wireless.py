#!/usr/bin/env python3
"""🔧 CORRECTION TÊTE PENCHÉE - Reachy Mini Wireless
Compatible Mac avec connexion réseau via Zenoh

Auteur: Athalia Luna (BBIA-SIM)
Basé sur: https://github.com/pollen-robotics/reachy_mini/issues (Batch Dec 2025)

Usage:
    python examples/reachy_mini/fix_head_tilted_wireless.py
    python examples/reachy_mini/fix_head_tilted_wireless.py --robot-ip 192.168.129.64
"""

import argparse
import sys
import time

try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose
except ImportError:
    print("❌ SDK reachy_mini non installé")
    print("   Installez-le avec: pip install reachy-mini")
    sys.exit(1)


def connect_robot(
    robot_ip: str | None = None, timeout: float = 30.0
) -> ReachyMini | None:
    """Se connecte au robot avec gestion d'erreur.

    Args:
        robot_ip: IP du robot (optionnel, pour vérification)
        timeout: Timeout de connexion en secondes

    Returns:
        Instance ReachyMini connectée ou None en cas d'erreur
    """
    try:
        print("🤖 Connexion au robot...")
        print(f"   Timeout: {timeout}s")

        if robot_ip:
            print(f"   IP attendue: {robot_ip}")
            print("   ⚠️  Note: Le backend Zenoh doit être démarré sur le robot")
            print("   💡 Lancez: reachy-mini-daemon (sur le robot ou via dashboard)")

        # Important: utilise localhost_only=False pour connexion réseau
        robot = ReachyMini(
            media_backend="no_media",  # Pas besoin de média pour cette correction
            use_sim=False,
            localhost_only=False,  # ← CRITIQUE pour connexion réseau
            timeout=timeout,
            spawn_daemon=False,  # Le daemon doit être lancé séparément
        )

        # Utiliser context manager (meilleure pratique)
        robot.__enter__()

        # Tester la connexion
        try:
            _ = robot.get_current_joint_positions()
            print("✅ Robot connecté et opérationnel")
            return robot
        except Exception as e:
            print(f"❌ Erreur lors du test de connexion: {e}")
            robot.__exit__(None, None, None)
            return None

    except TimeoutError:
        print("❌ Timeout : Le backend Zenoh n'est pas accessible")
        print("💡 Solutions:")
        print("   1. Vérifiez que le robot est allumé et connecté au WiFi")
        print("   2. Lancez le backend sur le robot: reachy-mini-daemon")
        print("   3. Ou utilisez le dashboard: http://<robot-ip>:8000")
        return None
    except Exception as e:
        print(f"❌ Erreur de connexion: {e}")
        print("💡 Vérifiez:")
        print("   1. Le robot est sur le même réseau WiFi")
        print("   2. Le backend Zenoh est démarré (reachy-mini-daemon)")
        print("   3. Le port 7447 (Zenoh) est accessible")
        return None


def fix_head_tilted(robot: ReachyMini) -> bool:
    """Corrige la tête penchée du Reachy Mini.

    Args:
        robot: Instance ReachyMini connectée

    Returns:
        True si succès, False sinon
    """
    try:
        print("\n" + "=" * 70)
        print("🔧 CORRECTION TÊTE PENCHÉE")
        print("=" * 70)
        print()

        # 1. Afficher la position actuelle
        print("📍 Position actuelle de la tête:")
        try:
            current_pose = robot.head.head_pose
            print(
                f"   X: {current_pose.x:.2f}mm, Y: {current_pose.y:.2f}mm, Z: {current_pose.z:.2f}mm"
            )
            print(
                f"   Roll: {current_pose.roll:.2f}°, Pitch: {current_pose.pitch:.2f}°, Yaw: {current_pose.yaw:.2f}°"
            )
        except Exception as e:
            print(f"   ⚠️  Impossible de lire la position: {e}")

        # 2. Correction - Position neutre
        print("\n🔧 Application de la correction...")
        print("   Position neutre: Roll=0°, Pitch=0°, Yaw=0°, Z=0mm")

        neutral_pose = create_head_pose(
            x=0, y=0, z=0, roll=0, pitch=0, yaw=0, degrees=True, mm=True
        )

        robot.goto_target(
            head=neutral_pose,
            antennas=[0.0, 0.0],  # Antennes neutres aussi
            duration=3.0,
            interpolation_mode="minjerk",  # Mouvement fluide
        )

        # 3. Attente et vérification
        print("   ⏳ Attente 3.5 secondes pour que le mouvement se termine...")
        time.sleep(3.5)

        try:
            final_pose = robot.head.head_pose
            print("\n✅ Correction appliquée !")
            print(
                f"   Nouvelle position - Roll: {final_pose.roll:.2f}°, Pitch: {final_pose.pitch:.2f}°"
            )
        except Exception as e:
            print(f"   ⚠️  Impossible de vérifier la position finale: {e}")
            print("   (Mais le mouvement a été envoyé)")

        # 4. Test de mobilité de la tête
        print("\n🎭 Test de mobilité de la tête...")

        # Légère inclinaison gauche
        print("   → Inclinaison gauche (roll=10°)...")
        robot.goto_target(head=create_head_pose(roll=10, degrees=True), duration=1.0)
        time.sleep(1.2)

        # Retour neutre
        print("   → Retour position neutre...")
        robot.goto_target(head=neutral_pose, duration=1.0)
        time.sleep(1.2)

        print("✅ La tête est maintenant fonctionnelle !")
        return True

    except Exception as e:
        print(f"\n❌ Erreur pendant la correction: {e}")
        import traceback

        traceback.print_exc()
        return False


def main() -> None:
    """Fonction principale."""
    parser = argparse.ArgumentParser(
        description="Corrige la tête penchée du Reachy Mini Wireless depuis un Mac distant"
    )
    parser.add_argument(
        "--robot-ip",
        type=str,
        default=None,
        help="IP du robot (optionnel, pour vérification uniquement)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Timeout de connexion en secondes (défaut: 30.0)",
    )

    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("🌙 BBIA-SIM × Reachy Mini - Head Tilt Fix (Wireless)")
    print("=" * 70)
    print(f"📅 Date: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    print("⚠️  PRÉREQUIS:")
    print("   1. Le robot doit être allumé et connecté au WiFi")
    print("   2. Le backend Zenoh doit être démarré sur le robot")
    print("      → Lancez: reachy-mini-daemon (sur le robot)")
    print("      → Ou utilisez le dashboard: http://<robot-ip>:8000")
    print("   3. Votre Mac doit être sur le même réseau WiFi")
    print()

    # Connexion au robot
    robot = connect_robot(robot_ip=args.robot_ip, timeout=args.timeout)

    if not robot:
        print("\n" + "=" * 70)
        print("❌ ÉCHEC - Impossible de se connecter au robot")
        print("=" * 70)
        print("\n🔍 Debug:")
        print("   1. Vérifiez que le robot est allumé et connecté au WiFi")
        if args.robot_ip:
            print(f"   2. Testez http://{args.robot_ip}:8000 dans votre navigateur")
        print("   3. Lancez le backend manuellement depuis le dashboard")
        print("   4. Vérifiez que le port 7447 (Zenoh) est accessible")
        print(
            "   5. Alternative: Utilisez fix_head_tilted_ssh.py pour exécution directe sur le robot"
        )
        sys.exit(1)

    try:
        # Correction
        success = fix_head_tilted(robot)

        print("\n" + "=" * 70)
        if success:
            print("✅ SUCCÈS - La tête est corrigée et fonctionnelle")
            print("=" * 70)
            print("\n💡 Prochaines étapes:")
            print("   1. Testez d'autres mouvements via le dashboard")
            print("   2. Si la tête re-penche, vérifiez les câbles internes")
            print("   3. Intégrez BBIA pour les émotions et la vision")
        else:
            print("⚠️  CORRECTION PARTIELLE")
            print("=" * 70)
            print("\n💡 Le mouvement a été envoyé mais il peut y avoir eu des erreurs.")
            print("   Solutions:")
            print("   1. Relancez ce script plusieurs fois")
            print("   2. Faites une calibration via l'app Reachy Mini Control")
            print("   3. Vérifiez la mise à jour du firmware")
    finally:
        # Déconnexion propre
        try:
            robot.__exit__(None, None, None)
            print("\n✅ Robot déconnecté proprement")
        except Exception:
            pass

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
