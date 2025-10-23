#!/usr/bin/env python3
"""
Système de gestion des processus BBIA avec détection de doublons et sécurité d'arrêt
Usage: python scripts/process_manager.py [start|stop|status|kill-all]
"""

import argparse
import logging
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import psutil

# Configuration du logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Fichier de verrouillage pour éviter les doublons
LOCK_FILE = Path.home() / ".bbia_sim.lock"
PID_FILE = Path.home() / ".bbia_sim.pid"


class ProcessManager:
    """Gestionnaire de processus BBIA avec sécurité et détection de doublons."""

    def __init__(self):
        self.project_root = Path(__file__).parent.parent
        self.launcher = self.project_root / "scripts" / "launch_complete_robot.py"

    def is_bbia_running(self) -> list[dict]:
        """Vérifie si des processus BBIA sont en cours d'exécution."""
        running_processes = []

        for proc in psutil.process_iter(
            ["pid", "name", "cmdline", "cpu_percent", "memory_info"]
        ):
            try:
                cmdline = " ".join(proc.info["cmdline"]) if proc.info["cmdline"] else ""

                # Détecter les processus BBIA
                if any(
                    keyword in cmdline.lower()
                    for keyword in [
                        "launch_complete_robot.py",
                        "bbia_sim",
                        "mujoco",
                        "reachy",
                    ]
                ):
                    running_processes.append(
                        {
                            "pid": proc.info["pid"],
                            "name": proc.info["name"],
                            "cmdline": cmdline,
                            "cpu_percent": proc.info["cpu_percent"],
                            "memory_mb": proc.info["memory_info"].rss / 1024 / 1024,
                        }
                    )
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        return running_processes

    def create_lock(self, pid: int) -> bool:
        """Crée un fichier de verrouillage pour éviter les doublons."""
        try:
            with open(LOCK_FILE, "w") as f:
                f.write(f"{pid}\n{time.time()}\n")
            with open(PID_FILE, "w") as f:
                f.write(f"{pid}\n")
            return True
        except Exception as e:
            logger.error(f"❌ Erreur création verrou: {e}")
            return False

    def remove_lock(self) -> bool:
        """Supprime le fichier de verrouillage."""
        try:
            if LOCK_FILE.exists():
                LOCK_FILE.unlink()
            if PID_FILE.exists():
                PID_FILE.unlink()
            return True
        except Exception as e:
            logger.error(f"❌ Erreur suppression verrou: {e}")
            return False

    def is_locked(self) -> Optional[int]:
        """Vérifie si un processus BBIA est verrouillé."""
        if not LOCK_FILE.exists():
            return None

        try:
            with open(LOCK_FILE) as f:
                pid = int(f.readline().strip())

            # Vérifier si le processus existe encore
            if psutil.pid_exists(pid):
                return pid
            else:
                # Processus mort, nettoyer le verrou
                self.remove_lock()
                return None
        except Exception:
            self.remove_lock()
            return None

    def start_process(
        self, mode: str = "graphical", duration: Optional[int] = None
    ) -> bool:
        """Démarre un processus BBIA avec vérification de doublons."""

        # Vérifier les doublons
        if self.is_locked():
            logger.error("❌ Un processus BBIA est déjà en cours d'exécution!")
            logger.error(
                "💡 Utilisez 'python scripts/process_manager.py stop' pour l'arrêter"
            )
            return False

        # Construire la commande
        cmd = ["python3", str(self.launcher)]

        if mode == "headless":
            cmd.append("--headless")
            if duration:
                cmd.extend(["--duration", str(duration)])
        elif mode == "test":
            cmd.extend(["--headless", "--duration", "2"])

        logger.info(f"🚀 Démarrage processus BBIA en mode {mode}")
        logger.info(f"📝 Commande: {' '.join(cmd)}")

        try:
            # Démarrer le processus
            process = psutil.Popen(cmd, cwd=str(self.project_root))

            # Créer le verrouillage
            if self.create_lock(process.pid):
                logger.info(f"✅ Processus démarré avec PID {process.pid}")
                logger.info(f"🔒 Verrouillage créé: {LOCK_FILE}")

                # Configurer l'arrêt automatique à la fermeture du terminal
                self._setup_terminal_exit_handler(process.pid)

                return True
            else:
                logger.error("❌ Échec de la création du verrouillage")
                process.terminate()
                return False

        except Exception as e:
            logger.error(f"❌ Erreur démarrage: {e}")
            return False

    def stop_process(self, force: bool = False) -> bool:
        """Arrête le processus BBIA en cours."""

        # Vérifier le verrouillage
        locked_pid = self.is_locked()
        if not locked_pid:
            logger.warning("⚠️ Aucun processus BBIA verrouillé trouvé")

            # Chercher tous les processus BBIA
            running = self.is_bbia_running()
            if not running:
                logger.info("✅ Aucun processus BBIA en cours")
                return True

            logger.info(f"🔍 {len(running)} processus BBIA trouvés:")
            for proc in running:
                logger.info(f"   PID {proc['pid']}: {proc['cmdline'][:80]}...")

            if not force:
                response = input("❓ Voulez-vous les arrêter? (y/N): ")
                if response.lower() != "y":
                    logger.info("❌ Arrêt annulé")
                    return False

            # Arrêter tous les processus BBIA
            for proc in running:
                try:
                    psutil.Process(proc["pid"]).terminate()
                    logger.info(f"🛑 Processus {proc['pid']} arrêté")
                except Exception as e:
                    logger.error(f"❌ Erreur arrêt PID {proc['pid']}: {e}")

            self.remove_lock()
            return True

        # Arrêter le processus verrouillé
        try:
            process = psutil.Process(locked_pid)
            logger.info(f"🛑 Arrêt du processus {locked_pid}")

            if force:
                process.kill()
                logger.info(f"💀 Processus {locked_pid} tué (force)")
            else:
                process.terminate()
                logger.info(f"🛑 Processus {locked_pid} terminé")

            # Attendre l'arrêt
            try:
                process.wait(timeout=5)
            except psutil.TimeoutExpired:
                logger.warning("⚠️ Processus ne répond pas, utilisation de kill")
                process.kill()

            self.remove_lock()
            logger.info("✅ Processus arrêté avec succès")
            return True

        except psutil.NoSuchProcess:
            logger.warning("⚠️ Processus déjà arrêté")
            self.remove_lock()
            return True
        except Exception as e:
            logger.error(f"❌ Erreur arrêt: {e}")
            return False

    def kill_all_processes(self) -> bool:
        """Tue tous les processus BBIA (mode force)."""
        logger.warning("💀 Mode KILL ALL activé")

        running = self.is_bbia_running()
        if not running:
            logger.info("✅ Aucun processus BBIA à tuer")
            return True

        logger.info(f"🔍 {len(running)} processus BBIA à tuer:")
        for proc in running:
            logger.info(f"   PID {proc['pid']}: {proc['cmdline'][:80]}...")

        response = input("❓ Confirmer le KILL ALL? (y/N): ")
        if response.lower() != "y":
            logger.info("❌ KILL ALL annulé")
            return False

        killed_count = 0
        for proc in running:
            try:
                psutil.Process(proc["pid"]).kill()
                logger.info(f"💀 PID {proc['pid']} tué")
                killed_count += 1
            except Exception as e:
                logger.error(f"❌ Erreur kill PID {proc['pid']}: {e}")

        self.remove_lock()
        logger.info(f"✅ {killed_count} processus tués")
        return True

    def show_status(self) -> None:
        """Affiche le statut des processus BBIA."""
        logger.info("📊 Statut des processus BBIA")
        logger.info("=" * 50)

        # Vérifier le verrouillage
        locked_pid = self.is_locked()
        if locked_pid:
            logger.info(f"🔒 Processus verrouillé: PID {locked_pid}")
        else:
            logger.info("🔓 Aucun processus verrouillé")

        # Lister tous les processus BBIA
        running = self.is_bbia_running()
        if running:
            logger.info(f"🔍 {len(running)} processus BBIA en cours:")
            for proc in running:
                status = "🔒 VERROUILLÉ" if proc["pid"] == locked_pid else "🔄 ACTIF"
                logger.info(
                    f"   {status} PID {proc['pid']}: {proc['cpu_percent']:.1f}% CPU, {proc['memory_mb']:.1f}MB RAM"
                )
                logger.info(f"      {proc['cmdline'][:100]}...")
        else:
            logger.info("✅ Aucun processus BBIA en cours")

    def _setup_terminal_exit_handler(self, pid: int) -> None:
        """Configure l'arrêt automatique quand le terminal se ferme."""

        def signal_handler(signum, frame):
            logger.info(
                f"\n🛑 Signal {signum} reçu - Arrêt automatique du processus {pid}"
            )
            try:
                process = psutil.Process(pid)
                process.terminate()
                logger.info("✅ Processus arrêté automatiquement")
            except psutil.NoSuchProcess:
                logger.info("✅ Processus déjà arrêté")
            except Exception as e:
                logger.error(f"❌ Erreur arrêt automatique: {e}")
            finally:
                self.remove_lock()
                sys.exit(0)

        # Enregistrer les handlers pour les signaux de fermeture de terminal
        signal.signal(signal.SIGTERM, signal_handler)
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGHUP, signal_handler)


def main():
    parser = argparse.ArgumentParser(description="Gestionnaire de processus BBIA")
    parser.add_argument(
        "action",
        choices=["start", "stop", "status", "kill-all"],
        help="Action à effectuer",
    )
    parser.add_argument(
        "--mode",
        choices=["graphical", "headless", "test"],
        default="graphical",
        help="Mode de démarrage",
    )
    parser.add_argument(
        "--duration", type=int, help="Durée en secondes (headless uniquement)"
    )
    parser.add_argument(
        "--force", action="store_true", help="Mode force (pas de confirmation)"
    )

    args = parser.parse_args()

    manager = ProcessManager()

    if args.action == "start":
        success = manager.start_process(args.mode, args.duration)
        if success:
            logger.info("✅ Démarrage réussi")
            logger.info(
                "💡 Utilisez 'python scripts/process_manager.py status' pour vérifier"
            )
            logger.info(
                "💡 Utilisez 'python scripts/process_manager.py stop' pour arrêter"
            )
        else:
            logger.error("❌ Échec du démarrage")
            sys.exit(1)

    elif args.action == "stop":
        success = manager.stop_process(args.force)
        if success:
            logger.info("✅ Arrêt réussi")
        else:
            logger.error("❌ Échec de l'arrêt")
            sys.exit(1)

    elif args.action == "status":
        manager.show_status()

    elif args.action == "kill-all":
        success = manager.kill_all_processes()
        if success:
            logger.info("✅ KILL ALL réussi")
        else:
            logger.error("❌ Échec KILL ALL")
            sys.exit(1)


if __name__ == "__main__":
    main()
