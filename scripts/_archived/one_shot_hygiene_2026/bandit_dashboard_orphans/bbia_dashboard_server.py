#!/usr/bin/env python3
"""bbia_dashboard_server.py - Serveur dashboard BBIA
Script pour lancer le dashboard web minimal
"""

import argparse
import logging
import subprocess
import sys
import time
from pathlib import Path

# Ajouter le chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.dashboard import FASTAPI_AVAILABLE, run_dashboard


def kill_processes_on_port(port: int) -> int:
    """Tue les processus utilisant le port spécifié."""
    killed = 0
    try:
        # Trouver les processus sur le port
        result = subprocess.run(
            ["lsof", "-ti", f":{port}"],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0 and result.stdout.strip():
            pids = result.stdout.strip().split("\n")
            for pid in pids:
                if pid:
                    try:
                        subprocess.run(["kill", "-TERM", pid], check=False, timeout=2)
                        killed += 1
                        logging.info(f"🛑 Processus {pid} sur port {port} arrêté")
                    except Exception as e:
                        logging.debug(f"Erreur arrêt PID {pid}: {e}")
            # Attendre un peu pour que les processus se terminent
            if killed > 0:
                time.sleep(1)
                # Force kill si toujours actif
                result = subprocess.run(
                    ["lsof", "-ti", f":{port}"],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                if result.returncode == 0 and result.stdout.strip():
                    pids = result.stdout.strip().split("\n")
                    for pid in pids:
                        if pid:
                            try:
                                subprocess.run(["kill", "-KILL", pid], check=False, timeout=2)
                                logging.info(f"💀 Force kill PID {pid}")
                            except Exception:
                                pass
    except FileNotFoundError:
        # lsof non disponible, essayer avec psutil si disponible
        try:
            import psutil

            for proc in psutil.process_iter(["pid", "name", "connections"]):
                try:
                    for conn in proc.info.get("connections", []):
                        if conn.laddr.port == port:
                            proc.terminate()
                            killed += 1
                            logging.info(f"🛑 Processus {proc.info['pid']} arrêté")
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            if killed > 0:
                time.sleep(1)
        except ImportError:
            logging.warning("⚠️ lsof non disponible, impossible de tuer les processus")
    except Exception as e:
        logging.warning(f"⚠️ Erreur lors de la recherche de processus: {e}")

    return killed


def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(description="Dashboard BBIA")
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Adresse d'écoute",
    )  # nosec B104
    parser.add_argument("--port", type=int, default=8000, help="Port d'écoute")
    parser.add_argument(
        "--backend",
        choices=["mujoco", "reachy"],
        default="mujoco",
        help="Backend robot à utiliser",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Mode headless (pour CI)",
    )

    args = parser.parse_args()

    # Configuration logging
    if args.headless:
        logging.basicConfig(level=logging.WARNING)
    else:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s",
        )

    print("🤖 Dashboard BBIA")
    print("=" * 40)

    if not FASTAPI_AVAILABLE:
        print("❌ FastAPI non disponible")
        print("Installez avec: pip install fastapi uvicorn websockets")
        return 1

    print(f"🌐 Serveur: {args.host}:{args.port}")
    print(f"🤖 Backend: {args.backend}")
    print(f"🔗 URL: http://{args.host}:{args.port}")

    if args.headless:
        print("🔇 Mode headless activé")

    # Tuer les processus existants sur le port
    killed = kill_processes_on_port(args.port)
    if killed > 0:
        print(f"🧹 {killed} processus existant(s) arrêté(s)")

    try:
        run_dashboard(host=args.host, port=args.port, backend=args.backend)
    except KeyboardInterrupt:
        print("\n👋 Arrêt du dashboard")
        return 0
    except Exception as e:
        print(f"❌ Erreur: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
