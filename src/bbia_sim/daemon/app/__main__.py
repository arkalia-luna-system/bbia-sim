"""Point d'entrée pour exécuter le daemon BBIA-SIM comme module.

Permet l'exécution via: python -m bbia_sim.daemon.app.main

Note: Fonctionne en mode simulation (MuJoCo) même sans robot physique.
Le daemon démarre automatiquement la simulation MuJoCo au démarrage.
"""

import logging
import shutil
import subprocess
import sys
import time
from pathlib import Path

# Ajouter src au path pour imports (depuis racine projet)
project_root = Path(__file__).parent.parent.parent.parent.parent
src_path = project_root / "src"
if str(src_path) not in sys.path:
    sys.path.insert(0, str(src_path))

# Imports après configuration du path (requis pour uvicorn)
import uvicorn  # noqa: E402

from bbia_sim.daemon.app.main import app  # noqa: E402


def kill_processes_on_port(port: int) -> int:
    """Tue les processus utilisant le port spécifié."""
    killed = 0
    try:
        # Trouver les processus sur le port
        # Utilisation de chemins absolus pour sécurité (bandit B607)
        lsof_path = shutil.which("lsof")
        kill_path = shutil.which("kill")
        if not lsof_path or not kill_path:
            raise FileNotFoundError("lsof or kill not found in PATH")

        result = subprocess.run(
            [lsof_path, "-ti", f":{port}"],  # nosec B603
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0 and result.stdout.strip():
            pids = result.stdout.strip().split("\n")
            for pid in pids:
                # Refuse toute valeur non numérique (évite injection args kill)
                if not pid or not pid.strip().isdigit():
                    continue
                pid = pid.strip()
                try:
                    subprocess.run(
                        [
                            kill_path,
                            "-TERM",
                            pid,
                        ],  # nosec B603
                        check=False,
                        timeout=2,
                    )
                    killed += 1
                    logging.info(f"🛑 Processus {pid} sur port {port} arrêté")
                except Exception as e:
                    logging.debug(f"Erreur arrêt PID {pid}: {e}")
            # Attendre un peu pour que les processus se terminent
            if killed > 0:
                time.sleep(1)
                # Force kill si toujours actif
                result = subprocess.run(
                    [lsof_path, "-ti", f":{port}"],  # nosec B603
                    capture_output=True,
                    text=True,
                    check=False,
                )
                if result.returncode == 0 and result.stdout.strip():
                    pids = result.stdout.strip().split("\n")
                    for pid in pids:
                        if not pid or not pid.strip().isdigit():
                            continue
                        pid = pid.strip()
                        try:
                            subprocess.run(
                                [
                                    kill_path,
                                    "-KILL",
                                    pid,
                                ],  # nosec B603
                                check=False,
                                timeout=2,
                            )
                            logging.info(f"💀 Force kill PID {pid}")
                        except Exception:  # nosec B110 - Ignorer erreurs force kill
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


if __name__ == "__main__":
    port = 8000
    # Tuer les processus existants sur le port
    killed = kill_processes_on_port(port)
    if killed > 0:
        logging.info(f"🧹 {killed} processus existant(s) arrêté(s)")

    logging.info("🚀 Démarrage du daemon BBIA-SIM (mode simulation)")
    logging.info("📍 Dashboard: http://127.0.0.1:8000/")
    logging.info("📚 API Docs: http://127.0.0.1:8000/docs")
    logging.info("💡 Note: Mode simulation activé (robot physique non requis)")

    from bbia_sim.daemon.ws.keepalive import uvicorn_ws_keepalive_kwargs  # noqa: E402

    # Importer directement l'app (pas de string) pour éviter problèmes reload
    # Keepalive WS (#1494) via BBIA_WS_KEEPALIVE_TIMEOUT
    uvicorn.run(
        app,  # Objet app directement (pas de string)
        host="127.0.0.1",
        port=port,
        reload=False,  # Pas de reload pour éviter problèmes
        log_level="info",
        **uvicorn_ws_keepalive_kwargs(),
    )
