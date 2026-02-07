#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
# À exécuter UNIQUEMENT SUR LE ROBOT (en SSH), pas sur ton Mac.
# ═══════════════════════════════════════════════════════════════════════════
# 1. Sur ton Mac :  ssh pollen@192.168.129.64
# 2. Sur le robot : bash install_and_run_testbench_on_robot.sh
# 3. Sur ton Mac, dans le navigateur : http://192.168.129.64:8042
#    (PAS http://0.0.0.0:8042 — 0.0.0.0 ne s'ouvre pas dans le navigateur.)
# ═══════════════════════════════════════════════════════════════════════════

set -e

# Vérifier qu'on est bien sur le robot (systemd existe, pas sur macOS)
if ! command -v systemctl >/dev/null 2>&1; then
  echo "❌ ERREUR: Ce script doit être exécuté SUR LE ROBOT (en SSH), pas sur ton Mac."
  echo "   Sur ton Mac lance:  ssh pollen@192.168.129.64"
  echo "   Puis sur le robot: bash install_and_run_testbench_on_robot.sh"
  exit 1
fi
INSTALL_DIR="${INSTALL_DIR:-$HOME/reachy_mini_testbench}"
REPO="https://huggingface.co/spaces/pollen-robotics/reachy_mini_testbench"

echo "📥 Clone de l'app depuis Hugging Face..."
if [ -d "$INSTALL_DIR" ]; then
  echo "   Déjà cloné dans $INSTALL_DIR (pull pour mettre à jour)"
  (cd "$INSTALL_DIR" && git pull --rebase 2>/dev/null || true)
else
  git clone "$REPO" "$INSTALL_DIR"
fi

echo "📦 Création d'un venv et installation..."
cd "$INSTALL_DIR"
if [ ! -d ".venv" ]; then
  python3 -m venv .venv
fi
. .venv/bin/activate
pip install -e .

# Sur Reachy Wireless le daemon Zenoh écoute sur l'IP WiFi, pas sur 127.0.0.1.
# L'app Testbench utilise par défaut connection_mode="localhost_only" → elle ne voit jamais le démon.
# On patch pour essayer en mode réseau (même machine = découverte Zenoh sur l'interface WiFi).
MAIN_PY="$INSTALL_DIR/reachy_mini_testbench/main.py"
if [ -f "$MAIN_PY" ] && grep -q 'connection_mode="localhost_only"' "$MAIN_PY"; then
  echo "🔧 Patch Reachy Wireless: connexion au démon en mode réseau (sinon DÉMON OFF sur 8042)"
  sed -i.bak 's/connection_mode="localhost_only"/connection_mode="network"/' "$MAIN_PY"
fi

# Pour que le Testbench voie le démon, le démon doit être démarré *avant* le Testbench.
if ! systemctl is-active --quiet reachy-mini-daemon 2>/dev/null; then
  echo "⚠️  reachy-mini-daemon n'est pas actif. Pour avoir DÉMON ON sur 8042:"
  echo "   sudo systemctl start reachy-mini-daemon"
  echo "   Puis relancer ce script."
  echo ""
fi

echo "💡 Rappel: scan/positions/check_all (moteurs) = 503 tant que le démon tient le port série."
echo "   Pour les diagnostics moteurs: arrêter le démon (sudo systemctl stop reachy-mini-daemon), puis relancer l'app."
echo ""
echo "🚀 Lancement de l'app (port 8042) SUR CE ROBOT..."
echo ""
echo "   Depuis ton Mac, ouvre dans le navigateur:  http://192.168.129.64:8042"
echo "   (remplace 192.168.129.64 par l'IP du robot si différente)"
echo ""
python -m reachy_mini_testbench.main
