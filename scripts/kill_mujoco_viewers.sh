#!/bin/bash
# ⚠️ DÉPRÉCIÉ : préférer cleanup_all.sh --ram-only ou process_manager.py stop
# Conservé pour compatibilité (viewers MuJoCo bloqués).
#
# Alternatives:
#   ./scripts/cleanup_all.sh --ram-only
#   python scripts/process_manager.py stop
#
# Date de dépréciation: Novembre 2024 (pointeurs maj 2 Nov 2026)

echo "⚠️  NOTE: Ce script est déprécié. Utilisez './scripts/cleanup_all.sh --ram-only' ou 'python scripts/process_manager.py stop'."
echo ""
echo "🔍 Recherche des processus MuJoCo..."

# Trouver tous les processus MuJoCo
MUJOCO_PIDS=$(ps aux | grep -i mujoco | grep -v grep | awk '{print $2}')

if [ -z "$MUJOCO_PIDS" ]; then
    echo "✅ Aucun processus MuJoCo trouvé"
    exit 0
fi

echo "⚠️  Processus MuJoCo trouvés :"
ps aux | grep -i mujoco | grep -v grep

echo ""
echo "🛑 Fermeture des processus MuJoCo..."
for pid in $MUJOCO_PIDS; do
    echo "   → Arrêt du processus $pid"
    kill -9 "$pid" 2>/dev/null
done

# Attendre un peu
sleep 1

# Vérifier s'il reste des processus
REMAINING=$(ps aux | grep -i mujoco | grep -v grep | wc -l)
if [ "$REMAINING" -gt 0 ]; then
    echo "⚠️  Certains processus persistent, tentative de force..."
    pkill -9 -f mujoco
    pkill -9 -f "mjpython"
    sleep 1
fi

echo "✅ Tous les processus MuJoCo ont été fermés"

