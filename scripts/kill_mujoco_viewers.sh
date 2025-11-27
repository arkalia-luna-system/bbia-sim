#!/bin/bash
# ⚠️ DÉPRÉCIÉ : Utiliser smart_process_cleanup.sh à la place
# Ce script est conservé pour compatibilité mais sera supprimé dans une future version
# Script pour forcer la fermeture de tous les viewers MuJoCo bloqués
#
# ⚠️  DÉPRÉCIÉ: Ce script est déprécié mais conservé pour compatibilité.
# 💡 Alternative recommandée: python scripts/process_manager.py stop
#    ou: scripts/smart_process_cleanup.sh
#
# Date de dépréciation: Novembre 2024

echo "⚠️  NOTE: Ce script est déprécié. Utilisez 'python scripts/process_manager.py stop' à la place."
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

