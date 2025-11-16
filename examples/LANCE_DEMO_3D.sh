#!/bin/bash
# Script pour lancer la démo Chat BBIA en 3D avec viewer 3D

echo "💬🤖 LANCEMENT DÉMO CHAT BBIA EN 3D"
echo "=================================="
echo ""

# Activer le venv (depuis examples/, remonter d'un niveau)
cd "$(dirname "$0")/.." || exit
source venv/bin/activate

# Utiliser mjpython pour voir le viewer 3D (fonctionne sur macOS)
if command -v mjpython &> /dev/null; then
    echo "✅ mjpython trouvé - lancement avec VIEWER 3D"
    echo "🖥️  Le viewer MuJoCo va s'ouvrir..."
    echo ""
    mjpython examples/demo_chat_bbia_3d.py
else
    echo "⚠️  mjpython non trouvé"
    echo "💡 Installer avec: pip install mujoco-viewer"
    echo ""
    echo "Lancement sans viewer 3D..."
    python examples/demo_chat_bbia_3d.py
fi

echo ""
echo "✅ Démo terminée"

