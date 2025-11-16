#!/bin/bash
# Lance la démo 3D avec fenêtre de taille fixe

echo "🎮 Lancement démo BBIA 3D (taille fixe)..."
echo ""

# Activer le venv (depuis examples/, remonter d'un niveau)
cd "$(dirname "$0")/.." || exit
source venv/bin/activate

# Utiliser mjpython pour voir le viewer 3D (fonctionne sur macOS)
if command -v mjpython &> /dev/null; then
    echo "✅ mjpython trouvé - lancement avec VIEWER 3D"
    echo "🖥️  La fenêtre va s'ouvrir..."
    echo "💡 Appuyez 2x sur (-) pour réduire, puis redimensionnez"
    echo ""
    
    # Lancer la démo
    mjpython examples/demo_chat_bbia_3d.py &
    DEMO_PID=$!
    
    echo "⏳ Attente ouverture fenêtre..."
    sleep 3
    
    # Script AppleScript pour redimensionner
    osascript <<'APPLESCRIPT'
tell application "System Events"
    tell application process "Python" to set frontmost to true
    try
        set theWindow to window 1 of application process "Python"
        set position of theWindow to {100, 100}
        set size of theWindow to {800, 600}
    on error
        log "Fenêtre non trouvée, essayez de réduire manuellement"
    end try
end tell
APPLESCRIPT
    
    # Attendre la fin de la démo
    wait $DEMO_PID
    
else
    echo "⚠️  mjpython non trouvé"
    echo "💡 Installer avec: pip install mujoco-viewer"
    echo ""
    python examples/demo_chat_bbia_3d.py
fi

echo ""
echo "✅ Démo terminée"

