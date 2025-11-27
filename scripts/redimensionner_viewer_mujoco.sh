#!/bin/bash
# Script pour redimensionner automatiquement la fenêtre MuJoCo
# Usage : ./redimensionner_viewer_mujoco.sh &

echo "🔧 Redimensionnement automatique fenêtre MuJoCo..."

# Attendre que la fenêtre s'ouvre (2 secondes)
sleep 2

# Trouver la fenêtre MuJoCo et la redimensionner
# Sur macOS, utiliser osascript
osascript <<EOF
tell application "System Events"
    repeat until exists (window 1 of application process "mujoco" whose name contains "MuJoCo")
        delay 0.5
    end repeat
    
    set theWindow to window 1 of application process "mujoco" whose name contains "MuJoCo"
    set position of theWindow to {100, 100}
    set size of theWindow to {800, 600}
end tell
EOF

echo "✅ Fenêtre redimensionnée à 800x600"

