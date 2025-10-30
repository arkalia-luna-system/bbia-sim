#!/bin/bash
# Synthèse et lecture voix macOS (fiable, sans dépendances Python)
# Usage:
#   ./scripts/voice_mac_say.sh "Texte à dire" "Amélie"
# Voix suggérées: Amélie, Audrey, Thomas (voir: say -v "?" )

set -euo pipefail

TEXT=${1:-"Bonjour, je suis BBIA. Je parle avec une voix plus douce et mignonne."}
VOICE=${2:-"Amélie"}

mkdir -p assets/voice
AIFF="assets/voice/bbia_say_tmp.aiff"
WAV="assets/voice/bbia_demo_say.wav"

# Génère un AIFF via say
say -v "$VOICE" -o "$AIFF" --data-format=LEI16@16000 "$TEXT"

# Convertit AIFF -> WAV (afconvert macOS)
afconvert -f WAVE -d LEI16@16000 "$AIFF" "$WAV"

echo "[OK] Fichier généré: $WAV"
echo "[PLAY] Lecture..."
afplay "$WAV"
echo "[PLAY] Terminé"

exit 0


