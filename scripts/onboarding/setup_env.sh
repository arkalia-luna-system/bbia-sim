#!/usr/bin/env bash
set -euo pipefail

# Prépare un venv local et installe BBIA-SIM + SDK Reachy Mini.
# Usage: ./setup_env.sh

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT_DIR"

echo "[onboarding] Création/activation venv ..."
python3 -m venv venv
source venv/bin/activate
pip install -U pip

echo "[onboarding] Installation du projet en mode editable ..."
pip install -e .

echo "[onboarding] Installation SDK Reachy Mini ..."
pip install reachy-mini

echo "[onboarding] Installation outils audio (si Homebrew présent) ..."
if command -v brew >/dev/null 2>&1; then
  brew list portaudio >/dev/null 2>&1 || brew install portaudio || true
  brew list ffmpeg >/dev/null 2>&1 || brew install ffmpeg || true
fi

echo "[onboarding] Terminé. Active le venv: 'source venv/bin/activate'"
exit 0


