#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
LOG_DIR="$ROOT_DIR/log"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/onboarding_demo.log"

CLI_AUDIO=false
for arg in "$@"; do
  case "$arg" in
    --cli-audio)
      CLI_AUDIO=true
      shift || true
      ;;
  esac
done

echo "[onboarding] $(date -Iseconds) démarrage (cli_audio=$CLI_AUDIO)" | tee -a "$LOG_FILE"

# Charger l'env si présent
ENV_FILE="$ROOT_DIR/scripts/onboarding/env_bbia_example.txt"
if [[ -f "$ENV_FILE" ]]; then
  # shellcheck disable=SC2046
  export $(grep -Ev '^\s*#|^\s*$' "$ENV_FILE" | tr -d '\r' | xargs) || true
  echo "[onboarding] env chargé depuis $ENV_FILE" | tee -a "$LOG_FILE"
else
  echo "[onboarding] ⚠️  $ENV_FILE introuvable, poursuite sans export auto" | tee -a "$LOG_FILE"
fi

if [[ -z "${BBIA_REACHY_HOST:-}" ]]; then
  echo "[onboarding] ❌ BBIA_REACHY_HOST non défini. Édite $ENV_FILE puis relance." | tee -a "$LOG_FILE"
  exit 2
fi

echo "[onboarding] Ping $BBIA_REACHY_HOST ..." | tee -a "$LOG_FILE"
if ! ping -c 3 "$BBIA_REACHY_HOST" >/dev/null 2>&1; then
  echo "[onboarding] ❌ Ping KO. Vérifie LAN/mDNS/IP." | tee -a "$LOG_FILE"
  exit 1
fi
echo "[onboarding] ✅ Réseau OK" | tee -a "$LOG_FILE"

# Activer le venv si trouvé
if [[ -f "$ROOT_DIR/venv/bin/activate" ]]; then
  # shellcheck disable=SC1090
  source "$ROOT_DIR/venv/bin/activate"
  echo "[onboarding] venv activé" | tee -a "$LOG_FILE"
else
  echo "[onboarding] ⚠️  venv non trouvé. Lance scripts/onboarding/setup_env.sh d'abord." | tee -a "$LOG_FILE"
fi

set +e
python "$ROOT_DIR/examples/demo_reachy_mini_corrigee.py" >> "$LOG_FILE" 2>&1
RC=$?
set -e

if [[ $RC -eq 0 ]]; then
  echo "[onboarding] ✅ Démo terminée avec succès" | tee -a "$LOG_FILE"
  if $CLI_AUDIO && [[ -f "/tmp/tts.wav" ]]; then
    echo "[onboarding] Lecture CLI de /tmp/tts.wav via afplay" | tee -a "$LOG_FILE"
    if command -v afplay >/dev/null 2>&1; then
      afplay /tmp/tts.wav || true
    else
      echo "[onboarding] ⚠️  afplay indisponible, lecture ignorée" | tee -a "$LOG_FILE"
    fi
  fi
else
  echo "[onboarding] ❌ Démo a retourné code $RC (voir $LOG_FILE)" | tee -a "$LOG_FILE"
  exit $RC
fi

exit 0


