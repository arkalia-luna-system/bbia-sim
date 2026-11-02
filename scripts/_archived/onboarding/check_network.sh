#!/usr/bin/env bash
set -euo pipefail

# Usage: ./check_network.sh <host_or_ip>
# Exemple: ./check_network.sh reachy-mini.local  |  ./check_network.sh 192.168.1.42

HOST="${1:-}"
if [[ -z "${HOST}" ]]; then
  echo "Usage: $0 <host_or_ip>" >&2
  exit 2
fi

echo "[onboarding] Ping ${HOST} ..."
if ping -c 3 "${HOST}" >/dev/null 2>&1; then
  echo "[onboarding] ✅ Réseau OK vers ${HOST}"
else
  echo "[onboarding] ❌ Impossible de joindre ${HOST}. Vérifie Wi‑Fi/LAN, mDNS, IP." >&2
  exit 1
fi

exit 0


