#!/bin/bash
# Démo BBIA - Salut (wave)
# Script de démonstration du salut avec épaule/bras

set -euo pipefail

# Configuration
API_URL="${API_URL:-http://localhost:8000}"
TOKEN="${TOKEN:-bbia-secret-key-dev}"
DURATION="${DURATION:-8}"

# Couleurs pour les logs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Fonction pour envoyer une commande de mouvement
send_motion() {
    local joints="$1"
    local step="$2"
    
    log_info "Étape $step: $joints"
    
    response=$(curl -s -w "%{http_code}" -o /tmp/response.json \
        -H "Authorization: Bearer $TOKEN" \
        -H "Content-Type: application/json" \
        -X POST \
        "$API_URL/api/motion/joints" \
        -d "$joints")
    
    http_code="${response: -3}"
    
    if [ "$http_code" -eq 200 ]; then
        log_success "Mouvement réussi"
        sleep 1.5
    else
        log_error "Échec mouvement (HTTP $http_code)"
        cat /tmp/response.json
        return 1
    fi
}

# Vérification de l'API
check_api() {
    log_info "Vérification de l'API BBIA-SIM..."
    
    if curl -s -f "$API_URL/health" > /dev/null; then
        log_success "API accessible"
    else
        log_error "API non accessible sur $API_URL"
        log_info "Assurez-vous que l'API est démarrée:"
        log_info "  uvicorn src.bbia_sim.daemon.app.main:app --port 8000"
        exit 1
    fi
}

# Séquence de salut
wave_sequence() {
    log_info "🤖 Démarrage du salut BBIA"
    log_info "Durée estimée: ${DURATION}s"
    echo "=========================================="
    
    # Étape 1: Position initiale (bras légèrement relevés)
    send_motion '[
        {"joint_name": "right_shoulder_pitch", "position": -0.3},
        {"joint_name": "left_shoulder_pitch", "position": -0.3}
    ]' "1/4 - Position initiale"
    
    # Étape 2: Lever le bras droit (salut)
    send_motion '[
        {"joint_name": "right_shoulder_pitch", "position": -0.8},
        {"joint_name": "right_elbow_pitch", "position": -0.5}
    ]' "2/4 - Lever le bras"
    
    # Étape 3: Mouvement de salutation (oscillation)
    send_motion '[
        {"joint_name": "right_shoulder_pitch", "position": -0.6},
        {"joint_name": "right_elbow_pitch", "position": -0.3}
    ]' "3/4 - Mouvement de salutation"
    
    # Étape 4: Retour à la position initiale
    send_motion '[
        {"joint_name": "right_shoulder_pitch", "position": -0.3},
        {"joint_name": "right_elbow_pitch", "position": 0.0},
        {"joint_name": "left_shoulder_pitch", "position": -0.3}
    ]' "4/4 - Retour position initiale"
    
    log_success "✅ Salut terminé!"
}

# Fonction principale
main() {
    echo "🤖 BBIA Demo - Salut (Wave)"
    echo "=========================="
    echo "API URL: $API_URL"
    echo "Token: ${TOKEN:0:10}..."
    echo "Durée: ${DURATION}s"
    echo ""
    
    check_api
    wave_sequence
    
    log_success "🎉 Démo salut terminée avec succès!"
}

# Gestion des signaux
cleanup() {
    log_warning "Arrêt demandé, nettoyage..."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Exécution
main "$@"
