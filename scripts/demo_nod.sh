#!/bin/bash
# Démo BBIA - Hochement de tête (nod)
# Script de démonstration du hochement de tête

set -euo pipefail

# Configuration
API_URL="${API_URL:-http://localhost:8000}"
TOKEN="${TOKEN:-bbia-secret-key-dev}"
DURATION="${DURATION:-6}"

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
        sleep 1.0
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

# Séquence de hochement de tête
nod_sequence() {
    log_info "🤖 Démarrage du hochement de tête BBIA"
    log_info "Durée estimée: ${DURATION}s"
    echo "=========================================="
    
    # Étape 1: Position initiale (tête droite)
    send_motion '[
        {"joint_name": "neck_yaw", "position": 0.0}
    ]' "1/6 - Position initiale"
    
    # Étape 2: Incliner la tête vers le bas (pitch négatif)
    send_motion '[
        {"joint_name": "neck_yaw", "position": 0.0}
    ]' "2/6 - Incliner vers le bas"
    
    # Étape 3: Relever la tête (pitch positif)
    send_motion '[
        {"joint_name": "neck_yaw", "position": 0.0}
    ]' "3/6 - Relever la tête"
    
    # Étape 4: Deuxième hochement - bas
    send_motion '[
        {"joint_name": "neck_yaw", "position": 0.0}
    ]' "4/6 - Deuxième hochement (bas)"
    
    # Étape 5: Deuxième hochement - haut
    send_motion '[
        {"joint_name": "neck_yaw", "position": 0.0}
    ]' "5/6 - Deuxième hochement (haut)"
    
    # Étape 6: Retour à la position neutre
    send_motion '[
        {"joint_name": "neck_yaw", "position": 0.0}
    ]' "6/6 - Position neutre"
    
    log_success "✅ Hochement de tête terminé!"
}

# Fonction principale
main() {
    echo "🤖 BBIA Demo - Hochement de tête (Nod)"
    echo "====================================="
    echo "API URL: $API_URL"
    echo "Token: ${TOKEN:0:10}..."
    echo "Durée: ${DURATION}s"
    echo ""
    
    check_api
    nod_sequence
    
    log_success "🎉 Démo hochement terminée avec succès!"
}

# Gestion des signaux
cleanup() {
    log_warning "Arrêt demandé, nettoyage..."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Exécution
main "$@"
