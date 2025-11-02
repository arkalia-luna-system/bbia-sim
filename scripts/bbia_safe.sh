#!/bin/bash
# Script wrapper BBIA avec gestion des processus et sécurité
# Usage: ./bbia_safe.sh [start|stop|status|kill-all] [mode]

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PROCESS_MANAGER="$SCRIPT_DIR/process_manager.py"

# Couleurs pour les logs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
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

log_header() {
    echo -e "${PURPLE}[BBIA]${NC} $1"
}

# Fonction d'aide
show_help() {
    echo "🤖 BBIA Reachy Mini - Gestionnaire Sécurisé"
    echo "============================================="
    echo ""
    echo "Usage: $0 ACTION [MODE] [OPTIONS]"
    echo ""
    echo "Actions disponibles:"
    echo "  start       Démarre le robot (avec détection de doublons)"
    echo "  stop        Arrête le robot (avec confirmation)"
    echo "  status      Affiche le statut des processus"
    echo "  kill-all    Tue tous les processus BBIA (DANGEREUX)"
    echo "  help        Affiche cette aide"
    echo ""
    echo "Modes disponibles (pour start):"
    echo "  graphical   Mode graphique avec fenêtre 3D (défaut)"
    echo "  headless    Mode headless sans fenêtre"
    echo "  test        Test rapide (2s)"
    echo ""
    echo "Options:"
    echo "  --duration N    Durée en secondes (headless uniquement)"
    echo "  --force         Mode force (pas de confirmation)"
    echo ""
    echo "Exemples:"
    echo "  $0 start                    # Démarre en mode graphique"
    echo "  $0 start headless           # Démarre en mode headless"
    echo "  $0 start headless --duration 10  # Headless pendant 10s"
    echo "  $0 stop                     # Arrête avec confirmation"
    echo "  $0 stop --force             # Arrête sans confirmation"
    echo "  $0 status                   # Affiche le statut"
    echo "  $0 kill-all                 # Tue tous les processus"
    echo ""
    echo "Sécurité:"
    echo "  • Détection automatique des doublons"
    echo "  • Arrêt automatique à la fermeture du terminal"
    echo "  • Fichiers de verrouillage pour éviter les conflits"
    echo "  • Confirmation avant arrêt des processus"
}

# Vérification des prérequis
check_prerequisites() {
    log_info "🔍 Vérification des prérequis..."
    
    # Vérifier Python
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 non trouvé"
        exit 1
    fi
    
    # Vérifier le gestionnaire de processus
    if [[ ! -f "$PROCESS_MANAGER" ]]; then
        log_error "Gestionnaire de processus non trouvé: $PROCESS_MANAGER"
        exit 1
    fi
    
    # Vérifier psutil
    if ! python3 -c "import psutil" 2>/dev/null; then
        log_warning "psutil non installé - installation..."
        pip install psutil
    fi
    
    log_success "✅ Prérequis OK"
}

# Fonction principale
main() {
    local action="${1:-help}"
    local mode="${2:-graphical}"
    
    # Afficher l'en-tête
    log_header "🤖 BBIA Reachy Mini - Gestionnaire Sécurisé"
    echo ""
    
    case "$action" in
        "help"|"-h"|"--help")
            show_help
            exit 0
            ;;
        "start")
            check_prerequisites
            log_info "🚀 Démarrage sécurisé du robot BBIA..."
            
            # Construire les arguments
            local args=("start" "--mode" "$mode")
            
            # Ajouter les options supplémentaires
            shift 2
            while [[ $# -gt 0 ]]; do
                args+=("$1")
                shift
            done
            
            # Exécuter le gestionnaire de processus
            cd "$PROJECT_ROOT"
            python3 "$PROCESS_MANAGER" "${args[@]}"
            ;;
        "stop")
            log_info "🛑 Arrêt sécurisé du robot BBIA..."
            
            # Construire les arguments
            local args=("stop")
            
            # Ajouter les options supplémentaires
            shift 1
            while [[ $# -gt 0 ]]; do
                args+=("$1")
                shift
            done
            
            # Exécuter le gestionnaire de processus
            cd "$PROJECT_ROOT"
            python3 "$PROCESS_MANAGER" "${args[@]}"
            ;;
        "status")
            log_info "📊 Vérification du statut..."
            cd "$PROJECT_ROOT"
            python3 "$PROCESS_MANAGER" status
            ;;
        "kill-all")
            log_warning "💀 Mode KILL ALL activé"
            cd "$PROJECT_ROOT"
            python3 "$PROCESS_MANAGER" kill-all
            ;;
        *)
            log_error "Action inconnue: $action"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Gestion des signaux pour l'arrêt propre
cleanup() {
    log_warning "🛑 Signal reçu - Arrêt propre en cours..."
    log_info "💡 Utilisez '$0 stop' pour arrêter les processus BBIA"
    exit 0
}

trap cleanup SIGINT SIGTERM SIGHUP

# Exécution
main "$@"
