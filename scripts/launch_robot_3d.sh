#!/bin/bash
# Script de lancement du robot Reachy Mini complet en 3D
# Usage: ./launch_robot_3d.sh [headless|graphical]

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LAUNCHER="$SCRIPT_DIR/launch_complete_robot.py"

# Si lancé depuis la racine via lien symbolique
if [[ "$(basename "$0")" == "launch_robot.sh" ]]; then
    SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
    PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
    LAUNCHER="$SCRIPT_DIR/launch_complete_robot.py"
fi

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

# Fonction d'aide
show_help() {
    echo "🤖 BBIA Reachy Mini - Lanceur 3D"
    echo "================================="
    echo ""
    echo "Usage: $0 [MODE]"
    echo ""
    echo "Modes disponibles:"
    echo "  graphical    Lance la fenêtre 3D (défaut)"
    echo "  headless     Mode headless sans fenêtre"
    echo "  test         Test rapide (2s headless)"
    echo ""
    echo "Exemples:"
    echo "  $0                    # Fenêtre 3D"
    echo "  $0 graphical          # Fenêtre 3D"
    echo "  $0 headless           # Mode headless"
    echo "  $0 test               # Test rapide"
    echo ""
    echo "Contrôles fenêtre 3D:"
    echo "  • Souris : Rotation de la vue"
    echo "  • Molette : Zoom"
    echo "  • Clic droit : Déplacer la vue"
    echo "  • Échap : Fermer la fenêtre"
}

# Vérification des prérequis
check_prerequisites() {
    log_info "🔍 Vérification des prérequis..."
    
    # Vérifier Python
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 non trouvé"
        exit 1
    fi
    
    # Vérifier mjpython (pour macOS)
    if [[ "$OSTYPE" == "darwin"* ]]; then
        if ! command -v mjpython &> /dev/null; then
            log_warning "mjpython non trouvé sur macOS"
            log_warning "La fenêtre 3D pourrait ne pas s'ouvrir"
            log_warning "Installez: pip install mujoco-python-viewer"
        fi
    fi
    
    # Vérifier le lanceur
    if [[ ! -f "$LAUNCHER" ]]; then
        log_error "Lanceur non trouvé: $LAUNCHER"
        exit 1
    fi
    
    log_success "✅ Prérequis OK"
}

# Lancement mode graphique
launch_graphical() {
    log_info "🎮 Lancement mode graphique..."
    
    if [[ "$OSTYPE" == "darwin"* ]]; then
        log_info "🍎 macOS détecté - utilisation de mjpython"
        if command -v mjpython &> /dev/null; then
            cd "$PROJECT_ROOT"
            mjpython "$LAUNCHER"
        else
            log_warning "mjpython non disponible, tentative avec python"
            cd "$PROJECT_ROOT"
            python3 "$LAUNCHER"
        fi
    else
        log_info "🐧 Linux/Windows - utilisation de python"
        cd "$PROJECT_ROOT"
        python3 "$LAUNCHER"
    fi
}

# Lancement mode headless
launch_headless() {
    log_info "🔄 Lancement mode headless..."
    cd "$PROJECT_ROOT"
    python3 "$LAUNCHER" --headless
}

# Test rapide
launch_test() {
    log_info "🧪 Test rapide (2s)..."
    cd "$PROJECT_ROOT"
    python3 "$LAUNCHER" --headless --duration 2
}

# Fonction principale
main() {
    local mode="${1:-graphical}"
    
    case "$mode" in
        "help"|"-h"|"--help")
            show_help
            exit 0
            ;;
        "graphical"|"gui"|"3d")
            check_prerequisites
            launch_graphical
            ;;
        "headless"|"no-gui")
            check_prerequisites
            launch_headless
            ;;
        "test"|"quick")
            check_prerequisites
            launch_test
            ;;
        *)
            log_error "Mode inconnu: $mode"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Exécution
main "$@"
