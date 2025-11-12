#!/usr/bin/env bash
# 🚀 Script All-in-One - BBIA Reachy Mini Simulation Starter
# Automatise l'installation, vérification et démarrage de BBIA-SIM
#
# Usage: ./scripts/reachy-mini-sim-starter.sh [--skip-install] [--skip-dashboard]
#
# Options:
#   --skip-install    : Saute l'installation des dépendances
#   --skip-dashboard  : Ne lance pas le dashboard automatiquement
#   --help            : Affiche cette aide

set -euo pipefail

# Couleurs pour output
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly CYAN='\033[0;36m'
readonly NC='\033[0m' # No Color

# Chemins
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT
readonly VENV_DIR="${PROJECT_ROOT}/venv"
readonly LOG_DIR="${PROJECT_ROOT}/log"

# Flags
SKIP_INSTALL=false
SKIP_DASHBOARD=false

# Fonctions utilitaires
log_info() {
    echo -e "${CYAN}ℹ️  $*${NC}" >&2
}

log_success() {
    echo -e "${GREEN}✅ $*${NC}" >&2
}

log_warning() {
    echo -e "${YELLOW}⚠️  $*${NC}" >&2
}

log_error() {
    echo -e "${RED}❌ $*${NC}" >&2
}

log_step() {
    echo -e "\n${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}" >&2
    echo -e "${BLUE}📋 $*${NC}" >&2
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}\n" >&2
}

# Afficher l'aide
show_help() {
    cat << EOF
🚀 BBIA Reachy Mini Simulation Starter

Script all-in-one qui automatise :
  1. Vérification des prérequis (Python, pip, mjpython)
  2. Création/activation de l'environnement virtuel
  3. Installation des dépendances BBIA-SIM
  4. Vérification de l'installation (bbia_doctor)
  5. Lancement automatique du dashboard

Usage:
    ./scripts/reachy-mini-sim-starter.sh [OPTIONS]

Options:
    --skip-install      Saute l'installation des dépendances
    --skip-dashboard    Ne lance pas le dashboard automatiquement
    --help              Affiche cette aide

Exemples:
    # Installation complète + dashboard
    ./scripts/reachy-mini-sim-starter.sh

    # Vérification uniquement (sans installation)
    ./scripts/reachy-mini-sim-starter.sh --skip-install --skip-dashboard

    # Installation sans dashboard
    ./scripts/reachy-mini-sim-starter.sh --skip-dashboard
EOF
}

# Parser les arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-install)
                SKIP_INSTALL=true
                shift
                ;;
            --skip-dashboard)
                SKIP_DASHBOARD=true
                shift
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            *)
                log_error "Option inconnue: $1"
                echo "Utilisez --help pour voir l'aide"
                exit 1
                ;;
        esac
    done
}

# Vérifier les prérequis système
check_prerequisites() {
    log_step "Vérification des prérequis système"

    local missing=0

    # Vérifier Python 3
    if ! command -v python3 &> /dev/null; then
        log_error "Python 3 n'est pas installé"
        log_info "Installez Python 3.11+ depuis https://www.python.org/"
        missing=$((missing + 1))
    else
        local python_version
        python_version=$(python3 --version 2>&1 | awk '{print $2}')
        log_success "Python 3 trouvé: ${python_version}"
    fi

    # Vérifier pip
    if ! command -v pip3 &> /dev/null && ! python3 -m pip --version &> /dev/null; then
        log_error "pip n'est pas installé"
        log_info "Installez pip: python3 -m ensurepip --upgrade"
        missing=$((missing + 1))
    else
        log_success "pip trouvé"
    fi

    # Vérifier mjpython sur macOS
    if [[ "$OSTYPE" == "darwin"* ]]; then
        if ! command -v mjpython &> /dev/null; then
            log_warning "mjpython non trouvé sur macOS"
            log_info "Pour la simulation 3D, installez: pip install mujoco-python-viewer"
            log_info "Le script continuera sans mjpython"
        else
            log_success "mjpython trouvé"
        fi
    fi

    if [[ $missing -gt 0 ]]; then
        log_error "${missing} prérequis manquant(s)"
        exit 1
    fi

    log_success "Tous les prérequis sont satisfaits"
}

# Créer ou activer l'environnement virtuel
setup_venv() {
    log_step "Configuration de l'environnement virtuel"

    if [[ -d "${VENV_DIR}" ]]; then
        log_info "Environnement virtuel existant trouvé"
    else
        log_info "Création de l'environnement virtuel..."
        python3 -m venv "${VENV_DIR}"
        log_success "Environnement virtuel créé"
    fi

    # Activer le venv
    log_info "Activation de l'environnement virtuel..."
    # shellcheck source=/dev/null
    source "${VENV_DIR}/bin/activate"

    # Mettre à jour pip
    log_info "Mise à jour de pip..."
    pip install --upgrade pip --quiet
    log_success "pip mis à jour"
}

# Installer les dépendances
install_dependencies() {
    if [[ "$SKIP_INSTALL" == true ]]; then
        log_warning "Installation des dépendances ignorée (--skip-install)"
        return 0
    fi

    log_step "Installation des dépendances BBIA-SIM"

    # Activer venv si pas déjà fait
    if [[ -z "${VIRTUAL_ENV:-}" ]]; then
        # shellcheck source=/dev/null
        source "${VENV_DIR}/bin/activate"
    fi

    # Installer le projet en mode editable
    log_info "Installation de BBIA-SIM en mode développement..."
    if pip install -e "${PROJECT_ROOT}[dev]" --quiet; then
        log_success "BBIA-SIM installé"
    else
        log_error "Échec de l'installation de BBIA-SIM"
        exit 1
    fi

    # Installer SDK Reachy Mini si disponible
    log_info "Vérification du SDK Reachy Mini..."
    if pip install reachy-mini --quiet 2>/dev/null; then
        log_success "SDK Reachy Mini installé"
    else
        log_warning "SDK Reachy Mini non disponible (normal en simulation)"
    fi
}

# Vérifier l'installation avec bbia_doctor
verify_installation() {
    log_step "Vérification de l'installation"

    # Activer venv si nécessaire
    if [[ -z "${VIRTUAL_ENV:-}" ]]; then
        # shellcheck source=/dev/null
        source "${VENV_DIR}/bin/activate"
    fi

    # Créer le répertoire log si nécessaire
    mkdir -p "${LOG_DIR}"

    # Lancer bbia_doctor
    if [[ -f "${PROJECT_ROOT}/scripts/bbia_doctor.py" ]]; then
        log_info "Exécution de bbia_doctor..."
        if python3 "${PROJECT_ROOT}/scripts/bbia_doctor.py" > "${LOG_DIR}/bbia_doctor.log" 2>&1; then
            log_success "Vérification terminée"
            log_info "Rapport complet: ${LOG_DIR}/bbia_doctor.log"
        else
            log_warning "bbia_doctor a rencontré des problèmes"
            log_info "Consultez: ${LOG_DIR}/bbia_doctor.log"
        fi
    else
        log_warning "bbia_doctor.py non trouvé, vérification ignorée"
    fi
}

# Lancer le dashboard
launch_dashboard() {
    if [[ "$SKIP_DASHBOARD" == true ]]; then
        log_warning "Lancement du dashboard ignoré (--skip-dashboard)"
        return 0
    fi

    log_step "Lancement du dashboard BBIA"

    # Activer venv si nécessaire
    if [[ -z "${VIRTUAL_ENV:-}" ]]; then
        # shellcheck source=/dev/null
        source "${VENV_DIR}/bin/activate"
    fi

    log_info "Démarrage du dashboard sur http://localhost:8000"
    log_info "Appuyez sur Ctrl+C pour arrêter le dashboard"

    # Lancer le dashboard
    if python3 -m bbia_sim.dashboard_advanced --port 8000 2>&1; then
        log_success "Dashboard arrêté proprement"
    else
        log_error "Erreur lors du lancement du dashboard"
        log_info "Essayez manuellement: python -m bbia_sim.dashboard_advanced"
        exit 1
    fi
}

# Afficher les instructions finales
show_next_steps() {
    log_step "Prochaines étapes"

    cat << EOF
${GREEN}🎉 BBIA-SIM est prêt !${NC}

${CYAN}Commandes utiles :${NC}

  ${BLUE}1. Activer l'environnement virtuel :${NC}
     source venv/bin/activate

  ${BLUE}2. Lancer une démo simulation :${NC}
     mjpython examples/demo_emotion_ok.py
     # ou sur Linux/Windows:
     python examples/demo_emotion_ok.py

  ${BLUE}3. Lancer le dashboard :${NC}
     python -m bbia_sim.dashboard_advanced --port 8000
     # Puis ouvrir http://localhost:8000

  ${BLUE}4. Vérifier l'installation :${NC}
     python scripts/bbia_doctor.py

${CYAN}Documentation :${NC}
  - Guide débutant : docs/guides/GUIDE_DEBUTANT.md
  - README complet : README.md

${CYAN}Support :${NC}
  - Issues GitHub : https://github.com/arkalia-luna-system/bbia-sim/issues
EOF
}

# Fonction principale
main() {
    # Parser les arguments
    parse_args "$@"

    # Afficher le header
    echo -e "${CYAN}"
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║  🚀 BBIA Reachy Mini Simulation Starter                   ║"
    echo "║  Script All-in-One - Installation & Démarrage           ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo -e "${NC}\n"

    # Créer le répertoire log
    mkdir -p "${LOG_DIR}"

    # Étapes principales
    check_prerequisites
    setup_venv
    install_dependencies
    verify_installation

    # Afficher les prochaines étapes si on ne lance pas le dashboard
    if [[ "$SKIP_DASHBOARD" == true ]]; then
        show_next_steps
    else
        launch_dashboard
    fi
}

# Point d'entrée
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi

