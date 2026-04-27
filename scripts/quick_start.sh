#!/bin/bash

# 🚀 Script de démarrage rapide BBIA-SIM
# Menu orienté scripts réellement présents dans ce dépôt.

set -e

# Couleurs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# Fonction pour afficher le menu
show_menu() {
    clear
    echo -e "${CYAN}🤖 === BBIA Reachy Mini Wireless - Menu Principal ===${NC}"
    echo ""
    echo -e "${YELLOW}Menu rapide pour simulation, diagnostic et maintenance locale.${NC}"
    echo ""
    echo -e "${GREEN}Options disponibles:${NC}"
    echo ""
    echo -e "${BLUE}1.${NC} 🧪 ${CYAN}Diagnostic environnement (bbia_doctor)${NC}"
    echo -e "${BLUE}2.${NC} 🎮 ${CYAN}Lancer une démo simulation rapide${NC}"
    echo -e "${BLUE}3.${NC} 📚 ${CYAN}Ouvrir la documentation principale${NC}"
    echo -e "${BLUE}4.${NC} 🔗 ${CYAN}Liens utiles (Discord, GitHub, etc.)${NC}"
    echo -e "${BLUE}5.${NC} 📋 ${CYAN}Spécifications du robot${NC}"
    echo -e "${BLUE}6.${NC} 🌐 ${CYAN}Lancer le serveur dashboard BBIA${NC}"
    echo -e "${BLUE}7.${NC} ✅ ${CYAN}Vérifier la documentation (liens/structure)${NC}"
    echo -e "${BLUE}8.${NC} 🔧 ${CYAN}Lancer API publique en mode check${NC}"
    echo -e "${BLUE}9.${NC} 🧹 ${CYAN}Nettoyer l'environnement${NC}"
    echo -e "${BLUE}10.${NC} 🚀 ${CYAN}Afficher procédure install dépôts Reachy${NC}"
    echo -e "${BLUE}0.${NC} 🚪 ${RED}Quitter${NC}"
    echo ""
    echo -e "${PURPLE}Choisissez une option (0-10):${NC} "
}

# Option 1: Diagnostic environnement
test_bbia() {
    echo -e "${GREEN}🧪 Diagnostic environnement BBIA...${NC}"
    echo ""
    python3 scripts/bbia_doctor.py || true
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 2: Démo simulation rapide
install_environment() {
    echo -e "${GREEN}🎮 Lancement démo simulation rapide...${NC}"
    echo -e "${YELLOW}Commande: bash scripts/run_demo_sim.sh happy 8${NC}"
    echo ""
    echo -e "${CYAN}Continuer ? (y/n):${NC} "
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        bash scripts/run_demo_sim.sh happy 8 || true
    else
        echo -e "${YELLOW}Démo annulée.${NC}"
    fi
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 3: Afficher la documentation
show_documentation() {
    echo -e "${GREEN}📚 Affichage de la documentation...${NC}"
    echo ""
    if [ -f "README.md" ]; then
        echo -e "${CYAN}Documentation trouvée !${NC}"
        echo -e "${YELLOW}Consultez README.md et docs/guides/GUIDE_DEMARRAGE.md${NC}"
        echo ""
        echo -e "${BLUE}Ou utilisez:${NC}"
        echo -e "${CYAN}less README.md${NC}"
        echo ""
        echo -e "${CYAN}Voulez-vous l'afficher maintenant ? (y/n):${NC} "
        read -r response
        if [[ "$response" =~ ^[Yy]$ ]]; then
            less README.md
        fi
    else
        echo -e "${RED}Documentation non trouvée.${NC}"
        echo -e "${YELLOW}Exécutez d'abord l'installation complète.${NC}"
    fi
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 4: Liens utiles
show_links() {
    echo -e "${GREEN}🔗 Liens utiles pour Reachy Mini Wireless:${NC}"
    echo ""
    echo -e "${CYAN}🏢 Officiels Pollen Robotics:${NC}"
    echo -e "${BLUE}  • Site web:${NC} https://www.pollen-robotics.com/reachy-mini-wireless/"
    echo -e "${BLUE}  • Documentation:${NC} https://docs.pollen-robotics.com/"
    echo -e "${BLUE}  • GitHub:${NC} https://github.com/pollen-robotics/"
    echo ""
    echo -e "${CYAN}💬 Communauté:${NC}"
    echo -e "${BLUE}  • Discord:${NC} https://discord.gg/pollen-robotics"
    echo -e "${BLUE}  • Hugging Face:${NC} https://huggingface.co/pollen-robotics"
    echo ""
    echo -e "${CYAN}📚 Ressources d'apprentissage:${NC}"
    echo -e "${BLUE}  • Cours Python IA/ML:${NC} https://www.coursera.org/"
    echo -e "${BLUE}  • Computer Vision:${NC} https://opencv.org/"
    echo -e "${BLUE}  • Speech Recognition:${NC} https://pytorch.org/audio/"
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 5: Spécifications du robot
show_specs() {
    echo -e "${GREEN}📋 Spécifications Reachy Mini Wireless:${NC}"
    echo ""
    echo -e "${CYAN}📏 Dimensions et poids:${NC}"
    echo -e "${BLUE}  • Hauteur:${NC} 28 cm (mode actif) / 23 cm (mode veille)"
    echo -e "${BLUE}  • Largeur:${NC} 16 cm"
    echo -e "${BLUE}  • Poids:${NC} 1,5 kg (3,3 lb)"
    echo ""
    echo -e "${CYAN}💻 Cerveau embarqué:${NC}"
    echo -e "${BLUE}  • Processeur:${NC} Raspberry Pi 5 intégré"
    echo -e "${BLUE}  • Connectivité:${NC} Wi-Fi intégré"
    echo -e "${BLUE}  • Stockage:${NC} Carte SD extensible"
    echo ""
    echo -e "${CYAN}🌐 Connectivité & alimentation:${NC}"
    echo -e "${BLUE}  • Batterie:${NC} Intégrée + alimentation USB-C"
    echo -e "${BLUE}  • Autonomie:${NC} Mobilité complète sans câble"
    echo ""
    echo -e "${CYAN}🗣️ Audio & micros:${NC}"
    echo -e "${BLUE}  • Microphones:${NC} 4 microphones pour reconnaissance vocale"
    echo -e "${BLUE}  • Haut-parleur:${NC} 5W pour voix claire"
    echo ""
    echo -e "${CYAN}📷 Caméra & capteurs:${NC}"
    echo -e "${BLUE}  • Caméra:${NC} Grand angle pour vision et reconnaissance"
    echo -e "${BLUE}  • Accéléromètre:${NC} Mesure mouvements/tremblements"
    echo ""
    echo -e "${CYAN}🤖 Mouvements & expressivité:${NC}"
    echo -e "${BLUE}  • Tête:${NC} 6 degrés de liberté (rotations précises)"
    echo -e "${BLUE}  • Corps:${NC} Rotation complète (yaw_body)"
    echo -e "${BLUE}  • Antennes:${NC} 2 antennes animables avec limites de sécurité (-0.3 à 0.3 rad)"
    echo ""
    echo -e "${CYAN}🛠️ Logiciel & écosystème:${NC}"
    echo -e "${BLUE}  • SDK Principal:${NC} Python (reachy-sdk)"
    echo -e "${BLUE}  • Open-source:${NC} 100% (matériel + logiciel)"
    echo -e "${BLUE}  • Hugging Face:${NC} Intégration native (1,7M+ modèles)"
    echo ""
    echo -e "${CYAN}💶 Prix & disponibilité:${NC}"
    echo -e "${BLUE}  • Prix:${NC} 449$ (~500€)"
    echo -e "${BLUE}  • Livraison:${NC} Fin 2025 - Début 2026"
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 6: Lancer serveur dashboard
launch_unity() {
    echo -e "${GREEN}🌐 Lancement du dashboard BBIA...${NC}"
    echo ""
    echo -e "${YELLOW}Commande: python src/bbia_sim/dashboard_advanced.py --port 8000${NC}"
    echo -e "${CYAN}Continuer ? (y/n):${NC} "
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        python src/bbia_sim/dashboard_advanced.py --port 8000 || true
    fi
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 7: Vérifier documentation
test_unity_config() {
    echo -e "${GREEN}✅ Vérification documentation...${NC}"
    echo ""
    python scripts/verify_documentation.py --consistency || true
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 8: API publique check
fix_unity_warnings() {
    echo -e "${GREEN}🔧 Vérification API publique...${NC}"
    echo ""
    python scripts/start_public_api.py --check || true
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 9: Nettoyer l'environnement
clean_environment() {
    echo -e "${GREEN}🧹 Nettoyage de l'environnement...${NC}"
    echo ""
    echo -e "${YELLOW}Cette opération va lancer cleanup_all.sh --yes${NC}"
    echo ""
    echo -e "${CYAN}Êtes-vous sûr ? (y/n):${NC} "
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}Nettoyage en cours...${NC}"
        bash scripts/cleanup_all.sh --yes || true
        echo -e "${GREEN}Nettoyage terminé !${NC}"
    else
        echo -e "${YELLOW}Nettoyage annulé.${NC}"
    fi
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 10: Procédure installation dépôts Reachy
install_reachy_repos() {
    echo -e "${GREEN}🚀 Procédure installation dépôts Reachy...${NC}"
    echo ""
    echo -e "${CYAN}Ce dépôt ne fournit plus d'installateur global unique des repos Reachy.${NC}"
    echo -e "${CYAN}Utilisez plutôt le repo officiel: https://github.com/pollen-robotics/reachy_mini${NC}"
    echo -e "${CYAN}Et le package: pip install -U reachy-mini${NC}"
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Fonction principale
main() {
    while true; do
        show_menu
        read -r choice
        
        case $choice in
            1)
                test_bbia
                ;;
            2)
                install_environment
                ;;
            3)
                show_documentation
                ;;
            4)
                show_links
                ;;
            5)
                show_specs
                ;;
            6)
                launch_unity
                ;;
            7)
                test_unity_config
                ;;
            8)
                fix_unity_warnings
                ;;
            9)
                clean_environment
                ;;
            10)
                install_reachy_repos
                ;;
            0)
                echo -e "${GREEN}👋 Au revoir ! Bonne préparation pour votre Reachy Mini Wireless !${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}❌ Option invalide. Veuillez choisir 0-10.${NC}"
                sleep 2
                ;;
        esac
    done
}

# Vérifier si on est dans le bon répertoire
if [ ! -f "pyproject.toml" ]; then
    echo -e "${RED}❌ Erreur: Ce script doit être exécuté depuis le répertoire du projet.${NC}"
    exit 1
fi

# Lancer le menu principal
main 