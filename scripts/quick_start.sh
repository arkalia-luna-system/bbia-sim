#!/bin/bash

# 🚀 Script de démarrage rapide pour Reachy Mini Wireless
# Options d'installation et de test pour BBIA

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
    echo -e "${YELLOW}Votre Reachy Mini Wireless arrive fin 2025 !${NC}"
    echo -e "${YELLOW}Préparez-vous dès maintenant avec BBIA !${NC}"
    echo ""
    echo -e "${GREEN}Options disponibles:${NC}"
    echo ""
    echo -e "${BLUE}1.${NC} 🧪 ${CYAN}Tester BBIA (simulation rapide)${NC}"
    echo -e "${BLUE}2.${NC} 🛠️  ${CYAN}Installation complète de l'environnement${NC}"
    echo -e "${BLUE}3.${NC} 📚 ${CYAN}Afficher la documentation complète${NC}"
    echo -e "${BLUE}4.${NC} 🔗 ${CYAN}Liens utiles (Discord, GitHub, etc.)${NC}"
    echo -e "${BLUE}5.${NC} 📋 ${CYAN}Spécifications du robot${NC}"
    echo -e "${BLUE}6.${NC} 🎮 ${CYAN}Lancer le simulateur Unity (si configuré)${NC}"
    echo -e "${BLUE}7.${NC} 🧪 ${CYAN}Tester la configuration Unity${NC}"
    echo -e "${BLUE}8.${NC} 🔧 ${CYAN}Corriger les avertissements Unity${NC}"
    echo -e "${BLUE}9.${NC} 🧹 ${CYAN}Nettoyer l'environnement${NC}"
    echo -e "${BLUE}10.${NC} 🚀 ${CYAN}Installer tous les dépôts GitHub Reachy${NC}"
    echo -e "${BLUE}0.${NC} 🚪 ${RED}Quitter${NC}"
    echo ""
    echo -e "${PURPLE}Choisissez une option (0-9):${NC} "
}

# Option 1: Tester BBIA
test_bbia() {
    echo -e "${GREEN}🧪 Lancement du test BBIA...${NC}"
    echo ""
    python3 test_bbia_reachy.py
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 2: Installation complète
install_environment() {
    echo -e "${GREEN}🛠️ Installation complète de l'environnement...${NC}"
    echo -e "${YELLOW}Cette opération peut prendre plusieurs minutes.${NC}"
    echo ""
    echo -e "${CYAN}Voulez-vous continuer ? (y/n):${NC} "
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        ./setup_reachy_environment.sh
    else
        echo -e "${YELLOW}Installation annulée.${NC}"
    fi
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 3: Afficher la documentation
show_documentation() {
    echo -e "${GREEN}📚 Affichage de la documentation...${NC}"
    echo ""
    if [ -f "REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md" ]; then
        echo -e "${CYAN}Documentation trouvée !${NC}"
        echo -e "${YELLOW}Ouvrez le fichier REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md dans votre éditeur.${NC}"
        echo ""
        echo -e "${BLUE}Ou utilisez:${NC}"
        echo -e "${CYAN}cat REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md | less${NC}"
        echo ""
        echo -e "${CYAN}Voulez-vous l'afficher maintenant ? (y/n):${NC} "
        read -r response
        if [[ "$response" =~ ^[Yy]$ ]]; then
            cat REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md | less
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
    echo -e "${BLUE}  • Corps:${NC} Rotation complète"
    echo -e "${BLUE}  • Antennes:${NC} 2 antennes animées pour expressivité"
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

# Option 6: Lancer le simulateur Unity
launch_unity() {
    echo -e "${GREEN}🎮 Lancement du simulateur Unity...${NC}"
    echo ""
    if [ -d "reachy-bbia-unity" ]; then
        echo -e "${CYAN}Projet Unity trouvé !${NC}"
        echo -e "${YELLOW}Lancement en cours...${NC}"
        echo ""
        echo -e "${CYAN}Pour lancer Unity :${NC}"
        echo -e "${BLUE}1. Ouvrez Unity Hub${NC}"
        echo -e "${BLUE}2. Cliquez sur 'Open'${NC}"
        echo -e "${BLUE}3. Sélectionnez le dossier: reachy-bbia-unity${NC}"
        echo -e "${BLUE}4. Le projet se chargera automatiquement${NC}"
        echo ""
        echo -e "${YELLOW}Ou utilisez la commande :${NC}"
        echo -e "${CYAN}open -a 'Unity Hub' reachy-bbia-unity${NC}"
        echo ""
        echo -e "${CYAN}Voulez-vous lancer Unity Hub maintenant ? (y/n):${NC} "
        read -r response
        if [[ "$response" =~ ^[Yy]$ ]]; then
            echo -e "${YELLOW}Lancement d'Unity Hub...${NC}"
            if [ -f "launch_unity.sh" ]; then
                ./launch_unity.sh
            else
                open -a "Unity Hub" reachy-bbia-unity
            fi
        fi
    else
        echo -e "${RED}Projet Unity non trouvé.${NC}"
        echo -e "${YELLOW}Le dossier reachy-bbia-unity n'existe pas.${NC}"
    fi
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 7: Tester la configuration Unity
test_unity_config() {
    echo -e "${GREEN}🧪 Test de la configuration Unity...${NC}"
    echo ""
    ./test_unity_setup.sh
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 8: Corriger les avertissements Unity
fix_unity_warnings() {
    echo -e "${GREEN}🔧 Correction des avertissements Unity...${NC}"
    echo ""
    ./fix_unity_warnings.sh
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 9: Nettoyer l'environnement
clean_environment() {
    echo -e "${GREEN}🧹 Nettoyage de l'environnement...${NC}"
    echo ""
    echo -e "${YELLOW}Cette opération va supprimer:${NC}"
    echo -e "${RED}  • Environnement virtuel reachy_env${NC}"
    echo -e "${RED}  • Dossier reachy-bbia-project${NC}"
    echo -e "${RED}  • Dossier external_repos${NC}"
    echo ""
    echo -e "${CYAN}Êtes-vous sûr ? (y/n):${NC} "
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}Nettoyage en cours...${NC}"
        rm -rf reachy_env reachy-bbia-project external_repos
        echo -e "${GREEN}Nettoyage terminé !${NC}"
    else
        echo -e "${YELLOW}Nettoyage annulé.${NC}"
    fi
    echo ""
    echo -e "${YELLOW}Appuyez sur Entrée pour continuer...${NC}"
    read
}

# Option 10: Installer tous les dépôts GitHub Reachy
install_reachy_repos() {
    echo -e "${GREEN}🚀 Installation de tous les dépôts GitHub Reachy...${NC}"
    echo ""
    echo -e "${YELLOW}Cette opération va installer:${NC}"
    echo -e "${CYAN}  • reachy-docs (Documentation officielle)${NC}"
    echo -e "${CYAN}  • pollen-vision (Vision par ordinateur)${NC}"
    echo -e "${CYAN}  • emotion_inference_hub (Détection d'émotions)${NC}"
    echo -e "${CYAN}  • reachy2-sdk-audio-server-rs (Serveur audio)${NC}"
    echo -e "${CYAN}  • reachy2-behaviors-dev (Comportements)${NC}"
    echo -e "${CYAN}  • reachy-dashboard (Interface web)${NC}"
    echo -e "${CYAN}  • reachy-face-tracking (Suivi de visage)${NC}"
    echo ""
    echo -e "${CYAN}Voulez-vous continuer ? (y/n):${NC} "
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        if [ -f "install_all_reachy_repos.sh" ]; then
            echo -e "${YELLOW}Lancement de l'installation...${NC}"
            ./install_all_reachy_repos.sh
        else
            echo -e "${RED}Script d'installation non trouvé.${NC}"
            echo -e "${YELLOW}Veuillez d'abord créer le script install_all_reachy_repos.sh${NC}"
        fi
    else
        echo -e "${YELLOW}Installation annulée.${NC}"
    fi
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
if [ ! -f "test_bbia_reachy.py" ]; then
    echo -e "${RED}❌ Erreur: Ce script doit être exécuté depuis le répertoire du projet.${NC}"
    exit 1
fi

# Lancer le menu principal
main 