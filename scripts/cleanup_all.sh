#!/bin/bash

# 🧹 Script de nettoyage complet - Fusion de cleanup_project.sh, cleanup_metadata_files.sh et smart_process_cleanup.sh
# Nettoie les fichiers cache ET libère la RAM en une seule commande

# Ne pas arrêter sur erreur (certaines commandes peuvent échouer normalement)
set +e

# Vérifier si le terminal supporte les couleurs
if [ -t 1 ] && command -v tput > /dev/null 2>&1 && [ "$(tput colors)" -ge 8 ]; then
    USE_COLORS=true
else
    USE_COLORS=false
fi

# Couleurs pour l'affichage (seulement si supportées)
if [ "$USE_COLORS" = true ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    NC='\033[0m' # No Color
else
    RED=''
    GREEN=''
    YELLOW=''
    BLUE=''
    NC=''
fi

# Options par défaut
CLEAN_CACHE=true
CLEAN_RAM=false
AUTO_CONFIRM=false

# Parser les arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --cache-only)
            CLEAN_CACHE=true
            CLEAN_RAM=false
            shift
            ;;
        --ram-only)
            CLEAN_CACHE=false
            CLEAN_RAM=true
            shift
            ;;
        --yes|-y)
            AUTO_CONFIRM=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --cache-only    Nettoie uniquement les fichiers cache"
            echo "  --ram-only      Nettoie uniquement la RAM (arrête processus gourmands)"
            echo "  --yes, -y       Confirmation automatique (pas de prompt)"
            echo "  --help, -h      Affiche cette aide"
            echo ""
            echo "Par défaut: nettoie les fichiers cache uniquement"
            exit 0
            ;;
        *)
            echo "Option inconnue: $1"
            echo "Utilisez --help pour voir les options disponibles"
            exit 1
            ;;
    esac
done

echo "🧹 Nettoyage complet du projet BBIA..."
echo "======================================"
echo ""

# ============================================================================
# PARTIE 1: NETTOYAGE FICHIERS CACHE (fusionné depuis cleanup_project.sh)
# ============================================================================

# Exclusions disques lourds (T7 / CI) — évite hang find sur .venv/.git
FIND_PRUNE=(-path "./.git" -o -path "./.venv" -o -path "./venv" -o -path "./venv-*" -o -path "./reachy_repos" -o -path "./node_modules" -o -path "./dist" -o -path "./build" -o -path "./.mypy_cache" -o -path "./.pytest_cache" -o -path "./.ruff_cache" -o -path "./htmlcov" -o -path "./site-packages")
find_prj() {
    # Usage: find_prj <find-args...>
    find . \( "${FIND_PRUNE[@]}" \) -prune -o "$@" -print
}
find_prj_delete() {
    find . \( "${FIND_PRUNE[@]}" \) -prune -o "$@" -print0 | xargs -0 rm -f 2>/dev/null || true
}

if [ "$CLEAN_CACHE" = true ]; then
    echo -e "${BLUE}📁 PHASE 1: Nettoyage des fichiers cache${NC}"
    echo "----------------------------------------"
    
    # 1. Supprimer les fichiers système macOS (métadonnées)
    echo "🗑️  Suppression des fichiers système macOS..."
    count_before=$(find_prj -name "._*" -type f 2>/dev/null | wc -l | tr -d ' ')

    # Supprimer métadonnées macOS hors arbres exclus (.venv/.git/…)
    find_prj_delete -name "._*" -type f
    find_prj_delete -name ".!*!._*" -type f
    find_prj_delete -name ".DS_Store" -type f

    count_after=$(find_prj -name "._*" -type f 2>/dev/null | wc -l | tr -d ' ')
    if [ "$count_before" -gt 0 ]; then
        echo -e "   ✅ ${GREEN}Fichiers métadonnées supprimés: $((count_before - count_after))${NC}"
    else
        echo "   ✅ Aucun fichier métadonnées trouvé"
    fi
    
    # 2. Supprimer les fichiers temporaires
    echo "🗑️  Suppression des fichiers temporaires..."
    rm -f reachy_commands.txt reachy_response.txt 2>/dev/null || true
    rm -f *.tmp *.log 2>/dev/null || true
    
    # Nettoyer les logs dans le dossier log/ (sauf fichiers JSON importants)
    if [ -d "log" ]; then
        log_count=$(find log/ -name "*.log" -o -name "*.txt" -o -name "*.err" -o -name "*.out" 2>/dev/null | wc -l | tr -d ' ')
        if [ "$log_count" -gt 0 ]; then
            find log/ -name "*.log" -delete 2>/dev/null || true
            find log/ -name "*.txt" -delete 2>/dev/null || true
            find log/ -name "*.err" -delete 2>/dev/null || true
            find log/ -name "*.out" -delete 2>/dev/null || true
            # Garder les fichiers JSON (rapports bandit, etc.)
            echo "   ✅ ${log_count} fichiers log supprimés dans log/"
        fi
    fi
    echo "   ✅ Fichiers temporaires supprimés"
    
    # 3. Nettoyer les caches Python
    echo "🐍 Nettoyage des caches Python..."
    pycache_count=$(find_prj -type d -name "__pycache__" 2>/dev/null | wc -l | tr -d ' ')
    pyc_count=$(find_prj -name "*.pyc" -type f 2>/dev/null | wc -l | tr -d ' ')
    # Dossiers __pycache__ : prune-aware
    find . \( "${FIND_PRUNE[@]}" \) -prune -o -type d -name "__pycache__" -print0 2>/dev/null | xargs -0 rm -rf 2>/dev/null || true
    find_prj_delete -name "*.pyc" -type f
    find_prj_delete -name "*.pyo" -type f
    echo "   ✅ Caches Python supprimés (${pycache_count} dossiers __pycache__, ${pyc_count} fichiers .pyc)"
    
    # 4. Nettoyer les caches mypy
    echo "🔍 Nettoyage des caches mypy..."
    if [ -d ".mypy_cache" ]; then
        rm -rf .mypy_cache 2>/dev/null || true
        echo "   ✅ Cache mypy supprimé"
    else
        echo "   ✅ Aucun cache mypy trouvé"
    fi
    
    # 5. Nettoyer les caches pytest
    echo "🧪 Nettoyage des caches pytest..."
    if [ -d ".pytest_cache" ]; then
        rm -rf .pytest_cache 2>/dev/null || true
        echo "   ✅ Cache pytest supprimé"
    fi
    
    # 6. Nettoyer les caches ruff
    echo "🔧 Nettoyage des caches ruff..."
    if [ -d ".ruff_cache" ]; then
        rm -rf .ruff_cache 2>/dev/null || true
        echo "   ✅ Cache ruff supprimé"
    fi
    
    # 7. Nettoyer les caches de couverture (coverage)
    echo "📊 Nettoyage des caches de couverture..."
    if [ -d "htmlcov" ]; then
        rm -rf htmlcov 2>/dev/null || true
        echo "   ✅ Cache htmlcov supprimé"
    fi
    if [ -f ".coverage" ]; then
        rm -f .coverage 2>/dev/null || true
        echo "   ✅ Fichier .coverage supprimé"
    fi
    # Garder coverage.xml (peut être utile pour CI/CD)
    
    # 8. Vérifier la structure de documentation
    echo "📚 Vérification de la structure de documentation..."
    mkdir -p docs/semaines docs/rapports docs/archives 2>/dev/null || true
    
    # 9. Déplacer les fichiers de documentation mal placés
    echo "📋 Organisation de la documentation..."
    if ls 📋_SEMAINE_*_*.md 1> /dev/null 2>&1; then
        mv 📋_SEMAINE_*_*.md docs/semaines/ 2>/dev/null || true
    fi
    if ls 📋_BILAN_*.md 📋_ARRET_*.md 📋_REORGANISATION_*.md 1> /dev/null 2>&1; then
        mv 📋_BILAN_*.md docs/rapports/ 2>/dev/null || true
        mv 📋_ARRET_*.md docs/rapports/ 2>/dev/null || true
        mv 📋_REORGANISATION_*.md docs/rapports/ 2>/dev/null || true
    fi
    
    echo ""
    echo -e "${GREEN}✅ Nettoyage fichiers cache terminé !${NC}"
    echo ""
fi

# ============================================================================
# PARTIE 2: NETTOYAGE RAM (fusionné depuis smart_process_cleanup.sh)
# ============================================================================

if [ "$CLEAN_RAM" = true ]; then
    echo -e "${BLUE}💾 PHASE 2: Nettoyage de la RAM${NC}"
    echo "----------------------------------------"
    
    # Fonction pour vérifier si un processus est critique
    is_critical_process() {
        local cmdline="$1"
        local critical_processes=(
            "WindowServer" "kernel" "launchd" "mds_stores" "mdsync" 
            "fskitd" "UVFSService" "XprotectService"
            "Cursor" "ChatGPT" "Perplexity" "Visual Studio Code" "Code"
            "cursor" "chatgpt" "perplexity"
        )
        
        for process in "${critical_processes[@]}"; do
            if [[ "$cmdline" == *"$process"* ]]; then
                return 0  # Processus critique
            fi
        done
        return 1  # Processus non critique
    }
    
    echo "🔍 Recherche des processus problématiques (non critiques)..."
    echo ""
    
    # Identifier les processus gourmands non critiques
    # Utiliser un fichier temporaire pour compter les processus (car while est dans un sous-shell)
    TEMP_FILE=$(mktemp)
    
    ps aux | awk 'NR>1 && ($3 > 15 || $6 > 1000000) {
        cmdline = ""
        for (i=11; i<=NF; i++) cmdline = cmdline " " $i
        
        # Vérifier si le processus est critique
        critical = 0
        if (cmdline ~ /WindowServer|kernel|launchd|mds_stores|mdsync|fskitd|UVFSService|XprotectService|Cursor|ChatGPT|Perplexity|Visual Studio Code|Code|cursor|chatgpt|perplexity/) {
            critical = 1
        }
        
        if (!critical) {
            printf "%s|%s|%.1f|%s\n", $2, $3, $6/1024, cmdline
        }
    }' > "$TEMP_FILE"
    
    problematic_count=$(wc -l < "$TEMP_FILE" | tr -d ' ')
    
    if [ "$problematic_count" -gt 0 ]; then
        while IFS='|' read -r pid cpu ram cmdline; do
            if [[ -n "$pid" ]]; then
                echo "⚠️  PID $pid: ${cpu}% CPU, ${ram}MB RAM"
                echo "   CMD: ${cmdline:0:80}..."
                echo ""
            fi
        done < "$TEMP_FILE"
    fi
    
    rm -f "$TEMP_FILE"
    
    if [ "$problematic_count" -eq 0 ]; then
        echo "✅ Aucun processus problématique trouvé"
    else
        # Demander confirmation (sauf si auto-confirm)
        if [ "$AUTO_CONFIRM" = false ]; then
            echo "❓ Voulez-vous arrêter ces processus problématiques? (y/N)"
            read -r response
        else
            response="y"
        fi
        
        if [[ "$response" =~ ^[Yy]$ ]]; then
            echo "🛑 Arrêt des processus problématiques..."
            
            # Arrêter les processus gourmands non critiques
            # Utiliser un fichier temporaire pour stocker les PIDs
            TEMP_PIDS=$(mktemp)
            ps aux | awk 'NR>1 && ($3 > 15 || $6 > 1000000) {
                cmdline = ""
                for (i=11; i<=NF; i++) cmdline = cmdline " " $i
                
                # Vérifier si le processus est critique
                critical = 0
                if (cmdline ~ /WindowServer|kernel|launchd|mds_stores|mdsync|fskitd|UVFSService|XprotectService|Cursor|ChatGPT|Perplexity|Visual Studio Code|Code|cursor|chatgpt|perplexity/) {
                    critical = 1
                }
                
                if (!critical) {
                    print $2  # PID
                }
            }' > "$TEMP_PIDS"
            
            killed_count=0
            while read -r pid; do
                if [[ -n "$pid" && "$pid" =~ ^[0-9]+$ ]]; then
                    echo "   🛑 Arrêt du processus PID $pid"
                    kill -TERM "$pid" 2>/dev/null || true
                    sleep 1
                    # Si le processus ne répond pas, le tuer
                    if kill -0 "$pid" 2>/dev/null; then
                        echo "   💀 Force kill du processus PID $pid"
                        kill -KILL "$pid" 2>/dev/null || true
                    fi
                    killed_count=$((killed_count + 1))
                fi
            done < "$TEMP_PIDS"
            
            rm -f "$TEMP_PIDS"
            
            echo ""
            if [ "$killed_count" -gt 0 ]; then
                echo -e "${GREEN}✅ $killed_count processus problématiques arrêtés${NC}"
            else
                echo -e "${GREEN}✅ Aucun processus à arrêter${NC}"
            fi
        else
            echo "❌ Arrêt annulé"
        fi
    fi
    
    echo ""
    echo "📊 Processus restants avec CPU > 10%:"
    ps aux | awk 'NR>1 && $3 > 10 {
        cmdline = ""
        for (i=11; i<=NF; i++) cmdline = cmdline " " $i
        printf "PID: %s, CPU: %s%%, RAM: %.1fMB, CMD: %s\n", $2, $3, $6/1024, cmdline
    }' | head -10
    
    echo ""
    echo -e "${GREEN}✅ Nettoyage RAM terminé !${NC}"
    echo ""
fi

# ============================================================================
# RÉSUMÉ FINAL
# ============================================================================

echo ""
echo "======================================"
echo -e "${GREEN}🎯 Nettoyage complet terminé !${NC}"
echo "======================================"
echo ""

# Afficher un résumé de ce qui a été nettoyé
if [ "$CLEAN_CACHE" = true ]; then
    echo -e "${BLUE}📊 Résumé du nettoyage cache:${NC}"
    
    # Compter les fichiers restants pour montrer l'état actuel
    remaining_pycache=$(find . -type d -name "__pycache__" 2>/dev/null | wc -l | tr -d ' ')
    remaining_pyc=$(find . -name "*.pyc" 2>/dev/null | wc -l | tr -d ' ')
    remaining_metadata=$(find . -name "._*" -type f ! -path "./venv/*" ! -path "./venv-*/*" ! -path "./dist/*" ! -path "./build/*" 2>/dev/null | wc -l | tr -d ' ')
    remaining_logs=$(find log/ -name "*.log" -o -name "*.txt" -o -name "*.err" -o -name "*.out" 2>/dev/null | wc -l | tr -d ' ' 2>/dev/null || echo "0")
    
    echo "   • Caches Python restants: ${remaining_pycache} dossiers __pycache__, ${remaining_pyc} fichiers .pyc"
    echo "   • Métadonnées macOS restantes: ${remaining_metadata} fichiers"
    echo "   • Logs restants dans log/: ${remaining_logs} fichiers"
    
    # Vérifier les caches
    [ -d ".mypy_cache" ] && echo "   • Cache mypy: présent" || echo "   • Cache mypy: nettoyé ✅"
    [ -d ".pytest_cache" ] && echo "   • Cache pytest: présent" || echo "   • Cache pytest: nettoyé ✅"
    [ -d ".ruff_cache" ] && echo "   • Cache ruff: présent" || echo "   • Cache ruff: nettoyé ✅"
    [ -d "htmlcov" ] && echo "   • Cache htmlcov: présent" || echo "   • Cache htmlcov: nettoyé ✅"
    [ -f ".coverage" ] && echo "   • Fichier .coverage: présent" || echo "   • Fichier .coverage: nettoyé ✅"
    echo ""
fi

if [ "$CLEAN_RAM" = true ]; then
    echo -e "${BLUE}📊 Résumé du nettoyage RAM:${NC}"
    # Le résumé RAM est déjà affiché dans la section RAM
    echo ""
fi

echo "💡 Commandes utiles:"
echo "   - Nettoyer uniquement cache:  ./scripts/cleanup_all.sh --cache-only"
echo "   - Nettoyer uniquement RAM:    ./scripts/cleanup_all.sh --ram-only"
echo "   - Nettoyer tout automatiquement: ./scripts/cleanup_all.sh --yes"
echo "   - Arrêter processus BBIA:     python scripts/process_manager.py stop"
echo "   - Statut processus BBIA:     python scripts/process_manager.py status"
echo ""

