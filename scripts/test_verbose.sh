#!/bin/bash
# Script pour lancer les tests avec une visibilité maximale
# Affiche la progression en temps réel et identifie les tests lents

set -e

# Couleurs pour les messages
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🧪 Lancement des tests avec visibilité maximale...${NC}"
echo ""

# Configuration
export PYTHONPATH=src:$PYTHONPATH
export MUJOCO_GL=disable
export DISPLAY=""
export BBIA_DISABLE_AUDIO=1

# Options pytest pour visibilité maximale
PYTEST_OPTS=(
    "-vv"                    # Double verbose - voir chaque test en temps réel
    "--durations=30"         # Afficher les 30 tests les plus lents
    "--tb=short"            # Traceback court mais informatif
    "--showlocals"          # Afficher variables locales en cas d'erreur
    "--log-cli-level=INFO"  # Niveau INFO pour voir les logs en temps réel
    "--color=yes"           # Couleurs pour meilleure lisibilité
    "-ra"                   # Afficher résumé des erreurs
)

# Filtrer les tests lents par défaut (peut être modifié)
MARKER_FILTER="${1:-not e2e and not slow and not heavy and not hardware}"

echo -e "${YELLOW}📊 Filtre de tests: ${MARKER_FILTER}${NC}"
echo -e "${YELLOW}💡 Astuce: Pour voir tous les tests, utilisez: ./scripts/test_verbose.sh 'not e2e'${NC}"
echo ""

# Lancer les tests
pytest tests/ \
    -m "${MARKER_FILTER}" \
    "${PYTEST_OPTS[@]}" \
    --timeout=60 \
    --maxfail=10

echo ""
echo -e "${GREEN}✅ Tests terminés!${NC}"
echo -e "${BLUE}💡 Les tests les plus lents sont affichés à la fin du rapport${NC}"

