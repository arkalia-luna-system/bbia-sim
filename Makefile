# Makefile pour BBIA-SIM
# Basé sur les standards Arkalia avec adaptations pour robotique

.PHONY: help install install-dev test test-cov lint format security clean build run demo

# Variables
PYTHON := python3
PIP := pip
VENV := venv
PYTHON_VENV := $(VENV)/bin/python
PIP_VENV := $(VENV)/bin/pip

# Couleurs pour les messages
GREEN := \033[0;32m
YELLOW := \033[1;33m
RED := \033[0;31m
BLUE := \033[0;34m
NC := \033[0m # No Color

help: ## Afficher l'aide
	@echo "$(GREEN)BBIA-SIM - Moteur cognitif pour Reachy Mini$(NC)"
	@echo ""
	@echo "$(BLUE)Commandes principales:$(NC)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  $(YELLOW)%-20s$(NC) %s\n", $$1, $$2}'

install: ## Installer les dépendances de base
	@echo "$(GREEN)Installation des dépendances BBIA...$(NC)"
	$(PIP) install -r requirements.txt
	$(PIP) install -e .

install-dev: ## Installer les dépendances de développement
	@echo "$(GREEN)Installation complète pour développement...$(NC)"
	$(PIP) install -r requirements.txt
	$(PIP) install -e ".[dev,test,docs]"
	$(PIP) install pre-commit
	@if [ -f .pre-commit-config.yaml ]; then pre-commit install; fi

test: ## Lancer les tests unitaires
	@echo "$(GREEN)🧪 Tests BBIA...$(NC)"
	$(PYTHON) -m pytest -v -m "not slow"

test-all: ## Lancer tous les tests (y compris lents)
	@echo "$(GREEN)🧪 Tests complets BBIA...$(NC)"
	$(PYTHON) -m pytest -v

test-cov: ## Tests avec couverture
	@echo "$(GREEN)📊 Tests avec couverture...$(NC)"
	$(PYTHON) -m pytest --cov=bbia_sim --cov-report=html --cov-report=term-missing --cov-report=xml

test-audio: ## Tests spécifiques audio (nécessite micro/speakers)
	@echo "$(GREEN)🎵 Tests audio BBIA...$(NC)"
	$(PYTHON) -m pytest tests/ -v -m "audio"

test-vision: ## Tests spécifiques vision (nécessite caméra)
	@echo "$(GREEN)👁️ Tests vision BBIA...$(NC)"
	$(PYTHON) -m pytest tests/ -v -m "vision"

lint: ## Vérifications de code
	@echo "$(GREEN)🔍 Linting BBIA...$(NC)"
	ruff check src/ tests/
	mypy src/bbia_sim/

format: ## Formater le code
	@echo "$(GREEN)✨ Formatage du code...$(NC)"
	black src/ tests/
	isort src/ tests/
	ruff format src/ tests/

security: ## Vérifications de sécurité
	@echo "$(GREEN)🔒 Audit sécurité...$(NC)"
	@find src -name "._*" -type f -delete 2>/dev/null || true
	@find tests -name "._*" -type f -delete 2>/dev/null || true
	@bandit -r src/bbia_sim/ -x tests -c .bandit
	@echo "$(YELLOW)Sécurité OK pour usage robotique$(NC)"

clean: ## Nettoyer les fichiers temporaires
	@echo "$(GREEN)🧹 Nettoyage...$(NC)"
	find . -type f -name "*.pyc" -delete
	find . -type d -name "__pycache__" -delete
	find . -type d -name "*.egg-info" -exec rm -rf {} +
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	find . -type d -name "htmlcov" -exec rm -rf {} +
	find . -type f -name ".coverage" -delete
	find . -type f -name "coverage.xml" -delete
	find . -name "._*" -delete
	find . -name ".DS_Store" -delete
	find . -name "*.wav" -path "./test*.wav" -delete

build: ## Construire le package
	@echo "$(GREEN)📦 Construction BBIA...$(NC)"
	$(PYTHON) -m build

# Commandes de démonstration BBIA
demo: ## Démonstration complète BBIA
	@echo "$(GREEN)🤖 Démonstration BBIA Reachy...$(NC)"
	$(PYTHON) tests/demo_bbia_complete.py

demo-voice: ## Démonstration synthèse vocale
	@echo "$(GREEN)🗣️ Test voix BBIA...$(NC)"
	$(PYTHON) src/bbia_sim/bbia_voice.py

demo-audio: ## Démonstration audio
	@echo "$(GREEN)🎵 Test audio BBIA...$(NC)"
	$(PYTHON) src/bbia_sim/bbia_audio.py

demo-emotions: ## Démonstration émotions
	@echo "$(GREEN)😊 Test émotions BBIA...$(NC)"
	$(PYTHON) src/bbia_sim/bbia_emotions.py

demo-vision: ## Démonstration vision
	@echo "$(GREEN)👁️ Test vision BBIA...$(NC)"
	$(PYTHON) src/bbia_sim/bbia_vision.py

# Commandes de qualité
check: format lint test security ## Vérifications complètes (format, lint, test, security)

ci: clean install-dev check build ## Pipeline CI complet

# Release et versioning
version: ## Afficher la version actuelle
	@echo "$(GREEN)Version BBIA-SIM:$(NC)"
	@$(PYTHON) -c "try: import tomllib; except ImportError: import tomli as tomllib; print(tomllib.load(open('pyproject.toml', 'rb'))['project']['version'])"

release-check: check ## Vérifier que tout est prêt pour release
	@echo "$(GREEN)✅ Vérification release BBIA...$(NC)"
	@echo "$(YELLOW)Prêt pour release !$(NC)"

# Documentation
docs: ## Générer la documentation
	@echo "$(GREEN)📚 Génération documentation...$(NC)"
	@if command -v mkdocs >/dev/null 2>&1; then \
		mkdocs build; \
		echo "$(YELLOW)Documentation générée dans site/$(NC)"; \
	else \
		echo "$(YELLOW)mkdocs non installé, skip...$(NC)"; \
	fi

docs-serve: ## Servir la documentation localement
	@echo "$(GREEN)📖 Serveur documentation...$(NC)"
	@if command -v mkdocs >/dev/null 2>&1; then \
		mkdocs serve; \
	else \
		echo "$(YELLOW)mkdocs non installé, skip...$(NC)"; \
	fi

# Configuration système Reachy
install-portaudio: ## Installer portaudio (macOS)
	@echo "$(GREEN)🎵 Installation portaudio...$(NC)"
	@if command -v brew >/dev/null 2>&1; then \
		brew install portaudio; \
		echo "$(YELLOW)Relancez 'make install' après installation$(NC)"; \
	else \
		echo "$(RED)Homebrew requis pour installer portaudio$(NC)"; \
	fi

check-system: ## Vérifier la configuration système
	@echo "$(GREEN)⚙️ Vérification système...$(NC)"
	@$(PYTHON) -c "import sys; print(f'Python: {sys.version}')"
	@$(PYTHON) -c "try: import pyaudio; print('✅ PyAudio OK'); except: print('❌ PyAudio manquant')"
	@$(PYTHON) -c "try: import cv2; print('✅ OpenCV OK'); except: print('❌ OpenCV manquant')"
	@$(PYTHON) -c "try: import pyttsx3; print('✅ pyttsx3 OK'); except: print('❌ pyttsx3 manquant')"

# Configuration développement
dev-setup: install-dev install-portaudio check-system ## Configuration complète développement
	@echo "$(GREEN)🚀 Configuration BBIA terminée !$(NC)"
	@echo "$(YELLOW)Commandes utiles:$(NC)"
	@echo "  $(BLUE)make demo$(NC)          - Démonstration complète"
	@echo "  $(BLUE)make test$(NC)          - Tests rapides"
	@echo "  $(BLUE)make demo-voice$(NC)    - Test voix uniquement"

# Aide rapide
quick-help: ## Aide rapide pour démarrer
	@echo "$(GREEN)🤖 BBIA-SIM - Démarrage rapide:$(NC)"
	@echo ""
	@echo "$(BLUE)1. Installation:$(NC) make dev-setup"
	@echo "$(BLUE)2. Tests:$(NC)        make test"
	@echo "$(BLUE)3. Démo:$(NC)         make demo"
	@echo "$(BLUE)4. Aide complète:$(NC) make help"
