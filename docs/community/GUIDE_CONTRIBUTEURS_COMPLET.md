# 🤝 Guide Contributeurs Complet - BBIA-SIM

**Dernière mise à jour : 15 Décembre 2025  
**Version** : 1.4.0  
**Objectif** : Guide complet pour contribuer à BBIA-SIM

---

## 🎯 Pourquoi Contribuer à BBIA-SIM ?

**BBIA-SIM** est un projet open source qui apporte :

- ✅ **RobotAPI Unifié** : Même code sim/robot réel
- ✅ **Solution 100% Gratuite** : Whisper, SmolVLM2, LLM local
- ✅ **IA** : 15+ modules, 21 comportements, 12 émotions
- ✅ **Qualité** : 1,743 tests, 68.86% coverage, 219 fichiers MD

**Votre contribution** aide à maintenir et améliorer le projet.

---

## 🚀 Démarrage Rapide

### 1. Fork et Clone

```bash
# Fork le projet sur GitHub, puis clonez votre fork
git clone https://github.com/votre-username/bbia-sim.git
cd bbia-sim
```

### 2. Installation

```bash
# Créer environnement virtuel
python -m venv venv
source venv/bin/activate  # Sur macOS/Linux
# ou
venv\Scripts\activate  # Sur Windows

# Installation en mode développement
pip install -e .[dev,test]

# Vérifier l'installation
python -m bbia_sim --doctor
```

### 3. Créer une Branche

```bash
# Depuis develop
git checkout develop
git pull origin develop

# Créer votre branche
git checkout -b feature/nom-de-votre-contribution
```

**Convention de nommage** :
- `feature/` - Nouvelle fonctionnalité
- `fix/` - Correction de bug
- `docs/` - Documentation
- `refactor/` - Refactoring
- `test/` - Tests uniquement

---

## 📋 Good First Issues

**Pour les nouveaux contributeurs**, voici les meilleures issues pour commencer :

### 🟢 Faciles (Bonnes pour débuter)

1. **Documentation** : Améliorer les guides, corriger les fautes
2. **Tests** : Ajouter des tests pour modules non couverts
3. **Exemples** : Créer des exemples d'usage simples
4. **Traduction** : Aider à traduire la documentation

**Étiquettes GitHub** : `good first issue`, `help wanted`

### 🟡 Moyennes (Niveau intermédiaire)

1. **Modules BBIA** : Améliorer les modules existants
2. **Dashboard** : Améliorer l'interface utilisateur
3. **Tests** : Améliorer la couverture de tests
4. **Performance** : Optimiser les hot-paths

### 🔴 Avancées (Niveau expert)

1. **Architecture** : Améliorer la structure modulaire
2. **Robot** : Améliorer l'intégration Reachy Mini
3. **IA** : Améliorer les modules d'intelligence
4. **Sécurité** : Renforcer la validation et sécurité

---

## 🔧 Standards de Code

### Formatage

```bash
# Formater avec Black
black src/ tests/

# Vérifier avec Ruff
ruff check src/ tests/

# Corriger automatiquement
ruff check --fix src/ tests/
```

**Règles** :
- ✅ Lignes ≤ 88 caractères (Black)
- ✅ Import triés (isort)
- ✅ Pas de warnings Ruff critiques

### Type Hints

```python
def fonction_exemple(param1: str, param2: int) -> bool:
    """Description courte."""
    return True
```

**Recommandation** : Ajouter type hints pour nouvelles fonctions

### Docstrings

```python
def fonction_exemple(param1: str, param2: int) -> bool:
    """Description courte de la fonction.
    
    Description détaillée si nécessaire.
    
    Args:
        param1: Description du paramètre 1
        param2: Description du paramètre 2
    
    Returns:
        Description de la valeur de retour
    
    Raises:
        ValueError: Si param2 est négatif
    
    Example:
        >>> fonction_exemple("test", 42)
        True
    """
    pass
```

---

## 🧪 Tests

### Écrire des Tests

**Structure** :

```python
import pytest
from bbia_sim.robot_factory import RobotFactory

class TestMaFonctionnalite:
    """Tests pour ma fonctionnalité."""
    
    @pytest.mark.unit
    @pytest.mark.fast
    def test_cas_simple(self):
        """Test cas simple."""
        # Arrange
        robot = RobotFactory.create_backend("mujoco")
        
        # Act
        result = robot.connect()
        
        # Assert
        assert result is True
```

**Markers pytest** :
- `@pytest.mark.unit` - Tests unitaires rapides
- `@pytest.mark.fast` - Tests rapides (< 1s)
- `@pytest.mark.e2e` - Tests end-to-end
- `@pytest.mark.robot` - Tests nécessitent robot physique

### Lancer les Tests

```bash
# Tous les tests
pytest tests/ -v

# Tests spécifiques
pytest tests/test_bbia_voice.py -v

# Avec couverture
pytest tests/ --cov=src --cov-report=html
```

---

## 📝 Workflow de Contribution

### 1. Développer

- ✅ Code formaté avec `black`
- ✅ Linting OK (`ruff check`)
- ✅ Type hints si possible
- ✅ Docstrings pour nouvelles fonctions

### 2. Tester

```bash
# Vérifier avant PR
pytest tests/ -v
ruff check src/ tests/
black --check src/ tests/
```

### 3. Commit

**Structure de commit** :

```text
type(scope): description courte

Description plus longue si nécessaire

Fixes #123
```

**Types** :
- `feat` - Nouvelle fonctionnalité
- `fix` - Correction de bug
- `docs` - Documentation
- `test` - Tests
- `refactor` - Refactoring
- `chore` - Maintenance

### 4. Push et Pull Request

```bash
# Pousser votre branche
git push origin feature/votre-contribution

# Ouvrir une PR sur GitHub
# Utiliser le template PR
```

**Checklist PR** :
- [ ] Code formaté avec `black`
- [ ] Linting OK (`ruff check`)
- [ ] Tests passent (`pytest`)
- [ ] Documentation mise à jour si nécessaire
- [ ] Pas de secrets/credentials dans le code
- [ ] Compatible Python 3.11+

---

## 🎯 Zones de Contribution

### Pour les Nouveaux Contributeurs 🟢

**Bons premiers pas** :
- 📝 Documentation : Améliorer les guides
- 🧪 Tests : Ajouter des tests
- 🌐 Traduction : Aider à traduire
- 🎨 Exemples : Créer des exemples simples

### Pour les Experts 🔴

**Domaines avancés** :
- 🏗️ Architecture : Améliorer la structure
- ⚡ Performance : Optimiser
- 🤖 Robot : Améliorer l'intégration
- 🔒 Sécurité : Renforcer la validation

---

## 🤝 Code de Conduite

### Communication

- ✅ Soyez respectueux et bienveillant
- ✅ Acceptez les critiques constructives
- ✅ Expliquez vos décisions techniques
- ✅ Aidez les nouveaux contributeurs

### Code Review

**En tant que contributeur** :
- ✅ Répondez rapidement aux commentaires
- ✅ Apprenez des suggestions
- ✅ Soyez ouvert au feedback

**En tant que reviewer** :
- ✅ Soyez constructif et bienveillant
- ✅ Expliquez vos suggestions
- ✅ Félicitez les bonnes contributions

---

## 📚 Ressources

### Documentation

- **Guide de Démarrage** : `docs/guides/GUIDE_DEMARRAGE.md`
- **Guide Avancé** : `docs/guides/GUIDE_AVANCE.md`
- **Architecture** : `docs/development/architecture/ARCHITECTURE_DETAILED.md`
- **Conformité SDK** : `docs/quality/compliance/CONFORMITE_REACHY_MINI_COMPLETE.md`

### Outils

- **CI/CD** : `.github/workflows/ci.yml`
- **Tests** : `pytest tests/ -v`
- **Linting** : `ruff check src/`
- **Formatage** : `black src/`

### Communauté

- **Issues** : [GitHub Issues](https://github.com/arkalia-luna-system/bbia-sim/issues)
- **Releases** : [GitHub Releases](https://github.com/arkalia-luna-system/bbia-sim/releases)

---

## ❓ Questions ?

Si vous avez des questions :

1. 📖 Consultez la documentation dans `docs/`
2. 🔍 Recherchez dans les issues existantes
3. 💬 Créez une issue avec le label `question`
4. 🤝 Contactez les maintainers

**Contributions bienvenues**

---

**Dernière mise à jour** : 8 Décembre 2025

