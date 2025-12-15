# 🤝 Guide de Contribution - BBIA-SIM

**Dernière mise à jour : 15 Décembre 2025  
**Version** : 1.4.0

> **Voir aussi** : [`docs/reference/INDEX_THEMATIQUE.md`](../reference/INDEX_THEMATIQUE.md) et [`docs/reference/project-status.md`](../reference/project-status.md)

Bienvenue dans la communauté BBIA-SIM ! Ce guide vous aidera à contribuer efficacement au projet.

> **💡 Nouveau contributeur ?**  
> Commencez par consulter les [Good First Issues](../getting-started/contributing.md) pour trouver des tâches adaptées aux nouveaux contributeurs.

---

## 🎯 Ce Guide vs Good First Issues

- **[Good First Issues](../getting-started/contributing.md)** : Liste d'issues suggérées pour nouveaux contributeurs
- **Ce guide** : Processus complet de contribution, templates, standards

---

## 🚀 Démarrage Rapide

### Pour les nouveaux contributeurs

1. **Fork le projet** sur GitHub
2. **Clone votre fork** : `git clone https://github.com/votre-username/bbia-sim.git`
3. **Créez une branche** : `git checkout -b feature/ma-contribution`
4. **Faites vos modifications**
5. **Testez** : `pytest tests/ -v`
6. **Poussez** : `git push origin feature/ma-contribution`
7. **Ouvrez une Pull Request** avec le template approprié

---

## 📋 Templates GitHub

Nous avons des templates pour faciliter la contribution :

### 🐛 Signaler un Bug

**Template** : `.github/ISSUE_TEMPLATE/bug_report.md`

Utilisez ce template pour :

- Décrire clairement le problème
- Fournir des étapes pour reproduire
- Indiquer l'environnement (OS, Python, version BBIA)
- Joindre logs et captures si possible

**Lien direct** : [Créer un bug report](https://github.com/arkalia-luna-system/bbia-sim/issues/new?template=bug_report.md)

### ✨ Proposer une Fonctionnalité

**Template** : `.github/ISSUE_TEMPLATE/feature_request.md`

Utilisez ce template pour :

- Décrire la fonctionnalité souhaitée
- Expliquer le problème/besoin
- Identifier les modules concernés
- Estimer la priorité

**Lien direct** : [Créer une feature request](https://github.com/arkalia-luna-system/bbia-sim/issues/new?template=feature_request.md)

### ❓ Poser une Question

**Template** : `.github/ISSUE_TEMPLATE/question.md` (à créer si besoin)

Pour les questions générales :

- Utilisez le label `question` sur une issue standard
- Référencez la documentation pertinente
- Soyez précis sur votre environnement

### 🤝 Pull Request

**Template** : `.github/PULL_REQUEST_TEMPLATE.md`

Avant d'ouvrir une PR :

- ✅ Lisez le template complet
- ✅ Cochez tous les éléments pertinents
- ✅ Assurez-vous que les tests passent
- ✅ Mettez à jour la documentation si nécessaire

**Lien direct** : [Créer une Pull Request](https://github.com/arkalia-luna-system/bbia-sim/compare)

### 🎯 First Run / Onboarding

**Template** : `.github/ISSUE_TEMPLATE/first_run.yml`

Utilisez ce template après votre première installation :

- Signalez les problèmes d'onboarding
- Partagez votre environnement
- Suggérez des améliorations

---

## 🔧 Workflow de Contribution

### 1. Préparer l'Environnement

```bash
# Installer en mode développement
pip install -e .[dev,test]

# Vérifier l'installation
pytest tests/ -v --tb=short

# Formater le code
black src/ tests/
ruff check --fix src/ tests/

```

### 2. Créer une Branche

```bash
# Depuis main
git checkout main
git pull origin main

# Créer votre branche
git checkout -b feature/nom-de-votre-contribution

```

**Convention de nommage** :

- `feature/` - Nouvelle fonctionnalité
- `fix/` - Correction de bug
- `docs/` - Documentation
- `refactor/` - Refactoring
- `test/` - Tests uniquement

### 3. Développer

**Standards de code** :

- ✅ Formatage : `black` (88 colonnes)
- ✅ Linting : `ruff`
- ✅ Type hints : `mypy` (si possible)
- ✅ Tests : Au moins un test pour nouvelle fonctionnalité

**Structure de commit** :

```text
type(scope): description courte

Description plus longue si nécessaire

Fixes #123

```

Types : `feat`, `fix`, `docs`, `test`, `refactor`, `chore`

### 4. Tester

```bash
# Tests unitaires
pytest tests/ -v

# Tests avec couverture
pytest tests/ --cov=src --cov-report=html

# Tests spécifiques
pytest tests/test_bbia_voice.py -v

```

### 5. Vérifier avant PR

**Checklist** :

- [ ] Code formaté avec `black`
- [ ] Linting OK (`ruff check`)
- [ ] Tests passent (`pytest`)
- [ ] Documentation mise à jour si nécessaire
- [ ] Pas de secrets/credentials dans le code
- [ ] Compatible Python 3.11+

### 6. Ouvrir la Pull Request

1. **Pousser votre branche** : `git push origin feature/votre-contribution`
2. **Ouvrir une PR** sur GitHub avec le template
3. **Remplir le template** complètement
4. **Attendre les reviews** et répondre aux commentaires

---

## 📚 Zones de Contribution

### Pour les Nouveaux Contributeurs 🟢

**Bons premiers pas** :

- 📝 Documentation : Améliorer les guides, corriger les fautes
- 🧪 Tests : Ajouter des tests pour modules non couverts
- 🌐 Traduction : Aider à traduire la documentation
- 🎨 Exemples : Créer des exemples d'usage simples

**Étiquettes GitHub** : `good first issue`, `help wanted`

### Pour les Experts 🔴

**Domaines avancés** :

- 🏗️ Architecture : Améliorer la structure modulaire
- ⚡ Performance : Optimiser les hot-paths
- 🤖 Robot : Améliorer l'intégration Reachy Mini
- 🔒 Sécurité : Renforcer la validation et sécurité

---

## 🎯 Roadmap Publique

La roadmap est documentée dans :

- `docs/reference/project-status.md` → Section "Axes futurs"
- `docs/reference/PROJECT_HISTORY.md` → Historique des releases
- Issues GitHub avec label `roadmap`

**Principes** :

- 3–5 items par release
- Statuts clairs (planifié, en cours, fait)
- Priorités explicites

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

## 📖 Documentation

### Standards de Documentation

**Markdown** :

- ✅ Utilisez des titres hiérarchiques (`##`, `###`)
- ✅ Ajoutez des exemples de code commentés
- ✅ Référencez les fichiers sources (numéros de lignes si pertinent)
- ✅ Utilisez des diagrammes Mermaid pour l'architecture

**Docstrings Python** :

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

### Où Documenter

- **Guides utilisateur** : `docs/guides/`
- **Guides techniques** : `docs/development/`
- **Architecture** : `docs/development/architecture/`
- **API** : Docstrings Python → génération Sphinx (futur)

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
- `@pytest.mark.e2e` - Tests end-to-end (peuvent être longs)
- `@pytest.mark.robot` - Tests nécessitent robot physique
- `@pytest.mark.skip` - Tests temporairement désactivés

### Couverture

**Objectif** : Maintenir/améliorer la couverture

```bash
# Voir la couverture
pytest tests/ --cov=src --cov-report=html
open htmlcov/index.html

```

---

## 🔗 Ressources

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
- **Discussions** : (À créer si besoin)
- **Releases** : [GitHub Releases](https://github.com/arkalia-luna-system/bbia-sim/releases)

---

## ❓ Questions ?

Si vous avez des questions :

1. 📖 Consultez la documentation dans `docs/`
2. 🔍 Recherchez dans les issues existantes
3. 💬 Créez une issue avec le label `question`
4. 🤝 Contactez les maintainers

**Merci de contribuer à BBIA-SIM !** 🚀
