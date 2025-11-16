# 🔍 ANALYSE COVERAGE - PROBLÈME D'IMPORTS

**Date** : Janvier 2025  
**Problème identifié** : Les tests existent mais les imports sont mal faits

---

## ❌ PROBLÈME IDENTIFIÉ

Les imports sont faits **dans des try/except à l'intérieur des fonctions** au lieu d'être au **niveau module**, ce qui empêche coverage de détecter les modules.

### Exemple du problème :

```python
# ❌ MAUVAIS (actuel dans test_bbia_integration.py)
def test_something(self):
    try:
        from bbia_sim.bbia_integration import BBIAIntegration
        # ...
    except ImportError:
        pytest.skip(...)
```

```python
# ✅ BON (à faire)
try:
    from bbia_sim.bbia_integration import BBIAIntegration
except ImportError:
    BBIAIntegration = None  # type: ignore

def test_something(self):
    if BBIAIntegration is None:
        pytest.skip("Module non disponible")
    # ...
```

---

## 📊 RÉSULTATS COVERAGE ACTUELS

### 1. **bbia_integration.py** : 20.1% ❌

**Tests existants** : `test_bbia_integration.py` (6 tests)

**Problème** :
- ✅ Tests existent et passent
- ❌ Imports dans try/except à l'intérieur des fonctions
- ❌ Coverage ne détecte pas le module

**Solution** : Déplacer imports au niveau module

---

### 2. **face_recognition.py** : 15.83% ⚠️

**Tests existants** : `test_face_recognition.py` (21 tests)

**Problème** :
- ✅ Import au niveau module (ligne 11) - **BON**
- ⚠️ Mais seulement 22 lignes sur 139 couvertes (15.83%)
- ⚠️ Tests ne couvrent pas toutes les fonctions

**Solution** : Ajouter tests pour fonctions non couvertes

---

### 3. **dashboard.py** : 31.29% ⚠️

**Tests existants** : `test_dashboard.py` (24 tests)

**Problème** :
- ⚠️ Imports conditionnels dans les fonctions
- ⚠️ Coverage ne détecte pas le module dans le rapport

**Solution** : Déplacer imports au niveau module

---

### 4. **reachy_backend.py** : 30.8% ❓

**Tests existants** : Pas de test dédié trouvé

**Problème** :
- ❓ Pas de test spécifique pour `reachy_backend.py`
- ❓ Peut-être testé indirectement via d'autres tests

**Solution** : Vérifier si testé indirectement, sinon créer test dédié

---

## ✅ ACTIONS À FAIRE

### Priorité 1 : Corriger imports (30 min)

1. **test_bbia_integration.py** :
   - Déplacer `from bbia_sim.bbia_integration import BBIAIntegration` au niveau module
   - Utiliser try/except au niveau module avec variable globale

2. **test_dashboard.py** :
   - Déplacer imports conditionnels au niveau module
   - Utiliser variable globale pour disponibilité

### Priorité 2 : Ajouter tests manquants (1-2h)

3. **face_recognition.py** :
   - Identifier fonctions non couvertes
   - Ajouter tests pour ces fonctions

4. **reachy_backend.py** :
   - Vérifier si testé indirectement
   - Créer test dédié si nécessaire

---

## 📝 EXEMPLE DE CORRECTION

### Avant (❌) :

```python
class TestBBIAIntegration:
    def test_something(self):
        try:
            from bbia_sim.bbia_integration import BBIAIntegration
            # test code
        except ImportError:
            pytest.skip("Module non disponible")
```

### Après (✅) :

```python
# Import au niveau module
try:
    from bbia_sim.bbia_integration import BBIAIntegration
except ImportError:
    BBIAIntegration = None  # type: ignore

class TestBBIAIntegration:
    @pytest.mark.skipif(BBIAIntegration is None, reason="Module non disponible")
    def test_something(self):
        # test code - module déjà importé
        integration = BBIAIntegration()
        # ...
```

---

## 🎯 RÉSULTAT ATTENDU

Après corrections :
- **bbia_integration.py** : 20% → 70%+
- **face_recognition.py** : 15% → 70%+
- **dashboard.py** : 31% → 70%+
- **reachy_backend.py** : 31% → 70%+

---

**Dernière mise à jour** : Janvier 2025

