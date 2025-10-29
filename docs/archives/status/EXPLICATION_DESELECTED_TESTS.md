# 🔍 EXPLICATION - POURQUOI 862 TESTS SONT DÉSÉLECTIONNÉS

**Question** : Pourquoi `40 passed, 1 skipped, 862 deselected` ?

---

## 📊 EXPLICATION

### Configuration Pytest Actuelle

Quand on lance :
```bash
pytest -m "unit and fast"
```

On **filtre** pour n'exécuter que les tests qui ont **LES DEUX** markers :
- `@pytest.mark.unit` **ET**
- `@pytest.mark.fast`

### Statistiques

- **Total tests dans le projet** : ~903 tests
- **Tests avec `@pytest.mark.unit` ET `@pytest.mark.fast`** : 41 tests
- **Tests désélectionnés** : 903 - 41 = **862 tests**

---

## 🎯 POURQUOI C'EST NORMAL

### Tests sans markers
Beaucoup de tests dans le projet n'ont **pas encore** les markers `@pytest.mark.unit` et `@pytest.mark.fast`.

Exemple : `tests/test_bbia_audio.py` utilise `unittest.TestCase` sans markers pytest.

### Tests avec d'autres markers
- Tests `@pytest.mark.e2e` → désélectionnés (pas `unit and fast`)
- Tests `@pytest.mark.slow` → désélectionnés (pas `fast`)
- Tests `@pytest.mark.integration` → désélectionnés (pas `unit`)

---

## ✅ C'EST CORRECT

C'est **voulu** ! On veut seulement lancer les tests **rapides et unitaires** pour vérification rapide.

Les autres tests peuvent être lancés avec :
- `pytest -m "not slow"` → Tous sauf les lents
- `pytest -m "unit"` → Tous les tests unitaires
- `pytest` → Tous les tests

---

## 🔧 SI TU VEUX TOUS LES TESTS

```bash
# Lancer TOUS les tests (pas de filtre)
pytest

# Lancer tous sauf les lents
pytest -m "not slow"

# Lancer seulement les unitaires (sans fast)
pytest -m "unit"
```

---

## 📝 SOLUTION SI TU VEUX AJOUTER LES MARKERS

Pour ajouter les markers aux tests existants :

1. **Tests unittest.TestCase** : Convertir vers pytest ou ajouter markers
2. **Tests sans markers** : Ajouter `@pytest.mark.unit` et `@pytest.mark.fast`

**Mais ce n'est pas nécessaire** - c'est normal qu'ils soient désélectionnés si on utilise `-m "unit and fast"`.

---

**Conclusion** : ✅ **C'EST NORMAL** - 862 désélectionnés car ils n'ont pas les markers `unit` ET `fast`.

