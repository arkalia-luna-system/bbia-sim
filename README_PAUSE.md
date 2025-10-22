# BBIA-SIM - État de Pause

## 📋 État Actuel (Octobre 2025)

Le projet `bbia-sim` est actuellement en **état de pause stable** en attente de la publication des assets officiels Reachy Mini.

### ✅ Ce qui fonctionne parfaitement

- **Simulation MuJoCo** : Durée headless respectée à ±0.05s
- **API REST/WebSocket** : Complète et fonctionnelle
- **Meshes 3D** : Placeholders générés automatiquement
- **Tests** : Suite complète (ruff/black/mypy/pytest/bandit)
- **CI/CD** : Pipeline vert et stable

### 🎯 Ce qui est prêt pour la reprise

- **Structure assets** : `src/bbia_sim/sim/assets/reachy_official/` préparée
- **Mapping Python** : `asset_mapping.py` centralisé
- **Documentation** : `OFFICIAL_ASSETS.md` avec instructions
- **Tests** : Smoke tests pour viewer 3D

## 🚀 Plan de Reprise (Quand les assets officiels seront publiés)

### 1. Intégration des assets officiels

```bash
# Copier les STL officiels
cp /path/to/official/stl/* src/bbia_sim/sim/assets/reachy_official/

# Mettre à jour le mapping
vim src/bbia_sim/sim/assets/reachy_official/asset_mapping.py
```

### 2. Tests de validation

```bash
# Vérifier la qualité
ruff check src/ tests/
black --check src/ tests/
mypy src/
pytest -q

# Tester la simulation
python -m bbia_sim --sim --headless --duration 1
```

### 3. Déploiement

```bash
# Commit et PR
git add .
git commit -m "feat(sim): integrate official Reachy assets"
git push origin feature/official-assets
```

## 🔧 Commandes de Vérification Rapide

```bash
# 1. Qualité du code
ruff check src/ tests/ && black --check src/ tests/ && mypy src/

# 2. Tests
pytest -q

# 3. Simulation headless
python -m bbia_sim --sim --headless --duration 1
```

## 📊 Métriques de Qualité

- **Durée headless** : 1.00s ±0.05s ✅
- **Ruff** : 0 erreurs ✅
- **Black** : Formatage OK ✅
- **MyPy** : 0 erreurs de type ✅
- **Pytest** : Tous les tests passent ✅
- **Bandit** : 25 warnings Low (acceptables) ✅

## 🎯 Prochaines Étapes

1. **Attendre** la publication des assets officiels Reachy Mini
2. **Intégrer** les STL officiels dans `reachy_official/`
3. **Tester** le rendu 3D avec les vrais meshes
4. **Déployer** la version finale

---

**Dernière mise à jour** : Octobre 2025  
**État** : Pause stable, prêt à reprendre  
**Responsable** : Équipe BBIA
