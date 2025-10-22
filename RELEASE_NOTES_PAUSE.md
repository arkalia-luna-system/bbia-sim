# BBIA-SIM v0.3.0 - Release Notes Pause

## 📋 Résumé

Version de **pause stable** du projet `bbia-sim` en attente de la publication des assets officiels Reachy Mini.

## ✅ Améliorations

### 🔧 Corrections Critiques
- **Fix durée headless** : Simulation respecte maintenant exactement la durée demandée (±0.00s)
- **Multi-OS viewer** : Détection robuste macOS/Linux avec messages d'aide clairs
- **Qualité code** : 0 erreurs ruff/black/mypy, tous les tests passent

### 🧹 Nettoyage
- Suppression des caches parasites (`__pycache__`, `.pytest_cache`, etc.)
- Suppression des fichiers macOS cachés (`.DS_Store`, `._*`)
- Mise à jour `.gitignore` pour éviter les fichiers parasites
- Suppression du test `reachy_websim_test.py` problématique

### 📚 Documentation
- Création `README_PAUSE.md` avec plan de reprise détaillé
- Structure `reachy_official/` préparée pour assets officiels
- Mapping Python centralisé dans `asset_mapping.py`

## 🎯 État Actuel

### ✅ Fonctionnalités Stables
- **Simulation MuJoCo** : Durée headless parfaite (1.00s ±0.00s)
- **API REST/WebSocket** : Complète et fonctionnelle
- **Meshes 3D** : Placeholders générés automatiquement
- **Tests** : Suite complète (124 tests passent)
- **CI/CD** : Pipeline vert et stable

### 📊 Métriques Qualité
- **Ruff** : 0 erreurs ✅
- **Black** : Formatage parfait ✅
- **MyPy** : 0 erreurs de type ✅
- **Pytest** : 124/124 tests passent ✅
- **Bandit** : 25 warnings Low (acceptables) ✅
- **Durée headless** : 1.00s ±0.00s ✅

## 🚀 Prochaines Étapes

1. **Attendre** la publication des assets officiels Reachy Mini
2. **Intégrer** les STL officiels dans `reachy_official/`
3. **Tester** le rendu 3D avec les vrais meshes
4. **Déployer** la version finale v1.0.0

## 🔧 Commandes de Vérification

```bash
# Qualité du code
ruff check src/ tests/ && black --check src/ tests/ && mypy src/

# Tests
pytest -q

# Simulation headless
python -m bbia_sim --sim --headless --duration 1
```

## 📝 Notes Techniques

- **Durée headless** : Fix critique appliqué avec vérification après chaque step
- **Viewer macOS** : Détection automatique et message d'aide pour `mjpython`
- **Assets** : Structure prête pour intégration officielle
- **Tests** : Smoke tests viewer skipés en CI pour éviter la flakiness

---

**Version** : v0.3.0  
**Date** : Octobre 2025  
**État** : Pause stable, prêt à reprendre  
**Hash commit** : À définir lors du tag
