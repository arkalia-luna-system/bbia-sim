# 📋 Tâches Restantes - Janvier 2025

**Date** : Janvier 2025  
**Statut** : Projet en excellent état, quelques améliorations mineures restantes

---

## 🔴 Priorité Haute (À Corriger Immédiatement)

### 1. Erreur de Linting - Import Non Trié
- **Fichier** : `src/bbia_sim/bbia_behavior.py` ligne 1074
- **Erreur** : `I001` - Import block is un-sorted or un-formatted
- **Action** : Trier les imports avec `ruff --fix`
- **Temps estimé** : 2 minutes

### 2. Erreurs de Collection de Tests
- **Fichiers** :
  - `tests/test_websocket_integration.py`
  - `tests/ws/test_telemetry_rate.py`
- **Problème** : 50 erreurs lors de la collection de tests
- **Action** : Vérifier les imports et la syntaxe de ces fichiers
- **Temps estimé** : 15-30 minutes

### 3. Vérifier Syntaxe vision_yolo.py
- **Fichier** : `src/bbia_sim/vision_yolo.py` ligne 529
- **Problème** : Erreur mentionnée lors du nettoyage cache
- **Action** : Vérifier la syntaxe Python
- **Temps estimé** : 5 minutes

---

## 🟡 Priorité Moyenne (Améliorations)

### 4. Vérifier Incohérences de Coverage
- **Problème** : Discrepance entre coverage pytest et README
  - `vision_yolo.py` : 15.83% (pytest) vs 99.45% (README)
  - `voice_whisper.py` : 11.51% (pytest) vs 92.52% (README)
- **Action** : Vérifier la configuration de coverage et corriger la documentation
- **Temps estimé** : 30 minutes

### 5. Améliorer Coverage bbia_voice_advanced.py
- **Couverture actuelle** : 15.61%
- **Action** : Créer des tests conditionnels pour Coqui TTS
- **Temps estimé** : 1-2 heures

### 6. Nettoyer TODO/FIXME
- **Fichiers avec TODO** :
  - `src/bbia_sim/daemon/app/dashboard/templates/sections/quick_actions.html` (2 TODO)
  - `src/bbia_sim/daemon/app/dashboard/static/js/robot_3d.js` (1 TODO)
  - `src/bbia_sim/daemon/app/dashboard/static/js/waveform.js` (1 TODO)
- **Action** : Implémenter ou documenter les TODO
- **Temps estimé** : 1-2 heures

---

## 🟢 Priorité Basse (Optionnel)

### 7. Fichiers Modifiés Non Commités
- **Fichiers** :
  - `coverage.xml` (généré automatiquement - normal)
  - `src/bbia_sim/__main__.py`
  - `src/bbia_sim/bbia_integration.py`
  - `tests/test_code_quality.py`
- **Action** : Vérifier les changements et commiter si nécessaire
- **Temps estimé** : 10 minutes

---

## 📊 Résumé

### Tâches Urgentes (Priorité Haute)
- ✅ 3 tâches à corriger immédiatement
- ⏱️ Temps total estimé : 20-40 minutes

### Tâches d'Amélioration (Priorité Moyenne)
- ✅ 3 tâches d'amélioration
- ⏱️ Temps total estimé : 2-4 heures

### Tâches Optionnelles (Priorité Basse)
- ✅ 1 tâche optionnelle
- ⏱️ Temps total estimé : 10 minutes

**Total temps estimé** : 2.5-5 heures pour toutes les tâches

---

## ✅ Actions Recommandées

1. **Immédiatement** :
   - Corriger l'erreur de linting dans `bbia_behavior.py`
   - Vérifier et corriger les erreurs de tests

2. **Cette semaine** :
   - Vérifier les incohérences de coverage
   - Nettoyer les TODO/FIXME

3. **Optionnel** :
   - Améliorer coverage `bbia_voice_advanced.py`
   - Vérifier les fichiers modifiés non commités

---

**Dernière mise à jour** : Janvier 2025

