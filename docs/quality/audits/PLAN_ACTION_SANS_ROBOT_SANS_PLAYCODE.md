# 🎯 Plan d'Action - Sans Robot Physique et Sans Play Code

**Date** : 24 Novembre 2025  
**Contexte** : En attente robot physique + réponse Play Console  
**Objectif** : Maximiser la valeur du projet pendant l'attente

---

## ✅ CE QUI VIENT D'ÊTRE FAIT (24 Nov 2025)

### 1. ✅ Implémentation TODO Quick Actions

- **Fichier** : `src/bbia_sim/daemon/app/dashboard/templates/sections/quick_actions.html`
- **Action** : Implémentation complète des TODO

  - ✅ Envoi émotions via `/api/motion/emotion` (POST JSON)
  - ✅ Envoi actions/comportements via `/api/ecosystem/behaviors/execute` (POST query)
  - ✅ Feedback visuel (vert = succès, rouge = erreur)
  - ✅ Gestion d'erreurs complète

- **Statut** : ✅ **TERMINÉ**

### 2. ✅ Guide Installation PWA

- **Fichier** : `docs/dashboard/GUIDE_INSTALLATION_PWA.md`
- **Contenu** :

  - ✅ Instructions Android (Chrome)
  - ✅ Instructions iOS (Safari)
  - ✅ Instructions Desktop (Chrome/Edge)
  - ✅ Vérification installation
  - ✅ Dépannage
  - ✅ Checklist complète

- **Statut** : ✅ **TERMINÉ**

### 3. ✅ Amélioration Graphiques Temps Réel

- **Fichier** : `src/bbia_sim/daemon/app/dashboard/templates/sections/telemetry_charts.html`
- **Actions** :

  - ✅ Légendes améliorées (Chart.js)
  - ✅ Zoom/pan interactif (chartjs-plugin-zoom)
  - ✅ Tooltips avec valeurs précises
  - ✅ Export données CSV/JSON
  - ✅ Support dark mode

- **Statut** : ✅ **TERMINÉ**

### 4. ✅ Dark Mode

- **Fichiers** :
  - `src/bbia_sim/daemon/app/dashboard/static/style.css` (variables CSS)
  - `src/bbia_sim/daemon/app/dashboard/templates/base.html` (script)
  - `src/bbia_sim/daemon/app/dashboard/static/js/dark_mode.js` (nouveau)
- **Actions** :

  - ✅ Variables CSS pour thème clair/sombre
  - ✅ Toggle dark mode (bouton flottant)
  - ✅ Persistance préférence (localStorage)
  - ✅ Support prefers-color-scheme
  - ✅ Adaptation couleurs graphiques

- **Statut** : ✅ **TERMINÉ**

---

## 🟡 CE QUI RESTE À FAIRE (Sans Robot, Sans Play Code)

### 🟡 PRIORITÉ MOYENNE (Optionnel)

#### 1. **Tests WebSocket Reconnexion** ⏱️ **1-2h**

**Pourquoi** : Robustesse maximale

**Actions** :

- [ ] Test reconnexion automatique après déconnexion
- [ ] Test gestion perte réseau temporaire
- [ ] Test queue messages pendant déconnexion
- [ ] Test heartbeat/ping pour détecter déconnexions

**Fichiers à créer** :

- `tests/test_websocket_reconnection.py` (nouveau)

**Impact** : ✅ **Robustesse améliorée**

---

#### 2. **Tests Dashboard Connexion Lente** ⏱️ **1-2h**

**Pourquoi** : Tester en conditions réelles (réseau lent)

**Actions** :

- [ ] Simuler latence réseau (Chrome DevTools)
- [ ] Tester chargement progressif
- [ ] Tester timeout/retry
- [ ] Tester affichage "Chargement..." pendant latence

**Fichiers à créer** :

- `tests/test_dashboard_slow_connection.py` (nouveau)

**Impact** : ✅ **Robustesse améliorée**

---

#### 3. **Tests PWA Cache Corrompu** ⏱️ **1h**

**Pourquoi** : Gérer cas edge Service Worker

**Actions** :

- [ ] Test cache corrompu (supprimer entrées manuellement)
- [ ] Test version Service Worker incompatible
- [ ] Test récupération après erreur cache
- [ ] Test nettoyage cache automatique

**Fichiers à créer** :

- `tests/test_pwa_cache_corruption.py` (nouveau)

**Impact** : ✅ **Robustesse améliorée**

---

### 🟢 PRIORITÉ BASSE (Plus Tard)

#### 4. **Nettoyer Autres TODO dans Codebase** ⏱️ **1-2h**

**Pourquoi** : Code propre, documentation claire

**Actions** :

- [ ] Documenter TODO `robot_3d.js` (charger modèle STL réel)
- [ ] Documenter TODO `waveform.js` (connecter sources audio WebSocket)
- [ ] Vérifier autres TODO dans codebase
- [ ] Créer issues GitHub pour TODO futurs (optionnel)

**Fichiers à modifier** :

- `src/bbia_sim/daemon/app/dashboard/static/js/robot_3d.js` (commentaire TODO)
- `src/bbia_sim/daemon/app/dashboard/static/js/waveform.js` (commentaire TODO)

**Impact** : ✅ **Code plus propre**

---

#### 5. **Optimisations Performance Optionnelles** ⏱️ **4-6h**

**Pourquoi** : Performance déjà excellente, optimisations marginales

**Actions** :

- [ ] Quantification modèles 8-bit (gain RAM ~2-4GB)
- [ ] Optimisation streaming audio (compression Opus)
- [ ] Cache LRU pour réponses LLM fréquentes
- [ ] Batch processing pour analyses sentiment

**Impact** : ✅ **Performance marginalement améliorée**

---

## 📊 RÉSUMÉ PAR PRIORITÉ

| Priorité | Tâches | Temps Estimé | Impact |
|----------|--------|--------------|--------|
| ✅ **TERMINÉ** | Graphiques + Dark Mode | 4-6h | ✅ UX améliorée |
| 🟡 **MOYENNE** | Tests robustesse | 3-5h | ✅ Robustesse |
| 🟢 **BASSE** | Nettoyage + Optimisations | 5-9h | ✅ Code propre |

---

## 🎯 PLAN D'ACTION RECOMMANDÉ

### 📅 **Jour 1 (Aujourd'hui - 24 Nov)** ✅ TERMINÉ

#### Matin (3-4h)

1. ✅ **Améliorer Graphiques Temps Réel** (2-3h)

   - Légendes, zoom, tooltips
   - Export données

#### Après-midi (2-3h)

2. ✅ **Ajouter Dark Mode** (2-3h)

   - Variables CSS, toggle, persistance
   - Test accessibilité

**Total Jour 1** : 4-6h ✅ **TERMINÉ**

---

### 📅 **Jour 2 (25 Nov - Optionnel)**

#### Matin (2-3h)

1. **Tests WebSocket Reconnexion** (1-2h)
2. **Tests Dashboard Connexion Lente** (1-2h)

#### Après-midi (1-2h)

3. **Tests PWA Cache Corrompu** (1h)
4. **Nettoyer TODO** (1h)

**Total Jour 2** : 3-5h (optionnel)

---

### 📅 **Jour 3 (26 Nov - Très Optionnel)**

#### Si temps disponible

5. **Optimisations Performance** (4-6h)

**Total Jour 3** : 4-6h (très optionnel)

---

## 💡 RECOMMANDATION FINALE

### 🎯 **Focus Recommandé**

**Priorité #1** : ✅ **TERMINÉ** - Graphiques + Dark Mode ⭐⭐⭐

- Impact UX élevé
- Visible immédiatement
- Pas de dépendances externes

**Priorité #2** : **Tests Robustesse** ⭐⭐

- Important pour production
- Peut attendre si temps limité

**Priorité #3** : **Nettoyage + Optimisations** ⭐

- Optionnel, peut attendre
- Améliorations marginales

---

## ✅ CHECKLIST FINALE

### ✅ Terminé (24 Nov 2025)

#### UX Dashboard

- [x] Améliorer graphiques temps réel (légendes, zoom, tooltips, export)
- [x] Ajouter dark mode (toggle, persistance, accessibilité)

### À Faire (Optionnel)

#### Tests Robustesse

- [ ] Tests WebSocket reconnexion
- [ ] Tests dashboard connexion lente
- [ ] Tests PWA cache corrompu

#### Code Quality

- [ ] Nettoyer TODO dans codebase
- [ ] Documenter TODO futurs

#### Optimisations (Optionnel)

- [ ] Quantification modèles 8-bit
- [ ] Cache LRU LLM
- [ ] Batch processing sentiment

---

## 📚 RESSOURCES

### Fichiers de Référence

- `docs/dashboard/ROADMAP_DASHBOARD.md` - Roadmap dashboard
- `docs/dashboard/GUIDE_INSTALLATION_PWA.md` - Guide PWA
- `docs/development/integration.md` - Guide intégration

### Documentation Technique

- [Chart.js - Plugins](https://www.chartjs.org/docs/latest/developers/plugins.html)
- [MDN - CSS Variables](https://developer.mozilla.org/en-US/docs/Web/CSS/Using_CSS_custom_properties)
- [Web.dev - Dark Mode](https://web.dev/prefers-color-scheme/)

---

**Dernière mise à jour** : 24 Novembre 2025  
**Statut** : ✅ **Priorités hautes terminées** - Graphiques + Dark Mode implémentés  
**Prochaines étapes** : Tests robustesse (optionnel)
