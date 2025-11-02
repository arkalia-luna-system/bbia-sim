# 📋 Liste RÉELLE des Tâches Pas Encore Faites

**Date** : Oct / Nov. 2025
**Source** : Audit exhaustif MD vs Code

---

## ✅ CE QUI EST DÉJÀ FAIT (mais mentionné comme "à faire" dans certains MD)

Les fichiers suivants ont été corrigés pour refléter la réalité :

1. ✅ **uptime** : Calcul implémenté dans `ecosystem.py`
2. ✅ **SmolVLM2** : Intégré dans `bbia_huggingface.py`
3. ✅ **VAD** : Implémenté dans `voice_whisper.py`
4. ✅ **NER extraction** : Méthodes `_extract_angle()` et `_extract_intensity()` implémentées
5. ✅ **Whisper streaming** : Implémenté avec VAD

---

## ⏳ VRAIES TÂCHES À FAIRE

### 🔴 Priorité Haute

#### 1. ✅ Tracking WebSocket Actif - **TERMINÉ**
- **Fichier** : `src/bbia_sim/daemon/app/routers/ecosystem.py`
- **Statut** : ✅ **COMPLÉTÉ** - Fonction `get_active_connections()` implémentée avec `get_ws_manager()`
- **Tests** : `test_ecosystem_priority_high.py` créé et tous passent
- **Statut** : ✅ **100% TERMINÉ**

#### 2. ✅ Logique Démarrage Démo - **TERMINÉ**
- **Fichier** : `src/bbia_sim/daemon/app/routers/ecosystem.py` (lignes 434-549)
- **Statut** : ✅ **COMPLÉTÉ** - Endpoint `/demo/start` complètement implémenté
- **Fonctionnalités** : Modes (simulation, robot_real, mixed), durée, émotion, arrêt automatique
- **Tests** : Tests créés et tous passent
- **Statut** : ✅ **100% TERMINÉ**

---

### 🟡 Priorité Moyenne

#### 3. Améliorer Coverage Tests
- **Modules** :
  - `vision_yolo.py` : 27.74% coverage (99 lignes non couvertes)
  - `voice_whisper.py` : 33.33% coverage (76 lignes non couvertes)
  - ⚠️ `dashboard_advanced.py` : **0.00% coverage** ⚠️ (**47 tests**, **1156 lignes**) - **À CORRIGER** (tests ne couvrent pas)
  - `daemon/bridge.py` : 0% coverage (283 lignes)
- **Estimation** : 8-12 heures

#### 4. Vérifier Liens Internes Cassés
- **Fichiers à vérifier** :
  - `docs/status.md` - Nombreux liens
  - `docs/INDEX_FINAL.md` - Liens vers archives
- **Action** : Script automatique pour vérifier tous les liens markdown
- **Estimation** : 1 heure

#### 5. Consolider Documents Redondants
- **Groupes identifiés** :
  - Résumés d'audit multiples (docs/audit/)
  - Fichiers de corrections similaires
- **Action** : Identifier plus récents et archiver autres
- **Estimation** : 2-3 heures

---

### 🟢 Priorité Basse

#### 6. Démos Vidéo
- **Source** : `PROJECTS.md` ligne 260
- **Action** : Créer démonstrations vidéo pour projets hardware
- **Estimation** : Variable

#### 7. Mise à Jour Statuts Projets
- **Source** : `PROJECTS.md` ligne 258
- **Action** : Mettre à jour statuts régulièrement
- **Estimation** : Maintenance continue

---

### 🔵 Hardware (En Attente Robot Physique)

#### 8. TODOs Robot Réel
- **Fichier** : `src/bbia_sim/backends/reachy_backend.py`
- **TODOs** :
  - Connexion réelle Reachy (ligne ~52)
  - Déconnexion réelle (ligne ~71)
  - Envoi commandes réelles (ligne ~104, ~165)
  - Synchronisation état (ligne ~127)
- **Statut** : ⏳ En attente réception robot physique
- **Estimation** : 3-4 heures (quand robot disponible)

---

## 📊 Résumé par Priorité

| Priorité | Nombre | Estimation Totale |
|----------|--------|-------------------|
| 🔴 Haute | 2 | 2-4 heures |
| 🟡 Moyenne | 3 | 11-16 heures |
| 🟢 Basse | 2 | Variable |
| 🔵 Hardware | 1 | 3-4 heures (attente) |

**Total (sans hardware)** : ~13-20 heures de travail

---

**Dernière mise à jour** : Oct / Nov. 2025
**Prochaine révision** : Oct / Nov. 2025

