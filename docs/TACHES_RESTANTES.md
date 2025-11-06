# 📋 Tâches Restantes - Documentation

**Statut** : ✅ **Réorganisation 95% terminée**

---

## ✅ TERMINÉ

### Réorganisation
- ✅ Audit complet de la documentation
- ✅ Réduction fichiers racine : 32 → 6 fichiers (-81%)
- ✅ Consolidation dossiers : 30 → ~15 dossiers
- ✅ Déplacement de ~20 fichiers
- ✅ Fusion de 9 dossiers
- ✅ Création README dans dossiers principaux
- ✅ Création GUIDE_NAVIGATION.md
- ✅ Mise à jour liens dans README.md et INDEX_FINAL.md

---

## ✅ TERMINÉ (Dernière Session)

### Corrections Finales
1. ✅ **Liens corrigés** :
   - `INDEX_FINAL.md` - Lien ASSISTANT_IA_GUIDE.md → development/assistant-ia-guide.md
   - `getting-started/contributing.md` - Liens GUIDE_SYSTEME_TESTS.md → development/testing.md

2. ✅ **Nettoyage** :
   - `development/hardware/` - Dossier vide supprimé
   - `archives/` → `archive/` - Consolidation terminée
   - Références archives mises à jour

---

## ✅ TERMINÉ (Session Actuelle - Décembre 2025)

### Fusion Coverage et Vérification Complète
1. ✅ **Fichiers coverage fusionnés** :
   - `AUDIT_COVERAGE_IMPORTS.md` fusionné dans `AUDIT_COVERAGE_IMPORTS_FINAL.md`
   - Déplacé vers `quality/audits/AUDIT_COVERAGE_IMPORTS.md`
   - Mis à jour `INDEX_AUDITS_CONSOLIDES.md`

2. ✅ **Vérification complète des liens** :
   - Script `scripts/verify_docs_links.py` créé
   - **0 erreur**, 244 liens valides, 3 avertissements mineurs
   - Tous les liens cassés corrigés dans :
     - `INDEX_FINAL.md`, `getting-started/troubleshooting.md`, `unity/UNITY_BBIA_GUIDE.md`
     - `reference/STATUT_PROJET.md`, `reference/PROJECT_HISTORY.md`, `development/README.md`

3. ✅ **README créés dans sous-dossiers** :
   - `quality/audits/README.md`
   - `quality/validation/README.md`
   - `archive/nettoyage-2025/README.md`
   - `archive/tasks/README.md`
   - `analyses/README.md`

4. ✅ **Fusion INDEX_FINAL.md + README.md → index.md** :
   - `docs/index.md` créé avec contenu consolidé
   - Structure organisée par catégorie et par rôle
   - Navigation complète et professionnelle

---

## 🔍 À VÉRIFIER (Optionnel - Non Prioritaire)

### Petites Améliorations
1. 🟢 **Dossier quality/checklists/** - Vide pour l'instant, peut être utilisé plus tard (OK)
2. ✅ **Vérifier tous les liens** - **TERMINÉ** (script créé, 0 erreur)

### Améliorations Futures (Non Prioritaire)
3. ✅ **Terminé** :
   - ✅ Fusionner INDEX_FINAL.md avec README.md → `index.md` créé
   - ✅ Créer README.md dans sous-dossiers principaux (13 README créés)
   - ✅ Vérifier tous les liens dans tous les fichiers (0 erreur)

---

## 📊 STATUT ACTUEL

### Structure
- ✅ **6 fichiers** à la racine (objectif atteint)
- ✅ **~15 dossiers** principaux (structure claire)
- ✅ **Navigation intuitive** (guide créé)
- ✅ **Liens principaux** mis à jour

### Qualité
- ✅ **Professionnelle** : Structure standardisée
- ✅ **Accessible** : Navigation intuitive
- ✅ **Maintenable** : Facile à étendre
- ✅ **Liens** : **100% vérifiés** - 0 erreur, 244 liens valides

---

## 🎯 PRIORITÉS (Toutes Optionnelles)

### ✅ TERMINÉ
1. ✅ Corriger référence ASSISTANT_IA_GUIDE.md dans INDEX_FINAL.md
2. ✅ Vérifier et corriger contributing.md
3. ✅ Supprimer dossiers vides (hardware/)
4. ✅ Consolider archives/ et archive/

### ✅ TERMINÉ
5. ✅ Vérifier tous les liens dans tous les fichiers - **TERMINÉ** (0 erreur)
6. ✅ Créer README dans sous-dossiers restants - **TERMINÉ** (13 README créés)
7. ✅ Fusionner INDEX_FINAL.md avec README.md - **TERMINÉ** (index.md créé)

---

**Recommandation** : Les tâches restantes sont **optionnelles** et n'impactent pas l'utilisation de la documentation. La réorganisation est **fonctionnelle et professionnelle**.

---

## 🎯 TÂCHES RESTANTES - Mise à jour

### Issues GitHub à Gérer

1. **Issue #2** (`bbia_memory.py`) : **À FERMER** ✅
   - Tests déjà complets (198 lignes dans `test_bbia_memory.py`)
   - Toutes les fonctionnalités testées
   - Action : Fermer l'issue avec message dans `docs/verification/MESSAGES_ISSUES_GITHUB.md`

2. **Issue #1** (`bbia_audio.py`) : **À MODIFIER** ⚠️
   - `detecter_son()` déjà bien testé
   - Manque : Tests pour `_capture_audio_chunk()` spécifiquement
   - Action : Ajouter précision dans l'issue (voir `docs/verification/MESSAGES_ISSUES_GITHUB.md`)

3. **Issue #3** (`bbia_emotions.py`) : **À MODIFIER** ⚠️
   - Historique et intensités limites déjà testés
   - Manque : Tests transitions complexes (séquences rapides, stress tests)
   - Action : Ajouter exemples dans l'issue (voir `docs/verification/MESSAGES_ISSUES_GITHUB.md`)

4. **Issue #4** (Bbox structure) : **PRÊTE** ✅
   - Code normalisé (center_x/center_y ajoutés aux visages MediaPipe)
   - Test à implémenter par @yummyash
   - Action : Aucune, l'issue est claire

5. **Issue #5** (Commandes vocales) : **À MODIFIER** ⚠️
   - Tests de base existent
   - Manque : Tests ponctuation, multi-mots complexes, variations linguistiques
   - Action : Ajouter exemples concrets dans l'issue (voir `docs/verification/MESSAGES_ISSUES_GITHUB.md`)

### Code - Normalisation Récente

✅ **TERMINÉ** : Normalisation structure bbox
- Ajout de `center_x` et `center_y` aux visages MediaPipe
- Tous les bbox ont maintenant la même structure
- Fichier : `src/bbia_sim/bbia_vision.py` (lignes 689-690, 890-891)

### Qualité Code

✅ **TERMINÉ** : Passage outils qualité
- Black : 123 fichiers formatés
- Ruff : Tous les checks passent
- MyPy : 1 erreur corrigée
- Bandit : Warnings mineurs (non bloquants)

### Documentation

✅ **TERMINÉ** : Rapports de vérification créés
- `docs/verification/RAPPORT_VERIFICATION_PROJET.md` : Vérification complète projet
- `docs/verification/ANALYSE_ISSUES_GITHUB.md` : Analyse des issues GitHub
- `docs/verification/MESSAGES_ISSUES_GITHUB.md` : Messages prêts pour GitHub

---

## 📋 ACTIONS IMMÉDIATES

### Priorité Haute
1. **Gérer les issues GitHub** :
   - Fermer Issue #2 (tests déjà faits)
   - Modifier Issues #1, #3, #5 (ajouter précisions)
   - Utiliser messages dans `docs/verification/MESSAGES_ISSUES_GITHUB.md`

### Priorité Moyenne
2. **✅ Coverage réel vérifié** (Janvier 2025) :
   - **Coverage total** : **64.98%** ✅
   - **24 modules < 70%** identifiés
   - **Priorité Haute** (9 modules < 30%) :
     - `daemon/app/__main__.py` : 0.0%
     - `daemon/app/routers/sanity.py` : 0.0%
     - `model_optimizer.py` : 0.0%
     - `__main__.py` : 19.0%
     - `bbia_awake.py` : 20.0%
     - `bbia_integration.py` : 20.1%
     - `face_recognition.py` : 20.1%
     - `backends/reachy_backend.py` : 30.8%
     - `dashboard.py` : 32.2%
   - **Priorité Moyenne** (4 modules 30-50%) :
     - `bbia_emotion_recognition.py` : 40.1%
     - `bbia_voice_advanced.py` : 42.5%
     - `daemon/app/routers/daemon.py` : 43.4%
     - `backends/mujoco_backend.py` : 45.3%
   - **Priorité Basse** (11 modules 50-70%) : Voir détails dans `docs/CE_QUI_RESTE_REEL.md`

### Priorité Basse
3. **TODO test optionnel** :
   - `tests/test_watchdog_monitoring.py` ligne 227
   - Test watchdog timeout robot déconnecté
   - Estimation : ~30 min (optionnel)

---

**Dernière mise à jour** : Janvier 2025

