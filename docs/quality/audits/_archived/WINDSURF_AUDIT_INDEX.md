# 🔍 INDEX DES PROMPTS D'AUDIT WINDSURF - BBIA-SIM v2.0

**Date :** 21 Novembre 2025  
**Score global :** 8.07/10

## 🎯 GUIDE D'UTILISATION OPTIMISÉ POUR WINDSURF

**Windsurf utilise la recherche sémantique et l'analyse de code. Ces prompts sont optimisés pour :**
- ✅ Recherche sémantique efficace (questions précises)
- ✅ Analyse ligne par ligne (instructions claires)
- ✅ Détection exhaustive (patterns spécifiques)
- ✅ Format visuel structuré (tableaux, listes, emojis)

---

## 📋 PROMPTS PAR PHASE (EXÉCUTION SÉQUENTIELLE)

**⚠️ IMPORTANT : Exécute UNE phase à la fois, dans l'ordre**

| Phase | Fichier | Actions | Durée estimée | Priorité |
|-------|---------|---------|---------------|----------|
| **1** | [PHASE 1 : Architecture](WINDSURF_AUDIT_PHASE1.md) | 3 actions | 10-15 min | 🔴 Critique |
| **2** | [PHASE 2 : Compatibilité SDK](WINDSURF_AUDIT_PHASE2.md) | 4 actions | 10-15 min | 🔴 Critique |
| **2B** | [PHASE 2B : Micro-détails](WINDSURF_AUDIT_PHASE2B.md) | 4 actions | 15-20 min | 🟠 Haute |
| **3** | [PHASE 3 : Qualité Code](WINDSURF_AUDIT_PHASE3.md) | 4 actions | 15-20 min | 🟠 Haute |
| **4** | [PHASE 4 : Tests](WINDSURF_AUDIT_PHASE4.md) | 3 actions | 10-15 min | 🟡 Moyenne |
| **5** | [PHASE 5 : Simulation MuJoCo](WINDSURF_AUDIT_PHASE5.md) | 3 actions | 10-15 min | 🟡 Moyenne |
| **6** | [PHASE 6 : Vision/IA](WINDSURF_AUDIT_PHASE6.md) | 3 actions | 15-20 min | 🟡 Moyenne |
| **7** | [PHASE 7 : Communication](WINDSURF_AUDIT_PHASE7.md) | 3 actions | 10-15 min | 🟠 Haute |
| **8** | [PHASE 8 : Performance](WINDSURF_AUDIT_PHASE8.md) | 3 actions | 15-20 min | 🟠 Haute |
| **9** | [PHASE 9 : Documentation](WINDSURF_AUDIT_PHASE9.md) | 3 actions | 10-15 min | 🟢 Faible |
| **10** | [PHASE 10 : CI/CD/Sécurité](WINDSURF_AUDIT_PHASE10.md) | 3 actions | 10-15 min | 🟠 Haute |
| **11** | [PHASE 11 : Synthèse](WINDSURF_AUDIT_PHASE11.md) | 4 actions | 20-30 min | 🔴 Critique |

---

## 🚀 MÉTHODE D'EXÉCUTION OPTIMALE

### Étape 1 : Préparation
1. Ouvre Windsurf dans le projet `/Volumes/T7/bbia-reachy-sim/`
2. Assure-toi que tous les fichiers sont indexés
3. Prépare un fichier de notes pour les résultats

### Étape 2 : Exécution par phase
1. **Ouvre** le fichier de phase (ex: `WINDSURF_AUDIT_PHASE1.md`)
2. **Copie** TOUT le contenu dans Windsurf
3. **Exécute** les actions dans l'ordre (1.1, 1.2, 1.3...)
4. **Note** les résultats dans un tableau
5. **Passe** à la phase suivante

### Étape 3 : Synthèse finale
1. Après les 10 phases, exécute la Phase 11 (Synthèse)
2. Compile tous les résultats
3. Génère le rapport final

---

## ⚡ OPTIMISATIONS WINDSURF INTÉGRÉES

Chaque prompt inclut :
- ✅ **Questions sémantiques précises** : "Where is X used?" au lieu de grep
- ✅ **Instructions ligne par ligne** : "Read file X, lines Y-Z"
- ✅ **Patterns spécifiques** : Recherche exacte de patterns
- ✅ **Format structuré** : Tableaux, listes, emojis pour clarté
- ✅ **Vérifications croisées** : Compare avec code existant

---

## 🚀 UTILISATION

1. **Commence par la Phase 1** (WINDSURF_AUDIT_PHASE1.md)
2. **Copie le contenu du fichier dans Windsurf**
3. **Exécute les actions dans l'ordre**
4. **Focus sur questions sémantiques** (pas de grep)
5. **Pousse l'analyse plus loin** (impact, causes, solutions)
6. **Rapporte les résultats**
7. **Passe à la phase suivante**

**Guide méthodologique :** [WINDSURF_AUDIT_GUIDE_OPTIMIZED.md](WINDSURF_AUDIT_GUIDE_OPTIMIZED.md)

---

## ⚠️ RÈGLES ABSOLUES (À RESPECTER IMPÉRATIVEMENT)

### 🚫 INTERDICTIONS FORMELLES
- ❌ **NE MODIFIE AUCUN FICHIER** (analyse statique uniquement)
- ❌ **NE CRÉE AUCUN FICHIER** (utilise les prompts existants)
- ❌ **NE SUPPRIME AUCUN FICHIER**
- ❌ **NE PROPOSE AUCUN CORRECTIF** (seulement identifier)

### ✅ TON RÔLE : AUDITEUR EXPERT
- 🔍 **Analyse en profondeur** : Lire les fichiers ligne par ligne
- 📊 **Identifie les problèmes** : Liste exhaustive avec fichiers/lignes
- 📝 **Documente les découvertes** : Format structuré (tableaux)
- 🎯 **Sois exhaustif mais concis** : Tous les problèmes, format clair

---

## 📊 SYNTHÈSE FINALE (PHASE 11)

Après avoir complété les phases 1-10, exécute la **Phase 11** qui génère :

1. **📊 Tableau de bord exécutif**
   - Scores par phase (X/10)
   - Statut (✅/⚠️/❌)
   - Impact (Critique/Haute/Moyenne/Faible)
   - Priorité (Immédiate/S1/S2/S3)

2. **🔥 Top 20 problèmes critiques**
   - Rang, problème, phase, impact, urgence
   - Correction proposée, effort estimé
   - Fichiers concernés (chemins exacts)

3. **🔧 Top 30 micro-problèmes**
   - Catégorie (Performance/Sécurité/Qualité/Tests)
   - Complexité (Facile/Moyen/Difficile)
   - Temps estimé par problème

4. **🗺️ Roadmap de correction**
   - Organisation par trimestre (S1, S2, S3, S4)
   - Priorités par sprint (2 semaines)
   - Ressources nécessaires
   - Métriques de suivi

---

## 🎨 AMÉLIORATIONS VISUELLES v2.0

- ✅ **Tableaux structurés** : Colonnes claires, alignement
- ✅ **Emojis sémantiques** : 🔴 Critique, 🟠 Haute, 🟡 Moyenne, 🟢 Faible
- ✅ **Code blocks** : Exemples de code avec lignes
- ✅ **Sections claires** : Objectif, Actions, Résultats, Score
- ✅ **Checklists** : Cases à cocher pour suivi

