# 🔍 GUIDE AUDIT WINDSURF OPTIMISÉ - BBIA-SIM

**Date :** 21 Novembre 2025  
**Score global :** 8.07/10

## 🎯 PRINCIPE

**Version optimisée pour Windsurf :**
- ✅ **Questions sémantiques précises** - Utilise la recherche intelligente
- ✅ **Instructions concises** - Pas de détails redondants
- ✅ **Analyse approfondie** - Pousse l'analyse au-delà de la surface
- ✅ **Format structuré** - Résultats clairs et actionnables

---

## 📋 PHASES D'AUDIT

| Phase | Fichier | Focus | Durée |
|-------|---------|-------|-------|
| **1** | [PHASE 1 : Architecture](WINDSURF_AUDIT_PHASE1.md) | Imports, dépendances | 15 min |
| **2** | [PHASE 2 : SDK](WINDSURF_AUDIT_PHASE2.md) | Compatibilité Reachy Mini | 20 min |
| **2B** | [PHASE 2B : Micro-détails](WINDSURF_AUDIT_PHASE2B.md) | Exceptions, timeouts | 20 min |
| **3** | [PHASE 3 : Qualité Code](WINDSURF_AUDIT_PHASE3.md) | Type hints, fonctions | 25 min |
| **4** | [PHASE 4 : Tests](WINDSURF_AUDIT_PHASE4.md) | Couverture, qualité | 20 min |
| **5** | [PHASE 5 : MuJoCo](WINDSURF_AUDIT_PHASE5.md) | Modèles, performance | 20 min |
| **6** | [PHASE 6 : Vision/IA](WINDSURF_AUDIT_PHASE6.md) | Modèles HF, performance | 25 min |
| **7** | [PHASE 7 : Communication](WINDSURF_AUDIT_PHASE7.md) | Zenoh, REST, WebSocket | 20 min |
| **8** | [PHASE 8 : Performance](WINDSURF_AUDIT_PHASE8.md) | RAM, CPU, optimisations | 25 min |
| **9** | [PHASE 9 : Documentation](WINDSURF_AUDIT_PHASE9.md) | Docstrings, TODO | 15 min |
| **10** | [PHASE 10 : CI/CD](WINDSURF_AUDIT_PHASE10.md) | Sécurité, dépendances | 20 min |
| **11** | [PHASE 11 : Synthèse](WINDSURF_AUDIT_PHASE11.md) | Tableau de bord, roadmap | 30 min |

---

## 🚀 MÉTHODE WINDSURF OPTIMISÉE

### 1. **Questions Sémantiques** (au lieu de grep)
❌ **Évite :** `grep -r "ReachyMini" src/`
✅ **Utilise :** "Where is ReachyMini class instantiated and how?"

### 2. **Analyse Profonde** (pas juste surface)
❌ **Évite :** Compter les lignes
✅ **Utilise :** "What are the potential race conditions in the watchdog thread?"

### 3. **Vérifications Croisées**
❌ **Évite :** Analyser un fichier isolément
✅ **Utilise :** "Compare goto_target implementation between mujoco_backend and reachy_mini_backend"

### 4. **Contexte SDK Officiel**
Toujours comparer avec : https://github.com/pollen-robotics/reachy_mini

---

## 📊 FORMAT DE RÉSULTAT STANDARD

Pour chaque action :

```markdown
### Action X.Y : [Titre]

**Question sémantique :** "..."

**Résultat :**
| Élément | État | Détails |
|---------|------|---------|
| ... | ✅/⚠️/❌ | ... |

**Analyse approfondie :**
- Point 1 avec justification
- Point 2 avec impact

**Score :** X/10
```

---

## ⚠️ RÈGLES

- **NE MODIFIE AUCUN FICHIER** (analyse uniquement)
- **Utilise recherche sémantique** (pas grep)
- **Lis fichiers complets** (pas juste extraits)
- **Compare avec SDK officiel** (toujours)
- **Pousse l'analyse** (pas juste surface)

---

## 🎯 EXÉCUTION

1. Ouvre le fichier de phase
2. Copie le contenu dans Windsurf
3. Exécute les actions dans l'ordre
4. Note les résultats au format standard
5. Passe à la phase suivante

---

## 📈 SCORES ACTUELS (Dernière vérification)

| Phase | Score | Statut |
|-------|-------|--------|
| Phase 1 | 9.2/10 | ✅ Excellent |
| Phase 2 | 9.3/10 | ✅ Excellent |
| Phase 2B | 8.3/10 | ✅ Bon |
| Phase 3 | 7.5/10 | ✅ Bon |
| Phase 4 | 7.0/10 | ✅ Bon |
| Phase 5 | 6.0/10 | ⚠️ Améliorable |
| Phase 6 | 6.5/10 | ⚠️ Améliorable |
| Phase 7 | 8.0/10 | ✅ Bon |
| Phase 8 | 8.25/10 | ✅ Bon |
| Phase 9 | 9.7/10 | ✅ Excellent |
| Phase 10 | 8.0/10 | ✅ Bon |

**Score global : 8.07/10**

---

## 🔗 RESSOURCES

- **Repo officiel :** https://github.com/pollen-robotics/reachy_mini
- **Vérification :** [VERIFICATION_REPO_OFFICIEL.md](VERIFICATION_REPO_OFFICIEL.md)
- **Index complet :** [WINDSURF_AUDIT_INDEX.md](WINDSURF_AUDIT_INDEX.md)

