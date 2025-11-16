# 🔍 INDEX DES PROMPTS D'AUDIT WINDSURF - BBIA-SIM

## 📋 PROMPTS PAR PHASE

**Utilise ces prompts UN PAR UN dans l'ordre :**

1. **[PHASE 1 : Architecture et Imports](WINDSURF_AUDIT_PHASE1.md)** - 3 actions
   - Analyser les imports
   - Dépendances circulaires
   - Fichiers orphelins macOS

2. **[PHASE 2 : Compatibilité SDK](WINDSURF_AUDIT_PHASE2.md)** - 4 actions
   - Utilisation ReachyMini
   - Utilisation create_head_pose
   - Versions dépendances
   - Arguments CLI

3. **[PHASE 2B : Micro-détails critiques](WINDSURF_AUDIT_PHASE2B.md)** - 4 actions
   - Exceptions silencieuses
   - Timeouts manquants
   - Context managers manquants
   - Validations manquantes

4. **[PHASE 3 : Qualité Code](WINDSURF_AUDIT_PHASE3.md)** - 4 actions
   - Type hints manquants
   - Fonctions trop longues
   - Usage de Any
   - Imports inutilisés

5. **[PHASE 4 : Tests](WINDSURF_AUDIT_PHASE4.md)** - 3 actions
   - Couverture par module
   - Qualité des tests
   - Tests manquants

6. **[PHASE 5 : Simulation MuJoCo](WINDSURF_AUDIT_PHASE5.md)** - 3 actions
   - Comparaison modèles XML
   - Performance simulation
   - Cohérence sim vs réel

7. **[PHASE 6 : Vision/IA](WINDSURF_AUDIT_PHASE6.md)** - 3 actions
   - Modèles Hugging Face
   - Performance vision
   - Gestion mémoire

8. **[PHASE 7 : Communication](WINDSURF_AUDIT_PHASE7.md)** - 3 actions
   - Bridge Zenoh
   - Endpoints REST
   - Fuites WebSocket

9. **[PHASE 8 : Performance](WINDSURF_AUDIT_PHASE8.md)** - 3 actions
   - deque vs list
   - Boucles bloquantes
   - @lru_cache manquants

10. **[PHASE 9 : Documentation](WINDSURF_AUDIT_PHASE9.md)** - 3 actions
    - Docstrings manquantes
    - TODO/FIXME
    - Documentation technique

11. **[PHASE 10 : CI/CD/Sécurité](WINDSURF_AUDIT_PHASE10.md)** - 3 actions
    - Entry points CLI
    - Secrets hardcodés
    - Dépendances obsolètes

---

## 🚀 UTILISATION

1. **Commence par la Phase 1**
2. **Copie le contenu du fichier dans Windsurf**
3. **Exécute les actions dans l'ordre**
4. **Rapporte les résultats**
5. **Passe à la phase suivante**

---

## ⚠️ RÈGLES COMMUNES

- **NE MODIFIE AUCUN FICHIER**
- **Analyse statique uniquement**
- **Sois exhaustif mais concis**

---

## 📊 SYNTHÈSE FINALE

Après avoir complété les 11 phases, crée un rapport de synthèse avec :

1. **Tableau de bord exécutif** (scores par phase)
2. **Top 20 problèmes critiques**
3. **Top 30 micro-problèmes**
4. **Roadmap de correction**

