# 🤖 Guide pour Assistants IA - BBIA-SIM

> **⚠️ ATTENTION : Ce guide est destiné aux ASSISTANTS IA, pas aux humains débutants !**  
> **👤 Si vous êtes un débutant humain, commencez par :** [Guide Débutant](../guides/GUIDE_DEBUTANT.md)

> **Date de mise à jour :** Oct / Nov. 2025  
> **Version :** 1.0

## 📋 Vue d'ensemble

Ce guide est destiné aux **assistants IA** (Claude, GPT, Cursor AI, etc.) pour comprendre rapidement l'état du projet, les scripts disponibles, les conventions, et les tâches à poursuivre.

> **💡 Pour les humains** : Si vous cherchez à apprendre à utiliser BBIA-SIM, consultez plutôt :
> - 🟢 **[Guide Débutant](../guides/GUIDE_DEBUTANT.md)** - Pour commencer
> - 🟡 **[Guide Avancé](../guides/GUIDE_AVANCE.md)** - Pour développeurs
> - 📚 **[README Documentation](../README.md)** - Navigation complète

---

## 🎯 État actuel du projet

### ✅ Statut général
- **Version :** 1.3.2
- **Documentation :** **131 fichiers MD** dans `docs/` (après nettoyage)
- **Tests :** **1362 tests sélectionnés** (1418 collectés, 56 deselected)
- **Coverage :** **68.86%** global (excellent)
- **Conformité SDK :** 100% validée

### 📊 Qualité documentation
- ✅ 0 lien brisé
- ✅ 16+ schémas Mermaid colorés
- ✅ 4756+ corrections auto appliquées
- ⚠️ 26 erreurs restantes (majoritairement faux positifs)
- ⚠️ 1001 avertissements (recommandations style)

---

## 🛠️ Scripts essentiels

### 1. Vérification documentation
```bash
# Script principal : scripts/verify_docs_complete.py

# Vérification complète
python scripts/verify_docs_complete.py

# Vérification spécifique
python scripts/verify_docs_complete.py --links-only      # Liens uniquement
python scripts/verify_docs_complete.py --mermaid-only    # Schémas Mermaid uniquement
python scripts/verify_docs_complete.py --spell-only      # Orthographe uniquement
python scripts/verify_docs_complete.py --code-consistency # Cohérence avec code

# Mode auto-correction
python scripts/verify_docs_complete.py --fix

# Mode complet (tous fichiers)
python scripts/verify_docs_complete.py --full-scan

# Vérifier liens externes
python scripts/verify_docs_complete.py --check-external-links
```

**Fonctionnalités :**
- ✅ Vérification liens (internes/externes)
- ✅ Validation syntaxe Mermaid
- ✅ Détection erreurs formatage (espaces, listes, tables)
- ✅ Vérification dates standardisées
- ✅ Orthographe basique (français)
- ✅ Cohérence avec code (fichiers, classes mentionnées)
- ✅ Auto-correction (mode `--fix`)

### 2. Diagnostic environnement
```bash
# Script : scripts/bbia_doctor.py

python scripts/bbia_doctor.py
```

**Fonctionnalités :**
- Vérification dépendances installées
- Vérification modèles IA disponibles
- Vérification variables d'environnement
- Vérification configuration projet

### 3. Tests
```bash
# Tests unitaires
pytest tests/ -v

# Tests spécifiques
pytest tests/test_bbia_*.py -v
pytest tests/test_reachy_mini_*.py -v

# Tests avec coverage
pytest tests/ --cov=src/bbia_sim --cov-report=html
```

---

## 📝 Conventions documentation

### Dates standardisées
- **Date de mise à jour :** Toujours utiliser `Oct / Nov. 2025`
- **Date de création :** Date du premier commit Git (immutable)

### Formatage Markdown
- ✅ Espaces : pas d'espaces doubles, pas d'espaces finaux
- ✅ Listes : toujours espace après `-` ou `*`
- ✅ Tables : toujours séparateur `---` après header
- ✅ Titres : toujours espace après `#`

### Schémas Mermaid
- ✅ Toujours ajouter des couleurs/styles pour meilleure visibilité
- ✅ Types supportés : `graph`, `flowchart`, `sequenceDiagram`, `gantt`, `pie`, `mindmap`, etc.

### Liens
- ✅ Liens internes : chemins relatifs depuis fichier MD
- ✅ Liens vers qualité : `../quality/compliance/`
- ✅ Liens vers guides : `../guides/` ou `../development/`

---

## 🔄 Tâches à poursuivre

### 1. Documentation (priorité haute)
- [ ] Corriger les 26 erreurs restantes
  - Tables sans séparateur (quelques cas)
  - Blocs code "non fermés" (faux positifs à améliorer dans script)
  - Listes sans espace (dans contextes valides)
- [ ] Réduire avertissements (1001 → ~500)
  - Espaces doubles dans contextes spéciaux
  - Orthographe (vérification manuelle recommandée)
- [ ] Ajouter schémas Mermaid colorés aux MD restants

### 2. Tests (priorité moyenne)
- [ ] Augmenter coverage `bbia_audio.py` (30-40% → 70%+)
- [ ] Tests `bbia_memory.py` (complet)
- [ ] Améliorer tests `bbia_emotions.py`

### 3. Performance (priorité moyenne)
- [ ] Benchmark latence E2E audio
- [ ] Optimiser chargement modèles (caching, lazy loading)

### 4. Features (priorité basse)
- [ ] UI avancée avec presets/sliders
- [ ] Script diagnostic (`bbia doctor`) amélioré
- [ ] Vidéos/GIF pour onboarding
- [ ] FAQ complète

---

## 📁 Structure documentation

```
docs/
├── guides/              # Guides utilisateurs (débutant, avancé)
├── development/   # Guides techniques (migration, testing, etc.)
├── development/architecture/       # Architecture détaillée
├── quality/audits/              # Audits, comparaisons, bilans
├── quality/compliance/         # Tests conformité SDK
├── deployment/                 # Documentation CI/CD
├── observabilite/      # Logs, métriques, health checks
├── development/api/                # Documentation API REST/WebSocket
├── dashboard/          # Roadmap dashboard/UX
├── performance/        # Optimisations performance
└── reference/         # Index, historique projet
```

---

## 🔍 Points d'attention

### Scripts à ne PAS modifier sans précaution
1. `scripts/verify_docs_complete.py`
   - Script optimisé et testé
   - Modifications nécessitent tests approfondis
   - Mode intelligent avec détection contextuelle

2. `scripts/bbia_doctor.py`
   - Diagnostic environnement critique
   - Modifications doivent maintenir compatibilité

### Fichiers critiques
1. `README.md` - Point d'entrée principal
2. `PROJECTS.md` - Portfolio projets
3. `docs/INDEX_FINAL.md` - Index documentation
4. `docs/getting-started/troubleshooting.md` - Questions fréquentes

### Conventions code
- **Python :** 3.11+
- **Formatage :** Black, Ruff, MyPy, Bandit
- **Tests :** pytest avec markers (`@pytest.mark.unit`, `@pytest.mark.fast`)
- **Documentation :** Markdown avec schémas Mermaid colorés

---

## 🚀 Workflow recommandé

### Pour corriger documentation
1. Lancer `python scripts/verify_docs_complete.py`
2. Identifier erreurs réelles (pas faux positifs)
3. Corriger manuellement ou avec `--fix`
4. Vérifier avec `--full-scan`
5. Commit avec message descriptif

### Pour ajouter fonctionnalité
1. Vérifier état actuel (`bbia doctor`)
2. Créer tests avant implémentation
3. Implémenter fonctionnalité
4. Lancer tests (`pytest`)
5. Mettre à jour documentation
6. Vérifier documentation (`verify_docs_complete.py`)

### Pour améliorer script
1. Comprendre logique actuelle
2. Tester avec `--full-scan` avant modifications
3. Modifier progressivement
4. Vérifier que faux positifs ne sont pas créés
5. Tester sur échantillon représentatif

---

## 📚 Ressources importantes

### Documentation clé
- `docs/INDEX_FINAL.md` - Index complet documentation
- `docs/guides/GUIDE_DEBUTANT.md` - Guide débutant
- `docs/guides/GUIDE_AVANCE.md` - Guide avancé
- `docs/development/architecture/ARCHITECTURE_OVERVIEW.md` - Vue d'ensemble architecture
- `docs/quality/audits/RESUME_ETAT_ACTUEL_BBIA.md` - État actuel détaillé

### Scripts principaux
- `scripts/verify_docs_complete.py` - Vérification documentation
- `scripts/bbia_doctor.py` - Diagnostic environnement

### Tests importants
- `tests/test_reachy_mini_*.py` - Tests conformité SDK
- `tests/test_bbia_*.py` - Tests modules BBIA
- `tests/test_robot_api.py` - Tests API unifiée

---

## ⚠️ Erreurs communes à éviter

1. **Ne pas corriger** les "erreurs" qui sont des faux positifs
   - Listes dans tableaux (valides)
   - Blocs code Python valides détectés comme non fermés
   - Formatage Markdown spécial

2. **Ne pas modifier** dates de création (immutables)
3. **Toujours vérifier** avec `verify_docs_complete.py` après modifications
4. **Toujours tester** scripts modifiés sur échantillon avant commit

---

## 📞 Support

- **Documentation :** `docs/` (complet et à jour)
- **Scripts :** `scripts/` (optimisés et testés)
- **Tests :** `tests/` (**1362 tests sélectionnés**, **68.86%** coverage)
- **FAQ :** `docs/getting-started/troubleshooting.md` (questions fréquentes)

---

## 🎯 Objectifs futurs

### Court terme (1-2 semaines)
- ✅ Documentation vérifiée et corrigée (TERMINÉ)
- ⏳ Réduire 26 erreurs → 0
- ⏳ Réduire 1001 avertissements → ~500

### Moyen terme (1 mois)
- ⏳ Coverage tests 70%+
- ⏳ Optimisations performance
- ⏳ UI avancée

### Long terme (3+ mois)
- ⏳ Vidéos/GIF onboarding
- ⏳ FAQ complète
- ⏳ Promotion projet

---

**Dernière mise à jour :** Oct / Nov. 2025  
**Version guide :** 1.0  
**Prochaine révision :** Oct / Nov. 2025

