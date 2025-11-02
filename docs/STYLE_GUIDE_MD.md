# 📐 Guide de Style Markdown - BBIA-SIM

**Version :** 1.0  
**Date :** Oct 25 / Nov 25  
**Objectif :** Documentation moderne, professionnelle, impactante

---

## 🎯 Principes Fondamentaux

### Clarté & Professionnalisme

- **Ton neutre et factuel** : Privilégier la précision aux superlatifs
- **Vouvoiement cohérent** : Utiliser "vous" dans toute la documentation
- **Preuves > Promesses** : Chiffres mesurés plutôt que promesses vagues

### Modernité & Impact Visuel

- **Structure claire** : Hiérarchie de titres cohérente
- **Emojis judicieux** : Dans les titres de section uniquement
- **Espacement optimal** : 1 ligne vide entre sections, max 2 lignes

---

## 📋 Formatage Standard

### Titres

```markdown
# Titre Principal (H1) - Avec emoji optionnel
## Section Principale (H2) - 🎯 Avec emoji descriptif
### Sous-section (H3) - ✅ Avec emoji fonctionnel
#### Détail (H4) - Rarement utilisé
```

**Règles :**
- Espace après `#`
- Pas de ponctuation finale
- Emojis uniquement si pertinents

### Listes

**Listes à puces modernes :**
```markdown
• ✅ Point clé avec emoji
• 🔄 Autre point avec emoji
- Point simple (sans emoji)
```

**Listes numérotées :**
```markdown
1. Première étape
2. Deuxième étape
3. Troisième étape
```

**Listes imbriquées :**
```markdown
- Item principal
  - Sous-item
  - Autre sous-item
```

### Blocs de Code

```markdown
```python
# Toujours spécifier le langage
code_here()
```
```

**Langages courants :**
- `python` pour code Python
- `bash` pour commandes shell
- `markdown` pour exemples MD
- `json` pour JSON
- `yaml` pour YAML

### Séparateurs

```markdown
---  # Standard (3 tirets, pas plus)
```

**Pas de variations :** `===`, `---`, `___`, etc.

---

## 🎨 Style Visuel

### Emojis Stratégiques

**Dans les titres :**
- 📋 Informations
- 🎯 Objectifs
- ✅ Accomplissements
- ⚠️ Avertissements
- ❌ Erreurs/Problèmes
- 🔍 Recherches/Audits
- 📊 Métriques/Stats
- 📝 Documentation
- 🚀 Démarrage/Quick Start
- 🏗️ Architecture
- 🧪 Tests
- 📚 Guides/Docs
- ⚡ Performance
- 🔒 Sécurité
- 🌟 Highlights

**Dans les listes :**
- ✅ Confirmé/Vrai
- ❌ Faux/Erreur
- ⚠️ À vérifier
- 🔄 Action
- 📝 Note
- 💡 Astuce
- 🎯 Objectif

### Formatage Texte

**Gras pour l'emphase :**
```markdown
**Texte important** - Fonctionnalités, métriques
```

**Italique pour notes :**
```markdown
*Note explicative* - Informations complémentaires
```

**Code inline :**
```markdown
`nom_variable` - Variables, fonctions, classes
```

### Citations

```markdown
> Citation importante ou note
> Multi-ligne si nécessaire
```

---

## 📊 Tableaux

```markdown
| Colonne 1 | Colonne 2 | Colonne 3 |
|-----------|-----------|-----------|
| Donnée 1  | Donnée 2  | Donnée 3  |
```

**Règles :**
- Alignement avec `|`
- Espaces autour des pipes
- En-tête séparé par `---`

---

## 🔗 Liens

```markdown
[Texte du lien](chemin/vers/fichier.md)
[Texte externe](https://example.com)
```

**Règles :**
- Chemins relatifs pour fichiers internes
- URLs complètes pour externes
- Texte descriptif (pas "ici", "ce lien")

---

## 📅 Dates Standardisées

### Format Dates

```markdown
**Date création :** Octobre 2024  # Fixe, ne jamais modifier
**Date mise à jour :** Oct 25 / Nov 25  # Récentes (octobre/novembre 2025)
**Date :** Octobre 2025  # Générales
**Date cible :** Décembre 2025  # Futures
```

### Historique

- **Octobre 2024** : Date création projet (première release v1.0.0)
- **Oct 25 / Nov 25** : Période actuelle (octobre/novembre 2025)
- **Octobre 2025** : Dates générales 2025

---

## ✅ Checklist Qualité

### Avant Publication

- [ ] Toutes les dates standardisées
- [ ] Tous les liens fonctionnels
- [ ] Tous les blocs de code avec langage
- [ ] Espacement cohérent (max 2 lignes vides)
- [ ] Emojis judicieux (titres uniquement)
- [ ] Métriques vérifiées contre code réel
- [ ] Ton neutre et professionnel
- [ ] Vouvoiement cohérent

### Vérification Contenu

- [ ] 12 émotions (pas 11, pas 13)
- [ ] Tests 1200+ (1157-1208 acceptable)
- [ ] Docs 280+ (300 fichiers réels)
- [ ] Architecture Factory+ABC confirmée
- [ ] CI/CD outils présents
- [ ] Caches globaux documentés

---

## 🎯 Exemples

### Bon Formatage

```markdown
## 🎯 Vue d'ensemble

BBIA-SIM est un moteur cognitif Python avec **12 émotions robotiques**.

### ✨ Points Clés

• ✅ **Conforme au SDK officiel** (100% validé)
• 🔄 **Backend unifié** : même code sim/hardware
• 🧪 **1200+ tests automatisés**

## 🚀 Quick Start

```bash
pip install -e .[dev]
python examples/demo_emotion_ok.py
```
```

### Mauvais Formatage

```markdown
##Vue d'ensemble  # Pas d'espace
BBIA-SIM est un moteur...  # Pas de structure

- Point 1
- Point 2

```code
pip install
```  # Langage manquant
```

---

## 📚 Ressources

- [Markdown Guide](https://www.markdownguide.org/)
- [GitHub Flavored Markdown](https://github.github.com/gfm/)
- [Emoji Cheat Sheet](https://www.webfx.com/tools/emoji-cheat-sheet/)

---

**Dernière mise à jour :** Oct 25 / Nov 25  
**Version :** 1.0

