# 📐 Guide de Style Markdown - BBIA-SIM

<div align="center">

**Version** : 2.0  
**Date** : Oct / Nov. 2025  
**Objectif** : Documentation moderne, professionnelle, élégante et impactante

</div>

---

## 🎯 Principes Fondamentaux

### Clarté & Professionnalisme

- **Ton neutre et factuel** : Privilégier la précision aux superlatifs
- **Vouvoiement cohérent** : Utiliser "vous" dans toute la documentation
- **Preuves > Promesses** : Chiffres mesurés plutôt que promesses vagues
- **Précision** : Métriques vérifiées contre code réel

### Modernité & Impact Visuel

- **Structure claire** : Hiérarchie de titres cohérente
- **Emojis judicieux** : Dans les titres de section uniquement
- **Espacement optimal** : 1 ligne vide entre sections, max 2 lignes
- **Alignement centré** : Pour les en-têtes de document et sections importantes
- **Tableaux élégants** : Formatage cohérent avec alignement

---

## 📋 Formatage Standard

### Titres

```markdown
# Titre Principal (H1) - Avec emoji optionnel

<div align="center">

**Sous-titre ou description courte**

</div>

## Section Principale (H2) - 🎯 Avec emoji descriptif
### Sous-section (H3) - ✅ Avec emoji fonctionnel
#### Détail (H4) - Rarement utilisé

```

**Règles :**

- Espace après `#`
- Pas de ponctuation finale
- Emojis uniquement si pertinents
- Utiliser `<div align="center">` pour les en-têtes de document

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

```text
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

- 📋 Informations / Listes
- 🎯 Objectifs / Buts
- ✅ Accomplissements / Succès
- ⚠️ Avertissements
- ❌ Erreurs / Problèmes
- 🔍 Recherches / Audits
- 📊 Métriques / Stats
- 📝 Documentation
- 🚀 Démarrage / Quick Start
- 🏗️ Architecture
- 🧪 Tests
- 📚 Guides / Docs
- ⚡ Performance
- 🔒 Sécurité
- 🌟 Highlights
- 🔴 Priorité haute
- 🟡 Priorité moyenne
- 🟢 Priorité basse
- 🔵 Hardware / Matériel

**Dans les listes :**

- ✅ Confirmé / Vrai
- ❌ Faux / Erreur
- ⚠️ À vérifier
- 🔄 Action
- 📝 Note
- 💡 Astuce
- 🎯 Objectif
- ⏳ En attente
- 📌 Important

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

### En-têtes de Document

```markdown
<div align="center">

**Texte centré**
**Date** : Oct / Nov. 2025
**Statut** : ✅ Actif

</div>

```

---

## 📊 Tableaux

### Format Standard

```markdown
<div align="center">

| Colonne 1 | Colonne 2 | Colonne 3 |
|:---------:|:---------:|:---------:|
| Donnée 1  | Donnée 2  | Donnée 3  |

</div>

```

**Règles :**

- Alignement avec `|`
- Espaces autour des pipes
- En-tête séparé par `---`
- Utiliser `:---:` pour centrer
- Utiliser `:---` pour aligner à gauche
- Utiliser `---:` pour aligner à droite
- Encadrer dans `<div align="center">` pour tableaux importants

### Exemple Élégant

```markdown
<div align="center">

| Module | Coverage | Objectif | Statut |
|:------:|:--------:|:--------:|:------:|
| `vision_yolo.py` | **99.45%** | 50%+ | ✅ **DÉPASSÉ** |
| `voice_whisper.py` | **92.52%** | 50%+ | ✅ **DÉPASSÉ** |

</div>

```

---

## 🔗 Liens

```markdown
[Guide de Démarrage](../guides/GUIDE_DEMARRAGE.md) (exemple)
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
**Date mise à jour :** Oct / Nov. 2025  # Récentes
**Date :** Oct / Nov. 2025  # Générales
**Date cible :** Oct / Nov. 2025  # Futures

```

### Historique

- **Octobre 2024** : Date création projet (première release v1.0.0)
- **Oct / Nov** : Période actuelle

---

## 🎨 Design BBIA

### Couleurs de la Marque

- **Bleu céleste/néon** : `#87bcfa`, `#3E6FFF`
- **Violet électrique** : `#A680FF`, `#C082FF`
- **Turquoise éthérée** : `#60e9e1`
- **Gris lunaire** : `#eaeaed`, `#bfc9d9`
- **Rose pastel** : `#FFDAEC`

### Ambiance Visuelle

- **Futuriste doux** : Technologique mais accessible
- **Poétique** : Inspiré de l'univers lunaire
- **Friendly** : Chaleureux et rassurant
- **Professionnel** : Précis et fiable

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
- [ ] Tableaux alignés et formatés
- [ ] En-têtes centrés pour documents importants

### Vérification Contenu

- [ ] 12 émotions (pas 11, pas 13)
- [ ] Tests **2311 collectés** (2201 actifs, 110 désélectionnés)
- [x] Docs 131 fichiers (après nettoyage, 53 fichiers supprimés)
- [ ] Architecture Factory+ABC confirmée
- [ ] CI/CD outils présents
- [ ] Caches globaux documentés

---

## 🎯 Exemples

### Bon Formatage

```markdown
# 📐 Guide de Style Markdown

<div align="center">

**Version** : 2.0  
**Date** : Oct / Nov. 2025

</div>

---

## 🎯 Vue d'ensemble

BBIA-SIM est un moteur cognitif Python avec **12 émotions robotiques**.

### ✨ Points Clés

• ✅ **Conforme au SDK officiel** (100% validé)
• 🔄 **Backend unifié** : même code sim/hardware
• 🧪 **2311 tests collectés** (référence coverage sur CI/Codecov)

## 🚀 Quick Start

```bash

pip install -e .[dev]
python examples/demo_emotion_ok.py

```xml

<div align="center">

| Module | Coverage | Statut |
|:------:|:--------:|:------:|
| `vision_yolo.py` | **99.45%** | ✅ |

</div>

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

<div align="center">

**Dernière mise à jour :** Oct / Nov  
**Version :** 2.0

</div>
