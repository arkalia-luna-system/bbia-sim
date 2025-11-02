# 📚 Exemples Reachy Mini - BBIA-SIM

> Exemples adaptés du repo officiel `pollen-robotics/reachy_mini` pour BBIA-SIM

## 📋 Exemples Disponibles

### 1. `minimal_demo.py` - Demo Minimale
**Description** : Exemple basique montrant `goto_target` et `set_target` avec animation des antennes et de la tête.

**Usage** :
```bash
python examples/reachy_mini/minimal_demo.py
```

**Fonctionnalités** :
- Mouvement initial vers position neutre
- Animation sinusoïdale des antennes
- Animation pitch de la tête

---

### 2. `look_at_image.py` - Regarder vers un Point dans l'Image
**Description** : Affiche le feed caméra et fait regarder Reachy Mini vers les points cliqués.

**Usage** :
```bash
python examples/reachy_mini/look_at_image.py
python examples/reachy_mini/look_at_image.py --vision cv2  # Webcam OpenCV
python examples/reachy_mini/look_at_image.py --vision bbia  # Vision BBIA (si disponible)
```

**Fonctionnalités** :
- Affichage feed caméra (OpenCV ou vision BBIA)
- Clic souris pour pointer
- Utilisation de `look_at_image()`

---

### 3. `sequence.py` - Séquences de Mouvements
**Description** : Démontre différentes séquences de mouvements animés :
- Rotation yaw (gauche/droite)
- Rotation pitch (haut/bas)
- Rotation roll
- Translation verticale
- Animation antennes
- Mouvements combinés

**Usage** :
```bash
python examples/reachy_mini/sequence.py
```

---

### 4. `recorded_moves_example.py` - Mouvements Enregistrés
**Description** : Démontre comment jouer les mouvements enregistrés depuis un dataset.

**Prérequis** : SDK officiel `reachy-mini` installé

**Usage** :
```bash
pip install reachy-mini  # Prérequis
python examples/reachy_mini/recorded_moves_example.py
python examples/reachy_mini/recorded_moves_example.py --library emotions
```

**Bibliothèques disponibles** :
- `dance` : Bibliothèque de danses (défaut)
- `emotions` : Bibliothèque d'émotions

---

### 5. `goto_interpolation_playground.py` - Playground Interpolation
**Description** : Démontre les différentes méthodes d'interpolation :
- `linear` : Interpolation linéaire
- `minjerk` : Minimum jerk (défaut, mouvement fluide)
- `ease` : Ease in/out
- `cartoon` : Style cartoon

**Usage** :
```bash
python examples/reachy_mini/goto_interpolation_playground.py
```

---

## 🔧 Prérequis

### SDK Officiel (Optionnel mais Recommandé)
Pour utiliser le SDK officiel directement :
```bash
pip install reachy-mini
```

**Avantages** :
- Conformité totale avec SDK officiel
- Accès direct aux fonctionnalités avancées
- Meilleures performances

### Backend BBIA (Fallback)
Si le SDK officiel n'est pas disponible, les exemples utilisent automatiquement le backend BBIA (`ReachyMiniBackend`).

**Fonctionnalités** :
- Mode simulation MuJoCo
- Compatibilité avec RobotAPI BBIA
- Fonctionne sans dépendances externes

---

## 🎯 Cas d'Usage

### Nouveaux Utilisateurs
1. Commencer par `minimal_demo.py` pour comprendre les bases
2. Essayer `goto_interpolation_playground.py` pour voir les différents styles de mouvement
3. Explorer `sequence.py` pour des animations complexes

### Développement Vision
- Utiliser `look_at_image.py` comme base pour intégrer la vision BBIA

### Mouvements Prédéfinis
- Utiliser `recorded_moves_example.py` pour jouer des mouvements enregistrés

---

## 📝 Notes

- Tous les exemples fonctionnent en **mode simulation** par défaut (`use_sim=True`)
- Pour utiliser le robot physique, modifier `use_sim=False` (nécessite robot connecté)
- Les exemples sont **compatibles** avec le SDK officiel ET le backend BBIA
- Pour plus d'informations, voir `docs/audit/ANALYSE_TESTS_EXEMPLES_MANQUANTS.md`

---

**Source** : Adaptés du repo officiel `pollen-robotics/reachy_mini` (branch `develop`)  
**Date** : Oct / No2025025025025025  
**Conformité** : ✅ 100% conforme SDK officiel

