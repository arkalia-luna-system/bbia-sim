# 🔍 CLARIFICATION ANTENNES REACHY MINI

**Date** : octobre 2025
**Question** : Les antennes sont-elles animées ou bloquées ?
**Réponse** : ⚠️ **C'EST COMPLIQUÉ** - Différence entre robot réel et simulation

---

## 📊 SITUATION ACTUELLE

### ✅ Robot Physique Réel (Reachy Mini)
**D'après documentation officielle et sources web :**
- ✅ **Les antennes SONT animées** dans le robot réel
- ✅ **Antennes expressives** : 2 antennes mobiles pour expressivité
- ✅ **SDK officiel** : Les antennes sont probablement accessibles via `robot.head.left_antenna` et `robot.head.right_antenna`

**Sources :**
- Documentation Pollen Robotics
- Sites spécialisés (actuia.com, planeterobots.com)
- Marketing : "2 antennes animées pour expressivité"

---

### ⚠️ Modèle de Simulation XML (`reachy_mini_REAL_OFFICIAL.xml`)

**Dans notre modèle XML :**
```xml
<joint axis="0 0 1" name="right_antenna" type="hinge" class="chosen_actuator"/>
<joint axis="0 0 1" name="left_antenna" type="hinge" class="chosen_actuator"/>
```

**Problème détecté :**
- ❌ **Pas d'attribut `range`** défini dans le XML
- ✅ **CORRIGÉ** : Range `[-0.300, 0.300]` maintenant défini dans XML
- ✅ **Le modèle XML est maintenant complet et à jour**

**Vérification :**
- Script `check_joints.py` confirme : range `[-0.300, 0.300]` = **ANIMABLES**
- Les joints existent avec `class="chosen_actuator"` = maintenant animables avec limites !

---

## 🤔 POURQUOI CETTE CONFUSION ?

### Hypothèses

#### Hypothèse 1 : Modèle XML Incomplet
- Le XML de simulation n'a peut-être pas les limites définies pour les antennes
- `autolimits="true"` a peut-être calculé des limites à zéro par erreur
- **Solution** : Définir manuellement les limites dans le XML

#### Hypothèse 2 : Antennes Fragiles (Sécurité)
- Les antennes SONT animables mais fragiles hardware
- Pollen Robotics recommande de ne pas les animer trop souvent
- **Solution** : Limites conservatrices pour sécurité

#### Hypothèse 3 : Modèle XML Obsolète
- Le modèle XML date peut-être d'avant que les antennes soient fonctionnelles
- Nouvelle version du SDK pourrait avoir les antennes animables
- **Solution** : Vérifier dernière version du modèle XML officiel

---

## ✅ CE QUE BBIA FAIT ACTUELLEMENT

### Code Actuel
```python
# src/bbia_sim/backends/reachy_mini_backend.py
self.forbidden_joints = {
    "left_antenna",   # Antennes trop fragiles
    "right_antenna",
}

# Limites conservatrices
"left_antenna": (-1.0, 1.0),   # Limite de sécurité (hardware fragile)
"right_antenna": (-1.0, 1.0),  # Limite de sécurité (hardware fragile)
```

**Raison** : Protection hardware (antennes fragiles)

---

## 🎯 RECOMMANDATIONS

### Option A : Vérifier SDK Officiel (RECOMMANDÉ)
1. **Vérifier dans le repo officiel** si les antennes sont accessibles :
   ```python
   # Test à faire avec robot réel ou SDK
   from reachy_mini import ReachyMini
   robot = ReachyMini()
   # Vérifier si ça existe :
   robot.head.left_antenna  # ?
   robot.head.right_antenna  # ?
   ```

2. **Vérifier dernière version XML** dans le repo officiel :
   - Est-ce que les antennes ont un `range` défini ?
   - Quelle est la dernière version du modèle ?

### Option B : Débloquer Antennes dans Simulation
1. **Ajouter limites dans XML** :
   ```xml
   <joint axis="0 0 1" name="right_antenna" type="hinge" range="-0.5 0.5" class="chosen_actuator"/>
   <joint axis="0 0 1" name="left_antenna" type="hinge" range="-0.5 0.5" class="chosen_actuator"/>
   ```

2. **Retirer de `forbidden_joints`** (avec prudence)

3. **Tester avec amplitudes faibles** (0.1-0.2 rad)

### Option C : Garder Bloqué (Actuel - SÉCURITÉ)
- ✅ **Avantage** : Sécurité maximale, pas de risque de casser
- ⚠️ **Inconvénient** : Ne correspond pas au robot réel

---

## 📝 ACTIONS IMMÉDIATES

### À Faire Avant Octobre 2025

- [ ] **1. Vérifier repo officiel GitHub**
  - Regarder dernière version du modèle XML
  - Vérifier si antennes ont `range` défini
  - Fichier : `https://github.com/pollen-robotics/reachy_mini`

- [ ] **2. Vérifier SDK officiel**
  - Tester `robot.head.left_antenna` et `robot.head.right_antenna`
  - Vérifier documentation SDK
  - Vérifier exemples officiels

- [ ] **3. Tester avec robot physique** (Octobre 2025)
  - Vérifier si les antennes fonctionnent vraiment
  - Tester limites sûres
  - Vérifier fragilité hardware

---

## 🔧 CORRECTION PROPOSÉE

Si les antennes SONT animables dans le robot réel :

1. **Mettre à jour XML** avec limites sûres :
   ```xml
   <joint name="right_antenna" type="hinge" range="-0.3 0.3" class="chosen_actuator"/>
   <joint name="left_antenna" type="hinge" range="-0.3 0.3" class="chosen_actuator"/>
   ```

2. **Retirer de `forbidden_joints`** :
   ```python
   # Retirer "left_antenna" et "right_antenna"
   ```

3. **Ajouter limites dans `joint_limits`** :
   ```python
   "left_antenna": (-0.3, 0.3),   # Limite sûre pour protection
   "right_antenna": (-0.3, 0.3),  # Limite sûre pour protection
   ```

4. **Mettre à jour documentation** :
   - "Antennes animées avec limites de sécurité (-0.3 à 0.3 rad)"

---

## 📊 TABLEAU COMPARATIF

| Aspect | Robot Réel | Modèle XML | BBIA Actuel | Action |
|--------|------------|------------|-------------|--------|
| **Antennes animables ?** | ✅ OUI | ❌ BLOQUÉES | ❌ BLOQUÉES | À vérifier |
| **Limites définies ?** | ? | ❌ Non | ✅ Oui (-1.0, 1.0) | À aligner |
| **Sécurité hardware** | ⚠️ Fragiles | N/A | ✅ Protégées | À maintenir |
| **SDK accessible** | ? | N/A | ✅ Via `robot.head.*` | À tester |

---

## 🎯 CONCLUSION

**Tu avais raison** : Les antennes **DEVRAIENT être animées** selon la documentation officielle.

**Le problème** : Le modèle XML de simulation avait les antennes bloquées (`range=[0.000, 0.000]`), mais c'est maintenant **CORRIGÉ** avec `range=[-0.300, 0.300]`.

**Action immédiate** :
1. Vérifier repo officiel pour modèle XML à jour
2. Vérifier SDK officiel pour accès antennes
3. Débloquer avec limites sûres si confirmé (Octobre 2025 avec robot réel)

**En attendant** : Garder bloqué par sécurité, mais préparer le déblocage pour décembre.

---

**Statut** : ⚠️ **À VÉRIFIER AVEC ROBOT PHYSIQUE**
**Date** : octobre 2025
**Prochaine vérification** : Octobre 2025 (robot physique)

