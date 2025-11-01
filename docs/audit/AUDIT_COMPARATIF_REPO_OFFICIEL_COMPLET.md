# 🔍 AUDIT COMPARATIF COMPLET - Repo Officiel Reachy-Mini vs BBIA

**Date** : 2025-10-31  
**Repo Officiel** : [pollen-robotics/reachy_mini](https://github.com/pollen-robotics/reachy_mini)  
**Version BBIA** : 1.3.2  
**Objectif** : Audit exhaustif comparant chaque aspect du repo officiel avec BBIA pour identifier micro-fautes et incohérences avant réception du robot (Octobre 2025)

---

## 📊 RÉSUMÉ EXÉCUTIF

### ✅ Points Conformes
- ✅ SDK officiel correctement intégré (`reachy_mini`)
- ✅ Modèle 3D officiel (`reachy_mini_REAL_OFFICIEL.xml`)
- ✅ 41 fichiers STL officiels présents
- ✅ Limites articulations extraites exactement du XML
- ✅ Dépendances SDK officiel présentes dans `pyproject.toml`
- ✅ Antennes correctement bloquées (`forbidden_joints`) - conforme XML officiel
- ✅ Documentation mise à jour (octobre 2024)

### ✅ Corrections Appliquées (2025-10-31)
- ✅ **Antennes** : Documentation corrigée - toutes mentions "antennes animées" → "antennes bloquées (sécurité hardware)"
- ✅ **Scripts** : `quick_start.sh` corrigé pour mentionner antennes bloquées
- ✅ **Global Config** : `antenna_animation` retiré des comportements valides (obsolète)
- ✅ **REACHY_MINI_REFERENCE.md** : Clarification expressivité (yeux + mouvements tête/corps)

### ⚠️ Points à Vérifier/Corriger (Actions Futures)
- ⚠️ **Version SDK** : Vérifier version exacte utilisée dans repo officiel
- ⚠️ **Software Release** : Vérifier version SDK disponible sur GitHub (email oct 2024 mentionne première version)
- ⚠️ **Configuration caméra** : Vérifier paramètres exacts (résolution, FOV) vs repo officiel
- ⚠️ **Beta Shipments** : 125 unités en octobre 2024 - vérifier retours communauté pour ajustements

---

## 1. 📏 SPÉCIFICATIONS TECHNIQUES & MESURES

### 1.1 Dimensions Globales

#### ✅ BBIA (Conforme)
```
Hauteur : 280mm (actif) / 230mm (veille)
Largeur : 160mm
Poids : 1.5kg
```
**Source** : `docs/reachy/REACHY_MINI_REFERENCE.md` ligne 100

#### ⚠️ Repo Officiel (Email Octobre 2024)
- **Hauteur** : 28 cm (actif) / 23 cm (veille) ✅ **CONFORME**
- **Largeur** : 16 cm ✅ **CONFORME**
- **Poids** : 1,5 kg ✅ **CONFORME**

**Conclusion** : ✅ Dimensions conformes

---

### 1.2 Position Tête (Frame "head")

#### BBIA (Extrait XML)
```xml
<site group="3" name="head" pos="-0.00611127 0.00370522 0.0291364"/>
```
**En mm** :
- X : -6.11mm (décalage gauche)
- Y : 3.7mm (décalage avant)
- Z : 29.14mm (hauteur depuis corps)

**Source** : `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml` ligne 496

#### ⚠️ À Vérifier dans Repo Officiel
- Position exacte de la caméra (`camera_optical`)
- Position des microphones

**Action** : Vérifier dans repo officiel si positions correspondent

---

### 1.3 Antennes

#### BBIA (Extrait XML)
```xml
<!-- Antenne Droite -->
<geom type="mesh" pos="8.00228e-15 -0.0588 -0.0103" mesh="antenna"/>
<!-- Antenne Gauche -->
<geom type="mesh" pos="6.79838e-15 -0.0588 -0.0103" mesh="antenna"/>
```
**Dimensions** :
- Hauteur : 58.8mm
- Profondeur : 10.3mm

**Source** : `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml` lignes 503, 523

#### ⚠️ IMPORTANT - Limites Antennes
```xml
<joint axis="0 0 1" name="right_antenna" type="hinge" class="chosen_actuator"/>
<joint axis="0 0 1" name="left_antenna" type="hinge" class="chosen_actuator"/>
```
**⚠️ PROBLÈME DÉTECTÉ** : Pas de `range` défini dans le XML officiel pour les antennes !
- Dans BBIA : Limites conservatrices `(-1.0, 1.0)` rad
- Dans XML : Pas de range → Antennes **bloquées** par défaut

**Action Requise** :
1. ✅ **Déjà fait** : BBIA bloque les antennes (`forbidden_joints`)
2. ⚠️ **Vérifier** : Dans repo officiel SDK, comment les antennes sont gérées
3. ⚠️ **Documenter** : Clarifier pourquoi antennes bloquées (fragilité hardware)

**Source** : `src/bbia_sim/backends/reachy_mini_backend.py` lignes 129-132

---

### 1.4 Corps (Body)

#### BBIA (Extrait XML)
```xml
<geom type="mesh" pos="3.79972e-17 -3.70588e-18 0.195" mesh="body_down_3dprint"/>
```
**Hauteur Z** : 195mm (19.5cm depuis base)

**Source** : `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml` ligne 90

#### ⚠️ À Vérifier
- Position exacte du centre de masse
- Inertie (`fullinertia`) dans XML

**Conclusion** : ✅ Dimensions extraites correctement

---

## 2. 🤖 SDK & DÉPENDANCES

### 2.1 Import SDK Officiel

#### ✅ BBIA (Conforme)
```python
try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose
    REACHY_MINI_AVAILABLE = True
except ImportError:
    REACHY_MINI_AVAILABLE = False
```
**Source** : `src/bbia_sim/backends/reachy_mini_backend.py` lignes 15-23

#### ⚠️ Repo Officiel (À Vérifier)
- **Nom package** : `reachy_mini` ✅ (déjà vérifié)
- **Version requise** : ? (v1.0.0?) ⚠️ **À VÉRIFIER**
- **Méthodes disponibles** : `goto_target()`, `look_at_world()`, `look_at_image()` ✅

**Action** : Vérifier version exacte dans repo officiel GitHub

---

### 2.2 Dépendances SDK

#### ✅ BBIA (pyproject.toml lignes 47-59)
```toml
"reachy_mini_motor_controller>=1.0.0",  ✅
"eclipse-zenoh>=1.4.0",                 ✅
"reachy-mini-rust-kinematics>=1.0.1",   ✅
"cv2_enumerate_cameras>=1.2.1",         ✅
"soundfile>=0.13.1",                     ✅
"huggingface-hub>=0.34.4",              ✅
"log-throttling>=0.0.3",                 ✅
"scipy>=1.15.3",                         ✅
"asgiref>=3.7.0",                        ✅
"aiohttp>=3.9.0",                        ✅
"psutil>=5.9.0",                         ✅
"jinja2>=3.1.0",                         ✅
"pyserial>=3.5",                         ✅
```

#### ⚠️ À Comparer avec Repo Officiel
**Action** : Vérifier `requirements.txt` ou `pyproject.toml` du repo officiel pour :
- Versions exactes requises
- Dépendances manquantes dans BBIA
- Dépendances obsolètes dans BBIA

---

### 2.3 Méthodes SDK Utilisées

#### ✅ BBIA (Conforme)
```python
# Méthodes utilisées dans BBIA
robot.goto_target(head_pose, duration=duration)
robot.look_at_world(point_3d, duration=duration)
robot.look_at_image(point_2d, duration=duration)
robot.head.motors[0].goal_position = angle
robot.body.turn_on() / turn_off()
```

**Source** : `src/bbia_sim/backends/reachy_mini_backend.py` (méthodes `move_head`, `look_at`)

#### ⚠️ À Vérifier dans Repo Officiel
- Toutes ces méthodes existent-elles ?
- Nouveaux paramètres ajoutés ?
- Méthodes dépréciées ?

**Action** : Comparer API complète du repo officiel avec BBIA

---

## 3. 📐 MODÈLES 3D & SCÈNES

### 3.1 Modèle XML Principal

#### ✅ BBIA
```
src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml
```
- **Source** : Modèle officiel OnShape → XML (via `onshape-to-robot`)
- **Ligne 4** : `<!-- Onshape https://cad.onshape.com/documents/... -->`
- **Ligne 3** : URL OnShape officielle ✅

#### ⚠️ À Vérifier
- **Version modèle** : Le XML est-il à jour avec la dernière version OnShape ?
- **Ligne 5** : `<compiler angle="radian" meshdir="../assets/reachy_official" autolimits="true"/>`
  - ✅ Chemin STL correct
  - ✅ `autolimits="true"` (extrait limites automatiquement)

**Action** : Vérifier si repo officiel utilise le même modèle ou une version plus récente

---

### 3.2 Assets STL

#### ✅ BBIA (41 fichiers STL)
```
src/bbia_sim/sim/assets/reachy_official/
├── body_down_3dprint.stl
├── body_top_3dprint.stl
├── head_front_3dprint.stl
├── head_back_3dprint.stl
├── stewart_*.stl (6 fichiers)
├── antenna*.stl (4 fichiers)
└── ... (31 autres)
```
**Total** : 41 fichiers STL ✅

#### ⚠️ À Vérifier
- **Source** : STL téléchargés depuis repo officiel ?
- **Version** : Dernière version des STL ?
- **Checksums** : Vérifier intégrité des fichiers

**Action** : Comparer liste STL avec repo officiel

---

### 3.3 Limites Articulations

#### ✅ BBIA (Extrait Exact du XML)
```python
self.joint_limits = {
    "stewart_1": (-0.8377580409572196, 1.3962634015955222),
    "stewart_2": (-1.396263401595614, 1.2217304763958803),
    "stewart_3": (-0.8377580409572173, 1.3962634015955244),
    "stewart_4": (-1.3962634015953894, 0.8377580409573525),
    "stewart_5": (-1.2217304763962082, 1.396263401595286),
    "stewart_6": (-1.3962634015954123, 0.8377580409573296),
    "yaw_body": (-2.792526803190975, 2.792526803190879),
}
```
**Source** : `src/bbia_sim/backends/reachy_mini_backend.py` lignes 108-125

#### ⚠️ Vérification XML
```xml
<joint axis="0 0 1" name="yaw_body" range="-2.792526803190975 2.792526803190879"/>
<joint axis="0 0 1" name="stewart_1" range="-0.8377580409572196 1.3962634015955222"/>
```
**Conclusion** : ✅ Limites extraites **exactement** du XML officiel

---

## 4. 📷 CAMÉRA & CAPTEURS

### 4.1 Configuration Caméra

#### ✅ BBIA (Extrait XML)
```xml
<camera name="eye_camera"
  mode="fixed"
  resolution="1280 720"
  fovy="80"
/>
```
**Source** : `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml` lignes 489-493

#### ⚠️ À Vérifier dans Repo Officiel
- **Résolution réelle** : 1280x720 ? (Email mentionne "caméra grand-angle HD")
- **FOV** : 80° correct ?
- **Format** : MJPG? H264?

**Action** : Vérifier spécifications caméra dans repo officiel

---

### 4.2 Position Caméra

#### ✅ BBIA (Extrait XML)
```xml
<site group="3" name="camera_optical" 
  pos="-0.0321159 -0.05047 0.00257878" 
  quat="0.44884 0.458499 0.740682 -0.199279"/>
```
**Position** :
- X : -32.1mm (décalage gauche)
- Y : -50.5mm (décalage arrière)
- Z : 2.6mm (hauteur depuis tête)

**Source** : `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml` ligne 485

**Conclusion** : ✅ Position extraite correctement

---

### 4.3 Microphones

#### ⚠️ BBIA (Documentation)
- **Version Wireless** : 4 microphones ✅
- **Version Lite** : 2 microphones ⚠️

**Source** : `docs/reachy/REACHY_MINI_REFERENCE.md` ligne 82

#### ⚠️ Email Officiel (Octobre 2024)
- **Wireless** : "4 microphones avec traitement en réseau" ✅
- **Lite** : "2 microphones" ✅

**Conclusion** : ✅ Documentation conforme

**Action** : Vérifier si BBIA gère différemment Wireless vs Lite (actuellement supposé Wireless)

---

## 5. 📚 DOCUMENTATION

### 5.1 README Principal

#### ✅ BBIA
- ✅ Badge "SDK Conformity 100%"
- ✅ Référence repo officiel : `pollen-robotics/reachy_mini`
- ✅ Guide démarrage rapide
- ✅ Spécifications techniques

**Source** : `README.md`

#### ⚠️ À Comparer
- Structure sections
- Exemples de code
- Guide installation

---

### 5.2 Guides Spécifiques

#### ✅ BBIA Guides
- `docs/guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md` ✅
- `docs/reachy/REACHY_MINI_REFERENCE.md` ✅
- `docs/conformite/CONFORMITE_REACHY_MINI_COMPLETE.md` ✅

#### ⚠️ À Vérifier
- Informations à jour avec dernières specs (octobre 2024)
- Erreurs typographiques
- Commandes SDK exactes

---

## 6. 🧪 TESTS & CONFORMITÉ

### 6.1 Tests Conformité SDK

#### ✅ BBIA
- `tests/test_reachy_mini_full_conformity_official.py` ✅
- `tests/test_reachy_mini_advanced_conformity.py` ✅
- 37/37 tests passent ✅

**Source** : `docs/audit/AUDIT_EXHAUSTIF_COMPLET_2025.md`

#### ⚠️ À Améliorer
- Tests avec robot physique (Octobre 2025)
- Tests performance latence
- Tests edge cases (timeouts, disconnections)

---

### 6.2 Coverage

#### ✅ BBIA
- **Modules core** : ~50% coverage ✅
- **Tests totaux** : 1005 tests ✅

#### ⚠️ À Améliorer
- Coverage `reachy_mini_backend.py` : 8.7% ⚠️ **FAIBLE**
- Tests mock SDK (quand robot pas disponible)

**Action** : Créer tests mock avancés (voir `docs/audit/PLAN_AMELIORATION_NOTATION_COMPLETE.md` ligne 33)

---

## 7. 🐛 MICRO-FAUTES IDENTIFIÉES

### 7.1 🔴 CRITIQUE - Limites Antennes

**Problème** :
- XML officiel : Antennes **sans range** (bloquées par défaut)
- BBIA : Limites conservatrices `(-1.0, 1.0)` mais joints interdits

**Impact** : ⚠️ Antennes fragiles hardware → Bloquer correct ✅

**Statut** : ✅ **DÉJÀ CORRIGÉ** (`forbidden_joints`)

---

### 7.2 🟡 MOYENNE - Version SDK

**Problème** :
- `pyproject.toml` : `reachy_mini_motor_controller>=1.0.0`
- Version exacte utilisée inconnue

**Impact** : Possibles incompatibilités futures

**Action** : 
1. Vérifier version exacte dans repo officiel
2. Pinner version si nécessaire

---

### 7.3 🟡 MOYENNE - Documentation Camera

**Problème** :
- Résolution caméra : 1280x720 dans XML
- Email officiel : "caméra grand-angle HD" (non spécifique)

**Impact** : Possible différence résolution réelle vs simulation

**Action** : Vérifier specs exactes caméra dans repo officiel

---

### 7.4 🟢 BASSE - Fichiers STL

**Problème** :
- 41 fichiers STL présents
- Version/checksums non vérifiés

**Impact** : Possible modèle obsolète

**Action** : Comparer avec repo officiel pour vérifier dernière version

---

## 8. ✅ CHECKLIST ACTIONS

### Actions Immédiates (Avant Octobre 2025)

- [ ] **1. Vérifier version SDK officielle**
  - Comparer `requirements.txt` du repo officiel
  - Pinner version si nécessaire
  - Fichier : `pyproject.toml`

- [ ] **2. Comparer API complète SDK**
  - Lister toutes méthodes `ReachyMini` dans repo officiel
  - Vérifier méthodes utilisées dans BBIA existent
  - Fichier : `src/bbia_sim/backends/reachy_mini_backend.py`

- [ ] **3. Vérifier spécifications caméra**
  - Résolution exacte
  - FOV
  - Format vidéo
  - Fichier : `docs/reachy/REACHY_MINI_REFERENCE.md`

- [ ] **4. Comparer liste fichiers STL**
  - Vérifier 41 fichiers présents
  - Vérifier checksums/intégrité
  - Fichier : `src/bbia_sim/sim/assets/reachy_official/`

- [ ] **5. Vérifier configuration microphones**
  - 2 vs 4 selon version (Lite vs Wireless)
  - BBIA suppose Wireless (4 microphones)
  - Fichier : `docs/reachy/REACHY_MINI_REFERENCE.md`

---

### Actions Moyen Terme (Octobre 2025 - Robot Physique)

- [ ] **6. Tests robot physique**
  - Tests conformité SDK sur robot réel
  - Tests performance latence
  - Tests edge cases (timeouts, disconnections)

- [ ] **7. Validation mesures**
  - Mesurer robot réel (hauteur, largeur, poids)
  - Comparer avec XML/simulations
  - Ajuster si nécessaire

- [ ] **8. Tests caméra réelle**
  - Résolution réelle
  - FOV réel
  - Calibration

---

## 9. 📊 TABLEAU RÉCAPITULATIF

| Aspect | BBIA | Repo Officiel | Statut | Action |
|--------|------|---------------|--------|--------|
| **Dimensions** | 28cm/16cm/1.5kg | 28cm/16cm/1.5kg | ✅ | Aucune |
| **Modèle XML** | `reachy_mini_REAL_OFFICIAL.xml` | OnShape officiel | ✅ | Vérifier version |
| **Assets STL** | 41 fichiers | ? | ⚠️ | Comparer liste |
| **SDK Import** | `from reachy_mini import ReachyMini` | `reachy_mini` | ✅ | Vérifier version |
| **Limites joints** | Extraites XML exact | XML officiel | ✅ | Aucune |
| **Antennes** | Bloquées (`forbidden_joints`) | Range vide (bloqué) | ✅ | Aucune |
| **Caméra résolution** | 1280x720 | "HD grand-angle" | ⚠️ | Vérifier exacte |
| **Caméra FOV** | 80° | ? | ⚠️ | Vérifier |
| **Microphones** | 4 (Wireless) | 2-4 selon version | ✅ | Clarifier version |
| **Dépendances SDK** | 13 dépendances | ? | ⚠️ | Comparer versions |

---

## 10. 🎯 RECOMMANDATIONS FINALES

### ✅ Points Excellents
1. **Limites articulations** : Extraites **exactement** du XML officiel ✅
2. **Modèle 3D** : Utilise modèle officiel OnShape ✅
3. **SDK Integration** : Import conditionnel propre ✅
4. **Documentation** : Références officielles présentes ✅

### ⚠️ Points à Améliorer
1. **Version SDK** : Pinner version exacte
2. **Tests coverage** : Augmenter coverage `reachy_mini_backend.py`
3. **Documentation caméra** : Spécifier résolution exacte
4. **Validation physique** : Tester sur robot réel (Octobre 2025)

### 🎯 Priorités
1. **🔴 Priorité 1** : Vérifier version SDK exacte dans repo officiel
2. **🟡 Priorité 2** : Comparer API complète SDK
3. **🟡 Priorité 3** : Vérifier specs caméra exactes
4. **🟢 Priorité 4** : Comparer fichiers STL

---

---

## 11. 📧 INFORMATIONS EMAIL OFFICIEL (OCTOBRE 2024)

### Nouveautés Communiquées

#### 🚀 Beta Shipments
- **125 unités** expédiées en octobre 2024
- **Programme** : Community Beta Program
- **Objectif** : Recueillir feedback avant rollout plus large
- **Sélection** : Testeurs sélectionnés pour capacité feedback régulier et constructif

#### 📦 Shipments Restants
- **~3,000 unités** prévues avant Noël (Lite + Wireless)
- **Calendrier** : Livraisons supplémentaires janvier-Octobre 2025
- **Qualité** : Équipe sur site avec fabricant pour qualité, tests, documentation

#### 💻 Software Release
- **Première version** : Disponible sur GitHub
- **Repo** : https://github.com/pollen-robotics/reachy_mini
- **Contenu** : Codebase, SDK, documentation précoce
- **Action BBIA** : Vérifier version exacte et comparer avec notre implémentation

#### 📅 Prochaines Mises à Jour
- **Prochaine email** : Mi-Octobre 2025
- **Contenu attendu** : Progrès production + software releases

#### ✨ Actualité Bonus
- **Reconnaissance** : TIME Best Inventions 2025 - Special Mentions
- **Impact** : Visibilité accrue pour le projet

#### 🤝 Améliorations Communauté
- **Feedback beta** : Déjà reçu et intégré
- **Bénéfice** : Software raffiné grâce aux testeurs beta
- **Communauté** : Croissance continue de builders et créateurs

---

## 12. ✅ ACTIONS CORRECTIVES APPLIQUÉES (2025-10-31)

### Corrections Documentation

#### 1. ✅ Antennes - Documentation Corrigée
- **Fichiers corrigés** :
  - `docs/reachy/REACHY_MINI_REFERENCE.md` : Ligne 157 - "Expressivité" clarifiée
  - `scripts/quick_start.sh` : Ligne 145 - Antennes → "bloquées (sécurité hardware)"
  - `src/bbia_sim/global_config.py` : `antenna_animation` retiré (obsolète)
  
- **Message standardisé** : "Antennes bloquées (sécurité hardware), utiliser yaw_body pour expressivité"
- **Statut** : ✅ **TOUTES LES MENTIONS CORRIGÉES**

#### 2. ✅ Clarifications Techniques
- **Expressivité** : Clarifiée comme "Yeux + mouvements tête/corps" (pas antennes)
- **Comportements** : `antenna_animation` → `body_yaw_animation`
- **Scripts** : Tous mis à jour avec information correcte

---

**Document créé le 2025-10-31**  
**Dernière mise à jour** : 2025-10-31 (Corrections appliquées)  
**Basé sur** : Email Pollen Robotics Octobre 2024, Repo officiel GitHub, Code BBIA v1.3.2  
**Prochaine mise à jour** : Après réception robot physique (Octobre 2025)

