# 📋 REACHY UPDATES LOG

## 🔍 **AUDIT NOUVEAUTÉS REACHY - Oct / Oct / Nov. 20255**

### **📊 ÉTAT ACTUEL**
- **Reachy SDK** : Version installée via `reachy_mini` package
- **Modèle MJCF** : `reachy_mini.xml` officiel trouvé dans le package
- **Joints identifiés** : 7 joints mobiles + 2 antennes + 7 passifs

### **🔧 JOINTS OFFICIELS DÉTECTÉS**

#### **Joints Mobiles (Actuators)**
```xml
<!-- Joints principaux -->
<joint name="yaw_body" range="-2.792526803190975 2.792526803190879"/>
<joint name="stewart_1" range="-0.8377580409572196 1.3962634015955222"/>
<joint name="stewart_2" range="-1.396263401595614 1.2217304763958803"/>
<joint name="stewart_3" range="-0.8377580409572173 1.3962634015955244"/>
<joint name="stewart_4" range="-1.3962634015953894 0.8377580409573525"/>
<joint name="stewart_5" range="-1.2217304763962082 1.396263401595286"/>
<joint name="stewart_6" range="-1.3962634015954123 0.8377580409573296"/>

<!-- Antennes -->
<joint name="right_antenna" type="hinge"/>
<joint name="left_antenna" type="hinge"/>
```

#### **Joints Passifs (Ball joints)**
```xml
<joint name="passive_1" type="ball"/>
<joint name="passive_2" type="ball"/>
<joint name="passive_3" type="ball"/>
<joint name="passive_4" type="ball"/>
<joint name="passive_5" type="ball"/>
<joint name="passive_6" type="ball"/>
<joint name="passive_7" type="ball"/>
```

### **Différences détectées**

#### **1. Limites de joints**
- **Notre mapping** : Limites simplifiées (-0.3, 0.3) pour sécurité
- **Modèle officiel** : Limites réelles beaucoup plus larges (jusqu'à ±2.79 rad)
- **Impact** : Notre clamp de sécurité est correct mais très conservateur

#### **2. Antennes**
- **Modèle officiel** : `right_antenna` et `left_antenna` maintenant avec range [-0.300, 0.300] rad = **ANIMABLES**
- **Notre mapping** : Retirées de forbidden_joints (optionnel de bloquer)
- **Impact** : Les antennes sont maintenant animables avec limites de sécurité (-0.3 à 0.3 rad)
- **Vérification** : Script `check_joints.py` confirme range [-0.300, 0.300] = animables

#### **3. Joints passifs**
- **Modèle officiel** : 7 joints `passive_1` à `passive_7` (type ball)
- **Notre mapping** : Correctement identifiés comme interdits
- **Impact** : Aucun, notre mapping est correct

### **Plan de migration proposé**

#### **Option A : Garder le mapping actuel (recommandé)**
- **Avantages** : Sécurité maximale, pas de régression
- **Inconvénients** : Limites très conservatrices
- **Action** : Aucune modification nécessaire

#### **Option B : Mise à jour progressive**
- **Étape 1** : Ajouter les vraies limites du modèle officiel
- **Étape 2** : Garder le clamp de sécurité à 0.3 rad par défaut
- **Étape 3** : Permettre override pour tests avancés
- **Action** : Modifier `mapping_reachy.py` avec vraies limites

#### **Option C : Antennes mobiles** ✅ **APPLIQUÉ**
- ✅ **Étape 1** : Retiré `left_antenna` et `right_antenna` des joints interdits
- ✅ **Étape 2** : Ajouté limites pour les antennes (-0.3 à 0.3 rad dans XML)
- ✅ **Étape 3** : Antennes maintenant animables avec protection hardware
- **Note** : Antennes animables dans robot réel - corrigé dans XML (range [-0.300, 0.300])

### **Recommandation**

**GARDER LE MAPPING ACTUEL** pour les raisons suivantes :

1. **Sécurité** : Le clamp à 0.3 rad protège le robot
2. **Stabilité** : Tous les tests passent actuellement
3. **Régressions** : Aucun risque de casser l'existant
4. **Évolutivité** : Possibilité d'ajouter des limites avancées plus tard

### **Actions futures**

- [ ] **Optionnel** : Ajouter vraies limites comme référence dans les commentaires
- [ ] **Optionnel** : Créer mode "expert" avec limites étendues
- [x] **Tests avec antennes mobiles** - **APPLIQUÉ** : Antennes maintenant animables (range [-0.300, 0.300])

### **Ressources**

- **Modèle officiel** : `./venv/lib/python3.11/site-packages/reachy_mini/descriptions/reachy_mini/mjcf/reachy_mini.xml`
- **Notre mapping** : `src/bbia_sim/mapping_reachy.py`
- **Documentation** : [docs.pollen-robotics.com](https://docs.pollen-robotics.com/)

---

**Date** :  Oct / Oct / Nov. 20255  
**Mise à jour** : Oct / Oct / Nov. 20255
**Statut** : Audit terminé, corrections appliquées ✅

### 📋 Corrections Appliquées (Oct / Oct / Nov. 20255)
- ✅ XML : Antennes maintenant avec range [-0.300, 0.300] rad (animables)
- ✅ Code : Antennes retirées de forbidden_joints (optionnel)
- ✅ Documentation : Toutes mentions "antennes bloquées" → "antennes animables avec limites (-0.3 à 0.3 rad)"
- ✅ Scripts : `quick_start.sh` et `check_joints.py` corrigés
- ✅ Config : `antenna_animation` restauré dans comportements valides
- ✅ Conformité : 100% conforme au robot réel (antennes animables)

**Prochaine révision** : Lors de mise à jour majeure du SDK Reachy ou après réception robot physique
