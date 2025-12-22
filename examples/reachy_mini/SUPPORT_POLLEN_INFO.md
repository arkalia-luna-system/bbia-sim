# 📧 Informations pour le support Pollen Robotics

## 🔴 **PROBLÈME RÉSUMÉ**

- ✅ **Reflash réussi** : Tous les moteurs (10-18) détectés et configurés
- ✅ **Câblage vérifié** : Démonter 2 fois, câbles changés, tout correct
- ✅ **Moteurs fonctionnels** : Le moteur 2 bouge (test réussi)
- ❌ **Moteur 2 clignote rouge** : Erreur matérielle persistante
- ❌ **Tête de travers** : stewart_2 à -22.85° en position "neutre"

## 📊 **DIAGNOSTICS EFFECTUÉS**

### Script `reachy-mini-reflash-motors` :
```
✅ Motor ID 10 (yaw_body) - Configuration correcte
✅ Motor ID 11 (stewart_1) - Configuration correcte
✅ Motor ID 12 (stewart_2) - Configuration correcte ← PROBLÈME ICI
✅ Motor ID 13 (stewart_3) - Configuration correcte
✅ Motor ID 14 (stewart_4) - Configuration correcte
✅ Motor ID 15 (stewart_5) - Configuration correcte
✅ Motor ID 16 (stewart_6) - Configuration correcte
✅ Motor ID 17 (left_antenna) - Configuration correcte
✅ Motor ID 18 (right_antenna) - Configuration correcte
```

### Positions des stewart joints en "neutre" :
```
stewart_1:   0.00°  ✅
stewart_2: -22.85° ❌ ← PROBLÈME
stewart_3: -11.34°
stewart_4:  32.78°
stewart_5: -19.16°
stewart_6:  43.51°
```

### Tests effectués :
- ✅ Le moteur 2 bouge (test de mouvement réussi)
- ✅ Position dans les limites (-22.85° est dans [-80°, 70°])
- ❌ Clignotement rouge persistant
- ❌ Tête de travers

## 🔧 **ACTIONS EFFECTUÉES**

1. ✅ Reflash complet avec `reachy-mini-reflash-motors`
2. ✅ Vérification câblage (démontage 2 fois)
3. ✅ Changement des câbles
4. ✅ Réinitialisation des erreurs moteurs
5. ✅ Tentatives de repositionnement de la tête
6. ❌ Problème persiste

## 💡 **DIAGNOSTIC PROBABLE**

**Problème de CALIBRATION/OFFSET d'usine** :
- Les offsets des moteurs sont incorrects
- La position "neutre" (tous à 0) ne correspond pas à une tête droite
- Le moteur 2 a probablement un offset incorrect qui cause :
  - Le clignotement rouge (erreur matérielle)
  - La tête de travers

**OU**

**Moteur 2 défectueux** :
- Le moteur répond mais a une erreur matérielle persistante
- Nécessite un remplacement

## 📋 **INFORMATIONS TECHNIQUES**

- **Robot** : Reachy Mini Wireless
- **Version** : Décembre 2025
- **Motor ID problématique** : 12 (stewart_2)
- **Position actuelle stewart_2** : -0.3988 rad (-22.85°)
- **Limites stewart_2** : [-1.396, 1.222] rad
- **Daemon** : Démarré et fonctionnel
- **SDK** : reachy_mini installé et fonctionnel

## 🎯 **DEMANDE AU SUPPORT**

1. **Recalibration des offsets** pour que la position "neutre" corresponde à une tête droite
2. **Vérification du moteur 2** (ID 12) - peut-être défectueux
3. **Remplacement du moteur 2** si défectueux

## 📧 **CONTACT**

**Formulaire Pollen Robotics** :
https://forms.gle/JdhMzadeCnbynw7Q6

**Informations à copier-coller dans le formulaire** :
```
Problème: Moteur 2 (stewart_2, ID 12) clignote rouge + tête de travers

Actions effectuées:
- Reflash réussi (tous moteurs détectés)
- Câblage vérifié 2 fois (démontage complet)
- Câbles changés
- Moteur 2 bouge mais clignote rouge
- Tête de travers (stewart_2 à -22.85° en position "neutre")

Diagnostic:
- Position stewart_2: -0.3988 rad (-22.85°)
- Position dans les limites mais tête de travers
- Clignotement rouge persistant malgré réinitialisation

Demande:
- Recalibration des offsets
- Vérification/remplacement moteur 2 si défectueux
```

