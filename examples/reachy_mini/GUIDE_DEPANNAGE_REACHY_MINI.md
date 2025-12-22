# Guide de Dépannage Reachy Mini

**Dernière mise à jour** : 22 Décembre 2025  
**Version SDK** : v1.2.4

---

## Diagnostic rapide

### Tableau des symptômes

| Symptôme | Cause probable | Section |
|----------|---------------|---------|
| Moteur clignote rouge | Erreur matérielle (surcharge, surchauffe, câblage) | [Problèmes moteurs](#problèmes-moteurs) |
| Tête penchée | Calibration/offset incorrect | [Problèmes calibration](#problèmes-calibration) |
| Moteur non détecté | Paramètres d'usine incorrects (ID=1, baudrate 57,600) | [Reflash moteurs](#reflash-moteurs) |
| "Missing motor" dans l'app | Moteur avec paramètres d'usine | [Reflash moteurs](#reflash-moteurs) |
| Tous les bras allumés en rouge | Problème système général | [Support Pollen](#support-pollen) |

---

## Problèmes moteurs

### Moteur clignote en rouge

**C'est anormal** - Un moteur qui clignote en rouge indique une erreur matérielle :
- Erreur de surcharge
- Surchauffe
- Problème de connexion/câblage
- Moteur en butée mécanique
- Moteur défectueux

#### Identifier le moteur problématique

**Correspondance Motor ID ↔ Moteur physique** :

| Motor ID | Nom physique | Emplacement |
|----------|--------------|-------------|
| **10** | Base | Rotation du corps |
| **11** | stewart_1 | Tête (moteur 1) |
| **12** | stewart_2 | Tête (moteur 2) |
| **13** | stewart_3 | Tête (moteur 3) |
| **14** | stewart_4 | Tête (moteur 4) |
| **15** | stewart_5 | Tête (moteur 5) |
| **16** | stewart_6 | Tête (moteur 6) |
| **17** | Antenne gauche | Antenne |
| **18** | Antenne droite | Antenne |

#### Solutions par ordre de probabilité

**1. Vérifier le câblage du moteur**

Le problème le plus fréquent est un **câble mal branché** ou **défectueux**.

**Étapes** :
1. **Éteignez le robot** (interrupteur OFF)
2. **Enlevez le capot de la tête**
3. **Vérifiez le câble du moteur** :
   - Le câble est-il **bien enfoncé** dans le connecteur?
   - Le câble n'est-il pas **déconnecté**?
   - Le câble n'est-il pas **endommagé** (coupure, pli, etc.)?
   - Le câble est-il dans le **bon ordre** (daisy-chain)?
4. **Rebranchez le câble** en vous assurant qu'il est bien enfoncé
5. **Rallumez le robot** (interrupteur ON)
6. **Vérifiez** si le clignotement a disparu

**2. Vérifier la butée mécanique**

Le moteur peut être **bloqué mécaniquement**.

**Étapes** :
1. **Éteignez le robot** (interrupteur OFF)
2. **Enlevez le capot de la tête**
3. **Vérifiez manuellement** :
   - Le moteur peut-il **bouger librement**?
   - Y a-t-il une **résistance anormale**?
   - Y a-t-il un **câble qui bloque** le mouvement?
   - Le moteur est-il **en butée** (position limite)?
4. **Déplacez légèrement le moteur** manuellement pour le sortir de la butée
5. **Rallumez le robot** (interrupteur ON)

**3. Réinitialiser les erreurs moteurs**

Si le problème persiste, réinitialisez les erreurs :

```bash
# Sur le robot (SSH)
ssh pollen@192.168.129.64
python3 examples/reachy_mini/reset_motor_2_errors.py
```

**4. Diagnostic automatique**

```bash
# Depuis votre Mac
python examples/reachy_mini/diagnostic_motor_errors.py
```

**5. Si le problème persiste**

Si toutes les solutions ci-dessus ne fonctionnent pas, le moteur est probablement **défectueux** et doit être **remplacé**.

Voir [Support Pollen](#support-pollen) pour procédure de remplacement.

---

## Problèmes calibration

### Tête penchée

**Situation** :
- ✅ Tous les moteurs sont détectés (reflash réussi)
- ✅ Le moteur bouge (test réussi)
- ✅ Les câbles sont corrects (vérifiés)
- ❌ La tête est toujours de travers
- ❌ Le moteur clignote en rouge

**Diagnostic** :

Ce n'est **PAS** un problème de câblage. C'est probablement un problème de **CALIBRATION/OFFSET**.

Les moteurs ont des **offsets différents** qui font que la position "neutre" (tous à 0) ne correspond pas à une tête droite.

**Preuve** :
- Le script montre que même en position "neutre", les stewart joints ne sont pas à 0 :
  - stewart_1: 0.00°
  - stewart_2: -22.85° ← **PROBLÈME ICI**
  - stewart_3: -11.34°
  - stewart_4: 32.78°
  - stewart_5: -19.16°
  - stewart_6: 43.51°

#### Solutions

**1. Script de correction forcée**

```bash
# Sur le robot
python3 /tmp/force_head_straight.py
```

Ce script va :
- Désactiver/réactiver les moteurs
- Faire des mouvements pour débloquer
- Essayer de repositionner la tête

**2. Recalibration des offsets**

Si le problème persiste, il faut **recalibrer les offsets** des moteurs.

**Option A : Via le script officiel (si disponible)**
```bash
reachy-mini-calibrate-offsets
```

**Option B : Manuellement via SSH**
```bash
ssh pollen@192.168.129.64
# Suivre procédure de calibration officielle
```

---

## Reflash moteurs

### Bug Décembre 2025

**Problème** : Tête penchée, moteur bloqué mais pas de LED rouge  
**Cause** : Moteur avec paramètres d'usine (ID=1, baudrate 57,600) au lieu de la config correcte  
**Solution** : Script officiel `reachy-mini-reflash-motors`

**⚠️ IMPORTANT** - Ce n'est PAS un problème de montage !

Ce bug affecte plusieurs utilisateurs du lot décembre 2025. Votre assemblage est correct, c'est un problème logiciel qui se corrige avec le script de reflash.

**Confirmé par** :
- robertodipizzamano: "Motor 1 issue fixed for me by running the 592 branch reachy-mini-reflash-motors script"
- Post officiel Augustin (Pollen Team): "Head tilted, motor n°1 not moving, but get stiff when powered on - SOLVED"

### Procédure officielle complète

#### Étape 1 : Préparation

1. **Alimenter le robot SANS démarrer le daemon** :
   - Brancher le robot
   - Mettre l'interrupteur sur **ON**
   - **NE PAS** ouvrir le dashboard
   - **NE PAS** démarrer le daemon

2. **Arrêter le daemon si déjà lancé** :
   ```bash
   # Sur le robot (SSH) ou localement:
   sudo systemctl stop reachy-mini-daemon
   ```

#### Étape 2 : Mettre à jour le package

```bash
pip install --upgrade reachy-mini
```

#### Étape 3 : Lancer le script de reflash

```bash
reachy-mini-reflash-motors
```

**Ce que fait le script** :
- Demande si vous avez la version **Lite** (USB) ou **Wireless** (WiFi)
- Détecte automatiquement les ports série
- Reflash tous les moteurs avec la configuration correcte
- Vérifie que tous les moteurs sont détectés

**Résultat attendu** :
```
✅ Motor ID 10 (yaw_body) - Configuration correcte
✅ Motor ID 11 (stewart_1) - Configuration correcte
✅ Motor ID 12 (stewart_2) - Configuration correcte
✅ Motor ID 13 (stewart_3) - Configuration correcte
✅ Motor ID 14 (stewart_4) - Configuration correcte
✅ Motor ID 15 (stewart_5) - Configuration correcte
✅ Motor ID 16 (stewart_6) - Configuration correcte
✅ Motor ID 17 (left_antenna) - Configuration correcte
✅ Motor ID 18 (right_antenna) - Configuration correcte
```

#### Étape 4 : Redémarrer le daemon

```bash
sudo systemctl start reachy-mini-daemon
```

#### Étape 5 : Vérifier

- Ouvrir le dashboard
- Vérifier que tous les moteurs sont détectés
- Tester les mouvements de la tête

---

## Support Pollen

### Informations pour le support

Si le problème persiste après avoir essayé toutes les solutions ci-dessus, contactez le support Pollen avec les informations suivantes :

#### Problème résumé

- ✅ **Reflash réussi** : Tous les moteurs (10-18) détectés et configurés
- ✅ **Câblage vérifié** : Démonter 2 fois, câbles changés, tout correct
- ✅ **Moteurs fonctionnels** : Le moteur bouge (test réussi)
- ❌ **Moteur clignote rouge** : Erreur matérielle persistante
- ❌ **Tête de travers** : stewart_2 à -22.85° en position "neutre"

#### Diagnostics effectués

**Script `reachy-mini-reflash-motors`** :
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

**Positions des stewart joints en "neutre"** :
```
stewart_1:   0.00°  ✅
stewart_2: -22.85° ❌ ← PROBLÈME
stewart_3: -11.34°
stewart_4:  32.78°
stewart_5: -19.16°
stewart_6:  43.51°
```

**Tests effectués** :
- ✅ Le moteur 2 bouge (test de mouvement réussi)
- ✅ Position dans les limites (-22.85° est dans [-80°, 70°])
- ❌ Clignotement rouge persistant
- ❌ Tête de travers

#### Actions effectuées

1. ✅ Reflash complet avec `reachy-mini-reflash-motors`
2. ✅ Vérification câblage (démontage 2 fois)
3. ✅ Changement des câbles
4. ✅ Réinitialisation des erreurs moteurs
5. ✅ Tentatives de repositionnement de la tête
6. ❌ Problème persiste

#### Informations techniques

- **Version SDK** : v1.2.4
- **Version Robot** : Wireless
- **Numéro de série** : [À compléter]
- **Date réception** : 18 Décembre 2025
- **Batch QC moteurs** : 2544/2543 (si applicable)

#### Contact

- **Email support** : support@pollen-robotics.com
- **Discord** : https://discord.gg/pollen-robotics
- **GitHub Issues** : https://github.com/pollen-robotics/reachy_mini/issues

---

## Références

- **SDK Officiel** : <https://github.com/pollen-robotics/reachy_mini>
- **Documentation BBIA** :
  - `docs/hardware/REACHY_MINI_SDK_v1.2.4.md` - Mise à jour SDK v1.2.4
  - `docs/hardware/PROBLEME_MOTEURS_QC_BATCH_DEC2025.md` - Problème batch QC
  - `docs/hardware/GUIDE_COMPLET_AVANT_RECEPTION.md` - Guide réception

---

*Guide consolidé - Décembre 2025*
