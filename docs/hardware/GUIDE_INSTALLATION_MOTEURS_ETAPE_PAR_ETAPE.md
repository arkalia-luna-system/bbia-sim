# 🔧 Guide d'Installation des Moteurs - Étape par Étape

**Dernière mise à jour** : 21 Janvier 2026  
**Objectif** : Guide complet pour installer les nouveaux moteurs et rallumer Reachy  
**Note** : ✅ Numéros QC vérifiés : **4x QC 2549** + **1x QC 2548** (batches sains)

---

## 📋 **PRÉPARATION**

### **1. Avant de commencer**

Exécutez le script de vérification :

```bash
python examples/reachy_mini/check_before_motor_installation.py
```

Ce script vérifie :

- ✅ Version SDK
- ✅ Documentation disponible
- ✅ État actuel des moteurs

### **2. Checklist de préparation**

- [x] ✅ **Nouveaux moteurs reçus** :
  - 3 moteurs QC 2549 (1, 2, 4) - 17 janvier 2026
  - 2 moteurs supplémentaires (1x QC 2549 + 1x QC 2548) - 26 janvier 2026
  - **Total** : 5 moteurs (4x QC 2549 + 1x QC 2548)
- [x] ✅ **Numéros QC vérifiés** :
  - **QC 2549** (21 janvier 2026) : Batch sain ✅ - Ne sont PAS QC 2542/2543/2544 (batches problématiques)
  - **QC 2548** (26 janvier 2026) : Nouveau batch, à vérifier
- [ ] **Test mécanique effectué** (chaque moteur doit tourner smooth)
- [ ] **Outils préparés** (tournevis, documentation)
- [ ] **Robot éteint** (éteindre avant de commencer)
- [ ] **Documentation lue** (ce guide + GUIDE_PREVENTION_PROBLEMES_MOTEURS.md)
- [ ] **Photos prises** (photos du câblage actuel pour référence)

---

## 🔧 **INSTALLATION DES MOTEURS**

### **Étape 1 : Éteindre le robot**

```bash
# Sur le robot (SSH ou directement)
sudo shutdown -h now

# OU simplement éteindre l'interrupteur
```

### **Étape 2 : Démonter la tête (si nécessaire)**

Suivez le guide d'assemblage officiel Pollen :

- [GitHub - Guide d'assemblage](https://github.com/pollen-robotics/reachy_mini/blob/develop/docs/platforms/reachy_mini/get_started.md)
- [Hugging Face - Guide interactif](https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide)

### **Étape 3 : Remplacer les moteurs**

#### **Moteur 1 (stewart_1) - Motor ID 11**

1. **Débrancher l'ancien moteur**
   - Débrancher les câbles
   - Retirer le moteur

2. **Vérifier le nouveau moteur**
   - ✅ Numéro QC vérifié (pas QC 2543)
   - ✅ Test mécanique : tourne smooth
   - ✅ Pas de dommages visibles

3. **Installer le nouveau moteur**
   - Brancher dans le slot n°1
   - Vérifier le câblage : Motor 1 → short → Motor 2
   - Vérifier l'alignement mécanique

#### **Moteur 2 (stewart_2) - Motor ID 12**

1. **Débrancher l'ancien moteur**
2. **Vérifier le nouveau moteur** (pas QC 2544)
3. **Installer le nouveau moteur**
   - Brancher dans le slot n°2
   - Câblage : Motor 1 → short → Motor 2 → long → Motor 3

#### **Moteur 4 (stewart_4) - Motor ID 14**

1. **Débrancher l'ancien moteur**
2. **Vérifier le nouveau moteur** (pas QC 2544)
3. **Installer le nouveau moteur**
   - Brancher dans le slot n°4
   - Câblage : Motor 4 → long → Motor 5

### **Étape 4 : Vérifier le câblage**

Vérifiez que tous les câbles sont :

- ✅ Bien branchés (pas de connecteurs lâches)
- ✅ Correctement orientés (pas de torsions)
- ✅ Pas de dommages (isolation intacte)
- ✅ Pas de courts-circuits visibles

**Configuration attendue** :

- Motor 1 (slot 1) → short → Motor 2 (slot 2) → long → Motor 3 (slot 3)
- Motor 4 (slot 4) → long → Motor 5 (slot 5) → short → Motor 6 (slot 6)

### **Étape 5 : Remonter la tête**

Suivez le guide d'assemblage officiel pour remonter la tête.

---

## 🔌 **RALLUMAGE ET VALIDATION**

### **Étape 1 : Allumer le robot**

```bash
# Allumer l'interrupteur
# OU démarrer via SSH si configuré
```

### **Étape 2 : Attendre le démarrage complet**

Attendre que :

- ✅ Le système soit complètement démarré
- ✅ Le daemon soit actif : `sudo systemctl status reachy-mini-daemon`
- ✅ Les LEDs des moteurs soient éteintes (après reflash automatique)

### **Étape 3 : Vérifier le reflash automatique**

Le SDK v1.2.4+ effectue automatiquement un reflash des moteurs lors de :

- La connexion au robot
- Le démarrage du robot
- L'ouverture du dashboard

**Vérifications** :

- ✅ Les LEDs des moteurs sont éteintes (après reflash)
- ✅ Pas d'erreurs dans les logs : `journalctl -u reachy-mini-daemon -f`

### **Étape 4 : Exécuter le script de validation**

```bash
python examples/reachy_mini/validate_motor_installation.py
```

Ce script effectue :

1. ✅ Vérification de la connexion
2. ✅ Scan des moteurs (baudrate et ID)
3. ✅ Test de chaque moteur individuellement
4. ✅ Test des mouvements de la tête
5. ✅ Vérification du reflash automatique
6. ✅ Génération d'un rapport complet

### **Étape 5 : Tests manuels**

#### **Test 1 : Test de chaque moteur**

```python
from reachy_mini import ReachyMini
from reachy_mini.utils import create_head_pose

with ReachyMini() as robot:
    # Test moteur 1
    robot.head.stewart_1.goal_position = 0.5
    time.sleep(1)
    robot.head.stewart_1.goal_position = 0.0
    time.sleep(1)
    
    # Test moteur 2
    robot.head.stewart_2.goal_position = 0.5
    time.sleep(1)
    robot.head.stewart_2.goal_position = 0.0
    time.sleep(1)
    
    # Test moteur 4
    robot.head.stewart_4.goal_position = 0.5
    time.sleep(1)
    robot.head.stewart_4.goal_position = 0.0
    time.sleep(1)
```

#### **Test 2 : Test des mouvements de la tête**

```python
with ReachyMini() as robot:
    # Mouvement vers le haut
    robot.goto_target(
        head=create_head_pose(z=10, degrees=True, mm=True),
        duration=1.0,
    )
    time.sleep(2)
    
    # Mouvement vers le bas
    robot.goto_target(
        head=create_head_pose(z=-10, degrees=True, mm=True),
        duration=1.0,
    )
    time.sleep(2)
    
    # Retour à la position neutre
    robot.goto_target(
        head=create_head_pose(z=0, roll=0, degrees=True, mm=True),
        duration=1.0,
    )
```

#### **Test 3 : Vérifier via l'API**

```bash
# Diagnostic via API
curl http://localhost:8000/api/motors/diagnostic

# OU via le dashboard
# Ouvrir http://localhost:8000 dans le navigateur
# Note: URLs locales (localhost) acceptées par le linter
```

---

## ✅ **VALIDATION FINALE**

### **Checklist de validation**

- [ ] ✅ Tous les moteurs détectés (scan OK)
- [ ] ✅ Tous les moteurs répondent (pas d'erreurs)
- [ ] ✅ Tous les moteurs bougent smooth (pas de saccades)
- [ ] ✅ Pas de LEDs rouges clignotantes
- [ ] ✅ Mouvements de la tête fluides
- [ ] ✅ Pas d'erreurs dans les logs
- [ ] ✅ Script de validation passe (validate_motor_installation.py)

### **Si tout est OK**

✅ **Installation réussie !**

**Prochaines étapes** :

1. Continuer à surveiller les moteurs (voir GUIDE_PREVENTION_PROBLEMES_MOTEURS.md)
2. Effectuer des tests réguliers (quotidien, hebdomadaire)
3. Mettre à jour le SDK vers v1.2.11 (recommandé, non critique)

### **Si problème détecté**

❌ **Actions à prendre** :

1. **Vérifier le câblage**
   - Tous les câbles bien branchés?
   - Pas de connecteurs lâches?
   - Pas de dommages aux câbles?

2. **Vérifier les logs**

   ```bash
   journalctl -u reachy-mini-daemon -f
   ```

3. **Relancer le scan**

   ```bash
   python examples/reachy_mini/scan_motors_baudrate.py
   ```

4. **Consulter le guide de troubleshooting**
   - `docs/hardware/PROBLEME_MOTEURS_QC_BATCH_DEC2025.md`
   - Documentation officielle Pollen

5. **Contacter Pollen Robotics si nécessaire**
   - Email : [sales@pollen-robotics.com](mailto:sales@pollen-robotics.com)
   - Discord : [Pollen Robotics Discord](https://discord.gg/pollen-robotics)

---

## 📚 **RESSOURCES**

### **Documentation BBIA**

- `GUIDE_PREVENTION_PROBLEMES_MOTEURS.md` - Guide de prévention complet
- `PROBLEME_MOTEURS_QC_BATCH_DEC2025.md` - Historique des problèmes
- `SUIVI_COMMUNICATION_POLLEN.md` - Communication avec Pollen

### **Documentation Officielle Pollen**

- Guide d'assemblage : [GitHub](https://github.com/pollen-robotics/reachy_mini/blob/develop/docs/platforms/reachy_mini/get_started.md)
- Guide interactif : [Hugging Face](https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide)
- Documentation SDK : [Docs Pollen](https://docs.pollen-robotics.com/)

### **Scripts Utiles**

- `check_before_motor_installation.py` - Vérification avant installation
- `validate_motor_installation.py` - Validation après installation
- `scan_motors_baudrate.py` - Scan des moteurs

---

## 🎯 **RÉSUMÉ**

1. **Préparation** : Exécuter `check_before_motor_installation.py`
2. **Installation** : Suivre les étapes ci-dessus
3. **Rallumage** : Allumer le robot et attendre le démarrage
4. **Validation** : Exécuter `validate_motor_installation.py`
5. **Tests** : Effectuer les tests manuels
6. **Surveillance** : Continuer à surveiller selon le guide de prévention

**Tout doit être nickel !** ✅

---

**Dernière mise à jour** : 21 Janvier 2026  
**Statut** : ✅ **PRÊT POUR INSTALLATION**
