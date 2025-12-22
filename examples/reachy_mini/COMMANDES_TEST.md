# 🧪 TOUTES LES COMMANDES DE TEST - Reachy Mini

**Date** : 22 Décembre 2025  
**Usage** : Copier-coller les commandes pour tester votre robot

---

## 📋 1. TESTS MOUVEMENT TÊTE

### Test Basique (Tête + Antennes)
```bash
python examples/reachy_mini/minimal_demo.py
```
**Description** : Test basique avec animation tête et antennes  
**Fonctionnalités** :
- Mouvement initial vers position neutre (tête droite)
- Animation sinusoïdale des antennes
- Animation pitch de la tête (haut/bas)

---

### Test Séquences de Mouvements
```bash
python examples/reachy_mini/sequence.py
```
**Description** : Test séquences complètes de mouvements  
**Fonctionnalités** :
- Rotation yaw (gauche/droite)
- Rotation pitch (haut/bas)
- Rotation roll
- Translation verticale (Z)
- Animation antennes
- Mouvements combinés

---

### Test Interpolation (Styles de Mouvement)
```bash
python examples/reachy_mini/goto_interpolation_playground.py
```
**Description** : Test différentes méthodes d'interpolation  
**Fonctionnalités** :
- `linear` : Interpolation linéaire
- `minjerk` : Minimum jerk (défaut, mouvement fluide)
- `ease` : Ease in/out
- `cartoon` : Style cartoon

---

## 📋 2. TESTS CAMÉRA

### Test Caméra + Regard vers Point
```bash
python examples/reachy_mini/look_at_image.py
```
**Description** : Affiche le feed caméra et fait regarder Reachy Mini vers les points cliqués  
**Fonctionnalités** :
- Affichage feed caméra
- Clic souris pour pointer
- Utilisation de `look_at_image()`

**Variantes** :
```bash
# Avec webcam OpenCV
python examples/reachy_mini/look_at_image.py --vision cv2

# Avec vision BBIA (si disponible)
python examples/reachy_mini/look_at_image.py --vision bbia
```

---

## 📋 3. DIAGNOSTIC

### Diagnostic Joints Stewart
```bash
python examples/reachy_mini/diagnostic_stewart.py
```
**Description** : Diagnostic complet des 6 joints stewart (plateforme parallèle)  
**Fonctionnalités** :
- Lecture positions actuelles des 6 joints stewart
- Détection câbles manquants/mal branchés
- Vérification limites officielles
- Analyse symétrie et équilibre
- Test mouvement pour vérifier réponse joints

**Détecte** :
- ✅ Joints dans les limites
- ❌ Joints hors limites (câble mal branché)
- ⚠️ Joints à zéro (câble manquant)
- ⚠️ Déséquilibre (tête penchée)

---

## 📋 4. MOUVEMENTS ENREGISTRÉS

### Test Mouvements Dance
```bash
python examples/reachy_mini/recorded_moves_example.py
```
**Description** : Test mouvements enregistrés (bibliothèque dance)

### Test Mouvements Emotions
```bash
python examples/reachy_mini/recorded_moves_example.py --library emotions
```
**Description** : Test mouvements émotions

**Prérequis** :
```bash
pip install reachy-mini  # SDK officiel requis
```

---

## 📋 5. TESTS AVEC ROBOT PHYSIQUE

**⚠️ IMPORTANT** : Par défaut, tous les scripts sont en mode simulation (`use_sim=True`)

### Pour utiliser le robot physique :

1. **Modifier dans le code** :
   - Changer `use_sim=True` → `use_sim=False`
   - Changer `localhost_only=True` → `localhost_only=False` (si robot en WiFi)

2. **Ou utiliser les scripts déjà configurés** :
   - `minimal_demo.py` : Déjà configuré pour robot physique (`use_sim=False, localhost_only=False`)
   - `diagnostic_stewart.py` : Déjà configuré pour robot physique

---

## 📋 6. ORDRE RECOMMANDÉ DE TEST

### Pour un nouveau robot (première fois) :

1. **Diagnostic joints** :
   ```bash
   python examples/reachy_mini/diagnostic_stewart.py
   ```
   → Vérifier que tous les joints répondent correctement

2. **Test mouvement basique** :
   ```bash
   python examples/reachy_mini/minimal_demo.py
   ```
   → Vérifier que la tête bouge correctement

3. **Test caméra** :
   ```bash
   python examples/reachy_mini/look_at_image.py --vision cv2
   ```
   → Vérifier que la caméra fonctionne

4. **Test séquences** :
   ```bash
   python examples/reachy_mini/sequence.py
   ```
   → Vérifier tous les types de mouvements

---

## 📋 7. CORRECTION TÊTE PENCHÉE

### Script de Correction Automatique

```bash
python examples/reachy_mini/fix_head_tilted.py
```

**Description** : Corrige automatiquement la tête penchée en forçant une position neutre  
**Fonctionnalités** :
- Force la tête en position neutre (droite) avec `np.eye(4)`
- Utilise interpolation `minjerk` (mouvement fluide recommandé)
- Vérifie que la correction a réussi
- Peut être lancé au démarrage pour auto-correction

**💡 Utilisation** :
- Lancer ce script **au démarrage** du robot pour corriger automatiquement
- Ou lancer manuellement si la tête se penche pendant l'utilisation

---

## 📋 8. DÉPANNAGE

### Si la tête reste penchée (bug décembre 2025) :

**⚠️ IMPORTANT: Si la tête est TRÈS penchée ET le moteur 1 ne bouge pas mais devient rigide,**
**c'est le bug du lot décembre 2025. Il faut reconfigurer le moteur AVANT de corriger la position.**

**📖 Guide complet:** Voir `examples/reachy_mini/REFLASH_GUIDE.md` pour la procédure détaillée.

#### 🔧 ÉTAPE 1: Reconfigurer le moteur (si bug décembre 2025)

**Symptômes du bug:**
- Tête très penchée
- Moteur 1 (stewart_1) ne bouge pas mais devient rigide quand alimenté
- Pas de LED rouge qui clignote
- Moteur avec paramètres d'usine (ID=1, baudrate 57,600) au lieu d'être préconfiguré

**⚠️ IMPORTANT: Ce n'est PAS un problème de montage ! C'est un bug logiciel du lot décembre 2025.**

**Solution officielle (Recommandée): Script de reflash**

**Procédure complète:**

1. **Alimenter le robot SANS démarrer le daemon:**
   - Brancher le robot, mettre l'interrupteur sur ON
   - **NE PAS** ouvrir le dashboard
   - **NE PAS** démarrer le daemon (`sudo systemctl stop reachy-mini-daemon`)

2. **Mettre à jour le package:**
   ```bash
   pip install --upgrade reachy-mini
   ```

3. **Lancer le script de reflash:**
   ```bash
   reachy-mini-reflash-motors
   ```
   
   Le script va:
   - Demander si vous avez la version **Lite** ou **Wireless**
   - Détecter automatiquement les ports série (ex: `/dev/ttyUSB0` sur Linux, `COM3` sur Windows)
   - Scanner tous les moteurs
   - Reprogrammer automatiquement ceux qui ont une mauvaise configuration

4. **Redémarrer normalement:**
   - Une fois le reflash terminé, démarrer le daemon
   - Tester les mouvements de la tête

**Alternative: Script wrapper**
```bash
# Si la commande directe ne fonctionne pas, utiliser le wrapper:
python examples/reachy_mini/reflash_motors_simple.py
```

**Alternative: Module Python**
```bash
python -m reachy_mini.tools.reflash_motors
```

**Solution 2: Script de correction manuelle**
```bash
# Script qui utilise les outils du SDK pour reconfigurer le moteur
python examples/reachy_mini/fix_motor_config_december_bug.py

# Avec port série spécifique:
python examples/reachy_mini/fix_motor_config_december_bug.py --serialport /dev/ttyAMA3
```

**Solution 3: Reconfiguration manuelle via SSH (si scripts ne fonctionnent pas)**
1. SSH dans le robot: `ssh pollen@<IP_ROBOT>` ou `ssh root@<IP_ROBOT>`
2. Arrêter le daemon: `sudo systemctl stop reachy-mini-daemon`
3. Scanner le bus Dynamixel sur `/dev/ttyAMA3`:
   - À 1,000,000 baud → tous les moteurs présents sauf ID 13
   - À 57,600 baud → trouver le moteur qui répond en tant qu'ID 1
4. Utiliser `reachy_mini.tools.setup_motor` pour:
   - Désactiver le couple
   - Changer le baudrate de 57,600 → 1,000,000
   - Changer l'ID de 1 → 13
5. Redémarrer le daemon: `sudo systemctl start reachy-mini-daemon`

#### 🔧 ÉTAPE 2: Corriger la position de la tête

**Après avoir reconfiguré le moteur, corriger la position:**
```bash
python examples/reachy_mini/fix_head_tilted.py
```
→ Le script va tenter de redresser la tête avec les corrections appropriées.

#### 🔧 ÉTAPE 3: Si ça ne marche toujours pas

1. **Diagnostic complet** :
   ```bash
   python examples/reachy_mini/diagnostic_stewart.py
   ```
   → Vérifie l'état des joints et détecte les problèmes

2. **Faire une calibration via l'app** :
   - Ouvrir l'application Reachy Mini Control
   - Aller dans les paramètres/calibration
   - Faire une calibration complète

3. **Vérifier la mise à jour du firmware** :
   ```bash
   pip show reachy-mini
   pip install --upgrade reachy-mini
   ```

4. **Consulter le guide officiel** :
   - https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide
   - https://github.com/pollen-robotics/reachy_mini/blob/develop/docs/troubleshooting.md
   - Discord Pollen Robotics: Post épinglé "Head tilted, motor n°1 not moving, but get stiff when powered on, and doesn't blink red - SOLVED"

### Si la caméra ne fonctionne pas :

1. **Test caméra OpenCV** :
   ```bash
   python examples/reachy_mini/look_at_image.py --vision cv2
   ```

2. **Vérifier permissions** (macOS) :
   - Système → Confidentialité → Caméra → Autoriser Terminal/Python

---

## 📋 9. COMMANDES RAPIDES (Copier-Coller)

```bash
# BUG DÉCEMBRE 2025: Reconfigurer moteur (si tête TRÈS penchée + moteur 1 rigide)
python examples/reachy_mini/reflash_motors_simple.py
# OU
python examples/reachy_mini/fix_motor_config_december_bug.py

# Correction tête penchée (APRÈS reconfiguration moteur si nécessaire)
python examples/reachy_mini/fix_head_tilted.py

# Diagnostic complet
python examples/reachy_mini/diagnostic_stewart.py

# Test mouvement tête
python examples/reachy_mini/minimal_demo.py

# Test caméra
python examples/reachy_mini/look_at_image.py --vision cv2

# Test séquences
python examples/reachy_mini/sequence.py

# Test interpolation
python examples/reachy_mini/goto_interpolation_playground.py
```

---

## ✅ RÉSUMÉ

| Test | Commande | Durée |
|------|----------|-------|
| **Correction tête** | `fix_head_tilted.py` | 3-5 sec |
| Diagnostic | `diagnostic_stewart.py` | 1-2 min |
| Mouvement basique | `minimal_demo.py` | Continu |
| Caméra | `look_at_image.py` | Continu |
| Séquences | `sequence.py` | Continu |
| Interpolation | `goto_interpolation_playground.py` | Continu |

---

**💡 Astuce** : Tous les scripts peuvent être arrêtés avec `Ctrl+C`

**📚 Documentation** : Voir `examples/reachy_mini/README.md` pour plus de détails

