# 🔍 Analyse Complète du Repo Officiel Reachy Mini - Janvier 2026

**Date d'analyse** : 26 Janvier 2026  
**Dernière vérification** : 26 Janvier 2026  
**Repo analysé** : https://github.com/pollen-robotics/reachy_mini  
**Dernière version** : v1.2.13 (21 janvier 2026)  
**Version installée BBIA** : 1.2.3  
**Note** : 10 versions de retard (v1.2.3 → v1.2.13)

---

## 📊 **RÉSUMÉ EXÉCUTIF**

### **Versions disponibles**

| Version | Date | Statut | Notes |
|---------|------|--------|-------|
| **v1.2.13** | 21 janvier 2026 | ✅ Latest | Dernière version stable |
| **v1.2.12** | - | ✅ Stable | - |
| **v1.2.11** | 14 janvier 2026 | ✅ Stable | - |
| **v1.2.10** | - | ✅ Stable | - |
| **v1.2.9** | - | ✅ Stable | - |
| **v1.2.8** | - | ✅ Stable | - |
| **v1.2.7** | - | ✅ Stable | - |
| **v1.2.6** | 3 janvier 2026 | ⚠️ Problèmes connus | Crashes, erreurs IK |
| **v1.2.5** | 26 décembre 2025 | ✅ Stable | - |
| **v1.2.4** | 22 décembre 2025 | ✅ Stable | - |
| **v1.2.3** | 18 décembre 2025 | ✅ Stable | Version installée BBIA |

**⚠️ IMPORTANT** : Vous êtes sur v1.2.3, la dernière version stable est v1.2.13. **Mise à jour recommandée** après installation des nouveaux moteurs.

---

## 🔧 **AMÉLIORATIONS CONCERNANT LES MOTEURS**

### **1. Outils de diagnostic moteurs**

**Commits récents** :
- `1c09b712` - Update motors_diagnosis.md
- `bd6fb83a` - Add pictures to motors_diagnosis.md
- `26c71ee0` - Motor diagnosis page using testbench app
- `5726429c` - Add scan motors' baudrate and ID - script and guide

**Nouveaux outils disponibles** :
- **Page de diagnostic moteurs** dans le dashboard
- **Script de scan des moteurs** (baudrate et ID)
- **Documentation améliorée** avec images

### **2. Reflash automatique amélioré**

**Commits récents** :
- `dbc69cdc` - light down the motors LEDs after reflash on startup
- `3567aaa6` - reflash motor on start
- `77f7cd91` - Merge pull request #593 (592-check-operating-mode)
- `e1ed4753` - Change the operating mode while reflashing motors
- `ed76be22` - Add operating mode check to reflash motor script
- `fd891a8f` - reflash motor id tool script
- `feff5bdd` - Document issue with Broken Motor 4 in troubleshooting

**Améliorations** :
- ✅ Reflash automatique au démarrage (déjà dans v1.2.4)
- ✅ Vérification du mode opératoire pendant le reflash
- ✅ LEDs des moteurs éteintes après reflash
- ✅ Outil CLI pour reflasher les moteurs manuellement
- ✅ Documentation du problème Motor 4 dans le troubleshooting

### **3. Scripts et outils**

**Nouveaux scripts disponibles** :
- `reachy-mini-reflash-motors` - Reflash manuel des moteurs
- `scan_motors_baudrate.py` - Scanner les moteurs (baudrate et ID)
- Outils de diagnostic dans le dashboard

**Branches importantes** :
- `592-check-operating-mode` - Vérification mode opératoire
- `reflash_motors_on_start` - Reflash au démarrage
- `reflash_motor_id_script` - Script de reflash

---

## 📦 **NOUVELLES FONCTIONNALITÉS (v1.2.3 → v1.2.13)**

### **v1.2.13** (21 janvier 2026) - Latest

**Améliorations** :
- Fix: Apps not showing as Installed when entry point name differs from Hugging Face space name
- Amélioration messages d'erreur quand Reachy Mini est éteint
- Documentation reflash Pi depuis macOS
- Report gstreamer latency
- Add raw write method
- Documentation Hugging Face améliorée
- Fix MyPy CI

### **v1.2.12** (Non publiée)

**Améliorations** :
- Corrections diverses

### **v1.2.11** (14 janvier 2026)

**Améliorations** :
- Documentation udev rules pour libusb
- Meilleure gestion des exceptions de connexion
- Tests média améliorés
- Support Windows amélioré (GStreamer)

### **v1.2.10**

**Améliorations** :
- Corrections diverses
- Améliorations stabilité

### **v1.2.9**

**Améliorations** :
- Corrections bugs
- Améliorations performance

### **v1.2.8**

**Améliorations** :
- Corrections bugs
- Améliorations stabilité

### **v1.2.7** (Stable)

**Améliorations** :
- Correction radio saccadée
- Améliorations diverses

### **v1.2.6** (⚠️ Problèmes connus)

**Problèmes rapportés** :
- Crashes du démon robot (Rust panic)
- Erreurs IK "Collision detected"
- Pertes de connexion
- Erreurs frontend 404s

**⚠️ NE PAS INSTALLER** cette version si vous avez un robot fonctionnel.

---

## 🔍 **CE QUI VOUS MANQUE**

### **1. Versions SDK**

**Vous avez** : v1.2.3  
**Dernière version** : v1.2.13  
**Différence** : 10 versions de retard (v1.2.3 → v1.2.4 → v1.2.5 → v1.2.6 → v1.2.7 → v1.2.8 → v1.2.9 → v1.2.10 → v1.2.11 → v1.2.12 → v1.2.13)

**Recommandation** : Mettre à jour vers v1.2.13 après installation des nouveaux moteurs.

### **2. Outils de diagnostic**

**Nouveaux outils disponibles** :
- ✅ Script de scan des moteurs (baudrate et ID)
- ✅ Page de diagnostic moteurs dans le dashboard
- ✅ Outil CLI de reflash manuel

**Action** : Installer la dernière version pour avoir accès à ces outils.

### **3. Documentation**

**Nouvelle documentation** :
- ✅ `motors_diagnosis.md` - Guide de diagnostic moteurs
- ✅ Documentation du problème Motor 4 dans troubleshooting
- ✅ Guide scan moteurs (baudrate et ID)

**Action** : Consulter la documentation officielle mise à jour.

---

## 📋 **BRANCHES IMPORTANTES À SURVEILLER**

### **Branches liées aux moteurs**

1. **`592-check-operating-mode`** ✅ Merged
   - Vérification du mode opératoire
   - Important pour éviter les problèmes de moteurs

2. **`reflash_motors_on_start`** ✅ Merged
   - Reflash automatique au démarrage
   - Déjà dans v1.2.4+

3. **`reflash_motor_id_script`** ✅ Merged
   - Script de reflash manuel
   - Utile pour dépannage

4. **`700-propagate-motor-controller-stats-to-backendstatus`** ✅ Merged
   - Statistiques moteurs dans le backend
   - Utile pour monitoring

### **Branches de features**

- `383-webrtc-use-rtp-component` - Amélioration WebRTC
- `388-wireless-webrtc-support` - Support WebRTC wireless
- `481-move-media-management-to-daemon-side` - Gestion média côté daemon
- `572-port-gstreamer-backend-to-windows` - Support Windows GStreamer

---

## 🎯 **RECOMMANDATIONS**

### **Court terme (après installation moteurs)**

1. **Mettre à jour vers v1.2.13**
   ```bash
   pip install --upgrade reachy-mini
   ```

2. **Tester les nouveaux outils**
   - Script de scan des moteurs
   - Page de diagnostic dans le dashboard
   - Outil de reflash manuel

3. **Vérifier la documentation**
   - Consulter `motors_diagnosis.md`
   - Vérifier le troubleshooting mis à jour

### **Moyen terme**

1. **Surveiller les nouvelles releases**
   - Vérifier GitHub régulièrement
   - Lire les release notes

2. **Tester en environnement de développement**
   - Avant mise à jour production
   - Vérifier compatibilité BBIA

3. **Documenter les différences**
   - Noter les changements importants
   - Mettre à jour la documentation BBIA

---

## 📚 **RESSOURCES**

- **GitHub officiel** : https://github.com/pollen-robotics/reachy_mini
- **Releases** : https://github.com/pollen-robotics/reachy_mini/releases
- **Documentation** : Voir `/docs` dans le repo
- **Issues** : https://github.com/pollen-robotics/reachy_mini/issues

---

## ⚠️ **IMPORTANT**

- **Ne pas mettre à jour** vers v1.2.6 (problèmes connus)
- **Mettre à jour** vers v1.2.13 après installation des nouveaux moteurs
- **Tester** en environnement de développement avant production
- **Documenter** tout problème rencontré

---

**En résumé** : Vous avez 10 versions de retard (v1.2.3 → v1.2.13). La dernière version (v1.2.13) contient de nombreuses améliorations pour les moteurs, la caméra (calibration améliorée), et la stabilité. Mise à jour recommandée après installation des nouveaux moteurs ! 🚀

---

## 📅 **MISE À JOUR 26 JANVIER 2026**

**Vérification complète effectuée** : 26 Janvier 2026

### **Résultat de la vérification**

✅ **Nouvelle version SDK** : v1.2.13 (21 janvier 2026)  
✅ **Dernier commit** : 26 janvier 2026 (amélioration calibration caméra)  
✅ **Toutes les branches analysées** : develop, main, et branches liées aux moteurs  
✅ **Nouvelles fonctionnalités** : Outils calibration caméra, amélioration WebRTC, support Windows GStreamer

### **Nouvelles informations identifiées (non liées aux moteurs)**

**Applications et IA** :
- Intégration Hugging Face Spaces pour déploiement d'applications
- Nouvelles applications : Hand Tracking, LLM Companion, Dance Dance Mini
- Installation complète : `pip install "reachy-mini[full]"` pour dépendances IA (transformers, tqdm, etc.)

**Firmware** :
- Firmware moteurs basé sur Rust (meilleure précision pour plateforme Stewart 6-DDL)
- Gestion temps réel améliorée

**Note** : Ces informations sont déjà intégrées dans le SDK v1.2.13 et n'affectent pas directement les moteurs ou leur installation.

### **Nouvelles fonctionnalités v1.2.12 → v1.2.13**

**Calibration caméra** (v1.2.13) :
- ✅ **Intégré dans BBIA** : Outils de calibration caméra avec Charuco board
- Scripts disponibles : `acquire.py` (acquisition d'images), `calibrate.py`, `scale_calibration.py`, `visualize_undistorted.py`, `analyze_crop.py`
- Support résolutions multiples avec crop/zoom (facteurs de crop documentés)
- Documentation complète dans `src/bbia_sim/tools/camera_calibration/README.md`
- Amélioration intrinsics caméra avec `scale_intrinsics()` pour résolutions multiples

**Améliorations WebRTC** (v1.2.13) :
- Amélioration gestion résolution caméra dans WebRTC streaming backend
- Support changement résolution dynamique (nécessite fermeture caméra avant changement)
- Fix/Improve camera calibration (#741) - commit 26 janvier 2026

**Documentation** (v1.2.13) :
- Migration documentation vers Hugging Face (https://huggingface.co/docs/reachy_mini/)
- Documentation locale simplifiée dans `docs/source/`
- ✅ **Intégré dans BBIA** : Documentation reflash Pi depuis macOS dans `docs/hardware/REFLASH_PI_MACOS.md`

**Autres améliorations v1.2.13** :
- Fix: Apps not showing as Installed quand nom entry point diffère du nom Hugging Face space
- Amélioration messages d'erreur quand Reachy Mini est éteint
- Report gstreamer latency
- Add raw write method
- Fix MyPy CI
