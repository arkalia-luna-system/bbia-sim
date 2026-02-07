# 🔍 Analyse Complète du Repo Officiel Reachy Mini - Janvier / Février 2026

**Date d'analyse** : 26 Janvier 2026  
**Dernière vérification** : 7 Février 2026  
**Repo analysé** : https://github.com/pollen-robotics/reachy_mini  
**Dernière version** : **v1.3.0** (5 février 2026)  
**Version précédente** : v1.2.13 (21 janvier 2026) — BBIA à jour en v1.3.0 (7 fév. 2026)  
**Version installée BBIA** : **1.3.0** (7 février 2026)  
**Note** : À jour avec Pollen (v1.3.0)

---

## 📊 **RÉSUMÉ EXÉCUTIF**

### **Versions disponibles**

| Version | Date | Statut | Notes |
|---------|------|--------|-------|
| **v1.3.0** | 5 février 2026 | ✅ **Latest** | Dernière version stable (HF auth, WebRTC, SDK simplifié) |
| **v1.2.13** | 21 janvier 2026 | ✅ Stable | - |
| **v1.2.12** | - | ✅ Stable | - |
| **v1.2.11** | 14 janvier 2026 | ✅ Stable | - |
| **v1.2.10** | - | ✅ Stable | - |
| **v1.2.9** | - | ✅ Stable | - |
| **v1.2.8** | - | ✅ Stable | - |
| **v1.2.7** | - | ✅ Stable | - |
| **v1.2.6** | 3 janvier 2026 | ⚠️ Problèmes connus | Crashes, erreurs IK |
| **v1.2.5** | 26 décembre 2025 | ✅ Stable | - |
| **v1.2.4** | 22 décembre 2025 | ✅ Stable | - |
| **v1.2.3** | 18 décembre 2025 | ✅ Stable | - |

**✅ À JOUR** : BBIA utilise **v1.3.0** (5 fév. 2026). Sur le robot physique (Pi), mettre à jour après installation des moteurs : `pip install --upgrade reachy-mini`.

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

## 📦 **NOUVELLES FONCTIONNALITÉS (v1.2.3 → v1.3.0)**

### **v1.3.0** (5 février 2026) - **Latest**

**Changements majeurs** :
- **HF Space auth sur webdashboard** : Authentification Hugging Face sur le dashboard web
- **WebRTC** : Intégration complète en cours pour apps JavaScript pures dans le navigateur
- **SDK simplifié** : Code de contrôle à distance déplacé dans une app (PR #781) — impact possible sur les scripts qui utilisaient le remote côté SDK
- **Documentation** : Ancienne doc supprimée (#754), tout sur Hugging Face
- **Calibration caméra** : Fix/amélioration (#741)
- **App assistant** : Vérifie le layout "src", nom d’app peut différer du nom du package (#763)
- **Création d’apps** : Option `--template conversation` pour forker l’app conversation (#780)
- **Apps forkées** : Noms de package Python uniques pour les apps conversation forkées (#785)
- **Install** : Instructions portaudio (#769), custom install (#705), instructions Ubuntu 22 (#776)
- **Wireless** : Guide développeur wireless (#775)
- **Logs** : Amélioration des logs (#793)
- **HF auth** : Nettoyage auth Hugging Face (#790, #796)
- **WebRTC data** : (#797)

### **v1.2.13** (21 janvier 2026)

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

**Vous avez** : **v1.3.0** (7 fév. 2026) ✅  
**Dernière version** : **v1.3.0** (5 fév. 2026)  
**Différence** : À jour avec Pollen.

**Recommandation** : Sur le robot physique (Pi), exécuter `pip install --upgrade reachy-mini` après installation des moteurs. Vérifier la compatibilité du « remote control » (déplacé dans une app en v1.3.0).

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

1. **Mettre à jour vers v1.3.0**
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
- **Mettre à jour** vers **v1.3.0** après installation des nouveaux moteurs
- **Tester** en environnement de développement avant production (notamment contrôle à distance / remote)
- **Documenter** tout problème rencontré

---

**En résumé** : BBIA est à jour en **v1.3.0** (7 fév. 2026). La dernière version officielle (5 fév. 2026) apporte auth HF sur le dashboard, WebRTC pour apps navigateur, SDK simplifié (remote déplacé en app), et les améliorations moteurs/caméra des v1.2.x. Sur le Pi (robot physique), faire `pip install --upgrade reachy-mini` après installation des moteurs. 🚀

---

## 📅 **MISE À JOUR 7 FÉVRIER 2026**

**Vérification complète effectuée** : 7 Février 2026  
**Dernière vérification repo officiel** : 7 Février 2026

### **Résultat de la vérification**

✅ **Dernière version SDK** : **v1.3.0** (5 février 2026) — HF auth dashboard, WebRTC, SDK simplifié (remote → app)  
✅ **Version précédente** : v1.2.13 (21 janvier 2026)  
✅ **Dernier commit officiel** : 26 janvier 2026 (Fix/Improve camera calibration #741 - `set_resolution()` pour WebRTC)  
✅ **Toutes les branches analysées** : develop, main, et branches liées aux moteurs  
✅ **Nouvelles fonctionnalités** : Outils calibration caméra, amélioration WebRTC (`set_resolution()`), support Windows GStreamer  
✅ **Documentation officielle** : Migration vers Hugging Face (https://huggingface.co/docs/reachy_mini/) - ancienne doc locale supprimée

### **Nouvelles informations identifiées (non liées aux moteurs)**

**Applications et IA** :
- Intégration Hugging Face Spaces pour déploiement d'applications
- Nouvelles applications : Hand Tracking, LLM Companion, Dance Dance Mini
- Installation complète : `pip install "reachy-mini[full]"` pour dépendances IA (transformers, tqdm, etc.)

**Firmware** :
- Firmware moteurs basé sur Rust (meilleure précision pour plateforme Stewart 6-DDL)
- Gestion temps réel améliorée

**Note** : Ces informations sont déjà intégrées dans le SDK v1.2.13 / v1.3.0 et n'affectent pas directement les moteurs ou leur installation.

### **Nouvelles fonctionnalités v1.2.12 → v1.2.13**

**Calibration caméra** (v1.2.13) :
- ✅ **Intégré dans BBIA** : Outils de calibration caméra avec Charuco board
- Scripts disponibles : `acquire.py` (acquisition d'images), `calibrate.py`, `scale_calibration.py`, `visualize_undistorted.py`, `analyze_crop.py`
- Support résolutions multiples avec crop/zoom (facteurs de crop documentés)
- Documentation complète dans `src/bbia_sim/tools/camera_calibration/README.md`
- Amélioration intrinsics caméra avec `scale_intrinsics()` pour résolutions multiples

**Améliorations WebRTC** (v1.2.13) :
- ✅ **Intégré dans BBIA** : Amélioration gestion résolution caméra dans WebRTC streaming backend
- ✅ **Intégré dans BBIA** : Support changement résolution dynamique via `set_resolution()` (nécessite fermeture caméra avant changement)
- ✅ **Intégré dans BBIA** : Fix/Improve camera calibration (#741) - commit 26 janvier 2026
- ✅ **Intégré dans BBIA** : Report gstreamer latency (mesure et reporting latence streaming vidéo)

**Documentation** (v1.2.13) :
- Migration documentation vers Hugging Face (https://huggingface.co/docs/reachy_mini/)
- Documentation locale simplifiée dans `docs/source/`
- ✅ **Intégré dans BBIA** : Documentation reflash Pi depuis macOS dans `docs/hardware/REFLASH_PI_MACOS.md`

**Autres améliorations v1.2.13** :
- ✅ **Intégré dans BBIA** : Fix apps not showing as Installed (vérification par nom space ET entry point)
- ✅ **Intégré dans BBIA** : Amélioration messages d'erreur quand Reachy Mini est éteint (messages plus clairs)
- Report gstreamer latency (non critique pour BBIA - WebSocket utilisé)
- Add raw write method (non critique pour BBIA - méthodes existantes suffisantes)
- Fix MyPy CI (erreurs MyPy dans calibrate.py sont normales - API OpenCV variable selon versions)
