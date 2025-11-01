# 🚀 Scripts BBIA

> **Scripts d'automatisation pour BBIA - Brain-Based Interactive Agent**

## ⚠️ **Scripts Dépréciés**

- ❌ **`start_api.py`** → Utiliser `start_public_api.py` (archivé dans `_archived/`)
- ❌ **`kill_greedy_processes.sh`** → Utiliser `smart_process_cleanup.sh` (archivé dans `_archived/`)
- ⚠️ **`kill_mujoco_viewers.sh`** → Utiliser `process_manager.py stop` (déprécié mais gardé)

### 🔄 **Scripts d'Audit Consolidés** (Octobre 2025)

Les scripts de comparaison/audit avec le SDK officiel Reachy Mini ont été consolidés :

- ✅ **`compare_with_official_exhaustive.py`** → Script principal (amélioré avec fusions)
- ✅ **`check_official_alignment.py`** → Alignement MJCF/STL (conservé)
- ✅ **`generate_conformity_report_reachy_mini.py`** → Génération rapports (conservé)

**Scripts archivés** dans `scripts/_archived/comparison_audit/` :
- 8 scripts redondants ou obsolètes (voir `scripts/_archived/comparison_audit/README.md`)

Voir `scripts/_archived/README.md` et `scripts/PLAN_CONSOLIDATION_AUDIT_SCRIPTS.md` pour plus de détails.

## 🎯 **Scripts Disponibles**

### 🎮 **Scripts Principaux**
- **`quick_start.sh`** - Menu interactif principal
- **`launch_unity.sh`** - Lancement de la simulation Unity 3D
- **`launch_robot_3d.sh`** - Lancement du robot Reachy Mini en 3D
- **`launch_robot.py`** - Script Python pour lancer le robot
- **`install_all_reachy_repos.sh`** - Installation automatique des dépôts GitHub
- **`setup_reachy_environment.sh`** - Configuration de l'environnement

### 🧪 **Scripts de Test**
- **`test_unity_setup.sh`** - Test de la configuration Unity
- **`fix_unity_warnings.sh`** - Correction des avertissements Unity
- **`hardware_dry_run.py`** - Validation hardware Reachy réel ✅
- **`record_trace.py`** - Enregistrement traces golden
- **`validate_trace.py`** - Validation traces contre référence

### 🎬 **Scripts de Démo (NOUVEAUX)**
- **`record_demo.sh`** - Enregistrement démo complète ✅
- **`plot_trace.py`** - Génération rapports d'analyse ✅

---

## 🚀 **Utilisation Rapide**

### 🎮 **Menu Interactif (Recommandé)**
```bash
./scripts/quick_start.sh
```
**Options disponibles :**
- Option 1 : Tester BBIA (simulation rapide)
- Option 6 : Lancer Unity 3D
- Option 7 : Tester la configuration Unity
- Option 8 : Corriger les avertissements Unity
- Option 10 : Installer dépôts GitHub

### 🧪 **Scripts de Validation**

#### **Hardware Dry Run**
```bash
# Test hardware complet (10s)
python scripts/hardware_dry_run.py --duration 10

# Test joint spécifique
python scripts/hardware_dry_run.py --joint yaw_body --duration 5
```

**Résultat attendu** :
```
✅ Robot Reachy connecté avec succès
✅ Tous les joints de test sont disponibles
✅ Limite d'amplitude respectée
✅ Joint interdit correctement rejeté
⏱️ Latence moyenne: 0.0ms
✅ Latence cible atteinte (<40ms)
🎉 Hardware dry run réussi !
```

#### **Scripts de Démo**
```bash
# Enregistrer une démo complète
bash scripts/record_demo.sh happy 15

# Générer un rapport d'analyse
python scripts/plot_trace.py --input assets/videos/demo_happy_*.jsonl --output assets/plots/rapport.txt
```

#### **Golden Tests**
```bash
# Enregistrer une trace de référence
python scripts/record_trace.py --emotion happy --duration 5 --out artifacts/golden/happy_mujoco.jsonl

# Valider une trace contre référence
python scripts/validate_trace.py --ref artifacts/golden/happy_mujoco.jsonl --cur current_trace.jsonl
```

**Résultat attendu** :
```
✅ Validation réussie
📊 Métriques:
   • Max abs qpos err: 0.1234
   • Ref hz: 60.0
   • Cur hz: 59.8
   • Diff rate percent: 0.3
   • Frames compared: 300
```

### 🤖 **Lancement Robot 3D**
```bash
# Script bash (recommandé)
./scripts/launch_robot_3d.sh

# Script Python
python scripts/launch_robot.py
```
**Fonctionnalités :**
- Robot Reachy Mini complet en 3D
- Mode graphique et headless
- Détection automatique macOS/Linux
- Test rapide disponible

### 🎯 **Lancement Unity**
```bash
./scripts/launch_unity.sh
```
**Fonctionnalités :**
- Détection automatique du projet Unity
- Lancement via Unity Hub
- Gestion des erreurs

### 📚 **Installation Dépôts**
```bash
./scripts/install_all_reachy_repos.sh
```
**Dépôts installés :**
- reachy-docs
- pollen-vision
- reachy2-tutorials
- reachy-dashboard
- reachy-face-tracking
- reachy2-behaviors-dev
- reachy2-sdk-audio-server-rs
- reachy-unity-package

### 🔧 **Configuration Environnement**
```bash
./scripts/setup_reachy_environment.sh
```
**Configuration :**
- Installation des dépendances Python
- Configuration de l'environnement virtuel
- Vérification des installations

---

## 🧪 **Tests et Dépannage**

### 🔍 **Test Unity**
```bash
./scripts/test_unity_setup.sh
```
**Vérifications :**
- Présence d'Unity Hub
- Configuration du projet Unity
- Permissions des fichiers

### 🔧 **Correction Avertissements**
```bash
./scripts/fix_unity_warnings.sh
```
**Corrections :**
- Mise à jour des packages Unity
- Correction des conflits de versions
- Optimisation des paramètres

---

## 📁 **Structure des Scripts**

```
scripts/
├── 🎮 quick_start.sh                              # Menu interactif principal
├── 🎯 launch_unity.sh                             # Lancement Unity 3D
├── 📚 install_all_reachy_repos.sh                # Installation dépôts GitHub
├── 🔧 setup_reachy_environment.sh                # Configuration environnement
├── 🧪 test_unity_setup.sh                        # Test configuration Unity
├── 🔧 fix_unity_warnings.sh                      # Correction avertissements Unity
├── 🔍 compare_with_official_exhaustive.py        # Comparaison exhaustive BBIA vs SDK officiel ✅
├── 🔍 check_official_alignment.py                # Vérification alignement MJCF/STL ✅
├── 📊 generate_conformity_report_reachy_mini.py   # Génération rapports conformité ✅
├── _archived/
│   ├── comparison_audit/                        # Scripts d'audit archivés (8 scripts)
│   └── start_api.py                              # Script API obsolète
└── 📖 README.md                                  # Ce fichier
```

---

## 🎯 **Détails des Scripts**

### 🎮 **quick_start.sh**
**Fonction :** Menu interactif principal avec toutes les options
**Utilisation :** Point d'entrée principal pour BBIA
**Options :** 10 options différentes pour toutes les fonctionnalités

### 🎯 **launch_unity.sh**
**Fonction :** Lancement de la simulation Unity 3D
**Détection :** Automatique du dossier `reachy-bbia-unity`
**Lancement :** Via Unity Hub avec gestion d'erreurs

### 📚 **install_all_reachy_repos.sh**
**Fonction :** Installation automatique de tous les dépôts GitHub
**Dépôts :** 8 dépôts officiels de Pollen Robotics
**Vérification :** Test automatique après installation

### 🔧 **setup_reachy_environment.sh**
**Fonction :** Configuration complète de l'environnement
**Dépendances :** Installation des packages Python
**Vérification :** Test de l'installation

### 🧪 **test_unity_setup.sh**
**Fonction :** Test de la configuration Unity
**Vérifications :** Unity Hub, projet, permissions
**Rapport :** État détaillé de la configuration

### 🔧 **fix_unity_warnings.sh**
**Fonction :** Correction des avertissements Unity
**Corrections :** Packages, versions, paramètres
**Optimisation :** Performance et stabilité

---

## 🎯 **Commandes Rapides**

### 🚀 **Démarrage Immédiat**
```bash
# Menu interactif
./scripts/quick_start.sh

# Unity 3D
./scripts/launch_unity.sh

# Installation complète
./scripts/install_all_reachy_repos.sh
```

### 🔍 **Tests et Vérifications**
```bash
# Test Unity
./scripts/test_unity_setup.sh

# Correction avertissements
./scripts/fix_unity_warnings.sh

# Configuration environnement
./scripts/setup_reachy_environment.sh
```

---

## 💡 **Conseils d'Utilisation**

1. **Commencez par le menu** : `./scripts/quick_start.sh`
2. **Utilisez Unity** : `./scripts/launch_unity.sh`
3. **Installez les dépôts** : `./scripts/install_all_reachy_repos.sh`
4. **Testez la configuration** : `./scripts/test_unity_setup.sh`
5. **Corrigez les problèmes** : `./scripts/fix_unity_warnings.sh`

---

## 🎯 **Dépannage**

### ❌ **Problèmes Courants**
- **Permission denied** : `chmod +x scripts/*.sh`
- **Unity non trouvé** : Vérifiez l'installation d'Unity Hub
- **Dépôts non installés** : Lancez `install_all_reachy_repos.sh`
- **Erreurs Python** : Lancez `setup_reachy_environment.sh`

### ✅ **Solutions**
- **Tous les scripts** sont testés et fonctionnels
- **Gestion d'erreurs** intégrée dans chaque script
- **Vérifications automatiques** après chaque installation
- **Documentation** complète pour chaque script

---

**BBIA** - Brain-Based Interactive Agent  
*Scripts d'automatisation* 🚀✨

**Version** : 2.0  
**Date** : 15 octobre 2025  
**Scripts** : ✅ 6 scripts fonctionnels  
**Automatisation** : ✅ Complète 