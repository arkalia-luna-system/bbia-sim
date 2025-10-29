# 🤖 BBIA - Moteur Cognitif pour Robot Reachy Mini

## 📋 **RÉSUMÉ EXÉCUTIF**

**BBIA** est un moteur cognitif Python avancé pour robot Reachy Mini Wireless, intégrant simulation MuJoCo, intelligence artificielle légère, et contrôle unifié via RobotAPI. Le projet offre une plateforme complète pour développer, tester et déployer des comportements robotiques intelligents.

---

## 🎯 **VALEUR PROPOSÉE**

### **🔬 Innovation Technique**
- **RobotAPI Unifié** : Interface abstraite pour simulation (MuJoCo) et robot réel
- **IA Légère** : Whisper STT + YOLOv8n + MediaPipe pour interactions naturelles
- **Golden Tests** : Système de validation non-régression avec traces de référence
- **Dashboard Web** : Interface de contrôle temps réel via WebSocket

### **🛡️ Sécurité & Fiabilité**
- **Limites de Sécurité** : Clamp automatique à 0.3 rad, joints interdits centralisés
- **Tests Automatisés** : Suite complète validée en CI (800+ tests)
- **CI/CD Robuste** : Pipeline GitHub Actions avec artefacts et validation
- **Déterminisme** : Seed fixé (SEED=42) pour reproductibilité

### **🚀 Facilité d'Usage**
- **Scripts One-Click** : Démo simulation et robot réel en une commande
- **Documentation Complète** : Guides développeur, architecture, contrats
- **Backend Flexible** : Switch simulation ↔ robot réel sans modification code

---

## 📊 **MÉTRIQUES CLÉS**

| Métrique | Valeur | Statut |
|----------|--------|--------|
| **Tests** | 800+ (CI) | ✅ |
| **Couverture** | Validée en CI | ✅ |
| **Modules BBIA** | 7 modules | ✅ |
| **Backends** | MuJoCo + Reachy mock | ✅ |
| **Golden Traces** | 3 références | ✅ |

---

## 🏗️ **ARCHITECTURE TECHNIQUE**

### **🎭 Vertical Slices (4 Fonctionnalités Complètes)**
1. **Émotion → Pose** : Mapping émotions vers mouvements joints
2. **Voix → Action** : Commandes vocales via Whisper STT
3. **Vision → Suivi** : Détection objets/visages avec YOLOv8n + MediaPipe
4. **Comportement → Scénario** : Scripts comportementaux complexes

### **🔧 Composants Principaux**
- **`RobotAPI`** : Interface abstraite unifiée
- **`MuJoCoBackend`** : Implémentation simulation MuJoCo
- **`ReachyBackend`** : Implémentation robot réel (mock)
- **`mapping_reachy.py`** : Source de vérité joints/limites
- **`hardware_dry_run.py`** : Validation matériel avec artefacts

### **🧪 Système de Tests**
- **Tests Unitaires** : Validation composants individuels
- **Tests d'Intégration** : Validation vertical slices
- **Golden Tests** : Validation non-régression avec traces
- **Tests Hardware** : Validation latence et limites sécurité

---

## 🎬 **DÉMOS DISPONIBLES**

### **🎥 Démo Simulation (3D)**
```bash
# Démo complète avec viewer MuJoCo
bash scripts/run_demo_sim.sh happy 15

# Résultat : Trace + rapport + vidéo 3D
```

### **🤖 Démo Robot Réel**
```bash
# Démo robot réel (headless)
bash scripts/run_demo_real.sh excited 20

# Résultat : Trace + rapport + comparaison référence
```

### **🎤 Démo IA Légère**
```bash
# Test commandes vocales
python scripts/stt_demo.py --command "salue" --backend mujoco

# Test microphone
python scripts/stt_demo.py --test-microphone --lang fr
```

### **🌐 Dashboard Web & API Publique**
```bash
# Interface web temps réel (dashboard)
python src/bbia_sim/dashboard_advanced.py --port 8000

# Démarrer l’API publique (mode dev)
python deploy/public_api.py --dev
```

---

## 🛠️ **INSTALLATION RAPIDE**

### **Prérequis**
- Python 3.10+
- macOS/Linux
- 4GB RAM minimum

### **Installation**
```bash
# Clone et setup
git clone https://github.com/arkalia-luna-system/bbia-sim.git
cd bbia-sim
python -m venv venv
source venv/bin/activate
pip install -e .[dev]

# Test installation
python scripts/hardware_dry_run.py --duration 5 --backend mujoco
```

### **Première Démo**
```bash
# Démo simulation 3D
bash scripts/run_demo_sim.sh happy 10

# Démo robot réel
bash scripts/run_demo_real.sh excited 15
```

---

## 🔗 **RESSOURCES**

### **📚 Documentation**
- **README** : Guide principal et quickstart
- **Architecture** : `docs/architecture/ARCHITECTURE.md`
- **Contrat RobotAPI** : `docs/references/CONTRACT.md`
- **Guide de Test** : `docs/guides_techniques/TESTING_GUIDE.md`

### **🎯 Scripts Utiles**
- **`hardware_dry_run.py`** : Test matériel complet
- **`record_demo.sh`** : Enregistrement démo + traces
- **`plot_trace.py`** : Analyse et rapports
- **`stt_demo.py`** : Test commandes vocales

### **🧪 Tests**
```bash
# Tests complets
python -m pytest tests/ -v

# Golden tests uniquement
python -m pytest tests/test_golden_traces.py -v

# Tests headless
python -m pytest tests/ -m "not e2e" -v
```

---

## 🎯 **PROCHAINES ÉTAPES**

### **🚀 Déploiement Robot Réel**
1. Configuration connexion Reachy SDK
2. Tests hardware dry run sur matériel
3. Validation latence et limites sécurité
4. Déploiement production

### **🧠 Améliorations IA**
1. Entraînement modèles personnalisés
2. Intégration reconnaissance gestes
3. Système de dialogue avancé
4. Apprentissage comportemental

### **🌐 Extensions Web**
1. Interface mobile responsive
2. Streaming vidéo temps réel
3. API REST complète
4. Intégration cloud

---

## 📞 **CONTACT & SUPPORT**

- **Repository** : [github.com/arkalia-luna-system/bbia-sim](https://github.com/arkalia-luna-system/bbia-sim)
- **Documentation** : Voir dossier `docs/`
- **Issues** : GitHub Issues pour bugs/features
- **CI/CD** : GitHub Actions pour validation continue

---

**Version** : 1.3.1  
**Date** : Octobre 2025  
**Statut** : Production Ready  
**Licence** : MIT
