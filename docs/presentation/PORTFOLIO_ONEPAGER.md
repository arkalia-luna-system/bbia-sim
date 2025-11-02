# 🤖 BBIA - Moteur Cognitif pour Robot Reachy Mini

## 📋 **Résumé exécutif**

**BBIA** est un moteur cognitif Python pour robot Reachy Mini Wireless, intégrant la simulation MuJoCo, une intelligence artificielle légère et un contrôle unifié via `RobotAPI`. Le projet fournit une plateforme pour développer, tester et déployer des comportements robotiques.

---

## 🎯 **Valeur proposée**

### **🔬 Innovation technique**
- **RobotAPI unifié** : interface abstraite pour simulation (MuJoCo) et robot réel
- **IA légère** : Whisper STT + YOLOv8n + MediaPipe pour interactions naturelles
- **Golden tests** : validation de non-régression avec traces de référence
- **Dashboard web** : interface de contrôle temps réel via WebSocket

### **🛡️ Sécurité et fiabilité**
- **Limites de sécurité** : clamp automatique à 0.3 rad, joints interdits centralisés
- **Tests automatisés** : suite validée en CI (800+ tests)
- **CI/CD** : pipeline GitHub Actions avec artefacts et validation
- **Déterminisme** : graine fixée (SEED=42) pour reproductibilité

### **🚀 Facilité d’usage**
- **Scripts one‑click** : démo simulation et robot réel en une commande
- **Documentation** : guides développeur, architecture, contrats
- **Backend flexible** : bascule simulation ↔ robot réel sans modification du code

---

## 📊 **Métriques clés**

| Métrique | Valeur | Statut |
|----------|--------|--------|
| **Tests** | 800+ (CI) | OK |
| **Couverture** | Validée en CI | OK |
| **Modules BBIA** | 7 modules | OK |
| **Backends** | MuJoCo + Reachy mock | OK |
| **Golden traces** | 3 références | OK |

---

## 🏗️ **Architecture technique**

### **🎭 Vertical slices (4 fonctionnalités)**
1. **Émotion → Pose** : Mapping émotions vers mouvements joints
2. **Voix → Action** : Commandes vocales via Whisper STT
3. **Vision → Suivi** : Détection objets/visages avec YOLOv8n + MediaPipe
4. **Comportement → Scénario** : Scripts comportementaux complexes

### **🔧 Composants principaux**
- **`RobotAPI`** : Interface abstraite unifiée
- **`MuJoCoBackend`** : Implémentation simulation MuJoCo
- **`ReachyBackend`** : Implémentation robot réel (mock)
- **`mapping_reachy.py`** : Source de vérité joints/limites
- **`hardware_dry_run.py`** : Validation matériel avec artefacts

### **🧪 Système de tests**
- **Tests Unitaires** : Validation composants individuels
- **Tests d'Intégration** : Validation vertical slices
- **Golden Tests** : Validation non-régression avec traces
- **Tests Hardware** : Validation latence et limites sécurité

---

## 🎬 **Démos disponibles**

### **🎥 Démo simulation (3D)**
```bash
# Démo complète avec viewer MuJoCo
bash scripts/run_demo_sim.sh happy 15

# Résultat : Trace + rapport + vidéo 3D
```

### **🤖 Démo robot réel**
```bash
# Démo robot réel (headless)
bash scripts/run_demo_real.sh excited 20

# Résultat : Trace + rapport + comparaison référence
```

### **🎤 Démo IA légère**
```bash
# Test commandes vocales
python scripts/stt_demo.py --command "salue" --backend mujoco

# Test microphone
python scripts/stt_demo.py --test-microphone --lang fr
```

### **🌐 Dashboard web et API publique**
```bash
# Interface web temps réel (dashboard)
python src/bbia_sim/dashboard_advanced.py --port 8000

# Démarrer l’API publique (mode dev)
python deploy/public_api.py --dev
```

---

## 🛠️ **Installation rapide**

### **Prérequis**
- Python 3.11+
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

### **Première démo**
```bash
# Démo simulation 3D
bash scripts/run_demo_sim.sh happy 10

# Démo robot réel
bash scripts/run_demo_real.sh excited 15
```

---

## 🔗 **Ressources**

### **📚 Documentation**
- **README** : Guide principal et quickstart
- **Architecture** : `docs/architecture/ARCHITECTURE.md`
- **Contrat RobotAPI** : `docs/references/CONTRACT.md`
- **Guide de Test** : `docs/guides_techniques/TESTING_GUIDE.md`

### **🎯 Scripts utiles**
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

## 🎯 **Prochaines étapes**

### **🚀 Déploiement robot réel**
1. Configuration connexion Reachy SDK
2. Tests hardware dry run sur matériel
3. Validation latence et limites sécurité
4. Déploiement production

### **🧠 Améliorations IA**
1. Entraînement modèles personnalisés
2. Intégration reconnaissance gestes
3. Système de dialogue avancé
4. Apprentissage comportemental

### **🌐 Extensions web**
1. Interface mobile responsive
2. Streaming vidéo temps réel
3. API REST complète
4. Intégration cloud

---

## 📞 **CONTACT & SUPPORT**

> Compatibilité Python et CI
>
> - Python: 3.11+
> - CI: `.github/workflows/ci.yml`
> - Setup rapide:
>   ```bash
>   pyenv install 3.11.9 && pyenv local 3.11.9
>   python -m pip install --upgrade pip
>   pip install -e .
>   ```

- **Repository** : [github.com/arkalia-luna-system/bbia-sim](https://github.com/arkalia-luna-system/bbia-sim)
- **Documentation** : Voir dossier `docs/`
- **Issues** : GitHub Issues pour bugs/features
- **CI/CD** : GitHub Actions pour validation continue

---

**Version** : 1.3.1
**Date** : Oct / Oct / Nov. 20255
**Statut** : Production Ready
**Licence** : MIT
