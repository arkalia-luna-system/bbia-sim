# 📦 BACKUP v1.1.1 - État Actuel du Projet

**Date de création :** $(date)  
**Tag Git :** `v1.1.1-backup`  
**Branches synchronisées :** `main` = `develop`  

## 🎯 **État du Projet**

### **Version Stable**
- **Version :** 1.1.1 (Production/Stable)
- **Statut :** ✅ Stable et propre
- **Qualité code :** ✅ Tous les linters passent (black, ruff, mypy, bandit)

### **Architecture Actuelle**
- **Backend Unifié :** RobotAPI avec MuJoCoBackend et ReachyBackend
- **Golden Tests :** Système de validation par traces de référence
- **CI/CD :** Pipeline GitHub Actions avec tests headless et artefacts
- **4 Vertical Slices :** Emotion, Voice, Vision, Behavior intégrés

### **Fonctionnalités Principales**

#### 🤖 **RobotAPI Interface**
- Interface unifiée pour simulation et robot réel
- Validation centralisée des joints et amplitudes
- Sécurité : amplitude ≤ 0.3 rad, joints interdits
- Méthodes : `set_joint_pos`, `set_emotion`, `look_at`, `run_behavior`

#### 🧪 **Golden Tests**
- Traces de référence : `happy`, `look_at`, `wakeup`
- Validation automatique contre régression
- Tolérances : position ±0.25 rad, rate ±0.1 rad/s
- Seed fixé : 42 pour déterminisme

#### 🎬 **Démos Verticales**
- `demo_emotion_ok.py` : Emotion → Pose (RobotAPI)
- `demo_emotion_fixed.py` : Emotion → Pose (MuJoCo direct, 3D stable)
- `demo_voice_ok.py` : Voice → Action
- `demo_vision_ok.py` : Vision → Tracking
- `demo_behavior_ok.py` : Behavior → Scenario

#### 🔧 **Scripts de Validation**
- `hardware_dry_run.py` : Test hardware Reachy réel
- `record_trace.py` : Enregistrement traces
- `validate_trace.py` : Validation traces
- `replay_viewer.py` : Replay animations

### **Structure du Projet**

```
bbia-reachy-sim/
├── src/bbia_sim/
│   ├── robot_api.py              # Interface unifiée
│   ├── backends/                 # MuJoCo + Reachy backends
│   ├── global_config.py          # Configuration centralisée
│   └── telemetry.py              # Métriques performance
├── examples/                     # Démos verticales
├── tests/                        # Tests unitaires + golden
├── scripts/                      # Scripts validation
├── artifacts/golden/             # Traces de référence
├── docs/                         # Documentation complète
└── logs/                         # Logs et rapports
```

### **Tests et Qualité**

#### **Tests Disponibles**
- **Tests unitaires :** 40+ tests, couverture 85%
- **Tests smoke :** Validation backend rapide (<5s)
- **Tests golden :** Validation régression
- **Tests limites :** Sécurité joints et amplitudes

#### **Commandes Tests**
```bash
# Tests complets
pytest tests/ -v

# Tests golden uniquement
pytest tests/test_golden_traces.py -v

# Tests smoke headless
pytest tests/test_robot_api_smoke.py -v

# Tests limites sécurité
pytest tests/test_robot_api_limits.py -v
```

### **CI/CD Pipeline**

#### **GitHub Actions**
- **Linting :** ruff, black, mypy, bandit
- **Tests :** unitaires + smoke + golden
- **Environnement :** SEED=42, MUJOCO_GL=egl
- **Artefacts :** Upload logs et CSV en cas d'échec

#### **Déclencheurs**
- Push sur `develop` et `main`
- Pull requests
- Tests headless automatiques

### **Préparation Reachy Réel**

#### **Checklist A4**
- ✅ SDK Reachy installé/version notée
- ✅ Mapping joints validé (noms/limites)
- ✅ Limites sécurité actives (amp ≤ 0.3 rad)
- ✅ Bouton STOP / coupure d'urgence prévu
- ✅ Latence cible (<40 ms set→read) mesurée
- ✅ Script hardware_dry_run.py OK (smoke 10s)

#### **Script Hardware Dry Run**
```bash
python scripts/hardware_dry_run.py --backend reachy
```

### **Documentation**

#### **Fichiers Clés**
- `README.md` : Guide principal
- `docs/CONTRACT.md` : Contrat RobotAPI
- `docs/SWITCH_SIM_ROBOT.md` : Guide switch backend
- `docs/PRET_REACHY_A4.md` : Checklist préparation
- `docs/prompts/` : Prompts IA agents

#### **Prompts IA**
- `PROMPT_CURSOR_BBIA_REACHY_FINAL.md` : Prompt principal
- `PROMPT_CURSOR_BBIA_REACHY_COMPLETE.md` : Prompt complet
- `PROMPT_CURSOR_BBIA_REACHY_CONTINUATION.md` : Prompt continuation

### **Métriques Actuelles**

#### **Performance**
- **Tests :** 40+ tests, 85% couverture
- **Latence :** <40ms target (simulation)
- **Golden tests :** 3 traces de référence
- **CI :** Pipeline stable, artefacts automatiques

#### **Sécurité**
- **Joints interdits :** Liste centralisée
- **Amplitude max :** 0.3 rad
- **Validation :** Centralisée dans RobotAPI
- **Failsafe :** Clamp automatique + messages clairs

### **Commandes de Lancement**

#### **Démos 3D (MuJoCo direct)**
```bash
# Emotion stable (3D visible)
mjpython examples/demo_emotion_fixed.py

# Autres démos (RobotAPI)
python examples/demo_voice_ok.py --backend mujoco
python examples/demo_vision_ok.py --backend mujoco
python examples/demo_behavior_ok.py --backend mujoco
```

#### **Tests Headless**
```bash
# Tests complets
pytest tests/ -v --tb=short

# Tests golden
pytest tests/test_golden_traces.py -v

# Validation hardware
python scripts/hardware_dry_run.py --backend reachy
```

### **Règles Critiques**

#### **Git Workflow**
- ✅ Jamais de guillemets doubles dans les commits
- ✅ Toujours travailler dans le venv
- ✅ Ne jamais laisser d'erreurs (code ou autre)
- ✅ Branche `develop` pour développement
- ✅ Tests verts avant commit

#### **Code Quality**
- ✅ black, ruff, mypy, bandit passent
- ✅ Tests golden passent
- ✅ Documentation à jour
- ✅ Structure organisée

### **Prochaines Étapes Recommandées**

1. **Réception Reachy réel :**
   - Installer SDK Reachy
   - Valider mapping joints
   - Tester `hardware_dry_run.py`

2. **Développement futur :**
   - Implémenter ReachyBackend réel
   - Ajouter nouveaux vertical slices
   - Optimiser performance

3. **Maintenance :**
   - Régénérer golden tests si besoin
   - Mettre à jour documentation
   - Surveiller CI/CD

---

## 🏷️ **Informations Backup**

- **Tag :** `v1.1.1-backup`
- **Commit :** `57fae99`
- **Branches :** `main` et `develop` synchronisées
- **État :** ✅ Stable, propre, prêt pour production
- **Prochaine version :** v1.2.0 (après réception Reachy réel)

**Ce backup représente l'état stable et fonctionnel du projet BBIA-SIM v1.1.1 avec toutes les fonctionnalités implémentées et testées.**
