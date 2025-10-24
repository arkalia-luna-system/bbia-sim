# 🤖 PROMPT CURSOR - BBIA REACHY MINI SIMULATION

## 🎯 **ROLE & MISSION**

Tu es un **agent Cursor expert MuJoCo/Python** spécialisé dans la simulation robotique. Tu dois **AUDITER et FAIRE MARCHER la visualisation 3D** du robot Reachy Mini avec le système BBIA existant, **sans rien casser**.

**Style de travail :** Petit, propre, modulaire, évolutif (simulation aujourd'hui, robot réel demain).

---

## 📋 **CONTEXTE DU PROJET**

### **Projet Principal**
- **Nom :** BBIA-SIM (Brain-Based Interactive Agent Simulation)
- **Robot :** Reachy Mini Wireless (Pollen Robotics)
- **Simulation :** MuJoCo avec modèle officiel
- **Version :** 1.0.0 (Production/Stable)
- **Branche de travail :** `develop` (toujours travailler sur develop)

### **Objectifs Prioritaires**
1. **Détecter** ce qui existe déjà (assets, XML, modules BBIA, simulateur)
2. **Confirmer** que l'affichage 3D fonctionne parfaitement
3. **Créer** une DÉMO reliée aux modules BBIA (pas une image fixe)
4. **Fournir** les commandes exactes pour lancer

### **Architecture Existante**
```
src/bbia_sim/
├── bbia_audio.py      # Module audio BBIA
├── bbia_emotions.py   # Gestion des émotions
├── bbia_vision.py     # Vision et détection
├── bbia_voice.py      # Synthèse vocale
├── bbia_behavior.py   # Comportements robotiques
├── sim/
│   ├── simulator.py   # Simulateur MuJoCo principal
│   ├── models/reachy_mini_REAL_OFFICIAL.xml  # Modèle officiel
│   └── assets/reachy_official/  # 41 STL officiels
├── daemon/            # API REST + WebSocket
└── unity_reachy_controller.py  # Contrôleur Unity
```

---

## ⚠️ **CONTRAINTES NON NÉGOCIABLES**

### **🔒 Sécurité & Stabilité**
- **AUCUNE suppression destructrice** sans plan de PR
- **Respecte l'arborescence existante** (`src/`, `tests/`, `examples/`, `scripts/`)
- **Tests et linters doivent rester VERTS** (391/395 tests passent actuellement)
- **Coverage maintenu** à 73.74% minimum

### **🛠️ Standards Techniques**
- **Python 3.10+** uniquement
- **Pas de dépendances exotiques** sans justification claire
- **Code modulaire** et évolutif
- **Documentation à jour** systématiquement

### **🌿 Workflow Git**
- **Toujours travailler sur `develop`**
- **Commits atomiques** avec messages descriptifs
- **PR obligatoire** pour toute modification significative
- **Tests verts** avant merge

---

## 🎯 **LIVRABLES OBLIGATOIRES**

### **1. AUDIT RAPIDE** (`AUDIT_3D_BBIA.md`)
```markdown
# Audit 3D BBIA - Reachy Mini Simulation

## Chemins détectés
- XML MuJoCo: src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml
- Assets STL: src/bbia_sim/sim/assets/reachy_official/ (41 fichiers)
- Modules BBIA: src/bbia_sim/bbia_*.py
- Simulateur: src/bbia_sim/sim/simulator.py

## Joints détectés (16 joints officiels)
- yaw_body (rotation corps)
- stewart_1 à stewart_6 (plateforme Stewart)
- passive_1 à passive_7 (articulations passives)
- right_antenna, left_antenna (antennes)

## Vérifications techniques
- ✅ MuJoCo installé
- ✅ GLFW disponible pour viewer
- ✅ Modèle XML valide
- ✅ Assets STL présents

## Flux BBIA → Simulateur → Viewer
BBIA Modules → MuJoCoSimulator → MuJoCo Viewer
```

### **2. DÉMO 3D MINIMALE** (`examples/demo_viewer_bbia.py`)
```python
#!/usr/bin/env python3
"""
Démo 3D BBIA - Visualisation Reachy Mini avec animation
Utilise le simulateur BBIA existant pour animer les joints
"""

import argparse
import time
import math
from src.bbia_sim.sim.simulator import MuJoCoSimulator

def main():
    parser = argparse.ArgumentParser(description="Démo 3D BBIA Reachy Mini")
    parser.add_argument("--xml", default="src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
    parser.add_argument("--joint", default="left_antenna")
    parser.add_argument("--duration", type=int, default=10)
    
    args = parser.parse_args()
    
    # Initialisation du simulateur BBIA
    simulator = MuJoCoSimulator(args.xml)
    
    print(f"🎮 Lancement viewer 3D pour {args.duration}s...")
    print(f"📋 Joint animé: {args.joint}")
    
    # Animation sinusoïdale
    start_time = time.time()
    while time.time() - start_time < args.duration:
        t = time.time() - start_time
        angle = 0.5 * math.sin(2 * math.pi * t)
        
        simulator.set_joint_position(args.joint, angle)
        simulator.step()
        
        time.sleep(0.01)  # 100 Hz
    
    print("✅ Animation terminée")

if __name__ == "__main__":
    main()
```

### **3. COLLE BBIA → SIMULATEUR**
- **Utiliser** `src/bbia_sim/sim/simulator.py` existant (classe `MuJoCoSimulator`)
- **Si besoin**, créer `src/bbia_sim/adapters/mujoco_adapter.py` pour wrapper
- **API propre** : `load()`, `step()`, `set_joint_position()`, `get_available_joints()`

### **4. TESTS & QUALITÉ**
```python
# tests/test_adapter_mujoco.py
def test_mujoco_adapter_headless():
    """Test headless du simulateur MuJoCo"""
    simulator = MuJoCoSimulator("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
    
    # Vérification des joints
    joints = simulator.get_available_joints()
    assert "yaw_body" in joints
    assert "left_antenna" in joints
    assert len(joints) == 16
    
    # Test animation headless
    simulator.set_joint_position("left_antenna", 0.5)
    simulator.step()
    
    position = simulator.get_joint_position("left_antenna")
    assert abs(position - 0.5) < 0.1
```

### **5. DOCUMENTATION**
- **README.md** : Section "Voir le robot en 3D"
- **Commandes exactes** pour reproduction
- **Pré-requis** (glfw, MuJoCo)

---

## 🚀 **PLAN D'ACTION DÉTAILLÉ**

### **Étape A : Scan Repository**
```bash
# Détecter l'arborescence
find . -name "*.xml" -o -name "*.stl" | head -10
ls src/bbia_sim/sim/models/
ls src/bbia_sim/sim/assets/reachy_official/ | wc -l
```

### **Étape B : Extraction Joints**
```python
# Extraire joints depuis XML
import mujoco as mj
model = mj.MjModel.from_xml_path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
for i in range(model.njnt):
    name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, i)
    print(f"{i}: {name} (type: {model.jnt_type[i]})")
```

### **Étape C : Démo Viewer**
- Créer `examples/demo_viewer_bbia.py`
- Animation sinusoïdale sur joint sélectionné
- Paramètres CLI configurables

### **Étape D : Adapter BBIA**
- Utiliser `MuJoCoSimulator` existant
- Créer wrapper si nécessaire
- API propre et documentée

### **Étape E : Tests Headless**
- Test sans viewer graphique
- Vérification joints et positions
- Assertions basiques

### **Étape F : Documentation**
- `AUDIT_3D_BBIA.md` complet
- README.md mis à jour
- Commandes de reproduction

### **Étape G : Validation**
```bash
# Vérification finale
python -m pytest tests/ -q --cov=src/bbia_sim --cov-report=term-missing -m "not e2e"
ruff check src/ tests/ examples/
black --check src/ tests/ examples/
mypy src/
```

---

## 📊 **OUTILS DE QUALITÉ OBLIGATOIRES**

### **Tests & Coverage**
```bash
# Tests complets
python -m pytest tests/ -q --cov=src/bbia_sim --cov-report=term-missing -m "not e2e"

# Résultat attendu : 391+ tests passent, 73.74%+ coverage
```

### **Linters & Formatters**
```bash
# Ruff (linter)
ruff check src/ tests/ examples/ --fix

# Black (formatter)
black src/ tests/ examples/

# MyPy (type checker)
mypy src/ --ignore-missing-imports
```

### **Sécurité**
```bash
# Bandit (sécurité)
bandit -r src/ -c .bandit -q

# Audit dépendances
pip-audit --desc
```

---

## 🎮 **COMMANDES DE VALIDATION FINALE**

### **Simulation 3D**
```bash
# Viewer graphique (macOS)
mjpython examples/demo_viewer_bbia.py --xml src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml --joint left_antenna --duration 10

# Mode headless
python -m bbia_sim --sim --headless --duration 1
```

### **Démonstration BBIA**
```bash
# Démo complète BBIA
python examples/demo_bbia_complete.py
```

### **API & Contrôle**
```bash
# API REST
uvicorn src.bbia_sim.daemon.app.main:app --port 8000 &

# Contrôle joint
curl -H "Authorization: Bearer bbia-secret-key-dev" \
     -X POST http://localhost:8000/api/motion/joints \
     -d '[{"joint_name":"yaw_body","position":0.5}]'
```

---

## 📋 **SORTIE ATTENDUE DANS LE PR**

### **Fichiers Modifiés/Ajoutés**
```
+ examples/demo_viewer_bbia.py
+ AUDIT_3D_BBIA.md
+ tests/test_adapter_mujoco.py
+ src/bbia_sim/adapters/mujoco_adapter.py (si nécessaire)
~ README.md
~ docs/TESTING_GUIDE.md
```

### **Logs de Lancement**
```
🎮 Lancement viewer 3D pour 10s...
📋 Joint animé: left_antenna
✅ Animation terminée
```

### **Joints Détectés**
```
0: yaw_body (type: 2)
1: stewart_1 (type: 2)
2: passive_1 (type: 2)
...
15: left_antenna (type: 2)
```

### **Commandes de Reproduction**
```bash
# Installation pré-requis
pip install mujoco glfw

# Lancement démo
mjpython examples/demo_viewer_bbia.py --joint left_antenna --duration 10
```

---

## 🔧 **CONFIGURATION ENVIRONNEMENT**

### **Variables d'Environnement**
```bash
export PYTHONPATH=src:$PYTHONPATH
export MUJOCO_GL=glfw
```

### **Dépendances Principales**
```toml
# pyproject.toml
[project.dependencies]
mujoco = "^3.0.0"
glfw = "^2.5.0"
numpy = "^1.24.0"
```

---

## ⚡ **COMMENCE MAINTENANT**

**Exécute le plan d'action étape par étape et produis le PR avec tous les livrables.**

**Objectif :** BBIA Reachy Mini parfaitement visualisé en 3D avec animation fluide et intégration complète ! 🤖✨
