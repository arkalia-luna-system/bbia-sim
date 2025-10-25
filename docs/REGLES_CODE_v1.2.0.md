# 📋 RÈGLES DE CODE BBIA-SIM v1.2.0 ✅ RELEASED

**Version** : 1.2.0 "Reachy-Ready + IA Légère"  
**Objectif** : Démo professionnelle avec robot réel  
**Focus** : Cycles courts, critères d'arrêt mesurables  

## 🎯 **RÈGLES STRATÉGIQUES**

### **✅ Ce qu'on FAIT (Phase 1)**
- **Démo professionnelle** avec robot réel
- **IA légère** : Whisper + YOLOv8n + MediaPipe seulement
- **Cycles courts** : 1-2 semaines max par fonctionnalité
- **Critères d'arrêt** : Tests verts + vidéo + artefacts CI
- **Sécurité maximale** : Kill-switch + validation centralisée

### **❌ Ce qu'on NE FAIT PAS (Phase 1)**
- **IA avancée** : FER deep, TTS émotionnel, DETR
- **Intégrations lourdes** : ROS complet, Unity avancé, Cloud
- **Interface complexe** : App mobile, dashboard avancé
- **Scope creep** : Pas de features supplémentaires

---

## 🔧 **RÈGLES TECHNIQUES**

### **🐍 Environnement Python**
```bash
# TOUJOURS travailler dans le venv
source venv/bin/activate

# Python 3.10+ uniquement
python --version  # Doit être 3.10+

# Dépendances minimales
pip install -r requirements.txt
```

### **🛡️ Sécurité & Validation**
```python
# Amplitude maximale : 0.3 rad
SAFE_AMPLITUDE_LIMIT = 0.3

# Joints interdits (JAMAIS contrôler)
FORBIDDEN_JOINTS = {
    "left_antenna", "right_antenna",
    "passive_1", "passive_2", "passive_3", 
    "passive_4", "passive_5", "passive_6", "passive_7"
}

# Joints sûrs recommandés
SAFE_JOINTS = {"yaw_body"}

# Validation centralisée
robot._validate_joint_pos(joint_name, position)
```

### **🧪 Tests & Qualité**
```bash
# Tests obligatoires avant commit
pytest tests/test_robot_api_smoke.py -v
pytest tests/test_robot_api_limits.py -v
pytest tests/test_golden_traces.py -v

# Linters obligatoires
black . && ruff check . && mypy src/

# Coverage minimum : 68.86%
pytest --cov=src/bbia_sim --cov-report=html
```

---

## 📝 **RÈGLES GIT**

### **🌿 Branches**
```bash
# TOUJOURS travailler sur develop
git checkout develop

# Pas de commits directs sur main
# Utiliser des PRs pour main
```

### **💬 Messages de Commit**
```bash
# JAMAIS de guillemets doubles
git commit -m 'Ajout fonctionnalité X'

# TOUJOURS des guillemets simples
git commit -m 'Fix bug Y'

# Messages en français
git commit -m 'Mise à jour documentation'
```

### **🏷️ Tags & Releases**
```bash
# Tags avec version claire
git tag -a v1.2.0 -m 'Release v1.2.0 - Reachy-Ready + IA Légère'

# Push tags
git push origin v1.2.0
```

---

## 🎭 **RÈGLES ÉMOTIONS**

### **✅ Émotions Valides**
```python
VALID_EMOTIONS = {
    "neutral", "happy", "sad", "angry", 
    "surprised", "confused", "determined", 
    "nostalgic", "proud", "calm", "excited", "curious"
}
```

### **🎯 Mapping Émotions**
```python
# Chaque émotion doit avoir :
- Description claire
- Mapping joints défini
- Intensité 0.0 à 1.0
- Transition fluide
- Validation dans GlobalConfig
```

---

## 🤖 **RÈGLES ROBOTAPI**

### **🔌 Connexion**
```python
# TOUJOURS vérifier la connexion
if not robot.is_connected:
    robot.connect()

# TOUJOURS déconnecter proprement
try:
    # ... code ...
finally:
    robot.disconnect()
```

### **🎮 Contrôle Joints**
```python
# TOUJOURS valider avant de contrôler
is_valid, clamped_pos = robot._validate_joint_pos(joint_name, position)
if not is_valid:
    logger.error(f"Joint invalide: {joint_name}")
    return False

# Utiliser la position clampée
robot.set_joint_pos(joint_name, clamped_pos)
```

### **🔄 Backend Switch**
```python
# Support des deux backends
robot = RobotFactory.create_backend("mujoco")  # Simulation
robot = RobotFactory.create_backend("reachy")  # Robot réel

# Même API pour les deux
robot.set_emotion("happy", intensity=0.8)
```

---

## 🧠 **RÈGLES IA**

### **🎯 IA Légère Seulement**
```python
# ✅ Autorisé :
- Whisper tiny/base (STT)
- YOLOv8n (détection objets)
- MediaPipe (face landmarks)
- pyttsx3 (TTS basique)

# ❌ Interdit (Phase 1) :
- FER deep learning
- TTS émotionnel complexe
- DETR object detection
- Modèles conversationnels
```

### **📊 Performance**
```python
# Latence cible : <40ms
# Précision cible : >90%
# CPU usage : <50%
# Mémoire : <2GB
```

---

## 🖥️ **RÈGLES INTERFACE**

### **🌐 Dashboard Minimal**
```python
# FastAPI + WebSocket + une page
# Émotions courantes seulement
# Contrôle yaw_body
# Log en temps réel
# Pas de configuration complexe
```

### **📱 Pas d'App Mobile**
```python
# Phase 1 : Web uniquement
# Phase 2 : App mobile possible
# Focus sur la démo professionnelle
```

---

## 📚 **RÈGLES DOCUMENTATION**

### **📝 Mise à Jour Obligatoire**
```python
# TOUJOURS mettre à jour :
- README.md
- docs/ROADMAP_STRATEGIQUE_v1.2.0.md
- docs/QUICKSTART_DEVELOPPEURS_v1.2.0.md
- docs/prompts/PROMPT_CURSOR_BBIA_REACHY_v1.2.0.md
```

### **🎯 Style Documentation**
```markdown
# Titres avec emojis
## Sections claires
- Listes à puces
- Code blocks avec syntaxe
- Exemples concrets
- Commandes copier-coller
```

---

## 🚨 **RÈGLES D'URGENCE**

### **🛑 Kill-Switch**
```bash
# Script d'arrêt d'urgence
python scripts/emergency_stop.py

# Bouton STOP matériel (rappel constant)
# Toujours prévoir un bouton STOP physique
```

### **🔍 Debug**
```bash
# Tests rapides
pytest tests/test_robot_api_smoke.py -v

# Logs détaillés
tail -f logs/bbia.log

# Hardware dry run
python scripts/hardware_dry_run.py --backend reachy
```

---

## ✅ **CHECKLIST AVANT COMMIT**

### **🔍 Vérifications Obligatoires**
- [ ] Tests passent : `pytest tests/ -v`
- [ ] Linters passent : `black . && ruff check . && mypy src/`
- [ ] Coverage maintenu : `pytest --cov=src/bbia_sim`
- [ ] Documentation mise à jour
- [ ] Sécurité validée (joints, amplitude)
- [ ] Backend switch fonctionne
- [ ] Golden tests passent

### **📝 Commit**
- [ ] Message en français
- [ ] Guillemets simples seulement
- [ ] Branche develop
- [ ] Pas d'erreurs laissées

---

## 🏆 **CRITÈRES DE SUCCÈS**

### **🎯 Semaine 1 - Reachy-Ready**
- [ ] Reachy SDK installé et fonctionnel
- [ ] Mapping joints physique validé
- [ ] hardware_dry_run.py étendu avec artefacts
- [ ] Vidéo démo MuJoCo 15s

### **🎯 Semaine 2 - IA Légère**
- [ ] Whisper STT intégré (FR/EN)
- [ ] YOLOv8n + MediaPipe fonctionnels
- [ ] Dashboard web minimal opérationnel
- [ ] Tests d'intégration IA passent

### **🎯 Semaine 3 - Polish Démo**
- [ ] Scripts one-click fonctionnels
- [ ] Documentation utilisateur complète
- [ ] Portfolio one-pager créé
- [ ] Release v1.2.0 taggée

---

**Ces règles garantissent la qualité et la sécurité du projet BBIA-SIM v1.2.0. Respectez-les pour une démo professionnelle réussie !** 🚀
