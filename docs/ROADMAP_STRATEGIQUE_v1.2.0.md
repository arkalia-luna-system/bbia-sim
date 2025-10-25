# 🚀 BBIA-SIM v1.1.1 → v1.2.0 - ROADMAP STRATÉGIQUE

**Date** : $(date)  
**Version cible** : v1.2.0 "Reachy-Ready + IA Légère"  
**Objectif** : Démo professionnelle avec robot réel  

## 🎯 **STRATÉGIE VALIDÉE**

### **✅ État Actuel (90-95% du socle)**
- **Base technique pro** : RobotAPI unifiée (sim ↔ réel), 4 vertical slices, démos 3D, golden tests, CI stable, dry-run matériel
- **Valeur immédiate** : Vous voyez le robot, vous validez sans régression, vous êtes prête à brancher le vrai Reachy
- **Architecture solide** : Tests robustes, documentation complète, sécurité centralisée

### **🎯 Objectif Prioritaire : Démo/Proto Professionnel**
**Pourquoi** : Vous avez 90-95% du socle. Convertissez ça en preuve tangible (robot réel) plutôt qu'en features en plus.

### **⏱️ Budget Temps : Cycles Courts**
- **1-2 semaines** par cycle avec critères d'arrêt mesurables
- **Tests verts** + vidéo + artefacts CI
- **Pas de scope creep** : Focus sur la démo professionnelle

### **👥 Cible Utilisateur : Développeurs/Chercheurs**
- **Documentation** : API/CLI, artefacts, one-pager portfolio
- **Grand public** : Plus tard (phase 2)

---

## 🔧 **PLAN TECHNIQUE VALIDÉ**

### **🎯 PRIORITÉ 1 - Pré-Reachy (Critique)**

#### **📦 Installation & Mapping**
```bash
# Actions immédiates :
1. Installer reachy-sdk
2. Mapper les noms/limites dans un seul fichier (évite la dérive)
3. Valider hardware_dry_run.py sur vrai robot
4. Mesurer latence réelle (<40ms target)
```

#### **🛡️ Sécurité Renforcée**
```python
# Kill-switch logiciel :
- Amplitude/clamp global (≤0.3 rad)
- Joints interdits centralisés
- Script "STOP" d'urgence
- Rappel "bouton STOP" matériel
```

#### **📊 Monitoring & Artefacts**
```python
# Étendre hardware_dry_run.py :
- 10 min de test complet
- Export latence CSV + log erreurs
- Artefacts CI automatiques
- Métriques de performance
```

### **🎯 PRIORITÉ 2 - IA Réaliste (Sans Déraper)**

#### **🗣️ Speech-to-Text**
```python
# Whisper tiny/base (CPU ok) :
- Support FR + EN
- Commandes simples : "look_at", "greet", "stop"
- Intégration avec comportements existants
```

#### **👁️ Computer Vision**
```python
# YOLOv8n + MediaPipe :
- Détection objet simple (personne, objet)
- Face landmarks (léger)
- Déclenchement : curious/greeting automatiques
- Assez pour une réaction crédible
```

#### **🔊 Text-to-Speech**
```python
# Garder pyttsx3 pour l'instant :
- TTS émotionnel viendra après
- Focus sur la reconnaissance d'abord
- Qualité suffisante pour démo
```

### **🎯 PRIORITÉ 3 - Dashboard Ultra-Léger**

#### **🖥️ Web UI Minimal**
```python
# FastAPI + WebSocket + une page :
- Émotions courantes (happy, sad, curious)
- Contrôle yaw_body
- Log en temps réel
- But : contrôle devant public + capture simple
```

---

## 📅 **ROADMAP 3 SEMAINES**

### **🗓️ SEMAINE 1 - Reachy-Ready (✅ ACCOMPLI)**
```bash
# Objectifs :
✅ reachy-sdk + mapping joints (fichier unique)
✅ dry-run 10 min + artefacts CI (latence csv/log)
✅ vidéo 15s "happy" (MuJoCo) + graphe qpos (déjà prêt)

# Livrables :
- Script installation Reachy SDK ✅
- Fichier mapping joints physique ✅
- hardware_dry_run.py étendu ✅
- Vidéo démo MuJoCo ✅
```

### **🗓️ SEMAINE 2 - IA Légère Utilisable**
```bash
# Objectifs :
✅ Whisper tiny/base + commandes FR/EN → look_at, greet
✅ YOLOv8n + face landmarks → curious/greeting déclenchés
✅ Web UI minimal (start/stop, combo émotion, logs)

# Livrables :
- Intégration Whisper STT
- Intégration YOLOv8n + MediaPipe
- Dashboard web minimal
- Tests d'intégration IA
```

### **🗓️ SEMAINE 3 - Polish Démo**
```bash
# Objectifs :
✅ Scripts "one-click" (run demo réel, run demo sim)
✅ One-pager PDF + README Quickstart + vidéo intégrée
✅ Tag v1.2.0 (Reachy-ready + IA légère + UI)

# Livrables :
- Scripts de démo automatisés
- Documentation utilisateur
- Portfolio one-pager
- Release v1.2.0
```

---

## 🎨 **FONCTIONNALITÉS CRÉATIVES VALIDÉES**

### **🎭 Nouvelles Émotions (Lisibles Visuellement)**
```python
# Ajouter :
- "calm" (calme, détendu)
- "excited" (excitation, énergie)
- "curious" (curiosité, attention)

# Mapping joints :
- Mouvements subtils mais perceptibles
- Intégration avec comportements existants
```

### **🤝 Comportements Sociaux (Faciles à Percevoir)**
```python
# Comportements simples :
- greeting (salutation)
- look_at_speaker (se tourner vers l'interlocuteur)
- nod (hochement de tête)

# Déclenchement automatique :
- Détection voix → look_at_speaker
- Détection visage → greeting
- Reconnaissance commande → nod
```

### **🎯 Cas d'Usage Idéal v1.2.0**
```python
# Scénario parfait :
"Je parle → il se tourne vers moi → il répond avec une émotion appropriée"

# Séquence :
1. Détection voix (Whisper)
2. Reconnaissance commande
3. Réaction visuelle (look_at_speaker)
4. Réponse émotionnelle appropriée
5. Synthèse vocale (pyttsx3)
```

---

## ⚠️ **RISQUES & GARDE-FOUS**

### **🛡️ Sécurité**
```python
# Clamp centralisé :
- Amplitude ≤ 0.3 rad
- Joints interdits listés
- Script "STOP" d'urgence
- Bouton STOP matériel (rappel constant)
```

### **🔍 Faux Positifs**
```python
# Déjà couverts :
- Golden tests existants
- Ajouter "golden réel" quand Reachy arrive
- Validation contre traces de référence
```

### **📈 Scope Creep IA**
```python
# Limite stricte :
- Whisper + YOLOv8n + face landmarks seulement
- Pas de FER deep, TTS émotionnel, DETR au début
- Focus sur la démo professionnelle
```

### **🌐 Intégrations Externes**
```python
# Phase 2 uniquement :
- ROS/Unity : utiles mais coûteux en intégration
- Cloud : éviter pour la démo (latence)
- Edge computing d'abord
```

---

## 🚫 **CE QU'ON NE FAIT PAS (Phase 1)**

### **❌ IA Avancée**
- FER deep learning
- TTS émotionnel complexe
- DETR object detection
- Modèles conversationnels

### **❌ Intégrations Lourdes**
- ROS Noetic complet
- Unity simulation avancée
- Cloud computing
- Docker/Kubernetes

### **❌ Interface Complexe**
- Application mobile
- Dashboard avancé
- Configuration complexe
- Multi-utilisateurs

---

## ✅ **CRITÈRES DE SUCCÈS**

### **🎯 Semaine 1**
- [ ] Reachy SDK installé et fonctionnel
- [ ] Mapping joints physique validé
- [ ] hardware_dry_run.py étendu avec artefacts
- [ ] Vidéo démo MuJoCo 15s

### **🎯 Semaine 2**
- [ ] Whisper STT intégré (FR/EN)
- [ ] YOLOv8n + MediaPipe fonctionnels
- [ ] Dashboard web minimal opérationnel
- [ ] Tests d'intégration IA passent

### **🎯 Semaine 3**
- [ ] Scripts one-click fonctionnels
- [ ] Documentation utilisateur complète
- [ ] Portfolio one-pager créé
- [ ] Release v1.2.0 taggée

---

## 🏆 **VISION FINALE v1.2.0**

### **🤖 Robot Compagnon Professionnel**
- **Démo crédible** : Robot réel qui réagit intelligemment
- **IA légère** : Reconnaissance voix + vision basique
- **Interface simple** : Contrôle web minimal
- **Sécurité maximale** : Kill-switch + validation centralisée

### **📊 Métriques de Succès**
- **Latence** : <40ms (robot réel)
- **Précision** : >90% (reconnaissance commandes)
- **Stabilité** : 0 crash en 10 min de démo
- **Sécurité** : 0 mouvement dangereux

### **🎯 Cas d'Usage Validé**
```python
# Démo professionnelle :
1. "Bonjour BBIA" → Robot se tourne + "Bonjour !"
2. "Comment ça va ?" → Robot répond + émotion happy
3. "Regarde cette personne" → Robot suit + curious
4. "Au revoir" → Robot salue + "Au revoir !"
```

---

## 📚 **RESSOURCES & RÉFÉRENCES**

### **📖 Documentation Officielle**
- [Reachy SDK Documentation](https://docs.pollen-robotics.com/)
- [Whisper Documentation](https://github.com/openai/whisper)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)
- [MediaPipe Documentation](https://mediapipe.dev/)

### **🔧 Scripts de Référence**
- `scripts/hardware_dry_run.py` - Test hardware
- `scripts/record_trace.py` - Enregistrement traces
- `scripts/validate_trace.py` - Validation traces
- `examples/demo_emotion_fixed.py` - Démo 3D stable

### **📁 Fichiers Clés**
- `src/bbia_sim/robot_api.py` - Interface unifiée
- `src/bbia_sim/global_config.py` - Configuration centralisée
- `src/bbia_sim/bbia_emotions.py` - Système émotions
- `docs/CONTRACT.md` - Contrat RobotAPI

---

**Cette roadmap est votre boussole pour les 3 prochaines semaines. Focus sur la démo professionnelle avec le vrai Reachy !** 🚀
