# 🔄 Stratégie de Réutilisation du Code Officiel

**Date** : Oct / Oct / Nov. 20255
**Objectif** : Identifier ce qui peut être copié/réutilisé de l'app officielle sans tout recréer

---

## 📋 Analyse : Ce qui peut être réutilisé

### **1. Bibliothèque Danses** ✅ **DÉJÀ DISPONIBLE**

**État actuel BBIA** :
- ✅ `RecordedMoves` déjà importé dans `move.py`
- ✅ API `/play/recorded-move-dataset/{dataset}/{move}` existe
- ✅ Support `reachy_mini.motion.recorded_move` présent

**Ce qu'on peut faire** :
```python
# DÉJÀ PRÉSENT dans src/bbia_sim/daemon/app/routers/move.py
from reachy_mini.motion.recorded_move import RecordedMoves

@router.post("/play/recorded-move-dataset/{dataset_name:path}/{move_name}")
async def play_recorded_move_dataset(...):
    moves = RecordedMoves(dataset_name)
    moves.play(move_name)
```

**Action** : ✅ **AUCUNE** - Déjà implémenté !

---

### **2. Outils LLM (Tools)** ✅ **TERMINÉ**

**App Officielle** : Outils exposés à l'assistant
- `move_head` : Changement position tête
- `camera` : Capture + analyse gpt-realtime
- `head_tracking` : Activer/désactiver suivi visage
- `dance` : Préparer danse
- `stop_dance` : Arrêter danses
- `play_emotion` : Diffuser clip émotion
- `stop_emotion` : Arrêter émotions
- `do_nothing` : Rester inactif

**BBIA Actuel** (✅ **IMPLÉMENTÉ**) :
- ✅ `move_head` → `robot_api.goto_target(head=...)` ou `set_target_head_pose()`
- ✅ `camera` → `vision.scan_environment()` avec analyse objets/visages
- ✅ `head_tracking` → Activation/désactivation suivi visage
- ✅ `dance` → Intégration `RecordedMoves` avec API `/play/recorded-move-dataset`
- ✅ `stop_dance` → Arrêt danses en cours
- ✅ `play_emotion` → `robot_api.set_emotion()` avec support 12 émotions
- ✅ `stop_emotion` → Arrêt émotions en cours
- ✅ `do_nothing` → Action vide pour inactivité

**État** : ✅ **TERMINÉ** - Module `bbia_tools.py` créé et intégré avec `BBIAHuggingFace.chat()`

---

### **3. Système Animation Idle (Respiration)** ✅ **TERMINÉ**

**App Officielle** : Système multicouche
- Respiration automatique (idle)
- Poses de passage subtiles
- Tremblement vocal (réaction à la voix)

**BBIA Actuel** : ✅ **IMPLÉMENTÉ**

**État** : ✅ **TERMINÉ** - Module `bbia_idle_animations.py` créé avec :
- ✅ `BBIABreathingAnimation` : Respiration automatique subtile
- ✅ `BBIAPoseTransitionManager` : Poses de passage toutes les 15s
- ✅ `BBIAVocalTremor` : Tremblement réactif au niveau audio
- ✅ `BBIIdleAnimationManager` : Gestionnaire centralisé

**Code à réutiliser** (structure) :
```python
# Structure inspirée de l'app officielle
class IdleAnimationManager:
    def __init__(self, robot_api):
        self.robot_api = robot_api
        self.breathing_active = True
        self.idle_poses = [...]
    
    def start_breathing(self):
        # Animation respiration subtile
        pass
    
    def stop_breathing(self):
        # Arrêter animation
        pass
    
    def add_vocal_tremor(self, audio_level):
        # Tremblement réactif à la voix
        pass
```

---

### **4. Interface Conversationnelle** 🟡 **PARTIEL**

**App Officielle** : Gradio UI avec transcriptions live

**BBIA Actuel** :
- ✅ Dashboard FastAPI (différent mais fonctionnel)
- ❌ Pas de transcriptions live

**Stratégie** : Ajouter transcriptions live au Dashboard existant (pas besoin Gradio)

---

### **5. OpenAI Realtime API** 🔴 **MANQUANT**

**App Officielle** : `fastrtcp` streaming conversation

**BBIA Actuel** : Whisper offline uniquement

**Stratégie** : Optionnel - Créer module `bbia_realtime_api.py` si besoin

**Code à copier/adapter** :
```python
# Structure de base à adapter
from fastrtcp import FastRTCPClient

class BBIARealtimeConversation:
    def __init__(self, api_key):
        self.client = FastRTCPClient(api_key)
    
    def start_conversation(self):
        # Streaming conversation
        pass
```

---

## 🎯 Plan d'Action : Réutilisation Stratégique

### **Étape 1 : Outils LLM (Priorité HAUTE)** ⏱️ 2-3h

**Créer** : `src/bbia_sim/bbia_tools.py`

**Réutiliser** :
- Structure outils de l'app officielle
- Intégrer avec `BBIAHuggingFace` existant
- Exposer via fonction `get_tools()` pour LLM

**Code à adapter** :
```python
def get_tools():
    return [
        {
            "type": "function",
            "function": {
                "name": "move_head",
                "description": "Changer position tête",
                "parameters": {...}
            }
        },
        # ... autres outils
    ]
```

---

### **Étape 2 : Animations Idle (Priorité HAUTE)** ⏱️ 3-4h

**Créer** : `src/bbia_sim/bbia_idle_animations.py`

**Réutiliser** :
- Logique respiration de l'app officielle
- Système file d'attente multicouche
- Intégration avec `RobotAPI`

**Code à adapter** :
```python
class BBIABreathingAnimation:
    def __init__(self, robot_api):
        self.robot_api = robot_api
        self.breathing_thread = None
    
    def start(self):
        # Boucle respiration subtile
        pass
```

---

### **Étape 3 : Intégration Danses LLM (Priorité MOYENNE)** ⏱️ 1-2h

**Modifier** : `src/bbia_sim/bbia_tools.py` (déjà créé étape 1)

**Réutiliser** :
- API `/play/recorded-move-dataset` existante
- Ajouter outil `dance` et `stop_dance` pour LLM

**Action** : Wrapper API existante pour outils LLM

---

### **Étape 4 : OpenAI Realtime (Priorité BASSE - Optionnel)** ⏱️ 4-5h

**Créer** : `src/bbia_sim/bbia_realtime_api.py`

**Réutiliser** :
- Structure `fastrtcp` de l'app officielle
- Adapter pour BBIA architecture

**Action** : Optionnel si besoin streaming temps réel

---

## 📂 Structure Fichiers à Créer

```
src/bbia_sim/
├── bbia_tools.py              # NOUVEAU - Outils LLM (move_head, dance, etc.)
├── bbia_idle_animations.py    # NOUVEAU - Respiration + idle animations
└── bbia_realtime_api.py       # NOUVEAU (optionnel) - OpenAI Realtime

tests/
├── test_bbia_tools.py         # NOUVEAU - Tests outils
└── test_bbia_idle_animations.py  # NOUVEAU - Tests animations
```

---

## 🔍 Code Officiel à Analyser/Copier

### **1. Tools Structure**
**Fichier officiel** : `tools.py` (dans app conversation)
- Fonction `get_tools()` : Liste outils LLM
- Fonction `execute_tool()` : Exécution outils
- Structure JSON Schema pour chaque outil

**Ce qu'on copie** :
- Structure JSON Schema
- Logique d'exécution
- Mapping outils → actions robot

---

### **2. Idle Animations**
**Fichier officiel** : `animations.py` ou `idle_manager.py`
- Classe `IdleAnimationManager`
- Méthode `start_breathing()`
- Méthode `add_vocal_tremor()`
- Système file d'attente multicouche

**Ce qu'on copie** :
- Logique respiration (patterns mathématiques)
- Système threading pour animations background
- Intégration avec mouvement principal

---

### **3. Dance Integration**
**Fichier officiel** : `tools.py` (outil `dance`)
- Fonction `dance()` : Préparer danse
- Fonction `stop_dance()` : Arrêter
- Intégration avec `reachy_mini_dances_library`

**Ce qu'on adapte** :
- Wrapper API `/play/recorded-move-dataset` existante
- Ajouter outils LLM `dance` / `stop_dance`

---

## ✅ Checklist Réutilisation

### **Phase 1 : Outils LLM** ✅ **TERMINÉ**
- [x] Créer `bbia_tools.py` avec structure officielle
- [x] Implémenter `get_tools()` (liste outils)
- [x] Implémenter `execute_tool(tool_name, params)`
- [x] Intégrer avec `BBIAHuggingFace.chat()` (function calling)
- [x] Méthode `_detect_and_execute_tools()` pour détection automatique
- [x] Tests `test_bbia_tools.py`

### **Phase 2 : Animations Idle** ✅ **TERMINÉ**
- [x] Créer `bbia_idle_animations.py`
- [x] Implémenter respiration automatique (`BBIABreathingAnimation`)
- [x] Implémenter poses de passage (`BBIAPoseTransitionManager`)
- [x] Implémenter tremblement vocal (`BBIAVocalTremor`)
- [x] Gestionnaire centralisé (`BBIIdleAnimationManager`)
- [x] Tests `test_bbia_idle_animations.py`
- [x] Démos `demo_idle_animations.py`

### **Phase 3 : Intégration Complète** ✅ **TERMINÉ**
- [x] Ajouter outils `dance` / `stop_dance` dans `bbia_tools.py`
- [x] Tester intégration LLM → outils → robot
- [x] Documenter usage (demos créés)
- [x] `demo_chat_with_tools.py` pour exemple complet

### **Phase 4 : Realtime API (Optionnel)**
- [ ] Créer `bbia_realtime_api.py`
- [ ] Implémenter streaming conversation
- [ ] Intégrer avec `BBIAVoice`
- [ ] Tests

---

## 🚀 Démarrage Rapide

### **1. Analyser Code Officiel**

**Repository à examiner** :
```
https://github.com/pollen-robotics/reachy-mini-conversation-app
```

**Fichiers clés** :
- `tools.py` : Outils LLM
- `animations.py` ou `idle_manager.py` : Animations idle
- `main.py` : Structure générale
- `conversation.py` : Logique conversation (si existe)

### **2. Copier Structure**

**Pour outils LLM** :
```python
# Adapter depuis tools.py officiel
def get_tools():
    return [
        {
            "type": "function",
            "function": {
                "name": "move_head",
                "description": "...",
                "parameters": {...}
            }
        }
    ]
```

**Pour animations idle** :
```python
# Adapter depuis idle_manager.py officiel
class BBIABreathingAnimation:
    def __init__(self, robot_api):
        self.robot_api = robot_api
    
    def start_breathing(self):
        # Pattern respiration
        pass
```

---

## 📊 Résumé : Ce qui peut être copié

| Fonctionnalité | Peut copier ? | Difficulté | Temps estimé |
|----------------|---------------|------------|---------------|
| **Outils LLM** | ✅ Oui (structure) | 🟡 Moyenne | 2-3h |
| **Animations Idle** | ✅ Oui (logique) | 🟡 Moyenne | 3-4h |
| **Danses** | ✅ Déjà présent | 🟢 Facile | 1-2h (intégration) |
| **Realtime API** | ✅ Oui (structure) | 🔴 Difficile | 4-5h (optionnel) |

---

## 💡 Recommandations

1. **Commencer par outils LLM** (plus simple, impact immédiat)
2. **Puis animations idle** (améliore expérience utilisateur)
3. **Enfin Realtime API** (optionnel, seulement si besoin)

**Avantages réutilisation** :
- ✅ Gain de temps (2-3h vs 10h+)
- ✅ Code testé et validé
- ✅ Compatibilité SDK garantie
- ✅ Architecture éprouvée

**À adapter** :
- ⚠️ Intégration avec architecture BBIA existante
- ⚠️ Tests pour BBIA
- ⚠️ Documentation BBIA

---

**Dernière mise à jour** : Oct / Oct / Nov. 20255

