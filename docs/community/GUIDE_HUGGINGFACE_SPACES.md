# 🌐 Guide Hugging Face Spaces - BBIA-SIM

**Date** : Décembre 2025  
**Version** : 1.4.0  
**Objectif** : Guide pour créer et publier des applications BBIA-SIM sur Hugging Face Spaces

---

## 🎯 Pourquoi Hugging Face Spaces ?

**Hugging Face Spaces** permet de :

- ✅ **Partager vos applications** BBIA-SIM publiquement
- ✅ **Démontrer les capacités** du robot en temps réel
- ✅ **Augmenter la visibilité** du projet
- ✅ **Faciliter l'adoption** par la communauté

---

## 🚀 Créer un Space

### 1. Prérequis

- Compte Hugging Face (gratuit)
- Application BBIA-SIM fonctionnelle
- Code prêt à être partagé

### 2. Créer le Space

1. **Aller sur** [Hugging Face Spaces](https://huggingface.co/spaces)
2. **Cliquer** "Create new Space"
3. **Remplir** :
   - **Nom** : `bbia-sim-conversation` (exemple)
   - **SDK** : `Gradio` ou `Streamlit`
   - **License** : `MIT`
   - **Visibility** : `Public`

### 3. Structure du Space

```
bbia-sim-conversation/
├── app.py              # Application principale
├── requirements.txt    # Dépendances
├── README.md          # Description
└── assets/            # Assets (optionnel)
```

---

## 📝 Exemples d'Applications

### 1. Application Conversationnelle

**Fichier** : `app.py`

```python
import gradio as gr
from bbia_sim import RobotFactory
from bbia_sim.bbia_chat import BBIAChat

# Initialiser robot
robot = RobotFactory.create_backend("mujoco")
chat = BBIAChat(robot_api=robot)

def chat_interface(message, history):
    """Interface de chat."""
    response = chat.chat(message)
    return response

# Interface Gradio
demo = gr.ChatInterface(
    fn=chat_interface,
    title="BBIA-SIM Conversation",
    description="Chat avec le robot Reachy Mini"
)

if __name__ == "__main__":
    demo.launch()
```

**Fichier** : `requirements.txt`

```
bbia-sim
gradio>=4.0.0
```

### 2. Application Vision

**Fichier** : `app.py`

```python
import gradio as gr
from bbia_sim import RobotFactory
from bbia_sim.bbia_vision import BBIAVision

# Initialiser robot
robot = RobotFactory.create_backend("mujoco")
vision = BBIAVision(robot_api=robot)

def vision_interface(image):
    """Interface vision."""
    result = vision.scan_environment()
    return result

# Interface Gradio
demo = gr.Interface(
    fn=vision_interface,
    inputs=gr.Image(),
    outputs=gr.JSON(),
    title="BBIA-SIM Vision",
    description="Détection objets et visages"
)

if __name__ == "__main__":
    demo.launch()
```

### 3. Application Émotions

**Fichier** : `app.py`

```python
import gradio as gr
from bbia_sim import RobotFactory
from bbia_sim.bbia_emotions import BBIAEmotions

# Initialiser robot
robot = RobotFactory.create_backend("mujoco")
emotions = BBIAEmotions(robot_api=robot)

def emotion_interface(emotion, intensity):
    """Interface émotions."""
    result = emotions.set_emotion(emotion, intensity)
    return f"Émotion {emotion} appliquée avec intensité {intensity}"

# Interface Gradio
demo = gr.Interface(
    fn=emotion_interface,
    inputs=[
        gr.Dropdown(
            choices=["happy", "sad", "angry", "excited", "neutral", "curious"],
            label="Émotion"
        ),
        gr.Slider(minimum=0.0, maximum=1.0, value=0.5, label="Intensité")
    ],
    outputs=gr.Textbox(),
    title="BBIA-SIM Émotions",
    description="Contrôler les émotions du robot"
)

if __name__ == "__main__":
    demo.launch()
```

---

## 📋 README.md du Space

**Fichier** : `README.md`

```markdown
---
title: BBIA-SIM Conversation
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: gradio
sdk_version: 4.0.0
app_file: app.py
pinned: false
---

# BBIA-SIM Conversation

Application conversationnelle pour robot Reachy Mini utilisant BBIA-SIM.

## Fonctionnalités

- ✅ Chat conversationnel avec le robot
- ✅ 12 émotions robotiques
- ✅ IA avancée (Whisper, SmolVLM2, LLM local)
- ✅ 100% gratuit et offline

## Utilisation

1. Entrez votre message dans le chat
2. Le robot répond avec intelligence
3. Profitez de l'interaction !

## Technologies

- **BBIA-SIM** : Moteur cognitif Python
- **Gradio** : Interface utilisateur
- **Whisper** : Reconnaissance vocale
- **SmolVLM2** : Vision par ordinateur

## Liens

- **GitHub** : [bbia-sim](https://github.com/arkalia-luna-system/bbia-sim)
- **Documentation** : [docs](https://github.com/arkalia-luna-system/bbia-sim/tree/main/docs)
```

---

## 🔧 Configuration

### Variables d'Environnement

**Fichier** : `.env` (optionnel)

```bash
BBIA_DISABLE_AUDIO=1
BBIA_TTS_BACKEND=pyttsx3
BBIA_STT_BACKEND=whisper
```

### Hardware Requirements

**Fichier** : `README.md`

```markdown
## Hardware Requirements

- CPU : 2+ cores
- RAM : 4GB+
- GPU : Optionnel (pour accélération IA)
```

---

## 📊 Métriques et Monitoring

### Ajouter des Métriques

```python
import gradio as gr
from bbia_sim.daemon.app.routers.metrics import get_metrics

def metrics_interface():
    """Interface métriques."""
    metrics = get_metrics()
    return metrics

demo = gr.Interface(
    fn=metrics_interface,
    outputs=gr.JSON(),
    title="BBIA-SIM Métriques"
)
```

---

## 🚀 Déploiement

### 1. Push sur Hugging Face

```bash
# Cloner votre space
git clone https://huggingface.co/spaces/votre-username/bbia-sim-conversation
cd bbia-sim-conversation

# Ajouter vos fichiers
git add .
git commit -m "Initial commit"
git push
```

### 2. Vérifier le Déploiement

1. **Aller sur** votre space Hugging Face
2. **Vérifier** que l'application se lance
3. **Tester** les fonctionnalités
4. **Partager** le lien !

---

## 📝 Bonnes Pratiques

### Performance

- ✅ **Lazy loading** : Charger les modèles à la demande
- ✅ **Cache** : Utiliser le cache pour les résultats
- ✅ **Optimisation** : Réduire la taille des modèles si possible

### Sécurité

- ✅ **Pas de secrets** : Ne pas exposer les clés API
- ✅ **Validation** : Valider les entrées utilisateur
- ✅ **Limites** : Limiter les ressources utilisées

### UX

- ✅ **Interface claire** : Design simple et intuitif
- ✅ **Feedback** : Afficher le statut des opérations
- ✅ **Documentation** : Expliquer comment utiliser

---

## 🎯 Exemples de Spaces

### Spaces Recommandés

1. **Conversation** : Chat avec le robot
2. **Vision** : Détection objets/visages
3. **Émotions** : Contrôle des émotions
4. **Comportements** : Démonstration comportements
5. **Dashboard** : Interface de contrôle complète

---

## ❓ Questions ?

Si vous avez des questions :

1. 📖 Consultez la documentation BBIA-SIM
2. 🔍 Recherchez dans les issues GitHub
3. 💬 Créez une issue avec le label `question`
4. 🤝 Contactez les maintainers

**Merci de partager vos applications BBIA-SIM !** 🚀

---

**Dernière mise à jour** : Décembre 2025

