# 🧠 Guide LLM Conversationnel BBIA

**Dernière mise à jour** : 26 Janvier 2026
**Version BBIA** : 1.4.0
**Objectif** : Guide complet pour utiliser l'intelligence conversationnelle de BBIA

---

## 🎯 Introduction

BBIA utilise un **LLM conversationnel** (Phi-2 ou TinyLlama) pour remplacer les règles par une compréhension contextuelle et des réponses adaptées.

### Fonctionnalités

- ✅ **Compréhension contextuelle** : Historique de 10 messages
- ✅ **Actions robot via conversation** : "Tourne la tête à droite"
- ✅ **Intégration émotions** : Détection et application automatique
- ✅ **5 personnalités** : Friendly, Professional, Playful, Calm, Enthusiastic
- ✅ **Apprentissage préférences** : Adaptation selon utilisateur

---

## 📦 Installation

### Dépendances

```bash
pip install transformers torch accelerate
```

### Modèles

BBIA charge automatiquement :
1. **Phi-2 2.7B** (Microsoft) - Recommandé
2. **TinyLlama 1.1B** - Fallback si Phi-2 indisponible
3. **Hugging Face Inference API** - Fallback si modèles locaux indisponibles

---

## 🚀 Utilisation Basique

### Exemple Simple

```python
from bbia_sim.bbia_chat import BBIAChat
from bbia_sim.robot_api import RobotAPI

# Créer instance chat
robot_api = RobotAPI()
chat = BBIAChat(robot_api=robot_api)

# Conversation
response = chat.chat("Bonjour !")
print(response)  # "Bonjour ! Comment puis-je vous aider aujourd'hui ?"

response = chat.chat("Comment vas-tu ?")
print(response)  # Réponse contextuelle avec référence à la conversation précédente
```

### Avec Actions Robot

```python
# BBIA comprend et exécute des actions
response = chat.chat("Tourne la tête à droite")
# → BBIA tourne la tête ET répond : "D'accord, je tourne la tête à droite."

response = chat.chat("Maintenant à gauche")
# → BBIA comprend "Maintenant" = référence à l'action précédente
```

---

## 🎭 Personnalités

### Changer de Personnalité

```python
# Personnalités disponibles
chat.set_personality("friendly")      # Amical et chaleureux (défaut)
chat.set_personality("professional")  # Formel et efficace
chat.set_personality("playful")       # Décontracté et humoristique
chat.set_personality("calm")          # Calme et apaisant
chat.set_personality("enthusiastic")  # Énergique et enthousiaste
```

### Exemple par Personnalité

```python
chat.set_personality("professional")
response = chat.chat("Bonjour")
# → "Bonjour. Comment puis-je vous assister aujourd'hui ?"

chat.set_personality("playful")
response = chat.chat("Bonjour")
# → "Salut ! Prêt pour une nouvelle aventure ? 😊"
```

---

## 🎯 Actions Robot via Conversation

BBIA détecte automatiquement 6 actions dans la conversation :

1. **look_right** : "Tourne la tête à droite", "Regarde à droite"
2. **look_left** : "Tourne la tête à gauche", "Regarde à gauche"
3. **look_up** : "Regarde en haut", "Lève la tête"
4. **look_down** : "Regarde en bas", "Baisse la tête"
5. **wake_up** : "Réveille-toi", "Allume-toi"
6. **sleep** : "Endors-toi", "Éteins-toi"

### Exemple

```python
response = chat.chat("Tourne la tête à droite puis regarde en haut")
# → BBIA exécute les deux actions et confirme
```

---

## 💡 Apprentissage Préférences

### Apprendre Préférences

```python
# Utilisateur préfère réponses courtes
chat.learn_preference("court", {"response_length": "short"})

# Utilisateur préfère réponses rapides
chat.learn_preference("rapide", {"response_speed": "fast"})
```

### Adaptation Automatique

BBIA adapte automatiquement ses réponses selon les préférences :

```python
# Après avoir appris "court"
response = chat.chat("Explique-moi comment fonctionne le robot")
# → Réponse courte et concise (2-3 phrases max)
```

---

## 🔧 Configuration

### Historique Conversation

```python
# Accéder à l'historique
history = list(chat.context)
print(f"Nombre de messages : {len(history)}")

# Historique contient :
# [
#   {"user": "Bonjour", "assistant": "Bonjour !", "timestamp": ...},
#   {"user": "Comment vas-tu ?", "assistant": "...", "timestamp": ...},
#   ...
# ]
```

### Timeout et Performance

```python
# Timeout pour génération (défaut: 5s)
chat.llm_timeout = 10.0  # 10 secondes

# Modèle Whisper pour STT (défaut: "tiny")
chat.whisper_model = "base"  # Plus précis mais plus lent
```

---

## 🧪 Tests

### Tests Unitaires

```bash
# Tests basiques
pytest tests/test_bbia_chat_llm.py

# Tests personnalités
pytest tests/test_bbia_chat_personalities.py
```

### Exemple de Test

```python
def test_chat_context():
    chat = BBIAChat()
    
    # Premier message
    response1 = chat.chat("Bonjour")
    assert "bonjour" in response1.lower()
    
    # Deuxième message avec référence
    response2 = chat.chat("Comment vas-tu ?")
    assert len(chat.context) == 2
    assert response2 != response1  # Réponses différentes
```

---

## 📚 API Référence

### Classe BBIAChat

```python
class BBIAChat:
    def __init__(self, robot_api: RobotAPI | None = None)
    def chat(self, user_message: str) -> str
    def set_personality(self, personality: str) -> None
    def learn_preference(self, user_action: str, context: dict) -> None
    def _detect_action(self, user_message: str) -> dict | None
    def _execute_action(self, action: dict) -> None
    def _extract_emotion(self, user_message: str) -> str | None
    def _apply_emotion(self, emotion: str) -> None
```

### Attributs

- `context` : `deque` (maxlen=10) - Historique conversation
- `personality` : `str` - Personnalité actuelle
- `user_preferences` : `dict` - Préférences utilisateur
- `llm_model` : Modèle LLM chargé
- `llm_tokenizer` : Tokenizer associé

---

## 🐛 Dépannage

### Problème : LLM ne charge pas

**Solution :**
```python
# Vérifier dépendances
import torch
print(torch.__version__)  # Doit être >= 2.0.0

# Vérifier RAM disponible
import psutil
print(f"RAM disponible: {psutil.virtual_memory().available / 1024**3:.1f} GB")
# Doit être >= 6GB pour Phi-2
```

### Problème : Latence élevée

**Solution :**
```python
# Utiliser modèle plus léger
chat = BBIAChat()
# TinyLlama sera chargé automatiquement si Phi-2 échoue

# Ou utiliser Hugging Face Inference API (fallback)
# Nécessite connexion internet
```

### Problème : Actions robot non détectées

**Solution :**
```python
# Vérifier que robot_api est fourni
chat = BBIAChat(robot_api=robot_api)

# Tester détection manuelle
action = chat._detect_action("Tourne la tête à droite")
print(action)  # {"action": "look_right", "confidence": 0.9}
```

---

## 🎉 Exemples Complets

### Conversation Complète

```python
from bbia_sim.bbia_chat import BBIAChat
from bbia_sim.robot_api import RobotAPI

robot_api = RobotAPI()
chat = BBIAChat(robot_api=robot_api)

# Conversation naturelle
chat.chat("Bonjour !")
chat.chat("Comment vas-tu ?")
chat.chat("Peux-tu tourner la tête à droite ?")
chat.chat("Maintenant à gauche")
chat.chat("Merci !")

# BBIA comprend le contexte et exécute les actions
```

### Avec Personnalité

```python
chat.set_personality("playful")

chat.chat("Raconte-moi une blague")
# → Réponse humoristique et décontractée

chat.set_personality("professional")
chat.chat("Raconte-moi une blague")
# → Réponse plus formelle (peut refuser poliment)
```

---

## 📖 Ressources

- **Code source** : `src/bbia_sim/bbia_chat.py`
- **Tests** : `tests/test_bbia_chat_llm.py`
- **Plan détaillé** : `docs/quality/audits/PLAN_INTELLIGENCE_CONVERSATIONNELLE.md`

---

---

## 🤗 Hugging Face Chat - Guide Complet (Issue #384)

### Introduction

BBIA intègre **Hugging Face Chat** via le module `BBIAHuggingFace` pour des conversations avec LLM. Ce guide explique comment utiliser cette fonctionnalité.

### Activation du Chat HF

```python
from bbia_sim.bbia_huggingface import BBIAHuggingFace

# Créer instance
hf = BBIAHuggingFace()

# Activer LLM conversationnel (optionnel, lourd)
hf.enable_llm_chat(model_name="phi2")  # ou "mistral", "llama", "tinyllama"

# Chat simple
response = hf.chat("Bonjour, comment allez-vous ?")
print(response["response"])

# Chat avec outils (function calling)
response = hf.chat("Tourne la tête à droite", enable_tools=True)
```

### Modèles Disponibles

| Modèle | Taille | RAM Requise | Recommandation |
|--------|--------|-------------|----------------|
| **phi2** | 2.7B | ~5GB | ✅ Recommandé pour RPi 5 |
| **tinyllama** | 1.1B | ~2GB | ✅ Ultra-léger |
| **mistral** | 7B | ~14GB | ❌ Trop lourd pour RPi |
| **llama** | 8B | ~16GB | ❌ Trop lourd pour RPi |

### Configuration

```python
# Utiliser modèle léger par défaut
os.environ["BBIA_HF_CHAT_MODEL"] = "phi2"

# Désactiver LLM pour économiser mémoire
hf.disable_llm_chat()

# Réactiver
hf.enable_llm_chat("phi2")
```

### Utilisation

```python
# Chat avec contexte
response = hf.chat(
    "Qu'est-ce que j'ai dit avant ?",
    use_context=True  # Utilise historique conversation
)

# Analyser sentiment
sentiment = hf.analyze_sentiment("Je suis très heureux !")
print(sentiment["sentiment"])  # "positive"

# Historique conversation
history = hf.get_conversation_history(limit=10)
for entry in history:
    print(f"{entry['role']}: {entry['content']}")
```

### Intégration avec Robot

```python
from bbia_sim.bbia_huggingface import BBIAHuggingFace
from bbia_sim.bbia_tools import BBIATools

# Créer outils robot
tools = BBIATools(robot_api=robot_api, hf_chat=hf)

# Chat avec actions robot automatiques
response = hf.chat("Regarde à droite", enable_tools=True)
# Le robot tourne automatiquement la tête !
```

### Troubleshooting

**Problème** : Modèle ne charge pas
**Solution** : Vérifier RAM disponible, utiliser modèle plus léger

**Problème** : Latence élevée
**Solution** : Utiliser `tinyllama` ou désactiver LLM (`disable_llm_chat()`)

**Problème** : Modèle non trouvé
**Solution** : Vérifier connexion internet, modèles téléchargés automatiquement

### Références

- Module : `src/bbia_sim/bbia_huggingface.py`
- Méthode principale : `enable_llm_chat()`, `chat()`, `disable_llm_chat()`
- Exemples : `examples/demo_chat_bbia_3d.py`

---

**Document créé le :** 26 Janvier 2026
**Dernière mise à jour :** 26 Janvier 2026 (Issue #384)
**Version BBIA :** 1.3.2
**Auteur :** Arkalia Luna System

