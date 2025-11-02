# Guide du chat intelligent BBIA

**Version :** 1.3.2
**Date :** Oct 25 / Nov 25  
**📚 [FAQ](../FAQ.md)** | **🧠 [Guide NLP](../guides/GUIDE_NLP_SMOLVLM.md)** | **📊 [État actuel](../audit/RESUME_ETAT_ACTUEL_BBIA.md)**

---

## Vue d'ensemble

Le chat intelligent BBIA permet d'avoir une conversation avec votre robot Reachy Mini. BBIA analyse le sentiment de vos messages et répond selon sa personnalité configurée.

### Architecture Chat BBIA

```mermaid
flowchart TB
    USER[Utilisateur] --> INPUT[Message Entrant]
    
    INPUT --> SENTIMENT[Analyse Sentiment<br/>RoBERTa]
    INPUT --> NLP{NLP Détection?}
    
    NLP -->|Oui| TOOL[Détection Outil<br/>sentence-transformers]
    NLP -->|Non| CHAT[Chat LLM]
    
    TOOL --> EXEC[Exécution Action]
    SENTIMENT --> EMOTION[Émotion Associée]
    CHAT --> RESPONSE[Réponse Textuelle]
    
    EMOTION --> ROBOT[RobotAPI]
    EXEC --> ROBOT
    RESPONSE --> TTS[Synthèse Vocale<br/>pyttsx3]
    
    ROBOT --> ACTION[Action Robot]
    TTS --> SPEAKER[Audio Sortie]
    
    style TOOL fill:#90EE90
    style SENTIMENT fill:#87CEEB
    style CHAT fill:#FFD700
```

---

## Démarrage rapide

### Installation

```bash
# Activer venv
source venv/bin/activate

# Installer dépendances (si nécessaire)
pip install transformers torch
```

### Utilisation basique

```python
from bbia_sim.bbia_huggingface import BBIAHuggingFace

# Initialiser BBIA
bbia = BBIAHuggingFace()

# Chat simple
response = bbia.chat("Bonjour")
print(response)  # 🤖 Bonjour ! Comment allez-vous ? Je suis BBIA, votre robot compagnon.
```

---

## Personnalités BBIA

BBIA peut adopter différentes personnalités qui influencent ses réponses :

### friendly_robot (défaut)
- **Style** : robot amical et professionnel
- **Emoji** : 🤖
- **Usage** : par défaut, conversations standards

```python
bbia.bbia_personality = "friendly_robot"
print(bbia.chat("Salut"))  # 🤖 Salut ! Comment allez-vous ?
```

### curious
- **Style** : curieux et interrogatif
- **Emoji** : 🤔
- **Usage** : explorer des questions, poser des questions

```python
bbia.bbia_personality = "curious"
print(bbia.chat("Comment ça va ?"))  # 🤔 Comment ça va ?
```

### enthusiastic
- **Style** : enthousiaste et énergique
- **Emoji** : 🎉
- **Usage** : motiver, encourager

```python
bbia.bbia_personality = "enthusiastic"
print(bbia.chat("Super projet !"))  # 🎉 Super projet !
```

### calm
- **Style** : calme et serein
- **Emoji** : 😌
- **Usage** : apaiser, rassurer

```python
bbia.bbia_personality = "calm"
print(bbia.chat("Je suis stressé"))  # 😌 Respirez, tout va bien...
```

---

## Référence API

### `chat(user_message: str, use_context: bool = True) -> str`

**Description :** Chat intelligent avec BBIA

**Paramètres :**
- `user_message` : Message utilisateur
- `use_context` : Utiliser contexte messages précédents (défaut: True)

**Retourne :** Réponse de BBIA avec emoji selon personnalité

**Exemples :**

```python
# Conversation simple
bbia = BBIAHuggingFace()

bbia.chat("Bonjour")
# 🤖 Bonjour ! Comment allez-vous ? Je suis BBIA, votre robot compagnon.

bbia.chat("Comment tu te portes ?")
# 🤖 Ça va bien, merci ! Et vous ?

bbia.chat("Je suis très content aujourd'hui")
# 🤖 C'est super ! Je suis content pour vous. Continuez comme ça !
```

### `conversation_history: list[dict]`

**Description :** Historique des conversations

**Structure :**
```python
[
    {
        "user": "Message utilisateur",
        "bbia": "Réponse BBIA",
        "sentiment": {"sentiment": "POSITIVE", "score": 0.95},
        "timestamp": "Oct / No2025025025025025"
    },
    ...
]
```

**Exemple :**

```python
# Voir historique
for entry in bbia.conversation_history:
    print(f"User: {entry['user']}")
    print(f"BBIA: {entry['bbia']}")
    print(f"Sentiment: {entry['sentiment']['sentiment']}")
    print()
```

---

## Cas d'usage

### Cas 1 : Conversation Simple

```python
from bbia_sim.bbia_huggingface import BBIAHuggingFace

bbia = BBIAHuggingFace()

# Conversation
print(bbia.chat("Bonjour"))           # 🤖 Bonjour ! ...
print(bbia.chat("Comment allez-vous ?"))  # 🤖 Ça va bien...
print(bbia.chat("Au revoir"))         # 🤖 Au revoir ! À bientôt !
```

### Cas 2 : Analyse Sentiment

```python
# BBIA analyse automatiquement le sentiment
bbia.chat("Je suis très heureux !")
# 🤖 C'est super ! Je suis content pour vous. Continuez comme ça !

bbia.chat("Je suis triste")
# 🤖 Je comprends. Parlez-moi de ce qui ne va pas.
```

### Cas 3 : Changer Personnalité

```python
# Enthousiaste
bbia.bbia_personality = "enthusiastic"
print(bbia.chat("Salut !"))
# 🎉 Salut ! C'est génial de te voir !

# Curieux
bbia.bbia_personality = "curious"
print(bbia.chat("Je travaille sur un projet"))
# 🤔 C'est intéressant ! Dis-moi en plus sur ton projet...
```

### Cas 4 : Historique et Contexte

```python
# BBIA conserve le contexte
bbia.chat("Je m'appelle Alice")
bbia.chat("Comment je m'appelle ?")
# BBIA se souviendra du contexte
```

---

## Dashboard interactif

### Accéder au dashboard

```bash
# Lancer le dashboard
python src/bbia_sim/dashboard_advanced.py --port 8000

# Ouvrir dans navigateur
# http://localhost:8000
```

### Utiliser le panel chat

1. **Ouvrir** le dashboard dans votre navigateur
2. **Localiser** le panel "💬 Chat avec BBIA"
3. **Taper** votre message dans le champ
4. **Cliquer** sur "Envoyer" ou appuyer sur Entrée
5. **Voir** la réponse de BBIA s'afficher

Fonctionnalités :
- messages en temps réel
- historique conservé (50 derniers)
- interface claire
- distinction user/BBIA

---

## Tests

### Lancer les tests

```bash
# Tous les tests chat
python -m pytest tests/test_bbia_huggingface_chat.py -v

# Test spécifique
python -m pytest tests/test_bbia_huggingface_chat.py::TestBBIAHuggingFaceChat::test_chat_simple_greeting -v
```

### Exemple de test

```python
def test_chat():
    hf = BBIAHuggingFace()

    # Test salutation
    response = hf.chat("Bonjour")
    assert "bonjour" in response.lower() or "hello" in response.lower()

    # Test historique
    assert len(hf.conversation_history) == 1
```

---

## Configuration avancée

### Modifier le comportement des réponses

```python
# Tous les mots-clés sont dans _generate_simple_response()
# Modifier src/bbia_sim/bbia_huggingface.py ligne ~471

def _generate_simple_response(self, message: str, sentiment: dict) -> str:
    message_lower = message.lower()

    # Ajouter vos mots-clés personnalisés
    if "votre_mot_cle" in message_lower:
        return "Votre réponse personnalisée"

    # ... reste du code
```

### Ajouter une nouvelle personnalité

```python
# Dans _adapt_response_to_personality() (ligne ~502)

personality_responses = {
    "friendly_robot": f"🤖 {response}",
    "your_personality": f"🆕 {response}",  # Nouvelle personnalité
}

# Utilisation
bbia.bbia_personality = "your_personality"
```

---

## Dépannage

### Problème : "Hugging Face transformers requis"

**Solution :**
```bash
pip install transformers torch
```

### Problème : "Je ne comprends pas bien"

**Cause :** Analyse sentiment échoue
**Solution :** BBIA retourne message de secours automatiquement

### Problème : Dashboard chat ne fonctionne pas

**Solution :**
1. Vérifier que le serveur tourne
2. Vérifier WebSocket connecté (indicateur vert)
3. Ouvrir console navigateur pour erreurs

---

## Exemples complets

Voir `examples/demo_chat_bbia.py` pour un exemple complet :

```bash
python examples/demo_chat_bbia.py
```

---

## Fonctionnement technique

### Flux Chat

```
1. User envoie message
   ↓
2. BBIA analyse sentiment
   ↓
3. BBIA génère réponse (mots-clés + sentiment)
   ↓
4. BBIA adapte selon personnalité
   ↓
5. Sauvegarde dans historique
   ↓
6. Retourne réponse avec emoji
```

### Analyse Sentiment

BBIA utilise le modèle `cardiffnlp/twitter-roberta-base-sentiment-latest` :
- **POSITIVE** : Messages positifs → "C'est super !"
- **NEGATIVE** : Messages négatifs → "Je comprends..."
- **NEUTRAL** : Messages neutres → Réponse contextuelle

---

## Intégration avec le robot

Le chat fonctionne avec simulation ET robot physique :

```python
from bbia_sim.bbia_huggingface import BBIAHuggingFace
from bbia_sim.robot_factory import RobotFactory

# BBIA
bbia = BBIAHuggingFace()

# Chat
response = bbia.chat("Bonjour")

# Interaction avec robot
robot = RobotFactory.create_backend('mujoco')  # ou 'reachy_mini'
robot.connect()

# Définir émotion selon chat
if "heureux" in response.lower():
    robot.set_emotion("happy", 0.8)
```

---

## Notes importantes

- Chat fonctionne sans robot (simulation)
- Historique persiste pendant la session
- Support français uniquement actuellement
- Pas de LLM lourd (réponses basiques rapides)
- Thread-safe (une instance par thread recommandée)

---

## Voir aussi

- Documentation API : docstrings dans `bbia_huggingface.py`
- Exemples : `examples/demo_chat_bbia.py`
- Tests : `tests/test_bbia_huggingface_chat.py`
- Dashboard : `src/bbia_sim/dashboard_advanced.py`

