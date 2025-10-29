# 🧠 Intelligence Conversationnelle LLM - Guide Complet

**Date :** Octobre 2025  
**Objectif :** Ajouter LLM pré-entraîné (Mistral 7B) pour conversations intelligentes

---

## 🎯 Objectif

Remplacer les réponses basées sur règles par un **vrai LLM conversationnel** qui comprend le contexte et génère des réponses naturelles.

**Avant :** Règles + sentiment analysis (limité)  
**Après :** Mistral 7B Instruct (conversations naturelles avec contexte)

---

## 📦 Installation

### Prérequis

```bash
# Activer venv
source venv/bin/activate

# Installer dépendances LLM
pip install transformers accelerate torch

# Optionnel : optimisations Apple Silicon
# (accélération automatique via MPS si disponible)
```

---

## 🚀 Utilisation

### Activation du LLM (Optionnel)

Le LLM est **désactivé par défaut** (pour éviter consommation mémoire inutile). Activer uniquement si nécessaire :

```python
from bbia_sim.bbia_huggingface import BBIAHuggingFace

# Initialiser BBIA
bbia = BBIAHuggingFace()

# Activer LLM conversationnel (peut prendre 1-2 minutes)
success = bbia.enable_llm_chat()
if success:
    print("✅ LLM activé - Conversations intelligentes disponibles")
else:
    print("⚠️  LLM non chargé - Utilisation réponses enrichies (règles)")
```

### Utilisation Automatique

Le LLM s'active automatiquement dans `ConversationBehavior` si disponible :

```python
from bbia_sim.bbia_behavior import ConversationBehavior

# Le comportement utilise automatiquement le LLM si activé
behavior = ConversationBehavior()
# Les conversations utiliseront Mistral 7B si disponible
```

### Conversation avec LLM

```python
from bbia_sim.bbia_huggingface import BBIAHuggingFace

bbia = BBIAHuggingFace()

# Activer LLM
bbia.enable_llm_chat("mistralai/Mistral-7B-Instruct-v0.2")

# Conversation intelligente
response = bbia.chat("Bonjour, comment ça va ?")
print(response)  # 🤖 Réponse naturelle générée par Mistral 7B

# Avec contexte (utilise historique)
response2 = bbia.chat("Tu te rappelles ce que je viens de dire ?", use_context=True)
print(response2)  # 🤖 Réponse qui référence la conversation précédente
```

### Désactiver LLM (Libérer Mémoire)

```python
# Désactiver pour libérer ~14GB RAM
bbia.disable_llm_chat()
```

---

## 🎭 Personnalités BBIA

Le LLM adapte ses réponses selon la personnalité BBIA :

```python
# Personnalité amicale (défaut)
bbia.bbia_personality = "friendly_robot"
response = bbia.chat("Salut !")
# → Réponse chaleureuse et professionnelle

# Personnalité curieuse
bbia.bbia_personality = "curious"
response = bbia.chat("Salut !")
# → Réponse avec questions et curiosité

# Personnalité enthousiaste
bbia.bbia_personality = "enthusiastic"
response = bbia.chat("Salut !")
# → Réponse énergique et positive

# Personnalité calme
bbia.bbia_personality = "calm"
response = bbia.chat("Salut !")
# → Réponse sereine et apaisante
```

---

## 📊 Comparaison Avant/Après

### Avant (Règles + Sentiment)

```python
# Réponses basées sur règles
User: "Bonjour"
BBIA: "Bonjour ! Ravi de vous revoir ! Comment allez-vous aujourd'hui ?"
# → Variantes limitées (6-8 réponses possibles)
```

### Après (LLM Mistral 7B)

```python
# Réponses générées intelligemment
User: "Bonjour"
BBIA: "Bonjour ! Content de te revoir. Comment s'est passée ta journée ?"
# → Réponses naturelles, variées, contextuelles
```

**Avantages LLM :**
- ✅ Réponses naturelles et variées
- ✅ Compréhension du contexte
- ✅ Référence à l'historique conversationnel
- ✅ Génération créative selon personnalité
- ✅ Adaptabilité à n'importe quelle question

---

## ⚙️ Configuration

### Modèles Disponibles

1. **Mistral 7B Instruct** ⭐ (Recommandé)
   - Qualité : Excellente
   - Français : Très bon
   - Taille : ~14GB RAM
   - Support MPS : ✅ (Apple Silicon)
   - Vitesse : ~1-3 secondes/réponse

2. **Llama 3 8B Instruct** (Alternative)
   - Qualité : Excellente
   - Français : Bon
   - Taille : ~16GB RAM
   - Support MPS : ✅
   - Vitesse : ~1-3 secondes/réponse

### Configuration Personnalisée

```python
# Charger modèle personnalisé
bbia.enable_llm_chat("meta-llama/Llama-3-8B-Instruct")

# Ou utiliser modèle local
bbia.enable_llm_chat("./models/mistral-7b-instruct")
```

---

## 🔧 Paramètres Génération

### Personnalisation Réponses

Le LLM utilise ces paramètres par défaut :
- `max_new_tokens=150` : Limite longueur réponse
- `temperature=0.7` : Créativité modérée
- `top_p=0.9` : Nucleus sampling
- `do_sample=True` : Génération variée

Pour modifier (dans `bbia_huggingface.py`) :

```python
outputs = self.chat_model.generate(
    **inputs,
    max_new_tokens=200,  # Réponses plus longues
    temperature=0.9,     # Plus créatif
    top_p=0.95,
    do_sample=True,
)
```

---

## 📈 Performance

### Ressources Nécessaires

**Mistral 7B Instruct :**
- RAM : ~14GB
- Premier chargement : 1-2 minutes
- Génération : ~1-3 secondes/réponse
- Disk : ~14GB (cache modèle)

**Optimisations :**
- Apple Silicon (M1/M2/M3) : Accélération MPS automatique
- CUDA : Accélération GPU si disponible
- Quantization : Réduire RAM à ~8GB (qualité légèrement réduite)

### Gestion Mémoire

```python
# Si mémoire limitée, charger seulement quand nécessaire
if need_intelligent_chat:
    bbia.enable_llm_chat()
    response = bbia.chat(user_message)
    bbia.disable_llm_chat()  # Libérer mémoire après
```

---

## 🧪 Tests

```bash
# Tester LLM conversationnel
cd /Volumes/T7/bbia-reachy-sim
source venv/bin/activate
python -c "
from bbia_sim.bbia_huggingface import BBIAHuggingFace

bbia = BBIAHuggingFace()
print('📥 Chargement LLM...')
if bbia.enable_llm_chat():
    print('✅ LLM chargé')
    response = bbia.chat('Bonjour, qui es-tu ?')
    print(f'🤖 Réponse: {response}')
else:
    print('❌ LLM non chargé')
"
```

---

## ⚠️ Limitations

1. **Mémoire :**
   - Requiert ~14GB RAM (Mistral 7B)
   - Peut être lourd sur machines limitées

2. **Vitesse :**
   - Premier chargement : 1-2 minutes
   - Génération : 1-3 secondes/réponse (CPU) ou <1s (MPS/CUDA)

3. **Qualité :**
   - Excellent français mais parfois génère en anglais
   - Réponses parfois trop longues (limité à 150 tokens)

4. **Fallback :**
   - Si LLM échoue, fallback automatique vers réponses enrichies (règles)

---

## 🔄 Migration Automatique

Le code existant **fonctionne sans modification** :

```python
# Code existant continue de fonctionner
from bbia_sim.bbia_huggingface import BBIAHuggingFace

bbia = BBIAHuggingFace()
response = bbia.chat("Bonjour")
# → Utilise réponses enrichies (règles) par défaut

# Activer LLM si souhaité
bbia.enable_llm_chat()
response = bbia.chat("Bonjour")
# → Utilise Mistral 7B pour réponse intelligente
```

---

## 📚 Références

- **Mistral 7B :** https://huggingface.co/mistralai/Mistral-7B-Instruct-v0.2
- **Llama 3 :** https://huggingface.co/meta-llama/Llama-3-8B-Instruct
- **Transformers :** https://huggingface.co/docs/transformers

---

## 🎯 Prochaines Étapes

1. ✅ LLM intégré dans `BBIAHuggingFace.chat()`
2. ✅ Activation optionnelle via `enable_llm_chat()`
3. ⏳ Tests unitaires
4. ⏳ Optimisation mémoire (quantization optionnelle)
5. ⏳ Support streaming (réponses au fil de l'eau)

---

**Status :** ✅ Phase 2 complétée - LLM conversationnel disponible !

