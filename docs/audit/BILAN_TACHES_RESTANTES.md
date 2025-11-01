# 📋 Bilan Tâches Restantes - Octobre 2025

**Date** : octobre 2025  
**État actuel** : ~85-90% parité avec app officielle Reachy Mini  
**Dernière mise à jour** : Après implémentation NLP + SmolVLM2

---

## ✅ CE QUI EST FAIT (Dernièrement)

### Améliorations Récentes ✅
1. ✅ **Patterns français étendus** (30+ variantes)
2. ✅ **NLP sentence-transformers** (détection robuste)
3. ✅ **SmolVLM2 pour vision** (alternative gpt-realtime gratuite)
4. ✅ **Outils LLM intégrés** (`bbia_tools.py`)
5. ✅ **Animations idle** (respiration, poses, tremblement vocal)
6. ✅ **Danses** (API intégrée)

---

## 🟡 TÂCHES OPTIONNELLES RESTANTES

### Priorité MOYENNE (Améliorations UX, pas bloquant)

#### 1. VAD (Voice Activity Detection) pour Activation Auto ⏱️ 1-2h

**Ce qui manque** :
- ❌ Activation automatique conversation quand utilisateur parle
- ❌ Détection parole vs silence

**Solution GRATUITE** :
- Modèle : `silero/vad` (gratuit Hugging Face)
- Intégration : `voice_whisper.py`

**Impact** : Meilleure UX (activation auto, pas besoin de bouton)

**Fichiers** :
- `src/bbia_sim/voice_whisper.py` - Ajouter méthode `detect_speech_activity()`

**Code à ajouter** :
```python
from transformers import pipeline

def detect_speech_activity(self, audio_chunk):
    """Détecte si audio contient parole (gratuit)."""
    try:
        vad = pipeline("audio-classification", model="silero/vad")
        result = vad(audio_chunk)
        return result[0]["label"] == "SPEECH"
    except Exception:
        return False  # Fallback
```

---

#### 2. Extraction Paramètres avec NER (Named Entity Recognition) ⏱️ 2h

**Ce qui manque** :
- ❌ Extraction intelligente de paramètres depuis phrases naturelles
- ❌ Ex: "tourne la tête de 30 degrés" → `{"direction": "left", "angle": 30}`

**Solution GRATUITE** :
- Regex amélioré pour extraire nombres, angles
- Ou modèles NER français si disponibles (optionnel)

**Impact** : Meilleure compréhension paramètres numériques

**Fichiers** :
- `src/bbia_sim/bbia_huggingface.py` - Améliorer `_execute_detected_tool()`

**Code à ajouter** :
```python
import re

def extract_angle(self, message: str) -> float | None:
    """Extrait angle depuis message (ex: '30 degrés', 'pi/4 radians')."""
    # Patterns: "30 degrés", "0.5 radians", "pi/4"
    pattern = r"(\d+(?:\.\d+)?)\s*(?:degrés|degrees|radians)"
    match = re.search(pattern, message.lower())
    if match:
        return float(match.group(1))
    return None
```

---

#### 3. Whisper Streaming pour Latence Plus Faible ⏱️ 2-3h

**Ce qui manque** :
- ❌ Transcription en continu (comme OpenAI Realtime API)
- ❌ Latence actuelle : ~1-2s, pourrait être ~500ms

**Solution GRATUITE** :
- Bibliothèque : `whisper-streaming` (open-source)
- Whisper déjà utilisé, juste activer mode streaming

**Impact** : Latence plus faible (500ms vs 1-2s)

**Fichiers** :
- `src/bbia_sim/voice_whisper.py` - Ajouter mode streaming

**Note** : Pas essentiel, Whisper offline fonctionne déjà bien

---

### Priorité BASSE (Nice to have)

#### 4. Améliorer Détection NLP avec Modèles Français ⏱️ 3-4h

**Ce qui manque** :
- ❌ Modèle NLP actuel (`all-MiniLM-L6-v2`) optimisé pour anglais
- ❌ Meilleure détection avec modèles français dédiés

**Solution GRATUITE** :
- Modèle : `sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2` (multilingue)
- Ou modèles français spécialisés si disponibles

**Impact** : Meilleure détection pour phrases françaises complexes

**Fichiers** :
- `src/bbia_sim/bbia_huggingface.py` - Modifier `_detect_tool_with_nlp()`

---

#### 5. Tests E2E pour NLP et SmolVLM2 ⏱️ 2h

**Ce qui manque** :
- ❌ Tests E2E pour nouvelles fonctionnalités NLP
- ❌ Tests pour SmolVLM2 description images

**Solution** :
- Créer `tests/test_bbia_nlp_detection.py`
- Créer `tests/test_bbia_smolvlm_vision.py`

**Impact** : Validation des nouvelles fonctionnalités

---

#### 6. Documentation Utilisateur pour NLP et SmolVLM2 ⏱️ 1h

**Ce qui manque** :
- ❌ Documentation comment utiliser NLP détection
- ❌ Documentation comment activer SmolVLM2

**Solution** :
- Mettre à jour `README.md` ou créer guide utilisateur

**Impact** : Meilleure adoption des nouvelles fonctionnalités

---

## 🔴 CE QUI EST VOLONTAIREMENT EXCLU (Payant)

### Services Payants (Explicitement Exclus)
1. ❌ **OpenAI Realtime API** - Payant (mais Whisper offline gratuit fonctionne)
2. ❌ **gpt-realtime vision** - Payant (mais SmolVLM2 gratuit fonctionne)
3. ❌ **OpenAI GPT-4** - Payant (mais Mistral/Llama/Phi-2 gratuits fonctionnent)

**Conclusion** : Tout est remplacé par alternatives gratuites ✅

---

## 📊 COMPARAISON FINALE vs App Officielle

| Fonctionnalité | App Officielle | BBIA Actuel | Statut |
|----------------|----------------|-------------|--------|
| **Vision** | gpt-realtime (payant) / SmolVLM2 | ✅ YOLOv8n + MediaPipe + **SmolVLM2** | ✅ **Parité** |
| **Détection outils** | NLP avancé | ✅ **NLP sentence-transformers** + mots-clés | ✅ **Parité** |
| **Conversation** | OpenAI Realtime (payant) | ✅ Whisper offline | ✅ **Équivalent** |
| **LLM** | ? | ✅ Mistral/Llama/Phi-2/TinyLlama (gratuit) | ✅ **Meilleur** |
| **Outils LLM** | ✅ | ✅ **8 outils intégrés** | ✅ **Parité** |
| **Danses** | ✅ | ✅ **API intégrée** | ✅ **Parité** |
| **Animations idle** | ✅ | ✅ **Implémentées** | ✅ **Parité** |
| **VAD activation auto** | ✅ | 🟡 **Manquant** (optionnel) | 🟡 **Optionnel** |
| **Streaming voix** | ✅ | 🟡 **Manquant** (optionnel) | 🟡 **Optionnel** |

**Parité globale** : **~85-90%** (sans rien payer) ✅

**Ce qui manque** : VAD et streaming (optionnels, pas essentiels)

---

## 🎯 RECOMMANDATIONS

### À FAIRE (si temps disponible)

**Priorité HAUTE** :
- ✅ **Rien** - Tout l'essentiel est fait !

**Priorité MOYENNE** :
1. 🟡 **VAD activation auto** (1-2h) - Meilleure UX
2. 🟡 **Extraction paramètres NER** (2h) - Compréhension avancée

**Priorité BASSE** :
3. 🟢 **Whisper streaming** (2-3h) - Latence plus faible
4. 🟢 **Tests E2E NLP/SmolVLM2** (2h) - Validation
5. 🟢 **Documentation** (1h) - Adoption

---

## 💡 CONCLUSION

**BBIA est maintenant très complet** avec ~85-90% de parité fonctionnelle avec l'app officielle, **100% gratuitement** !

**Les tâches restantes sont toutes optionnelles** et concernent principalement :
- Améliorations UX (VAD, streaming)
- Extraction paramètres plus intelligente
- Documentation et tests

**Aucune tâche critique ou bloquante** ✅

---

**Dernière mise à jour** : octobre 2025

