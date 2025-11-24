# 🚀 Améliorations en Attente CI

**Date :** 24 novembre 2025  
**Objectif :** Identifier et préparer les améliorations possibles pendant l'attente de la CI

---

## 🎉 Améliorations Phase 1 - Terminées (24 Nov. 2025)

### ✅ Quick Wins Implémentés

1. **Python 3.12 dans CI** ✅
   - Matrice ajoutée dans job `lint` (3.11 + 3.12)
   - Détection précoce problèmes compatibilité

2. **Pre-commit hooks améliorés** ✅
   - Gitleaks ajouté (scan secrets)
   - `check-json`, `check-toml` ajoutés
   - Versions mises à jour

3. **Scan secrets automatisé** ✅
   - Gitleaks intégré dans CI
   - Installation automatique si absent
   - Continue-on-error pour ne pas bloquer

4. **Métriques Prometheus complétées** ✅
   - `bbia_watchdog_heartbeat_age_seconds`
   - `bbia_robot_connected`
   - `bbia_latency_p50_ms`, `bbia_latency_p95_ms`, `bbia_latency_p99_ms`

5. **Dépendances système** ✅
   - `ffmpeg` ajouté dans tous les jobs CI

---

---

## 📊 État Actuel

### ✅ Réalisations Récentes

- **bbia_audio.py** : **98.47%** de couverture ✅ (amélioré de 87.76%)
- **Tests** : 1795 tests collectés, tous passent
- **Linting** : Aucune erreur (black, ruff, mypy, bandit)

### 📈 Modules à Améliorer

#### 1. **bbia_voice.py** : **37.41%** de couverture ⚠️

- **Lignes non couvertes identifiées** :
  - `162-163, 167-168, 172-173, 187-188` : Branches de sélection de voix (Aurelie/Amelie)
  - `251-269` : Gestion erreurs SDK dans `dire_texte` (play_audio, speaker.play, etc.)
  - `281-286, 288, 299` : Fallbacks sounddevice dans `dire_texte`
  - `312-384` : Gestion SDK media dans `dire_texte` (play_audio, speaker.play_file, etc.)
  - `425-476` : Gestion SDK microphone dans `reconnaitre_parole`
  - `493-494, 498-504` : Gestion erreurs speech_recognition
  - `521, 532-533` : Cas limites dans `lister_voix_disponibles`
  - `551-575` : Thread worker transcription asynchrone
  - `587-600, 607-624` : Start/stop transcription asynchrone
  - `648-682, 692-759` : Transcription asynchrone et synchrone
  - `782-849` : Fonction `transcribe_audio` (Whisper)

**Impact estimé** : +20-30% de couverture globale si amélioré à 70%+

#### 2. **bbia_voice_advanced.py** : **15.61%** de couverture ⚠️

- Module optionnel (Coqui TTS)
- Tests conditionnels nécessaires

#### 3. **Autres modules avec faible couverture**

- `vision_yolo.py` : 15.83% (mais 99.45% selon README - à vérifier)
- `voice_whisper.py` : 11.51% (mais 92.52% selon README - à vérifier)

---

## 🎯 Plan d'Action Proposé

### Phase 1 : Améliorer `bbia_voice.py` (Priorité Haute)

#### Tests à Créer

1. **Tests pour `dire_texte` - Gestion erreurs SDK** :
   - `test_dire_texte_sdk_play_audio_typeerror` : TypeError avec play_audio
   - `test_dire_texte_sdk_speaker_play_error` : Erreur speaker.play
   - `test_dire_texte_sdk_speaker_play_file_error` : Erreur speaker.play_file
   - `test_dire_texte_sdk_media_exception` : Exception générale media
   - `test_dire_texte_tts_backend_fallback_sounddevice` : Fallback sounddevice
   - `test_dire_texte_tts_backend_fallback_errors` : Erreurs fallback
   - `test_dire_texte_pyttsx3_exception` : Exception pyttsx3

2. **Tests pour `reconnaitre_parole` - Gestion SDK** :
   - `test_reconnaitre_parole_sdk_record_audio` : Enregistrement SDK
   - `test_reconnaitre_parole_sdk_audio_bytes` : Conversion bytes
   - `test_reconnaitre_parole_sdk_audio_ndarray` : Conversion numpy array
   - `test_reconnaitre_parole_sdk_exception` : Exception SDK
   - `test_reconnaitre_parole_unknown_value_error` : UnknownValueError
   - `test_reconnaitre_parole_microphone_exception` : Exception microphone

3. **Tests pour `get_bbia_voice` - Branches non couvertes** :
   - `test_get_bbia_voice_aurelie_enhanced_fr` : Aurelie Enhanced fr
   - `test_get_bbia_voice_amelie_enhanced_fr` : Amelie Enhanced fr
   - `test_get_bbia_voice_aurelie_fr_CA` : Aurelie fr-CA
   - `test_get_bbia_voice_amelie_fr_CA` : Amelie fr-CA
   - `test_get_bbia_voice_aurelie_any` : Toute Aurelie
   - `test_get_bbia_voice_amelie_any` : Toute Amelie

4. **Tests pour `lister_voix_disponibles` - Cas limites** :
   - `test_lister_voix_languages_decode_error` : Erreur decode
   - `test_lister_voix_languages_exception` : Exception générale

5. **Tests pour transcription asynchrone** :
   - `test_transcribe_audio_async_start_stop` : Start/stop thread
   - `test_transcribe_audio_async_timeout` : Timeout
   - `test_transcribe_audio_async_queue_full` : Queue pleine
   - `test_transcribe_audio_whisper_disabled` : Audio désactivé
   - `test_transcribe_audio_whisper_not_available` : Whisper non disponible
   - `test_transcribe_audio_whisper_load_failure` : Échec chargement modèle
   - `test_transcribe_audio_whisper_import_error` : ImportError
   - `test_transcribe_audio_whisper_exception` : Exception générale

**Fichier à créer** : `tests/test_bbia_voice_coverage_remaining.py`

### Phase 2 : Vérifier les Incohérences de Coverage

- Vérifier pourquoi `vision_yolo.py` montre 15.83% dans pytest mais 99.45% dans README
- Vérifier pourquoi `voice_whisper.py` montre 11.51% dans pytest mais 92.52% dans README
- Possible problème de mesure ou de configuration

### Phase 3 : Nettoyer le Code

- Vérifier et traiter les TODO/FIXME dans le codebase
- Vérifier les `# pragma: no cover` sont justifiés

---

## 📝 Notes Techniques

### Structure de Tests Proposée

```python
# tests/test_bbia_voice_coverage_remaining.py
"""Tests pour améliorer la couverture de bbia_voice.py - Lignes manquantes.

Objectif : Couvrir les lignes non testées pour atteindre 70%+ de couverture.
"""

import os
import unittest
from unittest.mock import MagicMock, patch

# Désactiver audio pour CI
os.environ["BBIA_DISABLE_AUDIO"] = "1"

from bbia_sim import bbia_voice

class TestBBIAVoiceCoverageRemaining(unittest.TestCase):
    """Tests pour couvrir les lignes manquantes de bbia_voice.py."""
    
    # Tests pour dire_texte
    # Tests pour reconnaitre_parole
    # Tests pour get_bbia_voice
    # Tests pour lister_voix_disponibles
    # Tests pour transcription asynchrone
```

### Mocking Stratégies

1. **SDK Media Mocking** :
   - `robot_api.media.play_audio()` avec TypeError
   - `robot_api.media.speaker.play()` avec Exception
   - `robot_api.media.speaker.play_file()` avec Exception

2. **TTS Backend Mocking** :
   - `get_tts_backend()` retournant un backend mock
   - `backend.synthesize_to_wav()` avec succès/échec
   - Fallback sounddevice avec ImportError/OSError

3. **Microphone SDK Mocking** :
   - `robot_api.media.record_audio()` avec différents formats
   - Conversion bytes/numpy array

4. **Whisper Mocking** :
   - `WHISPER_AVAILABLE = False`
   - `WhisperSTT.load_model()` échec
   - `ImportError` pour whisper

---

## 🎯 Objectifs

- **bbia_voice.py** : 37.41% → **70%+** de couverture
- **Couverture globale** : Maintenir **68.86%+**
- **Tests** : Ajouter ~20-30 nouveaux tests ciblés

---

## ✅ Prochaines Étapes

1. ✅ Documentation mise à jour (bbia_audio.py : 98.47%)
2. ⏳ Créer `test_bbia_voice_coverage_remaining.py`
3. ⏳ Exécuter tests et vérifier couverture
4. ⏳ Vérifier incohérences coverage (vision_yolo, voice_whisper)
5. ⏳ Nettoyer TODO/FIXME

---

**Status** : En attente validation CI actuelle
