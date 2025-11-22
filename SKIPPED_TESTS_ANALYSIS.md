# Analyse des Tests Skippés

## Résumé
- **Total de skips trouvés**: 479
- **Fichiers concernés**: 74

## Catégories de Skips

### ✅ DOIVENT RESTER SKIPPÉS (légitimes)

#### 1. Hugging Face (56 skips, 13 fichiers)
- **Raison**: Dépendance optionnelle lourde (transformers, torch ~2-4GB)
- **Action**: Aucune - c'est normal de skip si non installé
- **Exemples**: `test_bbia_huggingface_chat.py`, `test_bbia_phase2_modules.py`

#### 2. Hardware requis (9 skips, 8 fichiers)
- **Raison**: Nécessite robot physique ou périphériques audio réels
- **Action**: Aucune - tests hardware doivent rester skippés par défaut
- **Exemples**: `test_reachy_mini_backend.py`, `test_audio_latency_e2e.py`

#### 3. Variables d'environnement désactivées (14 skips, 10 fichiers)
- **Raison**: `BBIA_DISABLE_VISION=1` ou `BBIA_DISABLE_AUDIO=1` intentionnellement activés
- **Action**: Aucune - c'est une fonctionnalité de désactivation
- **Exemples**: `test_vision_fps_budget.py`, `test_audio_buffer_stability.py`

#### 4. Dépendances ML optionnelles (15 skips, 5 fichiers)
- **Raison**: MediaPipe, YOLO sont optionnels
- **Action**: Aucune - dépendances optionnelles
- **Exemples**: `test_vision_yolo_extended.py`, `test_vision_webcam_real.py`

#### 5. Dépendances optionnelles diverses (35 skips, 3 fichiers)
- **Raison**: CoquiTTS, KokoroTTS, NeuTTS, OpenVoiceTTS, zenoh, etc.
- **Action**: Aucune - dépendances optionnelles
- **Exemples**: `test_capabilities_remaining.py`, `test_daemon_bridge.py`

#### 6. Watchdog (3 skips, 1 fichier)
- **Raison**: Watchdog nécessite configuration spéciale
- **Action**: Aucune - fonctionnalité avancée
- **Exemples**: `test_watchdog_monitoring.py`

#### 7. macOS spécifique (1 skip, 1 fichier)
- **Raison**: Test spécifique macOS
- **Action**: Aucune - test spécifique plateforme
- **Exemples**: `test_cli_help.py`

### 🔧 PEUVENT ÊTRE CORRIGÉS

#### 1. BBIAChat skip inutile (3 skips, 2 fichiers)
- **Problème**: BBIAChat peut être initialisé même sans Hugging Face (a un fallback)
- **Fichiers**: 
  - `tests/test_capabilities_methods.py:115`
  - `tests/test_edge_cases_error_handling.py:60, 78, 370`
- **Action**: ✅ CORRIGER - Retirer le skip, tester avec fallback

#### 2. Tests qui skipent au lieu de gérer gracieusement (catégorie "other": 346 skips)
- **Problème**: Beaucoup de tests skipent au runtime au lieu d'utiliser des mocks
- **Action**: ⚠️ À VÉRIFIER au cas par cas
- **Exemples**: Tests qui skipent pour des modules qui devraient être disponibles

## Recommandations

### Priorité 1: Corriger BBIAChat
BBIAChat fonctionne sans Hugging Face, donc les tests ne devraient pas skip.

### Priorité 2: Vérifier les skips "other"
Analyser les 346 skips dans la catégorie "other" pour voir s'il y a des patterns récurrents à corriger.

### Priorité 3: Documenter les skips légitimes
Ajouter des commentaires explicatifs pour les skips qui doivent rester (hardware, dépendances optionnelles lourdes).

