# 📦 Scripts Archivés

> Scripts obsolètes ou non utilisés, conservés pour référence historique

## 📋 Scripts Archivés

### `start_api.py`

- **Date archivé**: Oct / Nov. 2025
- **Raison**: Remplacé par `start_public_api.py` (plus complet)
- **Utilisation**: ❌ Aucune référence dans le codebase
- **Alternative**: Utiliser `scripts/start_public_api.py`

### `kill_greedy_processes.sh`

- **Date archivé**: Oct / Nov. 2025
- **Raison**: Remplacé par `smart_process_cleanup.sh` (plus intelligent) et `process_manager.py`
- **Utilisation**: ❌ Aucune référence dans le codebase
- **Alternative**: Utiliser `scripts/smart_process_cleanup.sh` ou `python scripts/process_manager.py stop`

### Scripts Consolidés (Nov. 2025)

#### `audit_sdk_officiel_26NOV2025.py` → `scripts/_archived/comparison_audit/`
- **Date archivé**: Nov. 2025
- **Raison**: Fusionné dans `compare_with_official_exhaustive.py`
- **Alternative**: Utiliser `scripts/compare_with_official_exhaustive.py`

#### `comparaison_profonde_methodes_backend.py` → `scripts/_archived/comparison_audit/`
- **Date archivé**: Nov. 2025
- **Raison**: Fusionné dans `compare_with_official_exhaustive.py`
- **Alternative**: Utiliser `scripts/compare_with_official_exhaustive.py`

#### `audit_and_improve_md.py` → `scripts/_archived/`
- **Date archivé**: Nov. 2025
- **Raison**: Fusionné dans `verify_documentation.py`
- **Alternative**: Utiliser `scripts/verify_documentation.py --improve`


### `one_shot_hygiene_2026/` (1–2 Nov 2026)

- **Raison**: scripts d’audit/fix one-shot ou doublons liens MD, sans usage CI
- **Détail**: voir `one_shot_hygiene_2026/README.md` (+ sous-dossier `joints_media_demos/` du 2 Nov)
- **Alternatives**: `verify_docs_complete.py`, `verify_documentation.py`, `compare_with_official_exhaustive.py`, `check_joints.py`, `generate_bbia_image.py`

## 💡 Notes

Ces scripts sont conservés pour référence historique mais **ne doivent pas être utilisés**.
Utilisez les alternatives recommandées dans le README principal.
