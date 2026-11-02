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
- **Raison**: Remplacé par `cleanup_all.sh --ram-only` et `process_manager.py`
- **Utilisation**: ❌ Aucune référence dans le codebase
- **Alternative**: Utiliser `scripts/cleanup_all.sh --ram-only` ou `python scripts/process_manager.py stop`

### `cleanup_fused_2026/` (2 Nov 2026)

- **Raison**: doublons déjà fusionnés dans `cleanup_all.sh`
- **Contenu**: `cleanup_project.sh`, `cleanup_metadata_files.sh`, `smart_process_cleanup.sh`
- **Alternative**: `scripts/cleanup_all.sh`

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

### Suite hygiène 2 Nov 2026

| Élément | Raison | Alternative |
|---------|--------|-------------|
| `kill_mujoco_viewers.sh` | Déprécié, doublon cleanup | `cleanup_all.sh --ram-only` / `process_manager.py stop` |
| `docs_export/export_md_dark.sh`, `export_onepager.sh` | 0 réf. docs/CI/Makefile | `scripts/docs/build_docs_html.sh` |
| `utils/color_converter.py` | Utilitaire isolé, 0 réf. | — (CLI one-shot) |
| `onboarding/check_network.sh` | Orphelin (non appelé par setup/run_demo) | ping manuel / `bbia_doctor.py` |

Doc JSON déploiement racine → `docs/_archived/deployment/DEPLOYMENT_INSTRUCTIONS.json`.

## 💡 Notes

Ces scripts sont conservés pour référence historique mais **ne doivent pas être utilisés**.
Utilisez les alternatives recommandées dans le README principal.
