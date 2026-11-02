# Cleanup scripts fused (2 Nov 2026)

Doublons dépréciés, remplacés par `scripts/cleanup_all.sh`.

| Script | Raison |
|--------|--------|
| `cleanup_project.sh` | Cache/métadonnées ; fusionné dans `cleanup_all.sh` |
| `cleanup_metadata_files.sh` | AppleDouble / `.DS_Store` ; fusionné dans `cleanup_all.sh` |
| `smart_process_cleanup.sh` | RAM / processus ; fusionné via `cleanup_all.sh --ram-only` |

**Alternative** : `./scripts/cleanup_all.sh` (`--cache-only`, `--ram-only`, `--yes`).
