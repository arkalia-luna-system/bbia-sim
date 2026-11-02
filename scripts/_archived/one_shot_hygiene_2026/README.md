# Scripts one-shot archivés (ménage 1 Nov 2026)

Scripts d’audit / fix ponctuels **sans référence CI/Makefile**, ou doublons
supersédés par `verify_docs_complete.py` / `verify_documentation.py` /
`audit_and_correct_dates_md.py` / `compare_with_official_exhaustive.py`.

| Script | Raison |
|--------|--------|
| `audit_complet_md.py` | One-shot MD ; dates figées 2025 ; chemins obsolètes |
| `audit_documentation_md.py` | Référençait des MD déplacés/supprimés (`docs/conformite`, `docs/api`, …) |
| `analyse_exhaustive_conformite.py` | Doublon soft de `compare_with_official_exhaustive.py` |
| `validate_skipped_tests.py` | Inventaire skips figé / non branché CI |
| `fix_all_fstrings.py` / `fix_logging_fstrings.py` | One-shot migration logging |
| `verify_md_links.py` / `fix_md_links.py` / `verify_docs_links.py` | Remplacés par `verify_docs_complete.py` |

Ne pas réutiliser tels quels. Préférer les scripts actifs listés dans `scripts/README.md`.

### `joints_media_demos/` (2 Nov 2026)

Scripts orphelins (0 référence hors `scripts/`), doublons joints / médias / démos.

| Script | Raison |
|--------|--------|
| `analyze_joints_detailed.py` / `diagnose_joints.py` | Doublons de `check_joints.py` |
| `create_robot_gif.py` / `generate_emotions_sequence_for_gif.py` / `generate_bbia_image_simple.py` | One-shot médias ; préférer `generate_bbia_image.py` |
| `demo_nod.sh` / `demo_wave.sh` | Remplacés par `demo_mode_complete.py` / `demo_public_api.py` |
| `export_visual_stl.py` / `redimensionner_viewer_mujoco.sh` | Utilitaires ponctuels non branchés |
| `verify_project.py` | Remplacé par `verify_documentation.py` / `bbia_doctor.py` |
| `voice_demo.py` / `voice_mac_say.sh` | Démos voix non référencées |
| `veille_reachy_mini.py` | Veille one-shot ; deps optionnelles dans `requirements/requirements-veille.txt` |
