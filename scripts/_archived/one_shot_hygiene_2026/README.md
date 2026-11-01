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
