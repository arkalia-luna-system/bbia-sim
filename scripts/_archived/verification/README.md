# Scripts Archivés - Vérification Documentation

Ces scripts ont été fusionnés dans `verify_documentation.py`.

**Date archivage** : Oct 2025v. 20252025 Nov. 2025ct / Nov. 2025

## Scripts archivés

- `verify_doc_accuracy.py` - Vérifie précision (fichiers/test existent)
- `verify_md_vs_code.py` - Vérifie cohérence (fonctionnalités implémentées)

## Nouveau script unifié

Utiliser maintenant : `scripts/verify_documentation.py`

**Fonctionnalités** :

- Mode 1 : Vérification précision (de `verify_doc_accuracy.py`)
- Mode 2 : Vérification cohérence (de `verify_md_vs_code.py`)
- Par défaut : Les deux modes

**Usage** :

```bash
python scripts/verify_documentation.py           # Les deux modes
python scripts/verify_documentation.py --accuracy   # Précision uniquement
python scripts/verify_documentation.py --consistency # Cohérence uniquement
```

---

*Archivé pour référence historique*
