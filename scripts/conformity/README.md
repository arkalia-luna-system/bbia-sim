# 🧪 Tests de Conformité BBIA-SIM

Ce dossier contient les scripts de test de conformité pour valider que BBIA-SIM est conforme aux spécifications officielles du SDK Reachy Mini.

## 📋 Scripts Disponibles

### `test_conformity.py`
Test basique de conformité :
- Chargement modèle officiel
- Qualité assets STL
- Spécifications joints
- Endpoints API
- Modules BBIA
- Performance simulation

**Usage:**
```bash
python scripts/conformity/test_conformity.py
```

### `test_conformity_sdk_officiel.py`
Test complet de conformité avec le SDK officiel `reachy-mini` :
- Disponibilité SDK
- Conformité backend
- Compatibilité API
- Performances

**Usage:**
```bash
# Nécessite: pip install reachy-mini
python scripts/conformity/test_conformity_sdk_officiel.py
```

## 🎯 Objectif

Ces tests vérifient que BBIA-SIM respecte :
- Les spécifications officielles du SDK Reachy Mini
- Les interfaces et signatures attendues
- Les limites de sécurité
- Les performances minimales

## 📝 Notes

- Ces tests peuvent être automatisés à l'avenir dans `tests/`
- Pour l'instant, ils sont manuels pour permettre une exécution flexible
- Les résultats peuvent être utilisés pour générer des rapports de conformité

---

**Version**: 1.0  
**Date**: Janvier 2025

