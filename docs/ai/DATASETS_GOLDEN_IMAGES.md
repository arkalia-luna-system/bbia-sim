# 🗂️ Datasets & Golden Images

**Date** : Oct / Nov. 2025  
**Version** : 1.0

> **Voir aussi** : [`docs/references/INDEX_THEMATIQUE.md`](../references/INDEX_THEMATIQUE.md) et [`docs/status.md`](../status.md)

---

## 📁 Structure Recommandée
```
assets/datasets/
  vision/
    images/
    labels/
assets/golden/
  happy_mujoco.jsonl
  lookat_mujoco.jsonl
```

## 📝 Enregistrement

- **Script** : `scripts/record_trace.py --emotion happy --duration 5 --out assets/golden/happy_mujoco.jsonl`
- **Vision** : Conserver un sous-ensemble d'images annotées pour tests

## ✅ Validation

- **Comparaison trace** : `scripts/validate_trace.py --ref assets/golden/happy_mujoco.jsonl --cur current.jsonl`
- **Tests pytest** : `tests/test_golden_traces.py`

---

## 📚 Références

- **État par axe** : [`docs/status.md`](../status.md) → Vision / Audio / IA

---

**Dernière mise à jour** : Oct / Nov. 2025
