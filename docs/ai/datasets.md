# 🗂️ Datasets & Golden Images

**Date** : 26 Novembre 2025  
**Version** : 1.0

> **Voir aussi** : [`docs/reference/INDEX_THEMATIQUE.md`](../reference/INDEX_THEMATIQUE.md) et [`docs/reference/project-status.md`](../reference/project-status.md)

---

## 📁 Structure Actuelle

```text
artifacts/golden/
  happy_mujoco.jsonl          # Trace référence émotion "happy"
  lookat_mujoco.jsonl         # Trace référence "lookat"
  wakeup_mujoco.jsonl         # Trace référence "wakeup"
  happy_mujoco_long.jsonl     # Version longue
  lookat_mujoco_long.jsonl    # Version longue
  wakeup_mujoco_long.jsonl    # Version longue
  schema.md                   # Schéma des traces

```

> **Note** : Les traces golden sont stockées dans `artifacts/golden/` (pas `assets/golden/`)

## 📝 Enregistrement

### Scripts Disponibles

- **Enregistrer trace** : `scripts/record_trace.py --emotion happy --duration 5 --out artifacts/golden/happy_mujoco.jsonl`
- **Vision** : Conserver un sous-ensemble d'images annotées pour tests (optionnel)

### Exemple d'utilisation

```bash
# Enregistrer une trace de référence
python scripts/record_trace.py --emotion happy --duration 5 --out artifacts/golden/happy_mujoco.jsonl

# Enregistrer une trace de test
python scripts/record_trace.py --emotion happy --duration 2 --out current_trace.jsonl

```

## ✅ Validation

### Comparaison de traces

- **Script** : `scripts/validate_trace.py --ref artifacts/golden/happy_mujoco.jsonl --cur current_trace.jsonl`
- **Tolérances** : ±0.6 rad position, ±70% cadence (adaptées pour CI)

### Tests pytest

- **Fichier** : `tests/test_golden_traces.py`
- **Tests** : 3 traces de référence (happy, lookat, wakeup)
- **Validation** : Vérifie que les traces courantes correspondent aux références

---

## 📚 Références

- **État par axe** : [`docs/reference/project-status.md`](../reference/project-status.md) → Vision / Audio / IA

---

**Dernière mise à jour** : 21 Novembre 2025

---

## 🎯 Navigation

**Retour à** : [README Documentation](../README.md)  
**Voir aussi** : [Modules IA](modules.md) • [Intelligence LLM](llm.md) • [Index Thématique](../reference/INDEX_THEMATIQUE.md)
