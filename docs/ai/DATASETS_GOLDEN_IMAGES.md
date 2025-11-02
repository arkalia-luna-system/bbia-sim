# Datasets & Golden Images

> Voir aussi: `docs/references/INDEX_THEMATIQUE.md` et `docs/status.md`

## Structure recommandée
```
assets/datasets/
  vision/
    images/
    labels/
assets/golden/
  happy_mujoco.jsonl
  lookat_mujoco.jsonl
```

## Enregistrement
- Script: `scripts/record_trace.py --emotion happy --duration 5 --out assets/golden/happy_mujoco.jsonl`
- Vision: conserver un sous-ensemble d’images annotées pour tests

## Validation
- Comparaison trace: `scripts/validate_trace.py --ref assets/golden/happy_mujoco.jsonl --cur current.jsonl`
- Tests pytest: `tests/test_golden_traces.py`

## Références
- État par axe: `docs/status.md` → Vision / Audio / IA
