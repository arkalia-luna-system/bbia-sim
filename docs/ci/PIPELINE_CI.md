# Pipeline CI/CD

> Compatibilité Python: 3.11+ (prévoir matrice 3.12)
>
> Voir aussi: `docs/references/INDEX_THEMATIQUE.md` et `docs/status.md`

## État actuel
- GitHub Actions (`.github/workflows/ci.yml`), Python 3.11
- Phases: lint (ruff/black/mypy), tests, e2e headless, artifacts, codecov

## Axes d’amélioration
- Matrice Python: 3.11 / 3.12
- Hooks pre-commit: ruff/black/mypy
- Sharding/xdist tests si durée > 10 min
- Semgrep/gitleaks (sécurité) en jobs non-bloquants au début
- Perf baselines: exporter p50/p95 en JSONL et valider fourchette

## Artifacts & Reporting
- Coverage: `coverage.xml`, `htmlcov/`
- Logs e2e: upload sur échec

## Références
- État par axe: `docs/status.md` → CI/CD
 - Index: `docs/references/INDEX_THEMATIQUE.md`

## Pré-commit (optionnel)
```bash
pip install pre-commit
pre-commit install
pre-commit run --all-files
```
