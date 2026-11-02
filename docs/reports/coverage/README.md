# Rapports de couverture de code

Ce dossier accueille les artefacts **générés localement / CI** (XML/JSON).

## Génération

```bash
pytest --cov=src/bbia_sim --cov-report=xml --cov-report=json
```

## Git

Les `*.xml` / `*.json` de couverture **ne sont pas versionnés** (voir `.gitignore`).
Seul ce `README.md` est suivi. Régénérer les rapports à chaque run de tests.
