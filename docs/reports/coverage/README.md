# 📊 Rapports de Couverture de Code

Ce dossier contient les rapports de couverture de code générés par pytest et coverage.

## Fichiers

- `coverage.xml` - Rapport XML de couverture (généré automatiquement)
- `coverage.json` - Rapport JSON de couverture
- `coverage_report.json` - Rapport détaillé de couverture
- `coverage_analysis.xml` - Analyse XML de couverture
- `test_results.xml` - Résultats des tests au format XML

## Génération

Ces fichiers sont générés automatiquement lors de l'exécution des tests :

```bash
pytest --cov=src/bbia_sim --cov-report=xml --cov-report=json
```

## Note

Ces fichiers sont ignorés par Git (voir `.gitignore`) et sont régénérés à chaque exécution des tests.

