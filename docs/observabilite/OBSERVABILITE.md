# Observabilité (logs, métriques, santé)

> Compatibilité Python: 3.11+
>
> Liens utiles: `docs/references/INDEX_THEMATIQUE.md`, `docs/status.md`

## Logs structurés (proposé)
- Format: JSON par ligne
- Champs recommandés: timestamp, level, logger, message, module, request_id
- Sinks: console (dev), fichier `log/bbia.log` (prod), agrégateur (optionnel)

## Endpoints santé (proposé)
- Liveness: `GET /healthz` → 200 si process OK
- Readiness: `GET /readyz` → 200 si dépendances OK (SDK/Zenoh/config)

## Métriques Prometheus (proposé)
- Exposition: `GET /metrics`
- Métriques recommandées:
  - bbia_request_latency_seconds (histogram)
  - bbia_ws_clients_gauge
  - bbia_cpu_usage_percent, bbia_memory_usage_percent
  - bbia_watchdog_heartbeat_age_seconds

## Intégration CI
- Vérifier `/healthz` et `/readyz` en job e2e
- Publier `coverage.xml` + rapport perf (JSONL) en artifacts

## Références
- État par axe: `docs/status.md` → Observabilité
