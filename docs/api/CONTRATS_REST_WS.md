# Contrats REST & WebSocket

> Compatibilité Python: 3.11+
>
> Liens utiles: `docs/references/INDEX_THEMATIQUE.md`, `docs/status.md`

## REST
- Pagination: `?limit=50&offset=0`
- Filtre: paramètres explicites (`?emotion=happy`), pas d’opérateur caché
- Codes HTTP: 200/400/401/404/429/500 (minimaux)

## WebSocket
- Channel: `/ws/telemetry`
- Messages typés (`type`): ping/pong/status/telemetry
- Versionnement: champ `schema_version` (semver), compat descendante sur 1 version

## Sécurité
- Auth Bearer (facultatif en dev)
- CORS strict côté REST

## Références
- OpenAPI: `http://localhost:8000/openapi.json`
- État par axe: `docs/status.md` → API & SDK
 - Curl rapide:
```bash
curl -s http://localhost:8000/health || true
curl -s http://localhost:8000/openapi.json | head -n 20
```
