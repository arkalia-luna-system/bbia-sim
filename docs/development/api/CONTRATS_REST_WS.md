# 📡 Contrats REST & WebSocket

**Date** : Oct / Nov. 2025  
**Version** : 1.0  
**Compatibilité Python** : 3.11+

> **Liens utiles** : [`docs/reference/INDEX_THEMATIQUE.md`](../reference/INDEX_THEMATIQUE.md), [`docs/reference/project-status.md`](../reference/project-status.md)

**📚 [Guide intégration](../development/integration.md)** | **📊 [CI/CD Pipeline](../deployment/PIPELINE_CI.md)**

---

## Architecture REST & WebSocket

```mermaid
graph TB
    subgraph "Clients"
        HTTP[Client HTTP<br/>REST API]
        WS[Client WebSocket<br/>Temps Réel]
    end
    
    subgraph "BBIA API Server"
        REST[REST Endpoints<br/>FastAPI]
        WS_ENDPOINT[WebSocket Endpoint<br/>/ws/telemetry]
        AUTH[Authentification<br/>Bearer Token]
        CORS[CORS Policy<br/>Strict]
    end
    
    subgraph "Backend"
        ROBOTAPI[RobotAPI<br/>Contrôle Robot]
        TELEMETRY[Télémétrie<br/>État Temps Réel]
    end
    
    HTTP --> REST
    WS --> WS_ENDPOINT
    
    REST --> AUTH
    REST --> CORS
    REST --> ROBOTAPI
    
    WS_ENDPOINT --> TELEMETRY
    TELEMETRY --> ROBOTAPI
    
    style REST fill:#90EE90
    style WS_ENDPOINT fill:#87CEEB
    style AUTH fill:#FFD700
```

## REST
- Pagination: `?limit=50&offset=0`
- Filtre: paramètres explicites (`?emotion=happy`), pas d'opérateur caché
- Codes HTTP: 200/400/401/404/429/500 (minimaux)

### Flux REST

```mermaid
sequenceDiagram
    participant Client as Client HTTP
    participant API as REST API
    participant Auth as Auth Middleware
    participant Robot as RobotAPI
    
    Client->>API: GET /development/api/motion/status
    API->>Auth: Vérifier Bearer Token
    Auth-->>API: ✅ Token Valide
    
    API->>Robot: get_current_joint_positions()
    Robot-->>API: Positions joints
    
    API-->>Client: 200 OK + JSON
    
    Note over Client,Robot: Codes HTTP standards
```

## WebSocket
- Channel: `/ws/telemetry`
- Messages typés (`type`): ping/pong/status/telemetry
- Versionnement: champ `schema_version` (semver), compat descendante sur 1 version

### Flux WebSocket

```mermaid
sequenceDiagram
    participant Client as Client WebSocket
    participant WS as WebSocket Server
    participant Telemetry as Télémétrie
    participant Robot as RobotAPI
    
    Client->>WS: Connexion /ws/telemetry
    WS-->>Client: Connection Accepted
    
    loop Télémétrie Temps Réel
        Robot->>Telemetry: État Robot (100Hz)
        Telemetry->>WS: Message {type: "telemetry"}
        WS-->>Client: Envoi temps réel
    end
    
    Client->>WS: ping
    WS-->>Client: pong
    
    Note over Client,Robot: Messages typés + schema_version
```

## Sécurité
- Auth Bearer (facultatif en dev)
- CORS strict côté REST

### Flux Authentification

```mermaid
flowchart LR
    REQUEST[Requête Client] --> CHECK{Auth Requis?}
    
    CHECK -->|Dev Mode| ALLOW[✅ Autoriser<br/>Pas d'auth]
    CHECK -->|Production| TOKEN{Token Valide?}
    
    TOKEN -->|Oui| SUCCESS[✅ Requête Autorisée]
    TOKEN -->|Non| DENY[❌ 401 Unauthorized]
    
    style SUCCESS fill:#90EE90
    style DENY fill:#FFB6C1
    style CHECK fill:#FFD700
    style REQUEST fill:#87CEEB
```

## Références

- **OpenAPI** : `http://localhost:8000/openapi.json`
- **État par axe** : `docs/reference/project-status.md` → API & SDK

### Curl rapide
```bash
curl -s http://localhost:8000/health || true
curl -s http://localhost:8000/openapi.json | head -n 20
```

---

**Dernière mise à jour** : Oct / Nov. 2025
