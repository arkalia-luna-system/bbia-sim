# 🗺️ Roadmap Dashboard / UX

**Date** : Oct / Nov. 2025  
**Version** : 1.0

> **Voir aussi** : [`docs/reference/INDEX_THEMATIQUE.md`](../reference/INDEX_THEMATIQUE.md) et [`docs/reference/project-status.md`](../reference/project-status.md)

**📊 [CI/CD Pipeline](../deployment/PIPELINE_CI.md)** | **🔧 [Guide avancé](../guides/GUIDE_AVANCE.md)**

---

## Architecture Dashboard Proposée

```mermaid
graph TB
    subgraph "Frontend Dashboard"
        UI[Interface Utilisateur<br/>HTMX/Alpine ou Streamlit]
        CHARTS[Graphiques Temps Réel<br/>latence, FPS, CPU/RAM]
        SLIDERS[Sliders Émotions<br/>+ Presets JSON]
    end
    
    subgraph "Backend API"
        REST_API[REST API<br/>FastAPI]
        WS[WebSocket<br/>/ws/telemetry]
        AUTH[Authentification<br/>Bearer Token]
    end
    
    subgraph "Sources Données"
        ROBOT[RobotAPI<br/>État Robot]
        TELEMETRY[Télémétrie<br/>100Hz]
        METRICS[Métriques Système<br/>CPU/RAM]
    end
    
    UI --> REST_API
    UI --> WS
    
    CHARTS --> WS
    SLIDERS --> REST_API
    
    REST_API --> AUTH
    WS --> TELEMETRY
    TELEMETRY --> ROBOT
    
    style UI fill:#90EE90
    style WS fill:#87CEEB
    style ROBOT fill:#FFD700

```

## Roadmap Implémentation

```mermaid
gantt
    title Roadmap Dashboard / UX
    dateFormat  YYYY-MM-DD
    section Phase 1: Core
    API REST Endpoints        :done, api1, 2025-10-01, 2025-10-15
    WebSocket Télémétrie      :done, ws1, 2025-10-15, 2025-10-30
    Authentification          :active, auth1, 2025-11-01, 2025-11-15
    
    section Phase 2: UI
    Graphiques Temps Réel     :ui1, 2025-11-15, 2025-12-01
    Sliders Émotions          :ui2, 2025-12-01, 2025-12-15
    Presets Exportables       :ui3, 2025-12-15, 2025-01-01
    
    section Phase 3: Advanced
    Mode Démo Read-only       :adv1, 2025-01-01, 2025-01-15
    Export/Import Config      :adv2, 2025-01-15, 2025-02-01

```

## Mini UI télémétrie (proposé)

- Graphiques temps réel: latence, FPS, CPU/RAM
- Sliders émotions + presets exportables (JSON)
- Mode démo read-only (pas d'actions)

### Composants Dashboard

```mermaid
mindmap
  root((Dashboard BBIA))
    Télémétrie
      Graphiques Temps Réel
        Latence
        FPS Simulation
        CPU/RAM Usage
      WebSocket Stream
        100Hz Updates
    Contrôle
      Sliders Émotions
        12 Émotions
        Intensité 0-1
      Presets
        Export JSON
        Import JSON
        Sauvegarde
    Mode
      Démo Read-only
      Contrôle Complet
      Édition Presets

```

## Stack suggérée

- FastAPI + HTMX/Alpine (léger) ou Streamlit (rapide)

---

## 📚 Références

- **État par axe** : [`docs/reference/project-status.md`](../reference/project-status.md) → Dashboard / UX

---

**Dernière mise à jour** : Oct / Nov. 2025
