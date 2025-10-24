# Architecture BBIA-SIM

## Vue d'ensemble

BBIA-SIM est un système de simulation robotique modulaire avec API REST/WebSocket.

## Schéma d'architecture Mermaid

```mermaid
graph TB
    subgraph "Clients"
        CLI[CLI/Examples<br/>python -m bbia<br/>examples/*.py]
        WEB[Web Client<br/>Frontend<br/>Dashboard]
        EXT[External API<br/>Integration<br/>ROS Bridge]
    end
    
    subgraph "FastAPI Daemon"
        REST[REST API<br/>/api/state<br/>/api/motion]
        WS[WebSocket<br/>/ws/telemetry]
        MW[Middleware<br/>Security<br/>Rate Limit]
    end
    
    subgraph "Simulation Service"
        MUJOCO[MuJoCo Sim<br/>Headless<br/>100Hz Loop]
        STATE[Robot State<br/>Manager]
        JOINT[Joint Ctrl<br/>Manager]
    end
    
    subgraph "MuJoCo Engine"
        PHYSICS[Physics<br/>Engine]
        MODELS[3D Models<br/>reachy_mini.xml]
        SCENES[Scenes<br/>minimal.xml]
    end
    
    CLI --> REST
    WEB --> REST
    EXT --> REST
    
    CLI --> WS
    WEB --> WS
    EXT --> WS
    
    REST --> MW
    WS --> MW
    
    MW --> MUJOCO
    MW --> STATE
    MW --> JOINT
    
    MUJOCO --> PHYSICS
    STATE --> MODELS
    JOINT --> SCENES
    
    PHYSICS --> MODELS
    MODELS --> SCENES
```

## Architecture détaillée

### 🎮 **Couche Client**
- **CLI/Examples** : Scripts Python pour tests et démonstrations
- **Web Client** : Interface web pour contrôle et monitoring
- **External API** : Intégrations tierces (ROS, etc.)

### 🌐 **Couche API**
- **REST API** : Endpoints HTTP pour contrôle du robot
- **WebSocket** : Communication temps réel pour télémétrie
- **Middleware** : Sécurité, rate limiting, authentification

### 🤖 **Couche Simulation**
- **MuJoCo Sim** : Moteur de simulation physique
- **Robot State** : Gestionnaire d'état du robot
- **Joint Control** : Contrôleur des articulations

### ⚙️ **Couche Physique**
- **Physics Engine** : Moteur physique MuJoCo
- **3D Models** : Modèles 3D du robot Reachy Mini
- **Scenes** : Environnements de simulation

## Flux de données

```mermaid
sequenceDiagram
    participant C as Client
    participant API as FastAPI
    participant SIM as Simulation
    participant MJ as MuJoCo
    
    C->>API: POST /api/motion/joints
    API->>SIM: Update joint positions
    SIM->>MJ: Set joint targets
    MJ->>SIM: Physics step
    SIM->>API: Robot state update
    API->>C: WebSocket telemetry
    
    Note over C,MJ: Cycle de simulation à 100Hz
```

## Modules BBIA

```mermaid
graph LR
    subgraph "Modules BBIA"
        EMOTIONS[bbia_emotions.py<br/>8 émotions]
        VISION[bbia_vision.py<br/>Détection objets]
        AUDIO[bbia_audio.py<br/>Enregistrement]
        VOICE[bbia_voice.py<br/>TTS/STT]
        BEHAVIOR[bbia_behavior.py<br/>Comportements]
    end
    
    subgraph "Intégration"
        INTEGRATION[bbia_integration.py<br/>Orchestrateur]
    end
    
    subgraph "Simulation"
        SIMULATOR[simulator.py<br/>MuJoCo]
        JOINTS[joints.py<br/>16 joints]
    end
    
    EMOTIONS --> INTEGRATION
    VISION --> INTEGRATION
    AUDIO --> INTEGRATION
    VOICE --> INTEGRATION
    BEHAVIOR --> INTEGRATION
    
    INTEGRATION --> SIMULATOR
    SIMULATOR --> JOINTS
```

## Joints du robot Reachy Mini

```mermaid
graph TB
    subgraph "Joints Mobiles (7)"
        YAW[yaw_body<br/>Rotation corps<br/>-160° à +160°]
        STEWART1[stewart_1<br/>Plateforme<br/>-48° à +80°]
        STEWART2[stewart_2<br/>Plateforme<br/>-80° à +70°]
        STEWART3[stewart_3<br/>Plateforme<br/>-48° à +80°]
        STEWART4[stewart_4<br/>Plateforme<br/>-80° à +48°]
        STEWART5[stewart_5<br/>Plateforme<br/>-70° à +80°]
        STEWART6[stewart_6<br/>Plateforme<br/>-80° à +48°]
    end
    
    subgraph "Joints Bloqués (9)"
        PASSIVE1[passive_1-7<br/>Articulations<br/>mécaniques]
        ANTENNA1[left_antenna<br/>Antenne gauche<br/>Décorative]
        ANTENNA2[right_antenna<br/>Antenne droite<br/>Décorative]
    end
    
    YAW -.->|Recommandé| STEWART1
    STEWART1 -.-> STEWART2
    STEWART2 -.-> STEWART3
    STEWART3 -.-> STEWART4
    STEWART4 -.-> STEWART5
    STEWART5 -.-> STEWART6
```
```
python -m bbia_sim --sim --headless
    ↓
MuJoCoSimulator.launch_simulation()
    ↓
MuJoCo Engine (100Hz loop)
```

### 2. API → Simulation
```
curl POST /api/motion/joints
    ↓
FastAPI Router (motion.py)
    ↓
SimulationService.set_joint_position()
    ↓
MuJoCo mj_step() + mj_forward()
```

### 3. WebSocket → Client
```
Client connects to /ws/telemetry
    ↓
ConnectionManager.broadcast()
    ↓
SimulationService.get_robot_state()
    ↓
JSON telemetry (10Hz)
```

## Composants principaux

### `src/bbia_sim/sim/`
- **`simulator.py`** : Interface MuJoCo, gestion viewer/headless
- **`models/reachy_mini.xml`** : Modèle 3D robot Reachy Mini
- **`scenes/minimal.xml`** : Scène de test simple

### `src/bbia_sim/daemon/`
- **`app/main.py`** : Application FastAPI principale
- **`config.py`** : Configuration centralisée (dev/prod)
- **`middleware.py`** : Sécurité, rate limiting, headers
- **`models.py`** : Validations Pydantic strictes
- **`simulation_service.py`** : Service de simulation asynchrone
- **`ws/telemetry.py`** : WebSocket télémétrie temps réel

### `examples/`
- **`hello_sim.py`** : Test simulation MuJoCo
- **`goto_pose.py`** : Contrôle mouvement via API
- **`subscribe_telemetry.py`** : Télémétrie WebSocket

## Configuration

### Variables d'environnement
```bash
BBIA_ENV=dev|prod          # Mode environnement
BBIA_TOKEN=secret          # Token authentification
BBIA_CORS_ORIGINS=*        # CORS (dev) ou domaines (prod)
BBIA_SIM_HEADLESS=true     # Mode simulation
BBIA_TELEMETRY_FREQUENCY=10 # Fréquence télémétrie Hz
```

### Sécurité
- **Dev** : CORS permissif, logs verbeux
- **Prod** : CORS restrictif, headers sécurité, rate limiting

## Performance

### Simulation
- **Fréquence** : 100Hz (configurable)
- **Mode** : Headless optimisé pour API
- **Mémoire** : Buffers réutilisés, pas d'allocations dans boucle

### API
- **REST** : < 50ms réponse typique
- **WebSocket** : 10Hz télémétrie stable
- **Rate limit** : 100 req/min (prod)

## Déploiement

### Développement
```bash
uvicorn src.bbia_sim.daemon.app.main:app --reload
```

### Production
```bash
BBIA_ENV=prod BBIA_TOKEN=secure uvicorn src.bbia_sim.daemon.app.main:app
```

### Docker (futur)
```dockerfile
FROM python:3.10-slim
COPY . /app
WORKDIR /app
RUN pip install -e ".[prod]"
CMD ["uvicorn", "src.bbia_sim.daemon.app.main:app"]
```
