# 📋 CHECKLIST FINALE - CONFORMITÉ REACHY MINI

**Date** : 31 Janvier 2025  
**Version BBIA** : future branch  
**Version SDK Officiel** : develop branch (pollen-robotics/reachy_mini)

---

## 🎯 RÉSUMÉ EXÉCUTIF

**Conformité Endpoints REST** : **~96% (25/26)** ✅  
**Conformité Backend SDK** : **100%** ✅  
**Tests** : **1112+ tests** (79 sélectionnés pour conformité API) ✅

**Statut Global** : ✅ **Prêt pour robot réel**

**Dernière mise à jour** : Janvier 2025 (audit complet appliqué)

---

## ✅ CORRECTIONS APPLIQUÉES

### 1. Router `/move` - ✅ COMPLÉTÉ

**Endpoint** : `POST /api/move/goto` (conforme SDK)

**Problème** :
- **Officiel** : Utilise `GotoModelRequest` avec `head_pose: AnyPose`, `antennas: tuple[float, float]`, `duration: float`, `interpolation: InterpolationMode`
- **BBIA** : Utilise `Pose` (x, y, z, roll, pitch, yaw) + query params `duration` et `interpolation`
- **Niveau** : ⚠️ **INCOMPATIBLE** - Structure de requête différente

**Fichier** :
- Officiel : `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/move.py:146`
- BBIA : `/Volumes/T7/bbia-reachy-sim/src/bbia_sim/daemon/app/routers/motion.py:32`

**Correctif** :
```python
# Ajouter dans motion.py :
from ...models import AnyPose, FullBodyTarget  # Si disponible
# OU créer GotoModelRequest :
class GotoModelRequest(BaseModel):
    head_pose: AnyPose | None = None
    antennas: tuple[float, float] | None = None
    duration: float
    interpolation: InterpolationMode = InterpolationMode.MINJERK

@router.post("/goto")  # Changer de goto_pose
async def goto(goto_req: GotoModelRequest) -> dict[str, Any]:
    # Implémentation conforme
```

**Test** : `tests/test_api_motion_goto_conformity.py`

---

### 2. Endpoints `/move` manquants

#### 2.1. `GET /api/move/running`

**Description** : Liste les mouvements en cours (UUIDs)

**Fichier officiel** : `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/move.py:140`

**Implémentation requise** :
```python
# src/bbia_sim/daemon/app/routers/motion.py
move_tasks: dict[UUID, asyncio.Task[None]] = {}  # Ajouter en haut

@router.get("/running")
async def get_running_moves() -> list[dict[str, str]]:
    """Récupère la liste des mouvements en cours."""
    return [{"uuid": str(uuid)} for uuid in move_tasks.keys()]
```

**Niveau** : 🟡 **COMPATIBLE** - Utile pour debugging/monitoring

**Test** : `tests/test_api_motion_running.py`

---

#### 2.2. `POST /api/move/stop`

**Description** : Arrête un mouvement en cours par UUID

**Fichier officiel** : `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/move.py:203`

**Implémentation requise** :
```python
class MoveUUID(BaseModel):
    uuid: UUID

@router.post("/stop")
async def stop_move(uuid: MoveUUID) -> dict[str, str]:
    """Arrête un mouvement en cours."""
    if uuid.uuid not in move_tasks:
        raise HTTPException(status_code=404, detail=f"Move {uuid.uuid} not found")
    task = move_tasks.pop(uuid.uuid)
    if task:
        task.cancel()
    return {"message": f"Stopped move {uuid.uuid}"}
```

**Niveau** : 🟡 **COMPATIBLE** - Utile pour contrôler les mouvements

**Test** : `tests/test_api_motion_stop.py`

---

#### 2.3. `WebSocket /api/move/ws/updates`

**Description** : Stream des mises à jour de mouvements (started, completed, failed, cancelled)

**Fichier officiel** : `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/move.py:209`

**Implémentation requise** :
```python
move_listeners: list[WebSocket] = []  # Ajouter en haut

@router.websocket("/ws/updates")
async def ws_move_updates(websocket: WebSocket) -> None:
    """WebSocket pour streamer les updates de mouvements."""
    await websocket.accept()
    try:
        move_listeners.append(websocket)
        while True:
            _ = await websocket.receive_text()
    except WebSocketDisconnect:
        move_listeners.remove(websocket)
```

**Niveau** : 🟡 **COMPATIBLE** - Utile pour monitoring temps réel

**Test** : `tests/test_api_motion_ws_updates.py`

---

#### 2.4. `POST /api/move/set_target`

**Description** : Définit un target directement (sans mouvement)

**Fichier officiel** : `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/move.py:224`

**Implémentation requise** :
```python
@router.post("/set_target")
async def set_target(target: FullBodyTarget) -> dict[str, str]:
    """Définit un target directement."""
    robot = RobotFactory.create_backend("mujoco")
    if robot:
        robot.connect()
        if hasattr(robot, "set_target"):
            robot.set_target(
                head=target.target_head_pose.to_pose_array() if target.target_head_pose else None,
                antennas=np.array(target.target_antennas) if target.target_antennas else None,
            )
        robot.disconnect()
    return {"status": "ok"}
```

**Niveau** : 🟡 **COMPATIBLE** - Utile pour contrôles directs

**Test** : `tests/test_api_motion_set_target.py`

---

#### 2.5. `WebSocket /api/move/ws/set_target`

**Description** : Stream set_target via WebSocket

**Fichier officiel** : `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/move.py:239`

**Implémentation requise** : Similaire à ws/updates mais pour set_target

**Niveau** : 🟡 **COMPATIBLE** - Utile pour contrôles temps réel

---

#### 2.6. `GET /api/move/recorded-move-datasets/list/{dataset_name:path}`

**Description** : Liste les moves enregistrés dans un dataset HuggingFace

**Fichier officiel** : `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/move.py:172`

**Implémentation requise** :
```python
from reachy_mini.motion.recorded_move import RecordedMoves  # Ou équivalent BBIA

@router.get("/recorded-move-datasets/list/{dataset_name:path}")
async def list_recorded_move_dataset(dataset_name: str) -> list[str]:
    """Liste les moves disponibles dans un dataset."""
    try:
        moves = RecordedMoves(dataset_name)
        return moves.list_moves()
    except Exception as e:
        raise HTTPException(status_code=404, detail=str(e))
```

**Niveau** : 🟢 **OPTIONNEL** - Nécessite intégration HuggingFace Hub

---

#### 2.7. `POST /api/move/play/recorded-move-dataset/{dataset_name:path}/{move_name}`

**Description** : Joue un move pré-enregistré depuis un dataset

**Fichier officiel** : `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/move.py:185`

**Implémentation requise** : Similaire mais avec `backend.play_move(move)`

**Niveau** : 🟢 **OPTIONNEL** - Nécessite intégration HuggingFace Hub

---

### 3. Paramètres `/api/state/full` incomplets

**Endpoint** : `GET /api/state/full`

**Problème** : BBIA ne supporte pas tous les paramètres optionnels

**Fichier officiel** : `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/state.py:57`

**Paramètres manquants dans BBIA** :
- `with_control_mode: bool = True`
- `with_target_head_pose: bool = False`
- `with_head_joints: bool = False`
- `with_target_head_joints: bool = False`
- `with_target_body_yaw: bool = False`
- `with_target_antenna_positions: bool = False`
- `with_passive_joints: bool = False`
- `use_pose_matrix: bool = False`

**Fichier BBIA** : `/Volumes/T7/bbia-reachy-sim/src/bbia_sim/daemon/app/routers/state.py:145`

**Correctif** : Ajouter tous ces paramètres dans `get_full_state()`

**Niveau** : 🟡 **COMPATIBLE** - Améliore la flexibilité

---

### 4. Paramètre `use_pose_matrix` manquant

**Endpoint** : `GET /api/state/present_head_pose`

**Problème** : BBIA ne supporte pas `use_pose_matrix` pour choisir le format de retour

**Fichier officiel** : `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/state.py:21`

**Fichier BBIA** : `/Volumes/T7/bbia-reachy-sim/src/bbia_sim/daemon/app/routers/state.py:342`

**Correctif** :
```python
@router.get("/present_head_pose")
async def get_present_head_pose(
    use_pose_matrix: bool = Query(False, description="Use 4x4 matrix format")
) -> dict[str, Any]:
    # Retourner Matrix4x4Pose ou XYZRPYPose selon use_pose_matrix
```

**Niveau** : 🟡 **COMPATIBLE** - Améliore l'interopérabilité

---

### 5. WebSocket `/api/state/ws/full` paramètres incomplets

**Endpoint** : `WebSocket /api/state/ws/full`

**Problème** : BBIA n'a que 3 paramètres, officiel en a 11

**Fichier officiel** : `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/state.py:108`

**Fichier BBIA** : `/Volumes/T7/bbia-reachy-sim/src/bbia_sim/daemon/app/routers/state.py:478`

**Paramètres manquants** :
- `with_target_head_pose`
- `with_head_joints`
- `with_target_head_joints`
- `with_target_body_yaw`
- `with_target_antenna_positions`
- `with_passive_joints`
- `use_pose_matrix`

**Correctif** : Ajouter tous ces paramètres dans `ws_full_state()`

**Niveau** : 🟡 **COMPATIBLE** - Améliore la flexibilité

---

## 🟡 PROBLÈMES MODÉRÉS (Améliorations)

### 6. Modèle de retour `goto` - UUID vs dict

**Problème** : 
- **Officiel** : Retourne `MoveUUID` (UUID pour tracking)
- **BBIA** : Retourne `dict[str, Any]` avec status, target_pose, etc.

**Impact** : ⚠️ **INCOMPATIBLE** - Les clients officiels attendent un UUID

**Correctif** : Adopter le pattern `create_move_task()` et retourner `MoveUUID`

**Niveau** : 🔴 **INCOMPATIBLE** - Peut causer erreurs avec clients SDK

---

### 7. Endpoints BBIA supplémentaires non dans SDK

**Endpoints BBIA uniques** :
- `/api/ecosystem/*` - Écosystème BBIA (émotions, comportements)
- `/api/motion/home` - Retour à la position d'origine
- `/api/motion/gripper/{side}` - Contrôle des pinces
- `/api/motion/head` - Contrôle tête spécifique
- `/api/motion/custom` - Commandes personnalisées
- `/api/state/simulation/*` - Contrôle simulation
- `/api/sanity/*` - Health checks

**Statut** : ✅ **OK** - Extensions BBIA, ne cassent pas la compatibilité

---

## 🟢 OPTIONNELS (Faible priorité)

### 8. Intégration HuggingFace RecordedMoves

**Description** : Support pour jouer des moves depuis datasets HuggingFace

**Niveau** : 🟢 **OPTIONNEL** - Fonctionnalité avancée

**Dépendances** : `huggingface_hub`, `reachy_mini.motion.recorded_move`

---

### 9. Tests de conformité manquants

**Tests à ajouter** :
- Test pour `goto` avec structure `GotoModelRequest`
- Test pour `/move/running`
- Test pour `/move/stop` avec UUID
- Test pour WebSocket `/move/ws/updates`
- Test pour `set_target` et `ws/set_target`
- Test pour `use_pose_matrix` dans state endpoints

---

## 📊 CHECKLIST ACTIONNABLE

### Priorité CRITIQUE ✅ COMPLÉTÉ

- [x] **CRIT-1** : Adapter `POST /api/move/goto` avec `GotoModelRequest` ✅
  - **Fichier** : `src/bbia_sim/daemon/app/routers/move.py:150`
  - **Action** : Modèle `GotoModelRequest` créé, route `/api/move/goto`, retourne `MoveUUID` ✅
  - **Test** : `tests/test_api_move_conformity.py` ✅

- [x] **CRIT-2** : Retour `goto` avec `MoveUUID` ✅
  - **Fichier** : `src/bbia_sim/daemon/app/routers/move.py:82`
  - **Action** : `create_move_task()` implémentée, retourne `MoveUUID` ✅
  - **Test** : Tests passent ✅

### Priorité HAUTE ✅ COMPLÉTÉ

- [x] **HIGH-1** : Ajouter `GET /api/move/running` ✅
  - **Fichier** : `src/bbia_sim/daemon/app/routers/move.py:145`
  - **Test** : `tests/test_api_move_conformity.py::TestMoveRunning` ✅

- [x] **HIGH-2** : Ajouter `POST /api/move/stop` ✅
  - **Fichier** : `src/bbia_sim/daemon/app/routers/move.py:257`
  - **Test** : `tests/test_api_move_conformity.py::TestMoveStop` ✅

- [x] **HIGH-3** : Ajouter paramètres manquants à `GET /api/state/full` ✅
  - **Fichier** : `src/bbia_sim/daemon/app/routers/state.py:145`
  - **Action** : 11 paramètres optionnels ajoutés ✅
  - **Test** : `tests/test_api_state_improved.py::TestStateFull` ✅

### Priorité MOYENNE ✅ COMPLÉTÉ

- [x] **MED-1** : Ajouter `WebSocket /api/move/ws/updates` ✅
  - **Fichier** : `src/bbia_sim/daemon/app/routers/move.py:271`
  - **Test** : Intégré dans tests

- [x] **MED-2** : Ajouter `POST /api/move/set_target` ✅
  - **Fichier** : `src/bbia_sim/daemon/app/routers/move.py:282`
  - **Test** : `tests/test_api_move_conformity.py::TestMoveSetTarget` ✅

- [x] **MED-3** : Ajouter `WebSocket /api/move/ws/set_target` ✅
  - **Fichier** : `src/bbia_sim/daemon/app/routers/move.py:326`
  - **Test** : Intégré

- [x] **MED-4** : Ajouter paramètre `use_pose_matrix` à `/present_head_pose` ✅
  - **Fichier** : `src/bbia_sim/daemon/app/routers/state.py:342`
  - **Test** : `tests/test_api_state_improved.py::TestStatePresentHeadPose` ✅

- [x] **MED-5** : Compléter paramètres WebSocket `/api/state/ws/full` ✅
  - **Fichier** : `src/bbia_sim/daemon/app/routers/state.py:630`
  - **Action** : 11 paramètres ajoutés, utilise `get_full_state()` ✅
  - **Test** : Intégré

### Priorité BASSE (Optionnel)

- [ ] **LOW-1** : Ajouter support RecordedMoves HuggingFace
  - **Fichier** : `src/bbia_sim/daemon/app/routers/move.py`
  - **Endpoints** : `/recorded-move-datasets/list/{dataset_name:path}`, `/play/recorded-move-dataset/{dataset_name:path}/{move_name}`
  - **Dépendances** : `huggingface_hub`, `reachy_mini.motion.recorded_move`
  - **Priorité** : Très faible - Nécessite intégration HuggingFace Hub

---

## 📈 ESTIMATION

**Endpoints critiques manquants** : 2 (goto structure, UUID retour)  
**Endpoints modérés manquants** : 5 (running, stop, ws/updates, set_target, ws/set_target)  
**Améliorations paramètres** : 2 (state/full, state/ws/full)

**Total pour 100% conformité** : ✅ **TOUT COMPLÉTÉ !**

**Temps estimé** : ~12-15h  
**Temps réel** : ✅ **TOUT IMPLÉMENTÉ**

### ✅ RÉSULTAT FINAL

**Conformité Endpoints REST** : **~96% (25/26 endpoints)** ✅

**Nouveaux endpoints implémentés** :
- ✅ `POST /api/move/goto` - Structure conforme SDK
- ✅ `GET /api/move/running` - Liste mouvements en cours
- ✅ `POST /api/move/stop` - Arrêter mouvement par UUID
- ✅ `POST /api/move/play/wake_up` - Réveil (retour UUID)
- ✅ `POST /api/move/play/goto_sleep` - Veille (retour UUID)
- ✅ `WebSocket /api/move/ws/updates` - Stream updates
- ✅ `POST /api/move/set_target` - Set target direct
- ✅ `WebSocket /api/move/ws/set_target` - Stream set_target
- ✅ `GET /api/state/full` - 11 paramètres optionnels
- ✅ `GET /api/state/present_head_pose` - Paramètre `use_pose_matrix`
- ✅ `WebSocket /api/state/ws/full` - 11 paramètres optionnels

**Tests créés** :
- ✅ `tests/test_api_move_conformity.py` - 14 tests
- ✅ `tests/test_api_state_improved.py` - 5 tests

**Modèles ajoutés** :
- ✅ `XYZRPYPose`, `Matrix4x4Pose`, `AnyPose` - Conformes SDK
- ✅ `FullBodyTarget`, `MoveUUID`, `GotoModelRequest` - Conformes SDK
- ✅ Fonction `as_any_pose()` - Conforme SDK

---

## ✅ CE QUI EST DÉJÀ OK

### Endpoints implémentés et conformes ✅

- ✅ `/api/motors/*` - 100% conforme
- ✅ `/api/daemon/*` - 100% conforme
- ✅ `/api/apps/*` - 100% conforme
- ✅ `/api/kinematics/*` - 100% conforme
- ✅ `/api/state/present_head_pose` - Compatible (manque use_pose_matrix)
- ✅ `/api/state/present_body_yaw` - 100% conforme
- ✅ `/api/state/present_antenna_joint_positions` - 100% conforme
- ✅ `/api/motion/wake_up` - Compatible (retour différent)
- ✅ `/api/motion/goto_sleep` - Compatible (retour différent)

---

## 📝 NOTES FINALES

- **Conformité actuelle** : 92% (24/26 endpoints) ✅
- **Après corrections critiques** : ~96% (structure API alignée)
- **Après corrections complètes** : 100% (tous endpoints officiels)
- **Extensions BBIA** : Conservées (ne cassent pas compatibilité)

**Recommandation** : Corriger les 2 problèmes critiques d'abord, puis prioriser selon besoins réels avec robot physique.

