# 🔍 NOUVELLES DIFFÉRENCES DÉTECTÉES - BBIA vs SDK OFFICIEL

**Date** : Octobre 2025  
**Référence** : Comparaison approfondie des endpoints `/api/move` et `backend_adapter`

---

## 🟡 DIFFÉRENCES CRITIQUES DÉTECTÉES

### 1. **Endpoint `POST /goto` - Paramètre `method` non utilisé dans SDK officiel**

**SDK Officiel** (`reachy_mini/daemon/app/routers/move.py:146-157`):
```python
@router.post("/goto")
async def goto(
    goto_req: GotoModelRequest, backend: Backend = Depends(get_backend)
) -> MoveUUID:
    """Request a movement to a specific target."""
    return create_move_task(
        backend.goto_target(
            head=goto_req.head_pose.to_pose_array() if goto_req.head_pose else None,
            antennas=np.array(goto_req.antennas) if goto_req.antennas else None,
            duration=goto_req.duration,
            # ⚠️ AUCUN paramètre method ou body_yaw passé !
        )
    )
```

**BBIA** (`bbia_sim/daemon/app/routers/move.py:155-175`):
```python
@router.post("/goto")
async def goto(
    goto_req: GotoModelRequest,
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> MoveUUID:
    """Demande un mouvement vers une cible spécifique (conforme SDK)."""
    # Convertir InterpolationMode (str) vers InterpolationTechnique si nécessaire
    method_str = (
        goto_req.interpolation.value
        if hasattr(goto_req.interpolation, "value")
        else str(goto_req.interpolation)
    )

    return create_move_task(
        backend.goto_target(
            head=goto_req.head_pose.to_pose_array() if goto_req.head_pose else None,
            antennas=np.array(goto_req.antennas) if goto_req.antennas else None,
            duration=goto_req.duration,
            method=method_str,  # ⚠️ Paramètre method ajouté (non dans SDK)
        )
    )
```

**Différence** :
- SDK officiel : N'utilise PAS le paramètre `method` (interpolation) dans `goto_target()` depuis l'endpoint
- BBIA : Passe explicitement `method=method_str` à `goto_target()`

**Impact** : 🔴 **INCOMPATIBLE** - Le SDK officiel utilise probablement une valeur par défaut (MIN_JERK) ou gère l'interpolation différemment. BBIA passe explicitement la méthode, ce qui peut causer des différences de comportement.

**Recommandation** : ⚠️ **IMPORTANT** - Vérifier comment le SDK officiel gère réellement l'interpolation. Soit :
1. Retirer le paramètre `method` pour correspondre exactement au SDK
2. OU documenter que BBIA offre cette fonctionnalité supplémentaire (extension)

---

### 2. **Endpoint `POST /goto` - Paramètre `body_yaw` non présent dans SDK**

**SDK Officiel** : Ne passe pas de `body_yaw` dans l'appel `goto_target()`

**BBIA** : Ne passe pas non plus `body_yaw` dans l'appel → ✅ **CONFORME**

---

### 3. **Endpoint `POST /play/recorded-move-dataset` - Appel `play_move` synchrone vs async**

**SDK Officiel** (`reachy_mini/daemon/app/routers/move.py:185-200`):
```python
@router.post("/play/recorded-move-dataset/{dataset_name:path}/{move_name}")
async def play_recorded_move_dataset(
    dataset_name: str,
    move_name: str,
    backend: Backend = Depends(get_backend),
) -> MoveUUID:
    """Request the robot to play a predefined recorded move from a dataset."""
    try:
        recorded_moves = RecordedMoves(dataset_name)
    except RepositoryNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    try:
        move = recorded_moves.get(move_name)
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    return create_move_task(backend.play_move(move))  # ⚠️ Appel DIRECT (synchrone)
```

**BBIA** (`bbia_sim/daemon/app/routers/move.py:213-241`):
```python
@router.post("/play/recorded-move-dataset/{dataset_name:path}/{move_name}")
async def play_recorded_move_dataset(
    dataset_name: str,
    move_name: str,
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> MoveUUID:
    """Demande au robot de jouer un mouvement enregistré depuis un dataset (conforme SDK officiel)."""
    try:
        from reachy_mini.motion.recorded_move import RecordedMoves

        recorded_moves = RecordedMoves(dataset_name)
        move = recorded_moves.get(move_name)

        # Conforme SDK officiel : play_move est async, créer coroutine
        async def play_recorded_move_coro() -> None:
            """Coroutine pour jouer un mouvement enregistré (conforme SDK)."""
            await backend.play_move(move)  # ⚠️ Utilise await (async)

        return create_move_task(play_recorded_move_coro())
```

**Différence** :
- SDK officiel : Appelle directement `backend.play_move(move)` (synchrone) dans `create_move_task()`
- BBIA : Crée une coroutine async qui appelle `await backend.play_move(move)`

**Impact** : 🔴 **INCOMPATIBLE** - Si `play_move` dans le SDK officiel n'est pas async, BBIA ne fonctionnera pas correctement.

**Vérification nécessaire** :
- Vérifier si `Backend.play_move()` est synchrone ou asynchrone dans le SDK officiel
- Vérifier si `BackendAdapter.play_move()` doit être synchrone ou asynchrone

---

### 4. **Endpoint `POST /set_target` - Paramètre `body_yaw` manquant**

**SDK Officiel** (`reachy_mini/daemon/app/routers/move.py:224-236`):
```python
@router.post("/set_target")
async def set_target(
    target: FullBodyTarget,
    backend: Backend = Depends(get_backend),
) -> dict[str, str]:
    """POST route to set a single FullBodyTarget."""
    backend.set_target(
        head=target.target_head_pose.to_pose_array()
        if target.target_head_pose
        else None,
        antennas=np.array(target.target_antennas) if target.target_antennas else None,
        # ⚠️ AUCUN body_yaw passé (utilise valeur par défaut None)
    )
    return {"status": "ok"}
```

**BBIA** (`bbia_sim/daemon/app/routers/move.py:263-293`):
```python
@router.post("/set_target")
async def set_target(
    target: FullBodyTarget,
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> dict[str, str]:
    """Définit un target directement (sans mouvement) - conforme SDK."""
    # ... conversion head_pose ...
    backend.set_target(
        head=head_pose_array,
        antennas=np.array(target.target_antennas) if target.target_antennas else None,
        # ⚠️ AUCUN body_yaw passé (utilise valeur par défaut None) → ✅ CONFORME
    )
    return {"status": "ok"}
```

**Différence** : Aucune - ✅ **CONFORME** (les deux n'utilisent pas `body_yaw`)

---

### 5. **WebSocket `/ws/set_target` - Différence d'implémentation**

**SDK Officiel** (`reachy_mini/daemon/app/routers/move.py:239-257`):
```python
@router.websocket("/ws/set_target")
async def ws_set_target(
    websocket: WebSocket, backend: Backend = Depends(ws_get_backend)
) -> None:
    """WebSocket route to stream FullBodyTarget set_target calls."""
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            try:
                target = FullBodyTarget.model_validate_json(data)
                await set_target(target, backend)  # ⚠️ Appelle la fonction set_target

            except Exception as e:
                await websocket.send_text(
                    json.dumps({"status": "error", "detail": str(e)})
                )
    except WebSocketDisconnect:
        pass
```

**BBIA** (`bbia_sim/daemon/app/routers/move.py:296-343`):
```python
@router.websocket("/ws/set_target")
async def ws_set_target(
    websocket: WebSocket,
    backend: BackendAdapter = Depends(ws_get_backend_adapter),
) -> None:
    """WebSocket pour streamer les appels set_target - conforme SDK."""
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            try:
                target = FullBodyTarget.model_validate_json(data)
                # ⚠️ Code dupliqué au lieu d'appeler set_target()
                # ... conversion head_pose inline ...
                backend.set_target(...)  # Appel direct
                await websocket.send_text(json.dumps({"status": "ok"}))
            except Exception as e:
                await websocket.send_text(
                    json.dumps({"status": "error", "detail": str(e)})
                )
    except WebSocketDisconnect:
        pass
```

**Différence** :
- SDK officiel : Réutilise la fonction `set_target()` (DRY)
- BBIA : Code dupliqué (conversion head_pose répétée)

**Impact** : 🟡 **MAINTENABILITÉ** - Code dupliqué, mais fonctionnellement équivalent.

**Recommandation** : Refactoriser pour réutiliser `set_target()` comme le SDK.

---

### 6. **BackendAdapter.play_move() - Synchrone vs Async**

**SDK Officiel** : `backend.play_move(move)` appelé directement (synchrone présumé)

**BBIA BackendAdapter** : A une méthode `play_move()` mais doit vérifier si elle doit être async.

**Vérification nécessaire** : Voir si le SDK officiel utilise `async def play_move()` ou `def play_move()`.

---

## ✅ RÉSUMÉ PRIORITÉS

### 🔴 **HAUTE PRIORITÉ - À CORRIGER**

1. **Endpoint `POST /goto` - Retirer paramètre `method`** pour correspondre exactement au SDK
   - **OU** Documenter comme extension BBIA si c'est intentionnel

2. **Endpoint `POST /play/recorded-move-dataset` - Vérifier si `play_move` est sync ou async**
   - Si sync : Retirer le wrapper async coroutine
   - Si async : Vérifier que BackendAdapter.play_move() est async

### 🟡 **MOYENNE PRIORITÉ - À AMÉLIORER**

3. **WebSocket `/ws/set_target` - Réutiliser fonction `set_target()`**
   - Refactoriser pour éviter duplication de code

---

## 🔧 CORRECTIONS PROPOSÉES

### Correction 1 : Retirer `method` de `POST /goto` (si conforme SDK)

**Fichier** : `src/bbia_sim/daemon/app/routers/move.py`

**Ligne** : ~155-175

**Changement** :
```python
# AVANT
return create_move_task(
    backend.goto_target(
        head=goto_req.head_pose.to_pose_array() if goto_req.head_pose else None,
        antennas=np.array(goto_req.antennas) if goto_req.antennas else None,
        duration=goto_req.duration,
        method=method_str,  # ⚠️ À retirer pour conformité stricte
    )
)

# APRÈS (conforme SDK officiel)
return create_move_task(
    backend.goto_target(
        head=goto_req.head_pose.to_pose_array() if goto_req.head_pose else None,
        antennas=np.array(goto_req.antennas) if goto_req.antennas else None,
        duration=goto_req.duration,
        # Pas de method - SDK utilise valeur par défaut (MIN_JERK)
    )
)
```

---

### Correction 2 : Vérifier et corriger `play_recorded_move_dataset`

**Fichier** : `src/bbia_sim/daemon/app/routers/move.py`

**Ligne** : ~213-241

**Vérification nécessaire** : Si `Backend.play_move()` dans SDK officiel est synchrone :

**Changement** :
```python
# AVANT
async def play_recorded_move_coro() -> None:
    """Coroutine pour jouer un mouvement enregistré (conforme SDK)."""
    await backend.play_move(move)

return create_move_task(play_recorded_move_coro())

# APRÈS (si SDK utilise play_move synchrone)
return create_move_task(backend.play_move(move))  # Direct, comme SDK
```

---

### Correction 3 : Réutiliser `set_target()` dans WebSocket

**Fichier** : `src/bbia_sim/daemon/app/routers/move.py`

**Ligne** : ~296-343

**Changement** :
```python
# AVANT (code dupliqué)
backend.set_target(...)  # Code inline

# APRÈS (réutilisation)
await set_target(target, backend)  # Réutiliser fonction existante
```

---

**Dernière mise à jour** : Octobre 2025

