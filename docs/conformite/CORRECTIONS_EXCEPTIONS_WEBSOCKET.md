# Corrections Gestion Exceptions et WebSocket - Conformité SDK

**Date**: octobre 2025  
**Fichiers modifiés**:
- `src/bbia_sim/daemon/app/routers/move.py`
- `src/bbia_sim/daemon/app/routers/kinematics.py`

---

## ✅ Corrections Appliquées

### 1. Import `RecordedMoves` - Position dans fichier

**Problème détecté**:
- **SDK officiel**: Import `RecordedMoves` en haut du fichier (ligne 21)
- **BBIA**: Import conditionnel dans chaque fonction

**Correction**:
```python
# AVANT
@router.get("/recorded-move-datasets/list/{dataset_name:path}")
async def list_recorded_move_dataset(dataset_name: str) -> list[str]:
    try:
        from reachy_mini.motion.recorded_move import RecordedMoves  # ❌ Import dans fonction
        ...

# APRÈS (conforme SDK)
try:
    from reachy_mini.motion.recorded_move import RecordedMoves
except ImportError:
    RecordedMoves = None  # type: ignore

# Puis dans fonction:
@router.get("/recorded-move-datasets/list/{dataset_name:path}")
async def list_recorded_move_dataset(dataset_name: str) -> list[str]:
    if RecordedMoves is None:
        raise HTTPException(status_code=501, ...)
    ...
```

---

### 2. Gestion Exceptions HTTPException - Exception Chaining

**Problème détecté**:
- **SDK officiel**: `raise HTTPException(...)` sans `from e`
- **BBIA**: `raise HTTPException(...) from e` (exception chaining)

**Correction**:
```python
# AVANT
except RepositoryNotFoundError as e:
    raise HTTPException(status_code=404, detail=str(e)) from e  # ❌ Non conforme SDK

# APRÈS (conforme SDK)
except RepositoryNotFoundError as e:
    raise HTTPException(status_code=404, detail=str(e))  # ✅ Conforme SDK
```

**Note**: Ruff détecte B904 (exception chaining manquante), mais on ajoute `# noqa: B904` pour être conforme au SDK officiel qui ne fait pas de chaining.

---

### 3. Gestion FileNotFoundError - kinematics.py

**Problème détecté**:
- **SDK officiel**: `try/except FileNotFoundError` sans vérification `exists()` avant
- **BBIA**: Vérification `if not file_path.exists()` avant try

**Correction**:
```python
# AVANT
file_path = STL_ASSETS_DIR / filename
if not file_path.exists():  # ❌ Vérification avant try
    raise HTTPException(...)
try:
    with open(file_path, "rb") as file:
        ...

# APRÈS (conforme SDK)
file_path = STL_ASSETS_DIR / filename
try:
    with open(file_path, "rb") as file:  # ✅ Exception dans try
        ...
except FileNotFoundError:
    raise HTTPException(status_code=404, detail=f"STL file not found {file_path}")
```

---

### 4. WebSocket ws_move_updates - Gestion remove()

**Problème détecté**:
- **SDK officiel**: `move_listeners.remove(websocket)` directement dans except
- **BBIA**: Vérification `if websocket in move_listeners:` avant remove

**Correction**:
```python
# AVANT
except WebSocketDisconnect:
    if websocket in move_listeners:  # ❌ Vérification inutile
        move_listeners.remove(websocket)

# APRÈS (conforme SDK)
except WebSocketDisconnect:
    move_listeners.remove(websocket)  # ✅ Direct comme SDK
```

---

### 5. notify_listeners - Gestion move_listeners.remove()

**Problème détecté**:
- **SDK officiel**: `move_listeners.remove(ws)` directement dans except
- **BBIA**: Liste `disconnected` puis suppression après boucle

**Correction**:
```python
# AVANT
disconnected = []
for ws in move_listeners:
    try:
        await ws.send_json(...)
    except (RuntimeError, WebSocketDisconnect):
        disconnected.append(ws)  # ❌ Liste puis suppression après

for ws in disconnected:
    if ws in move_listeners:
        move_listeners.remove(ws)

# APRÈS (conforme SDK)
for ws in move_listeners:
    try:
        await ws.send_json(...)
    except (RuntimeError, WebSocketDisconnect):
        move_listeners.remove(ws)  # ✅ Direct comme SDK
```

---

### 6. wrap_coro - Logging

**Problème détecté**:
- **SDK officiel**: Pas de `logger.error()` dans wrap_coro
- **BBIA**: `logger.error()` ajouté

**Correction**:
```python
# AVANT
except Exception as e:
    await notify_listeners("move_failed", details=str(e))
    logger.error(f"Erreur dans la tâche de mouvement {uuid}: {e}")  # ❌ Non présent SDK

# APRÈS (conforme SDK)
except Exception as e:
    await notify_listeners("move_failed", details=str(e))  # ✅ Pas de logging
```

---

## 📊 Résultat

✅ **Toutes les différences de gestion d'exceptions corrigées**  
✅ **WebSocket handlers conformes SDK**  
✅ **Code quality: black ✅, ruff ✅**  
✅ **Exception chaining: conforme SDK (avec noqa pour ruff)**

---

**Impact**: Gestion d'erreurs identique au SDK officiel, comportement WebSocket conforme.

