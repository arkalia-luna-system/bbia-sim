# Corrections Routers et Prefixes - Conformité SDK Officiel

**Date**: 1er Octobre 2025  
**Fichiers modifiés**: 
- `src/bbia_sim/daemon/app/main.py`
- `src/bbia_sim/daemon/app/routers/*.py`

---

## ✅ Corrections Appliquées

### 1. Structure des Prefixes Routers

**Avant** (BBIA):
- Prefixes définis dans `main.py` lors de l'inclusion
- Exemple: `app.include_router(state.router, prefix="/api/state")`

**Après** (Conforme SDK):
- Prefixes définis dans la déclaration du router
- Structure avec router parent `/api` incluant sous-routers
- Exemple: `router = APIRouter(prefix="/state")` dans `state.py`

### 2. Routers Corrigés

| Router | Ancien Prefix | Nouveau Prefix | Fichier |
|--------|---------------|----------------|---------|
| state | `/api/state` (main.py) | `/state` (dans router) | `routers/state.py` |
| move | `/api/move` (main.py) | `/move` (dans router) | `routers/move.py` |
| motors | `/api/motors` (main.py) | `/motors` (dans router) | `routers/motors.py` |
| kinematics | `/api/kinematics` (main.py) | `/kinematics` (dans router) | `routers/kinematics.py` |
| daemon | `/api/daemon` (main.py) | `/daemon` (dans router) | `routers/daemon.py` |
| apps | `/api/apps` (main.py) | `/apps` (dans router) | `routers/apps.py` |

### 3. Structure main.py

**Avant**:
```python
app.include_router(state.router, prefix="/api/state", ...)
app.include_router(move.router, prefix="/api/move", ...)
```

**Après** (Conforme SDK):
```python
api_router = APIRouter(prefix="/api")
api_router.include_router(state.router)
api_router.include_router(move.router)
api_router.include_router(motors.router)
api_router.include_router(daemon.router)
api_router.include_router(kinematics.router)
api_router.include_router(apps.router)
app.include_router(api_router, dependencies=[Depends(verify_token)])
```

### 4. Endpoint /goto

**Correction**: Retirer le paramètre `method` de l'appel `goto_target()` pour correspondre exactement au SDK officiel qui ne le passe pas (utilise valeur par défaut `MIN_JERK`).

---

## 📊 Résultat

✅ **Tous les routers ont maintenant leur prefix défini dans leur déclaration**  
✅ **Structure conforme SDK avec router parent `/api`**  
✅ **Endpoints identiques: `/api/state/*`, `/api/move/*`, etc.**  
✅ **Code style: black ✅, ruff ✅**

---

**Impact**: Aucun changement d'URL pour les clients (tous les endpoints restent `/api/...`), mais structure interne conforme SDK.

