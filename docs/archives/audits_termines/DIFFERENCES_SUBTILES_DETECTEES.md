---
**⚠️ ARCHIVE - DOCUMENT HISTORIQUE ⚠️**

Ce document a été archivé car il est devenu obsolète ou a été remplacé par une version plus récente.
Il est conservé à des fins de référence historique uniquement.

**Date d'archivage** : Oct / No2025025025025025
**Raison** : Document terminé/obsolète/remplacé
---

# 🔍 DIFFÉRENCES SUBTILES DÉTECTÉES - BBIA vs SDK OFFICIEL

**Date** : Oct / No2025025025025025  
**Référence** : Comparaison exhaustive `bbia-reachy-sim` vs `pollen-robotics/reachy_mini` (branch develop)

---

## 📋 RÉSUMÉ

Différences subtiles mais importantes détectées lors de l'analyse approfondie des signatures, validations et comportements.

---

## 🟡 DIFFÉRENCES DÉTECTÉES

### 1. **Signature `goto_target()` - Type `head`**

**SDK Officiel** (`reachy_mini.py:232-241`):
```python
def goto_target(
    self,
    head: Optional[npt.NDArray[np.float64]] = None,  # 4x4 pose matrix
    antennas: Optional[Union[npt.NDArray[np.float64], List[float]]] = None,
    duration: float = 0.5,
    method: InterpolationTechnique = InterpolationTechnique.MIN_JERK,
    body_yaw: float | None = 0.0,  # Body yaw angle in radians
) -> None:
```

**BBIA** (`reachy_mini_backend.py:911-918`):
```python
def goto_target(
    self,
    head: Optional["HeadPose"] = None,
    antennas: npt.NDArray[np.float64] | list[float] | None = None,
    duration: float = 0.5,
    method: str = "minjerk",
    body_yaw: float = 0.0,
) -> None:
```

**Différence** :
- ✅ **Type `head`** : SDK attend `npt.NDArray[np.float64]` (matrice 4x4), BBIA utilise type hint `HeadPose` (alias/TypeVar)
- ⚠️ **Type `body_yaw`** : SDK accepte `float | None = 0.0` (None pour garder position actuelle), BBIA force `float = 0.0`
- ✅ **Type `method`** : SDK utilise `InterpolationTechnique` enum, BBIA accepte `str` avec conversion (plus flexible)

**Impact** : Mineur - BBIA convertit automatiquement, mais le type hint est moins précis.

**Recommandation** : Aligner les type hints pour correspondre exactement au SDK (optionnel, car fonctionnel).

---

### 2. **Validation `duration` dans `goto_target()`**

**SDK Officiel** (`reachy_mini.py:260-263`):
```python
if duration <= 0.0:
    raise ValueError(
        "Duration must be positive and non-zero. Use set_target() for immediate position setting."
    )
```

**BBIA** (`reachy_mini_backend.py:922-923`):
```python
if duration_float < 0.0:
    raise ValueError(f"Duration must be >= 0, got {duration_float}")
```

**Différence** :
- SDK : `duration <= 0.0` (rejette 0)
- BBIA : `duration < 0.0` (accepte 0)

**Impact** : Mineur - BBIA accepte `duration=0`, ce qui est équivalent à `set_target()` selon la doc SDK.

**Recommandation** : Aligner sur SDK (`<= 0.0`) pour conformité stricte (optionnel).

---

### 3. **Validation `look_at_image()` - Caméra**

**SDK Officiel** (`reachy_mini.py:348-349`):
```python
if self.media_manager.camera is None:
    raise RuntimeError("Camera is not initialized.")
```

**BBIA** (`reachy_mini_backend.py:887-889`):
```python
if not self.is_connected or not self.robot:
    logger.info(f"Mode simulation: look_at_image({u}, {v})")
    return np.eye(4, dtype=np.float64)  # Retourne pose par défaut
```

**Différence** :
- SDK : Lève `RuntimeError` si caméra non initialisée
- BBIA : Retourne pose par défaut en mode simulation (pas d'erreur)

**Impact** : Comportement différent - SDK strict, BBIA permissif.

**Recommandation** : Si caméra non disponible, lever `RuntimeError` comme SDK (ou option pour mode simulation).

---

### 4. **Validation coordonnées `look_at_image()`**

**SDK Officiel** (`reachy_mini.py:351-356`):
```python
assert 0 < u < self.media_manager.camera.resolution[0], (
    f"u must be in [0, {self.media_manager.camera.resolution[0]}], got {u}."
)
assert 0 < v < self.media_manager.camera.resolution[1], (
    f"v must be in [0, {self.media_manager.camera.resolution[1]}], got {v}."
)
```

**BBIA** (`reachy_mini_backend.py:891-909`):
```python
try:
    result = self.robot.look_at_image(u, v, duration, perform_movement)
    # ... conversion ...
except Exception as e:
    logger.error(f"Erreur look_at_image: {e}")
    return np.eye(4, dtype=np.float64)  # Pas de validation explicite des coordonnées
```

**Différence** :
- SDK : Valide explicitement avec `assert` (strict)
- BBIA : Laisse le SDK valider (pas de pré-validation)

**Impact** : Mineur - Le SDK valide, mais BBIA ne pré-vérifie pas.

**Recommandation** : Ajouter validation explicite avant appel SDK pour messages d'erreur plus clairs (optionnel).

---

### 5. **Type retour `look_at_world()`**

**SDK Officiel** (`reachy_mini.py:384-391`):
```python
def look_at_world(
    self,
    x: float,
    y: float,
    z: float,
    duration: float = 1.0,
    perform_movement: bool = True,
) -> npt.NDArray[np.float64]:  # Toujours retourne matrice 4x4
```

**BBIA** (`reachy_mini_backend.py:1289-1296`):
```python
def look_at_world(
    self,
    x: float,
    y: float,
    z: float,
    duration: float = 1.0,
    perform_movement: bool = True,
) -> npt.NDArray[np.float64] | None:  # Peut retourner None
```

**Différence** :
- SDK : Retourne toujours `npt.NDArray[np.float64]` (jamais None)
- BBIA : Retourne `npt.NDArray[np.float64] | None` (peut retourner None en cas d'erreur)

**Impact** : Mineur - BBIA gère mieux les erreurs mais type moins strict.

**Recommandation** : Si nécessaire, aligner sur SDK (toujours retourner matrice, lever exception si erreur).

---

### 6. **Paramètre `body_yaw` dans `goto_target()` - Sémantique**

**SDK Officiel** (`reachy_mini.py:240`):
```python
body_yaw: float | None = 0.0,  # Body yaw angle in radians. Use None to keep the current yaw.
```

**BBIA** (`reachy_mini_backend.py:917`):
```python
body_yaw: float = 0.0,  # Pas de support pour None
```

**Différence** :
- SDK : `body_yaw=None` signifie "garder position actuelle"
- BBIA : `body_yaw=0.0` signifie toujours "aller à 0.0"

**Impact** : Fonctionnel différent - Impossible de garder position actuelle du body_yaw avec BBIA.

**Recommandation** : ⚠️ **IMPORTANT** - Ajouter support `body_yaw=None` pour conformité SDK.

---

### 7. **Validation `set_target()` - Vérification arguments**

**SDK Officiel** (`reachy_mini.py:190-204`):
```python
if head is None and antennas is None and body_yaw is None:
    raise ValueError(
        "At least one of head, antennas or body_yaw must be provided."
    )

if head is not None and not head.shape == (4, 4):
    raise ValueError(f"Head pose must be a 4x4 matrix, got shape {head.shape}.")

if antennas is not None and not len(antennas) == 2:
    raise ValueError(
        "Antennas must be a list or 1D np array with two elements."
    )

if body_yaw is not None and not isinstance(body_yaw, (int, float)):
    raise ValueError("body_yaw must be a float.")
```

**BBIA** (`reachy_mini_backend.py:1089-1103`):
```python
def set_target(
    self,
    head: Optional["HeadPose"] = None,
    antennas: npt.NDArray[np.float64] | list[float] | None = None,
    body_yaw: float | None = None,
) -> None:
    # ... délègue directement au SDK sans pré-validation
    self.robot.set_target(head=head, antennas=antennas, body_yaw=body_yaw)
```

**Différence** :
- SDK : Valide explicitement avant appel
- BBIA : Délègue validation au SDK (pas de pré-validation)

**Impact** : Mineur - Validation se fait quand même (dans SDK), mais messages d'erreur moins directs.

**Recommandation** : Optionnel - Pré-valider pour messages d'erreur plus clairs.

---

## ✅ RÉSUMÉ PRIORITÉS

### 🔴 **HAUTE PRIORITÉ**
1. **Support `body_yaw=None` dans `goto_target()`** - Fonctionnalité manquante SDK

### 🟡 **MOYENNE PRIORITÉ**
2. Aligner validation `duration` (`<= 0.0` vs `< 0.0`)
3. Type hint `head` pour correspondre exactement au SDK (optionnel si conversion automatique fonctionne)

### 🟢 **BASSE PRIORITÉ (Optionnel)**
4. Validation explicite coordonnées `look_at_image()`
5. Validation explicite arguments `set_target()`
6. Aligner type retour `look_at_world()` (toujours matrice, jamais None)

---

## 🔧 CORRECTIONS RECOMMANDÉES

### Correction 1 : Support `body_yaw=None` dans `goto_target()`

**Fichier** : `src/bbia_sim/backends/reachy_mini_backend.py`

**Ligne** : ~917

**Changement** :
```python
# AVANT
body_yaw: float = 0.0,

# APRÈS
body_yaw: float | None = 0.0,  # None = garder position actuelle (conforme SDK)
```

Et dans l'appel SDK (~993) :
```python
# AVANT
body_yaw=float(body_yaw) if body_yaw is not None else 0.0,

# APRÈS
body_yaw=body_yaw,  # Passer None directement si None (SDK gère)
```

---

### Correction 2 : Validation `duration` stricte

**Fichier** : `src/bbia_sim/backends/reachy_mini_backend.py`

**Ligne** : ~922

**Changement** :
```python
# AVANT
if duration_float < 0.0:
    raise ValueError(f"Duration must be >= 0, got {duration_float}")

# APRÈS
if duration_float <= 0.0:
    raise ValueError(
        "Duration must be positive and non-zero. Use set_target() for immediate position setting."
    )
```

---

## 📝 NOTES

- **Type hints** : Les différences de type hints (`HeadPose` vs `npt.NDArray`) sont généralement acceptables si la conversion automatique fonctionne correctement.
- **Validation** : Les validations peuvent être déléguées au SDK sans problème, mais pré-valider améliore les messages d'erreur.
- **Comportement simulation** : BBIA retourne des valeurs par défaut en mode simulation, ce qui est acceptable mais diffère du SDK strict.

---

**Dernière mise à jour** : Oct / No2025025025025025

