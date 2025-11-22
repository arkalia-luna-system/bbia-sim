# 🔍 Analyse Problèmes Duplication RAM

## Problèmes détectés

### 1. ⚠️ BBIAVision - Instances multiples possibles

**Problème** : 
- Singleton existe (`get_bbia_vision_singleton`) mais le code peut créer des instances directement avec `BBIAVision()`
- Chaque instance charge YOLO (~500MB) et MediaPipe (~100MB)
- Si 3 instances créées → ~1.8GB RAM consommée

**Fichiers concernés** :
- `src/bbia_sim/bbia_vision.py` : Singleton disponible mais pas forcé
- `src/bbia_sim/dashboard_advanced.py` : Utilise singleton (OK)
- `src/bbia_sim/bbia_behavior.py` : Utilise singleton (OK)
- Mais d'autres fichiers peuvent créer `BBIAVision()` directement

**Solution** :
- Forcer l'utilisation du singleton partout
- Ou ajouter vérification dans `__init__` pour éviter chargement modèles si instance existe déjà

---

### 2. ⚠️ BBIAHuggingFace - Instances multiples possibles

**Problème** :
- Système de tracking (`_shared_instances`) mais chaque instance charge ses propres modèles
- Modèles LLM peuvent faire 2-7GB chacun
- Si 2 instances créées → modèles dupliqués en RAM

**Fichiers concernés** :
- `src/bbia_sim/bbia_huggingface.py` : Tracking instances mais pas de singleton
- `src/bbia_sim/dashboard_advanced.py` : Crée instance si nécessaire
- `src/bbia_sim/bbia_chat.py` : Peut créer instance

**Solution** :
- Implémenter singleton pour BBIAHuggingFace
- Ou partager modèles entre instances (complexe)

---

### 3. ⚠️ start_async_scanning - Threads multiples possibles

**Problème** :
- `BBIAVision.start_async_scanning()` peut être appelé plusieurs fois
- Vérifie `if self._async_scan_active` mais si appelé rapidement, peut créer plusieurs threads
- Chaque thread consomme ~10-50MB RAM

**Fichier concerné** :
- `src/bbia_sim/bbia_vision.py` ligne 1308

**Solution** :
- Ajouter lock pour éviter création threads multiples
- Vérifier si thread existe déjà et est actif avant de créer nouveau

---

### 4. ⚠️ Animations Idle - Managers multiples possibles

**Problème** :
- `BBIIdleAnimationManager` peut être créé plusieurs fois
- Chaque manager démarre ses propres threads (respiration, poses)
- Si 2 managers créés → 4 threads supplémentaires

**Fichier concerné** :
- `src/bbia_sim/bbia_idle_animations.py`

**Solution** :
- Implémenter singleton pour BBIIdleAnimationManager
- Ou vérifier si animations déjà actives avant démarrage

---

### 5. ⚠️ SimulationService - Instances multiples possibles

**Problème** :
- `SimulationService` peut être créé plusieurs fois
- Chaque instance peut démarrer sa propre simulation MuJoCo
- Simulation MuJoCo consomme ~200-500MB RAM

**Fichiers concernés** :
- `src/bbia_sim/daemon/simulation_service.py`
- `src/bbia_sim/daemon/app/main.py` : Crée instance au démarrage
- Mais d'autres fichiers peuvent créer leurs propres instances

**Solution** :
- Utiliser singleton pour SimulationService
- Ou vérifier si simulation déjà en cours avant démarrage

---

### 6. ⚠️ Threads Watchdog - Un par instance ReachyMiniBackend

**Problème** :
- Chaque instance `ReachyMiniBackend` démarre son propre thread watchdog
- Si plusieurs instances créées → plusieurs threads watchdog
- Chaque thread consomme ~5-10MB RAM

**Fichier concerné** :
- `src/bbia_sim/backends/reachy_mini_backend.py` ligne 333

**Solution** :
- Vérification existe déjà (`if self._watchdog_thread is not None and self._watchdog_thread.is_alive()`)
- Mais si plusieurs instances → plusieurs threads
- Solution : Partager watchdog entre instances ou limiter nombre d'instances

---

## Corrections proposées

### Correction 1 : Forcer singleton BBIAVision

```python
# Dans bbia_vision.py __init__
def __init__(self, robot_api: Any | None = None) -> None:
    global _bbia_vision_singleton
    if _bbia_vision_singleton is not None:
        logger.warning("⚠️ Instance BBIAVision déjà existante, réutilisation singleton")
        # Réutiliser instance existante au lieu de créer nouvelle
        self.__dict__.update(_bbia_vision_singleton.__dict__)
        return
    # ... reste du code
```

### Correction 2 : Ajouter lock pour start_async_scanning

```python
def start_async_scanning(self, interval: float = 0.1) -> bool:
    with self._scan_lock:
        if self._async_scan_active:
            logger.warning("Scan asynchrone déjà actif")
            return False
        
        if self._scan_thread is not None and self._scan_thread.is_alive():
            logger.warning("Thread scan déjà actif")
            return False
        
        # ... reste du code
```

### Correction 3 : Singleton pour BBIIdleAnimationManager

```python
_idle_animation_manager_singleton: "BBIIdleAnimationManager | None" = None
_idle_animation_manager_lock = threading.Lock()

def get_idle_animation_manager(robot_api: "RobotAPI | None" = None) -> "BBIIdleAnimationManager":
    global _idle_animation_manager_singleton
    if _idle_animation_manager_singleton is None:
        with _idle_animation_manager_lock:
            if _idle_animation_manager_singleton is None:
                _idle_animation_manager_singleton = BBIIdleAnimationManager(robot_api)
    return _idle_animation_manager_singleton
```

### Correction 4 : Singleton pour SimulationService

```python
_simulation_service_singleton: "SimulationService | None" = None
_simulation_service_lock = threading.Lock()

def get_simulation_service(model_path: str | None = None) -> "SimulationService":
    global _simulation_service_singleton
    if _simulation_service_singleton is None:
        with _simulation_service_lock:
            if _simulation_service_singleton is None:
                _simulation_service_singleton = SimulationService(model_path)
    return _simulation_service_singleton
```

---

## Impact RAM estimé

| Problème | RAM par duplication | Si 2x dupliqué | Si 3x dupliqué |
|----------|---------------------|----------------|----------------|
| BBIAVision (YOLO+MediaPipe) | ~600MB | ~1.2GB | ~1.8GB |
| BBIAHuggingFace (modèles LLM) | ~3-7GB | ~6-14GB | ~9-21GB |
| Threads (respiration, poses, scan) | ~30MB | ~60MB | ~90MB |
| SimulationService (MuJoCo) | ~300MB | ~600MB | ~900MB |
| **TOTAL** | **~4-8GB** | **~8-16GB** | **~12-24GB** |

---

### 7. ⚠️ _start_metrics_collection - Tasks asyncio multiples possibles

**Problème** :
- `BBIAAdvancedWebSocketManager._start_metrics_collection()` peut créer plusieurs tasks asyncio si appelé rapidement
- Chaque task consomme ~10-20MB RAM et fait des collectes en parallèle
- Impact : ~20MB par task supplémentaire + CPU inutile

**Fichier concerné** :
- `src/bbia_sim/dashboard_advanced.py` ligne 500

**Solution** :
- Vérifier si `_metrics_task` existe déjà et est actif avant de créer nouveau
- ✅ CORRIGÉ : Ajout vérification `if self._metrics_task is not None and not self._metrics_task.done()`

---

## Recommandations

1. **Priorité 1** : Forcer singleton BBIAVision partout ✅ PARTIELLEMENT CORRIGÉ
2. **Priorité 2** : Implémenter singleton BBIAHuggingFace
3. **Priorité 3** : Ajouter locks pour éviter threads multiples ✅ PARTIELLEMENT CORRIGÉ
4. **Priorité 4** : Singleton pour SimulationService et BBIIdleAnimationManager
5. **Priorité 5** : Vérifier tasks asyncio avant création ✅ CORRIGÉ

---

## Vérification

Pour vérifier les duplications en cours d'exécution :

```python
import threading
import gc

# Compter instances BBIAVision
vision_instances = [obj for obj in gc.get_objects() if isinstance(obj, BBIAVision)]
print(f"Instances BBIAVision: {len(vision_instances)}")

# Compter instances BBIAHuggingFace
hf_instances = [obj for obj in gc.get_objects() if isinstance(obj, BBIAHuggingFace)]
print(f"Instances BBIAHuggingFace: {len(hf_instances)}")

# Compter threads actifs
threads = threading.enumerate()
print(f"Threads actifs: {len(threads)}")
for t in threads:
    print(f"  - {t.name}: {t.is_alive()}")
```

