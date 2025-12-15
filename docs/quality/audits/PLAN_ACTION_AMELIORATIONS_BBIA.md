# 🎯 PLAN D'ACTION DÉTAILLÉ - Améliorations BBIA

**Date** : 15 Décembre 2025  
**Objectif** : Plan d'action détaillé pour les améliorations les plus importantes avec tests associés  
**Référence** : [`TOP_AMELIORATIONS_IMPORTANTES_BBIA.md`](TOP_AMELIORATIONS_IMPORTANTES_BBIA.md)

Ce plan réutilise les documents existants et évite les doublons.

---

## 📋 RÉSUMÉ EXÉCUTIF

**Top 5 améliorations** avec plan d'action détaillé :

1. 🔴 Fallback automatique sim → robot (2-3h)
2. 🔴 Heartbeat WebSocket adaptatif (3-4h)
3. 🟡 Finaliser découverte automatique robots (2-3h)
4. 🟡 Lifespan context manager robuste (3-4h)
5. 🟡 Mode simplifié dashboard (4-6h)

**Total** : 14-20h avec tests complets

---

## 🔴 AMÉLIORATION 1 : Fallback Automatique Sim → Robot

### Audit de l'Existant

**Fichiers concernés** :

- ✅ `src/bbia_sim/robot_factory.py` (lignes 21-75) - `create_backend()` existe
- ✅ `src/bbia_sim/backends/reachy_backend.py` (lignes 100-106) - Fallback partiel
- ✅ `src/bbia_sim/dashboard_advanced.py` (lignes 3669-3683) - Fallback partiel
- ✅ `tests/test_robot_factory.py` (26 tests) - Tests existants

**Ce qui existe** :

- ✅ `create_backend()` avec support `mujoco`, `reachy`, `reachy_mini`
- ✅ Fallback partiel dans `reachy_backend.py` (catch exceptions → sim)
- ✅ Fallback partiel dans `dashboard_advanced.py` (try reachy_mini → mujoco)
- ✅ Tests complets pour `create_backend()` (26 tests)

**Ce qui manque** :

- ❌ Support `backend='auto'` dans `create_backend()`
- ❌ Détection automatique robot disponible
- ❌ Fallback systématique robot → sim
- ❌ Tests pour mode `auto`

### Plan d'Action Détaillé

#### Étape 1 : Modifier `robot_factory.py` (1h)

**Fichier** : `src/bbia_sim/robot_factory.py`

**Modifications** :

```python
@staticmethod
def create_backend(
    backend_type: str = "mujoco",
    **kwargs: Any,
) -> "RobotAPI | None":
    """Crée un backend RobotAPI.
    
    Args:
        backend_type: Type de backend ("mujoco", "reachy", "reachy_mini", ou "auto")
            - "auto": Détecte automatiquement robot, fallback vers sim si absent
        **kwargs: Arguments spécifiques au backend
    """
    # NOUVEAU: Support mode "auto"
    if backend_type.lower() == "auto":
        # Essayer robot réel d'abord
        try:
            backend = RobotFactory.create_backend("reachy_mini", use_sim=False, **kwargs)
            if backend and backend.is_connected:
                logger.info("✅ Robot réel détecté et connecté")
                return backend
        except Exception as e:
            logger.debug("Robot réel non disponible: %s", e)
        
        # Fallback vers simulation
        logger.info("⚠️ Robot réel non disponible, utilisation simulation")
        return RobotFactory.create_backend("mujoco", **kwargs)
    
    # ... reste du code existant ...
```

**Tests à créer** : `tests/test_robot_factory_auto_fallback.py` (voir section Tests)

#### Étape 2 : Tests (1h)

**Fichier** : `tests/test_robot_factory_auto_fallback.py` (NOUVEAU)

**Tests à créer** :

- `test_auto_detects_robot_when_available()` - Détecte robot si disponible
- `test_auto_fallback_to_sim_when_robot_unavailable()` - Fallback sim si robot absent
- `test_auto_handles_connection_errors()` - Gestion erreurs connexion
- `test_auto_preserves_kwargs()` - Préserve kwargs lors fallback

**Critères** :

- ✅ Tests légers (< 1s chacun)
- ✅ Utilisent mocks (pas de robot réel requis)
- ✅ Coverage 100% du code ajouté

#### Étape 3 : Mise à jour documentation (30min)

**Fichier** : Réutiliser `TOP_AMELIORATIONS_IMPORTANTES_BBIA.md` (pas de nouveau MD)

**Actions** :

- Mettre à jour section "Fallback automatique" avec statut "✅ FAIT"
- Ajouter exemple code dans section existante

### Tests Détaillés

**Fichier** : `tests/test_robot_factory_auto_fallback.py`

```python
#!/usr/bin/env python3
"""Tests pour fallback automatique sim → robot."""

from unittest.mock import MagicMock, patch

import pytest

from bbia_sim.robot_factory import RobotFactory
from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


class TestAutoFallback:
    """Tests pour mode 'auto' avec fallback automatique."""
    
    def test_auto_detects_robot_when_available(self):
        """Test détection robot si disponible."""
        with patch('bbia_sim.robot_factory.ReachyMiniBackend') as mock_backend:
            mock_instance = MagicMock()
            mock_instance.is_connected = True
            mock_backend.return_value = mock_instance
            
            backend = RobotFactory.create_backend('auto')
            
            assert backend is not None
            assert backend.is_connected is True
            mock_backend.assert_called_once()
    
    def test_auto_fallback_to_sim_when_robot_unavailable(self):
        """Test fallback sim si robot absent."""
        with patch('bbia_sim.robot_factory.ReachyMiniBackend', side_effect=Exception("No robot")):
            with patch('bbia_sim.robot_factory.MuJoCoBackend') as mock_sim:
                mock_instance = MagicMock()
                mock_sim.return_value = mock_instance
                
                backend = RobotFactory.create_backend('auto')
                
                assert backend is not None
                assert isinstance(backend, type(mock_instance))
                mock_sim.assert_called_once()
    
    def test_auto_handles_connection_errors(self):
        """Test gestion erreurs connexion."""
        with patch('bbia_sim.robot_factory.ReachyMiniBackend') as mock_backend:
            mock_instance = MagicMock()
            mock_instance.is_connected = False  # Robot non connecté
            mock_backend.return_value = mock_instance
            
            with patch('bbia_sim.robot_factory.MuJoCoBackend') as mock_sim:
                mock_sim_instance = MagicMock()
                mock_sim.return_value = mock_sim_instance
                
                backend = RobotFactory.create_backend('auto')
                
                # Devrait fallback vers sim car robot non connecté
                assert backend is not None
                mock_sim.assert_called_once()
    
    def test_auto_preserves_kwargs(self):
        """Test préservation kwargs lors fallback."""
        with patch('bbia_sim.robot_factory.ReachyMiniBackend', side_effect=Exception()):
            with patch('bbia_sim.robot_factory.MuJoCoBackend') as mock_sim:
                mock_instance = MagicMock()
                mock_sim.return_value = mock_instance
                
                backend = RobotFactory.create_backend('auto', fast=True)
                
                assert backend is not None
                # Vérifier que kwargs sont passés
                mock_sim.assert_called_once_with(fast=True)
```

**Temps estimé** : 1h (écriture + validation)

### Validation

**Commandes de validation** :

```bash
# Tests
pytest tests/test_robot_factory_auto_fallback.py -v

# Linting
black src/bbia_sim/robot_factory.py tests/test_robot_factory_auto_fallback.py
ruff check src/bbia_sim/robot_factory.py tests/test_robot_factory_auto_fallback.py
mypy src/bbia_sim/robot_factory.py tests/test_robot_factory_auto_fallback.py
bandit -r src/bbia_sim/robot_factory.py

# Coverage
pytest tests/test_robot_factory_auto_fallback.py --cov=src/bbia_sim/robot_factory --cov-report=term
```

**Critères de succès** :

- ✅ Tous les tests passent
- ✅ Pas d'erreurs linting
- ✅ Coverage 100% du code ajouté
- ✅ Pas de régression (tests existants passent)

---

## 🔴 AMÉLIORATION 2 : Heartbeat WebSocket Adaptatif

### Audit de l'Existant - Heartbeat

**Fichiers concernés** :

- ✅ `src/bbia_sim/dashboard_advanced.py` (lignes 383-392) - Heartbeat fixe 30s
- ✅ `src/bbia_sim/daemon/ws/telemetry.py` - ConnectionManager existe
- ✅ `tests/test_websocket_reconnection.py` (15 tests) - Tests existants
- ✅ Reconnection automatique dans dashboard (lignes 2004-2056)

**Ce qui existe** :

- ✅ Heartbeat fixe 30s dans `dashboard_advanced.py`
- ✅ Reconnection automatique avec backoff exponentiel
- ✅ Tests de reconnexion WebSocket
- ✅ ConnectionManager avec gestion connexions

**Ce qui manque** :

- ❌ Heartbeat adaptatif selon latence
- ❌ Ajustement automatique intervalle (10s-60s)
- ❌ Tests pour heartbeat adaptatif

### Plan d'Action Détaillé - Heartbeat

#### Étape 1 : Modifier `dashboard_advanced.py` (1.5h)

**Fichier** : `src/bbia_sim/dashboard_advanced.py`

**Modifications** :

```python
class BBIAAdvancedWebSocketManager:
    def __init__(self):
        # ... code existant ...
        self._heartbeat_interval = 30.0  # Valeur initiale
        self._last_heartbeat: float = 0.0
        self._latency_history: list[float] = []  # NOUVEAU: Historique latence
        self._max_latency_history = 10  # NOUVEAU: Garder 10 dernières mesures
    
    def _calculate_adaptive_heartbeat(self) -> float:
        """Calcule intervalle heartbeat adaptatif selon latence.
        
        Returns:
            Intervalle heartbeat en secondes (10s-60s)
        """
        if not self._latency_history:
            return 30.0  # Valeur par défaut
        
        # Calculer latence moyenne
        avg_latency = sum(self._latency_history) / len(self._latency_history)
        
        # Ajuster heartbeat selon latence (2x latence, min 10s, max 60s)
        heartbeat = max(10.0, min(60.0, avg_latency * 2))
        
        return heartbeat
    
    def _update_latency(self, latency_ms: float) -> None:
        """Met à jour historique latence pour calcul heartbeat adaptatif."""
        self._latency_history.append(latency_ms)
        if len(self._latency_history) > self._max_latency_history:
            self._latency_history.pop(0)
        
        # Recalculer heartbeat adaptatif
        self._heartbeat_interval = self._calculate_adaptive_heartbeat()
    
    async def _send_heartbeat(self) -> None:
        """Envoie un heartbeat adaptatif selon latence."""
        current_time = time.time()
        if current_time - self._last_heartbeat >= self._heartbeat_interval:
            heartbeat_data = {
                "type": "heartbeat",
                "timestamp": datetime.now().isoformat(),
                "interval": self._heartbeat_interval,  # NOUVEAU: Inclure intervalle
            }
            await self.broadcast(json.dumps(heartbeat_data))
            self._last_heartbeat = current_time
```

**Tests à créer** : `tests/test_websocket_heartbeat_adaptive.py` (voir section Tests)

#### Étape 2 : Modifier `telemetry.py` (1h)

**Fichier** : `src/bbia_sim/daemon/ws/telemetry.py`

**Modifications** :

- Ajouter heartbeat adaptatif similaire dans `ConnectionManager`
- Utiliser latence réelle si disponible

#### Étape 3 : Tests (1h)

**Fichier** : `tests/test_websocket_heartbeat_adaptive.py` (NOUVEAU)

**Tests à créer** :

- `test_heartbeat_adapts_to_low_latency()` - Heartbeat plus rapide si latence faible
- `test_heartbeat_adapts_to_high_latency()` - Heartbeat plus lent si latence élevée
- `test_heartbeat_stays_within_bounds()` - Heartbeat reste entre 10s-60s
- `test_heartbeat_updates_on_latency_change()` - Heartbeat s'ajuste dynamiquement

**Critères** :

- ✅ Tests légers (< 1s chacun)
- ✅ Utilisent mocks (pas de WebSocket réel requis)
- ✅ Coverage 100% du code ajouté

### Tests Détaillés - Heartbeat

**Fichier** : `tests/test_websocket_heartbeat_adaptive.py`

```python
#!/usr/bin/env python3
"""Tests pour heartbeat WebSocket adaptatif."""

import time
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager


class TestAdaptiveHeartbeat:
    """Tests pour heartbeat adaptatif selon latence."""
    
    def test_heartbeat_adapts_to_low_latency(self):
        """Test heartbeat plus rapide si latence faible."""
        manager = BBIAAdvancedWebSocketManager()
        
        # Simuler latence faible (10ms)
        for _ in range(5):
            manager._update_latency(10.0)
        
        heartbeat = manager._calculate_adaptive_heartbeat()
        
        # Latence faible → heartbeat plus rapide (proche de 10s)
        assert heartbeat < 30.0
        assert heartbeat >= 10.0
    
    def test_heartbeat_adapts_to_high_latency(self):
        """Test heartbeat plus lent si latence élevée."""
        manager = BBIAAdvancedWebSocketManager()
        
        # Simuler latence élevée (200ms)
        for _ in range(5):
            manager._update_latency(200.0)
        
        heartbeat = manager._calculate_adaptive_heartbeat()
        
        # Latence élevée → heartbeat plus lent (proche de 60s)
        assert heartbeat > 30.0
        assert heartbeat <= 60.0
    
    def test_heartbeat_stays_within_bounds(self):
        """Test heartbeat reste entre 10s-60s."""
        manager = BBIAAdvancedWebSocketManager()
        
        # Test latence très faible
        for _ in range(5):
            manager._update_latency(1.0)
        heartbeat_low = manager._calculate_adaptive_heartbeat()
        assert heartbeat_low >= 10.0
        
        # Test latence très élevée
        for _ in range(5):
            manager._update_latency(1000.0)
        heartbeat_high = manager._calculate_adaptive_heartbeat()
        assert heartbeat_high <= 60.0
    
    def test_heartbeat_updates_on_latency_change(self):
        """Test heartbeat s'ajuste dynamiquement."""
        manager = BBIAAdvancedWebSocketManager()
        
        # Latence faible initiale
        for _ in range(5):
            manager._update_latency(10.0)
        heartbeat1 = manager._heartbeat_interval
        
        # Latence élevée
        for _ in range(5):
            manager._update_latency(200.0)
        heartbeat2 = manager._heartbeat_interval
        
        # Heartbeat devrait augmenter
        assert heartbeat2 > heartbeat1
    
    @pytest.mark.asyncio
    async def test_heartbeat_sends_with_adaptive_interval(self):
        """Test envoi heartbeat avec intervalle adaptatif."""
        manager = BBIAAdvancedWebSocketManager()
        manager.active_connections = [AsyncMock()]
        
        # Simuler latence faible
        manager._update_latency(10.0)
        
        # Vérifier que heartbeat utilise intervalle adaptatif
        initial_interval = manager._heartbeat_interval
        
        await manager._send_heartbeat()
        
        # Vérifier que heartbeat a été envoyé
        assert manager._last_heartbeat > 0
        assert manager._heartbeat_interval == initial_interval
```

**Temps estimé** : 1h (écriture + validation)

### Validation - Heartbeat

**Commandes de validation** :

```bash
# Tests
pytest tests/test_websocket_heartbeat_adaptive.py -v

# Linting
black src/bbia_sim/dashboard_advanced.py src/bbia_sim/daemon/ws/telemetry.py tests/test_websocket_heartbeat_adaptive.py
ruff check src/bbia_sim/dashboard_advanced.py src/bbia_sim/daemon/ws/telemetry.py tests/test_websocket_heartbeat_adaptive.py
mypy src/bbia_sim/dashboard_advanced.py src/bbia_sim/daemon/ws/telemetry.py tests/test_websocket_heartbeat_adaptive.py
bandit -r src/bbia_sim/dashboard_advanced.py src/bbia_sim/daemon/ws/telemetry.py

# Coverage
pytest tests/test_websocket_heartbeat_adaptive.py --cov=src/bbia_sim/dashboard_advanced --cov-report=term
```

**Critères de succès** :

- ✅ Tous les tests passent
- ✅ Pas d'erreurs linting
- ✅ Coverage 100% du code ajouté
- ✅ Pas de régression (tests existants passent)

---

## 🟡 AMÉLIORATION 3 : Finaliser Découverte Automatique Robots

### Audit de l'Existant - Découverte

**Fichiers concernés** :

- ✅ `src/bbia_sim/robot_registry.py` (lignes 32-97) - Infrastructure créée
- ✅ `tests/test_robot_registry.py` (13 tests, 93.85% coverage) - Tests existants

**Ce qui existe** :

- ✅ Classe `RobotRegistry` avec méthode `discover_robots()`
- ✅ Tests complets (13 tests)
- ✅ Infrastructure Zenoh créée
- ⚠️ **Problème** : Découverte incomplète (TODO ligne 82), utilise variables d'env

**Ce qui manque** :

- ❌ Vraie découverte via Zenoh (actuellement fallback variables d'env)
- ❌ Intégration dans `RobotFactory` pour utilisation automatique
- ❌ API endpoint `/api/robots/list`

### Plan d'Action Détaillé - Découverte

#### Étape 1 : Finaliser `robot_registry.py` (1h)

**Fichier** : `src/bbia_sim/robot_registry.py`

**Modifications** :

- Implémenter vraie découverte Zenoh (remplacer TODO ligne 82)
- Utiliser `zenoh.discover()` ou subscriber pour détecter robots

#### Étape 2 : Intégrer dans `RobotFactory` (30min)

**Fichier** : `src/bbia_sim/robot_factory.py`

**Modifications** :

- Utiliser `RobotRegistry` dans mode `auto` pour découverte automatique

#### Étape 3 : Ajouter endpoint API (30min)

**Fichier** : `src/bbia_sim/daemon/app/routers/state.py` ou nouveau router

**Modifications** :

- Ajouter endpoint `GET /api/robots/list` pour lister robots découverts

#### Étape 4 : Tests (1h)

**Fichier** : Améliorer `tests/test_robot_registry.py` (tests existants)

**Tests à améliorer** :

- Tests découverte Zenoh réelle (actuellement mock)
- Tests intégration `RobotFactory` avec `RobotRegistry`

**Critères** :

- ✅ Tests légers (< 1s chacun)
- ✅ Utilisent mocks (pas de Zenoh réel requis)
- ✅ Coverage maintenu à 93.85%+

### Validation - Découverte

**Commandes de validation** :

```bash
# Tests
pytest tests/test_robot_registry.py -v

# Linting
black src/bbia_sim/robot_registry.py src/bbia_sim/robot_factory.py
ruff check src/bbia_sim/robot_registry.py src/bbia_sim/robot_factory.py
mypy src/bbia_sim/robot_registry.py src/bbia_sim/robot_factory.py
bandit -r src/bbia_sim/robot_registry.py

# Coverage
pytest tests/test_robot_registry.py --cov=src/bbia_sim/robot_registry --cov-report=term
```

---

## 🟡 AMÉLIORATION 4 : Lifespan Context Manager Robust

### Audit de l'Existant - Lifespan

**Fichiers concernés** :

- ✅ `src/bbia_sim/daemon/app/main.py` (lignes 93-150) - Lifespan basique
- ✅ `tests/e2e/test_api_simu_roundtrip.py` - Tests lifespan existants

**Ce qui existe** :

- ✅ Lifespan basique avec startup/shutdown
- ✅ Gestion WebSocket cleanup
- ✅ Gestion simulation MuJoCo
- ⚠️ **Problème** : Pas de retry, pas de fallback si startup échoue

**Ce qui manque** :

- ❌ Retry automatique si startup échoue
- ❌ Fallback gracieux si composants non disponibles
- ❌ Health check avant de marquer "ready"

### Plan d'Action Détaillé - Lifespan

#### Étape 1 : Modifier `main.py` (1.5h)

**Fichier** : `src/bbia_sim/daemon/app/main.py`

**Modifications** :

```python
@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Gestionnaire de cycle de vie robuste avec retry."""
    logger.info("🚀 Démarrage de l'API BBIA-SIM")
    
    # NOUVEAU: Retry automatique startup simulation
    max_retries = 3
    retry_delay = 1.0
    
    for attempt in range(max_retries):
        try:
            sim_config = settings.get_simulation_config()
            success = await simulation_service.start_simulation(
                headless=sim_config["headless"]
            )
            
            if success:
                logger.info("✅ Simulation MuJoCo démarrée avec succès")
                app_state["simulator"] = simulation_service
                app_state["is_running"] = True
                break
            else:
                if attempt < max_retries - 1:
                    logger.warning("⚠️ Tentative %d/%d échouée, retry...", attempt + 1, max_retries)
                    await asyncio.sleep(retry_delay)
                else:
                    logger.warning("⚠️ Échec démarrage simulation après %d tentatives", max_retries)
                    app_state["simulator"] = None
                    app_state["is_running"] = False
        except Exception as e:
            if attempt < max_retries - 1:
                logger.warning("⚠️ Erreur démarrage (tentative %d/%d): %s", attempt + 1, max_retries, e)
                await asyncio.sleep(retry_delay)
            else:
                logger.error("❌ Échec démarrage après %d tentatives: %s", max_retries, e)
                app_state["simulator"] = None
                app_state["is_running"] = False
    
    yield
    
    # ... reste du code existant (shutdown) ...
```

#### Étape 2 : Tests (1.5h)

**Fichier** : `tests/test_lifespan_robust.py` (NOUVEAU)

**Tests à créer** :

- `test_lifespan_retries_on_startup_failure()` - Retry si startup échoue
- `test_lifespan_fallback_if_sim_unavailable()` - Fallback si sim non disponible
- `test_lifespan_continues_without_sim()` - App démarre même sans sim

**Critères** :

- ✅ Tests légers (< 1s chacun)
- ✅ Utilisent mocks (pas de simulation réelle)
- ✅ Coverage 100% du code ajouté

### Tests Détaillés - Lifespan

**Fichier** : `tests/test_lifespan_robust.py`

```python
#!/usr/bin/env python3
"""Tests pour lifespan context manager robuste."""

import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from bbia_sim.daemon.app.main import app, lifespan


class TestLifespanRobust:
    """Tests pour lifespan avec retry et fallback."""
    
    @pytest.mark.asyncio
    async def test_lifespan_retries_on_startup_failure(self):
        """Test retry si startup échoue."""
        with patch('bbia_sim.daemon.app.main.simulation_service') as mock_sim:
            # Simuler 2 échecs puis succès
            mock_sim.start_simulation = AsyncMock(side_effect=[
                False, False, True
            ])
            
            async with lifespan(app):
                # Vérifier que retry a été fait
                assert mock_sim.start_simulation.call_count == 3
    
    @pytest.mark.asyncio
    async def test_lifespan_fallback_if_sim_unavailable(self):
        """Test fallback si sim non disponible."""
        with patch('bbia_sim.daemon.app.main.simulation_service') as mock_sim:
            # Simuler échec permanent
            mock_sim.start_simulation = AsyncMock(return_value=False)
            
            async with lifespan(app):
                # App devrait démarrer même si sim échoue
                assert app is not None
    
    @pytest.mark.asyncio
    async def test_lifespan_continues_without_sim(self):
        """Test app démarre même sans sim."""
        with patch('bbia_sim.daemon.app.main.simulation_service') as mock_sim:
            mock_sim.start_simulation = AsyncMock(return_value=False)
            
            async with lifespan(app):
                # Vérifier que app_state est correct
                from bbia_sim.daemon.app.main import app_state
                assert app_state["simulator"] is None
                assert app_state["is_running"] is False
```

**Temps estimé** : 1.5h (écriture + validation)

### Validation - Lifespan

**Commandes de validation** :

```bash
# Tests
pytest tests/test_lifespan_robust.py -v

# Linting
black src/bbia_sim/daemon/app/main.py tests/test_lifespan_robust.py
ruff check src/bbia_sim/daemon/app/main.py tests/test_lifespan_robust.py
mypy src/bbia_sim/daemon/app/main.py tests/test_lifespan_robust.py
bandit -r src/bbia_sim/daemon/app/main.py

# Coverage
pytest tests/test_lifespan_robust.py --cov=src/bbia_sim/daemon/app/main --cov-report=term
```

---

## 🟡 AMÉLIORATION 5 : Mode Simplifié Dashboard

### Audit de l'Existant - Dashboard

**Fichiers concernés** :

- ✅ `src/bbia_sim/dashboard_advanced.py` - Dashboard complet
- ⚠️ **Problème** : Interface complexe pour nouveaux utilisateurs

**Ce qui existe** :

- ✅ Dashboard complet avec tous les contrôles
- ✅ Interface avancée pour experts

**Ce qui manque** :

- ❌ Mode simplifié avec contrôles essentiels
- ❌ Toggle mode simplifié/avancé
- ❌ Tests pour mode simplifié

### Plan d'Action Détaillé - Dashboard

#### Étape 1 : Ajouter mode simplifié (2h)

**Fichier** : `src/bbia_sim/dashboard_advanced.py`

**Modifications** :

- Ajouter paramètre `mode='expert'` dans `BBIAAdvancedWebSocketManager`
- Créer template HTML simplifié pour mode simplifié
- Toggle mode simplifié/avancé

#### Étape 2 : Tests (2h)

**Fichier** : `tests/test_dashboard_beginner_mode.py` (NOUVEAU)

**Tests à créer** :

- `test_simplified_mode_shows_simple_controls()` - Mode simplifié affiche contrôles essentiels
- `test_toggle_simplified_advanced_mode()` - Toggle mode fonctionne
- `test_simplified_mode_hides_advanced_features()` - Mode simplifié cache fonctionnalités avancées

**Critères** :

- ✅ Tests légers (< 1s chacun)
- ✅ Utilisent mocks (pas de dashboard réel)
- ✅ Coverage 100% du code ajouté

### Validation - Dashboard

**Commandes de validation** :

```bash
# Tests
pytest tests/test_dashboard_beginner_mode.py -v

# Linting
black src/bbia_sim/dashboard_advanced.py tests/test_dashboard_beginner_mode.py
ruff check src/bbia_sim/dashboard_advanced.py tests/test_dashboard_beginner_mode.py
mypy src/bbia_sim/dashboard_advanced.py tests/test_dashboard_beginner_mode.py
bandit -r src/bbia_sim/dashboard_advanced.py

# Coverage
pytest tests/test_dashboard_beginner_mode.py --cov=src/bbia_sim/dashboard_advanced --cov-report=term
```

---

## 📋 CHECKLIST FINALE AVANT PUSH

### Validation Complète

**Pour chaque amélioration** :

- [ ] Code implémenté
- [ ] Tests créés/améliorés
- [ ] Tous les tests passent
- [ ] Pas d'erreurs linting (black, ruff, mypy, bandit)
- [ ] Coverage 100% du code ajouté
- [ ] Pas de régression (tests existants passent)
- [ ] Documentation mise à jour (réutiliser MD existants)

### Commandes de Validation Globale

```bash
# Tests complets
pytest tests/ -v --tb=short

# Linting complet
black src/ tests/
ruff check src/ tests/
mypy src/bbia_sim/
bandit -r src/bbia_sim/

# Coverage global
pytest tests/ --cov=src/bbia_sim --cov-report=term-missing --cov-fail-under=50

# Tests spécifiques nouvelles améliorations
pytest tests/test_robot_factory_auto_fallback.py tests/test_websocket_heartbeat_adaptive.py tests/test_lifespan_robust.py tests/test_dashboard_beginner_mode.py -v
```

### Push sur Develop

**Quand tout est validé** :

```bash
# Vérifier statut git
git status

# Ajouter fichiers modifiés
git add src/bbia_sim/robot_factory.py
git add src/bbia_sim/dashboard_advanced.py
git add src/bbia_sim/daemon/app/main.py
git add src/bbia_sim/robot_registry.py
git add tests/test_robot_factory_auto_fallback.py
git add tests/test_websocket_heartbeat_adaptive.py
git add tests/test_lifespan_robust.py
git add tests/test_dashboard_beginner_mode.py
git add docs/quality/audits/PLAN_ACTION_AMELIORATIONS_BBIA.md

# Commit
git commit -m "feat: Améliorations importantes BBIA (fallback auto, heartbeat adaptatif, lifespan robuste, mode simplifié)

- Fallback automatique sim → robot dans RobotFactory
- Heartbeat WebSocket adaptatif selon latence
- Lifespan context manager avec retry automatique
- Mode simplifié dashboard avec contrôles essentiels
- Tests complets pour toutes les améliorations

Référence: TOP_AMELIORATIONS_IMPORTANTES_BBIA.md"

# Push sur develop
git push origin develop
```

---

## 📊 RÉSUMÉ TEMPS ESTIMÉ

| Amélioration | Code | Tests | Total |
|--------------|------|-------|-------|
| Fallback auto sim→robot | 1h | 1h | **2h** |
| Heartbeat adaptatif | 2.5h | 1h | **3.5h** |
| Découverte auto robots | 2h | 1h | **3h** |
| Lifespan robuste | 1.5h | 1.5h | **3h** |
| Mode simplifié | 2h | 2h | **4h** |
| **TOTAL** | **9h** | **6.5h** | **15.5h** |

**Avec validation et documentation** : ~18-20h total

---

**Dernière mise à jour** : 15 Décembre 2025  
**Référence** : [`TOP_AMELIORATIONS_IMPORTANTES_BBIA.md`](TOP_AMELIORATIONS_IMPORTANTES_BBIA.md)

---

## 🎯 PLAN D'ACTION - AMÉLIORATIONS RESTANTES (22 améliorations)

**Statut Top 5** : ✅ **100% TERMINÉ** (15 Déc 2025)

### 📊 Classement par Impact/Performance

#### 🔴 PRIORITÉ HAUTE - Impact Performance/UX (5 améliorations)

1. **Tests conformité SDK exhaustifs** (6-8h) - ⭐⭐⭐⭐⭐ Impact
   - **Pourquoi** : Garantit compatibilité SDK officiel, détecte régressions
   - **Impact** : 🔴 **CRITIQUE** - Qualité code, conformité production
   - **Tests existants** : ✅ `test_reachy_mini_full_conformity_official.py` (37 tests), `test_reachy_mini_complete_conformity.py` (16 tests)
   - **À faire** : Améliorer coverage edge cases, tests limites

2. **Tests headless MuJoCo robustes** (3-4h) - ⭐⭐⭐⭐ Impact
   - **Pourquoi** : Tests CI fiables, validation automatique
   - **Impact** : 🔴 **ÉLEVÉ** - Robustesse CI, moins de flaky tests
   - **Tests existants** : ✅ `test_vertical_slices.py` (headless), `test_simulator.py` (headless)
   - **À faire** : Améliorer gestion erreurs, timeouts adaptatifs

3. **Support simultané sim/robot réel** (6-8h) - ⭐⭐⭐⭐ Impact
   - **Pourquoi** : Tests sim pendant utilisation robot réel
   - **Impact** : 🔴 **ÉLEVÉ** - Productivité développement
   - **Infrastructure** : ✅ `RobotFactory.create_multi_backend()` existe
   - **À faire** : Routing API selon commande, tests intégration

4. **Intégration HF Spaces plus poussée** (6-8h) - ⭐⭐⭐ Impact
   - **Pourquoi** : Installation apps directement depuis dashboard
   - **Impact** : 🟡 **MOYEN-ÉLEVÉ** - UX améliorée, écosystème
   - **Infrastructure** : ✅ Recherche apps existe
   - **À faire** : Installation automatique, gestion dépendances

5. **Cache modèles agressif** (2-3h) - ⭐⭐⭐ Impact Performance
   - **Pourquoi** : Réduction RAM, démarrage plus rapide
   - **Impact** : 🟡 **MOYEN-ÉLEVÉ** - Performance Mac Mini
   - **À faire** : Cache LRU pour modèles MuJoCo, assets STL

#### 🟡 PRIORITÉ MOYENNE - Impact UX/Robustesse (5 améliorations)

6. **Guides par niveau** (4-6h) - ⭐⭐⭐ Impact
   - **Pourquoi** : Accessibilité nouveaux utilisateurs
   - **Impact** : 🟡 **MOYEN** - Adoption, documentation
   - **À faire** : Organiser guides (premiers pas → intermédiaire → expert)

7. **Batch processing mouvements** (4-6h) - ⭐⭐⭐ Impact Performance
   - **Pourquoi** : Mouvements plus fluides, moins de latence
   - **Impact** : 🟡 **MOYEN** - Performance, UX
   - **À faire** : File d'attente multicouche, traitement batch

8. **Chargement lazy assets STL** (3-4h) - ⭐⭐ Impact Performance
   - **Pourquoi** : Démarrage plus rapide, moins de RAM
   - **Impact** : 🟡 **MOYEN** - Performance Mac Mini
   - **À faire** : Chargement à la demande, cache

9. **Timestep adaptatif** (3-4h) - ⭐⭐ Impact Performance
   - **Pourquoi** : Performance optimale selon complexité scène
   - **Impact** : 🟡 **MOYEN** - Performance simulation
   - **À faire** : Ajustement dynamique timestep (0.005s-0.02s)

10. **Scènes complexes avec objets** (4-6h) - ⭐⭐ Impact
    - **Pourquoi** : Tests manipulation objets, interactions
    - **Impact** : 🟡 **MOYEN** - Fonctionnalités avancées
    - **À faire** : Créer scènes XML avec objets MuJoCo

#### 🟢 PRIORITÉ BASSE - Améliorations Futures (12 améliorations)

11. **Rate limiting granulaire** (2-3h) - ⭐⭐ Impact
12. **OpenAPI détaillée** (3-4h) - ⭐⭐ Impact
13. **Sharding tests** (2-3h) - ⭐ Impact Performance
14. **Pre-commit hooks complets** (2-3h) - ⭐ Impact Qualité
15. **Exemples erreurs communes** (3-4h) - ⭐ Impact Documentation
16. **Exemples exécutables validés** (4-6h) - ⭐ Impact Documentation
17. **MyPy strict mode** (8-12h) - ⭐ Impact Qualité
18. **Multi-robots simultanés** (8-12h) - ⭐ Impact Scalabilité
19. **Et 4 autres...**

---

### 🎯 PLAN D'ATTAQUE RECOMMANDÉ

#### Phase 1 : Performance & Robustesse (Priorité #1-2)
**Temps** : 9-12h  
**Impact** : 🔴 **CRITIQUE**

1. **Tests conformité SDK exhaustifs** (6-8h)
   - Améliorer `test_reachy_mini_full_conformity_official.py`
   - Ajouter tests edge cases, limites
   - Tests : ✅ Existent, à améliorer

2. **Tests headless MuJoCo robustes** (3-4h)
   - Améliorer gestion erreurs, timeouts
   - Tests : ✅ Existent, à améliorer

#### Phase 2 : Productivité Développement (Priorité #3)
**Temps** : 6-8h  
**Impact** : 🔴 **ÉLEVÉ**

3. **Support simultané sim/robot réel** (6-8h)
   - Finaliser routing API selon commande
   - Tests : ❌ À créer `tests/test_multi_backend_routing.py`
   - Infrastructure : ✅ `create_multi_backend()` existe

#### Phase 3 : UX & Écosystème (Priorité #4-5)
**Temps** : 8-11h  
**Impact** : 🟡 **MOYEN-ÉLEVÉ**

4. **Intégration HF Spaces plus poussée** (6-8h)
   - Installation automatique apps
   - Tests : ❌ À créer `tests/test_hf_spaces_installation.py`

5. **Cache modèles agressif** (2-3h)
   - Cache LRU pour modèles MuJoCo
   - Tests : ❌ À créer `tests/test_cache_models.py`

---

### 📋 DÉTAILS PAR AMÉLIORATION

#### 1. Tests Conformité SDK Exhaustifs

**Fichiers concernés** :
- ✅ `tests/test_reachy_mini_full_conformity_official.py` (37 tests existants)
- ✅ `tests/test_reachy_mini_complete_conformity.py` (16 tests existants)

**À améliorer** :
- Ajouter tests edge cases (valeurs limites, erreurs)
- Tests limites joints (min/max)
- Tests performance (latence, throughput)
- Tests erreurs réseau/timeout

**Tests à créer** :
- `tests/test_conformity_edge_cases.py` (nouveau)
- Améliorer tests existants

**Validation** :
```bash
pytest tests/test_reachy_mini_full_conformity_official.py -v
pytest tests/test_reachy_mini_complete_conformity.py -v
pytest tests/test_conformity_edge_cases.py -v
```

---

#### 2. Tests Headless MuJoCo Robustes

**Fichiers concernés** :
- ✅ `tests/test_vertical_slices.py` (tests headless existants)
- ✅ `tests/test_simulator.py` (test headless existant)

**À améliorer** :
- Gestion erreurs plus robuste
- Timeouts adaptatifs selon complexité
- Tests récupération après erreur

**Tests à créer** :
- `tests/test_headless_robustness.py` (nouveau)
- Améliorer tests existants

**Validation** :
```bash
pytest tests/test_vertical_slices.py tests/test_simulator.py tests/test_headless_robustness.py -v
```

---

#### 3. Support Simultané Sim/Robot Réel

**Fichiers concernés** :
- ✅ `src/bbia_sim/robot_factory.py` (ligne 217 : `create_multi_backend()` existe)
- ⚠️ `src/bbia_sim/daemon/app/main.py` (routing API à ajouter)

**À faire** :
- Ajouter routing API selon commande (paramètre `backend` dans requête)
- Gestion multi-backends dans `BackendAdapter`
- Tests intégration

**Tests à créer** :
- `tests/test_multi_backend_routing.py` (nouveau)
- Tests : ❌ À créer

**Validation** :
```bash
pytest tests/test_multi_backend_routing.py -v
```

---

#### 4. Intégration HF Spaces Plus Poussée

**Fichiers concernés** :
- ✅ `src/bbia_sim/daemon/app/routers/apps.py` (recherche apps existe)
- ⚠️ Installation automatique à ajouter

**À faire** :
- Installation apps depuis dashboard
- Gestion dépendances
- Tests installation

**Tests à créer** :
- `tests/test_hf_spaces_installation.py` (nouveau)
- Tests : ❌ À créer

**Validation** :
```bash
pytest tests/test_hf_spaces_installation.py -v
```

---

#### 5. Cache Modèles Agressif

**Fichiers concernés** :
- ⚠️ `src/bbia_sim/robot_factory.py` (cache à ajouter)
- ⚠️ `src/bbia_sim/sim/simulator.py` (cache modèles MuJoCo)

**À faire** :
- Cache LRU pour modèles MuJoCo fréquemment utilisés
- Cache assets STL
- Tests cache

**Tests à créer** :
- `tests/test_cache_models.py` (nouveau)
- Tests : ❌ À créer

**Validation** :
```bash
pytest tests/test_cache_models.py -v
```

---

### 📊 RÉSUMÉ TEMPS ESTIMÉ - AMÉLIORATIONS RESTANTES

| Phase | Amélioration | Code | Tests | Total | Impact |
|-------|--------------|------|-------|-------|--------|
| **Phase 1** | Tests conformité exhaustifs | 4h | 2-4h | **6-8h** | 🔴 CRITIQUE |
| **Phase 1** | Tests headless robustes | 2h | 1-2h | **3-4h** | 🔴 ÉLEVÉ |
| **Phase 2** | Support simultané sim/robot | 4h | 2-4h | **6-8h** | 🔴 ÉLEVÉ |
| **Phase 3** | HF Spaces poussé | 4h | 2-4h | **6-8h** | 🟡 MOYEN |
| **Phase 3** | Cache modèles agressif | 1h | 1-2h | **2-3h** | 🟡 MOYEN |
| **TOTAL Phase 1-3** | **5 améliorations** | **15h** | **8-16h** | **23-31h** | |

**Autres améliorations** (priorité basse) : ~40-60h restantes

---

### ✅ CHECKLIST VALIDATION

**Pour chaque amélioration** :
- [ ] Code implémenté
- [ ] Tests créés/améliorés
- [ ] Tous les tests passent
- [ ] Pas d'erreurs linting (black, ruff, mypy, bandit)
- [ ] Coverage 100% du code ajouté
- [ ] Pas de régression (tests existants passent)
- [ ] Documentation mise à jour (TOUS les MD concernés)
- [ ] Push sur develop quand tout OK
