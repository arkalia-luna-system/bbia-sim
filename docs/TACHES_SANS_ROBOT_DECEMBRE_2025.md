# ✅ TÂCHES RÉALISABLES SANS ROBOT - Décembre 2025

**Date** : Décembre 2025  
**Statut** : Toutes ces tâches peuvent être faites maintenant, sans attendre le robot réel

---

## 🎯 RÉSUMÉ EXÉCUTIF

**Tâches réalisables maintenant** : **15+ tâches**  
**Priorité** : 🟢 Basse à 🟡 Moyenne (toutes optionnelles)  
**Temps total estimé** : **8-15 heures**

---

## 🔴 PRIORITÉ MOYENNE - Améliorations Code

### 1. ✅ Migration Imports RobotFactory (TERMINÉ - Décembre 2025)

**Fichier** : `src/bbia_sim/robot_api.py`

**Statut** : ✅ **TERMINÉ**
- Tous les scripts/exemples utilisent maintenant `robot_factory` directement
- Warning de dépréciation ajouté pour compatibilité ascendante
- Tests passent

### 2. ✅ Métriques Performance (TERMINÉ - Décembre 2025)

**Fichiers** : `src/bbia_sim/daemon/app/routers/metrics.py`

**Statut** : ✅ **TERMINÉ**
- Endpoint `/metrics/prometheus` (Prometheus metrics)
- Endpoint `/metrics/healthz` (Liveness probe)
- Endpoint `/metrics/readyz` (Readiness probe)
- Endpoint `/metrics/health` (Health check détaillé)
- Support psutil et prometheus_client optionnels
- Tests créés : `tests/test_metrics.py` (4 tests passent)

### 3. ✅ Script Diagnostic (TERMINÉ - Décembre 2025)

**Fichier** : `src/bbia_sim/__main__.py`

**Statut** : ✅ **TERMINÉ**
- Commande `python -m bbia_sim --doctor` disponible
- Vérifie : Python version, Reachy Mini SDK, MuJoCo, Audio, Camera, Network, Permissions
- Tests fonctionnels

### 4. ⚠️ Implémenter Auth WebSocket (1-2h)

**Fichier** : `src/bbia_sim/daemon/app/main.py` (ligne 243)

**Objectif** : Sécuriser les connexions WebSocket avec authentification

**Implémentation** :
```python
# Option 1 : Via query params
@app.websocket("/ws/full")
async def websocket_endpoint(websocket: WebSocket, token: str = Query(...)):
    # Vérifier token
    if not verify_token(token):
        await websocket.close(code=1008, reason="Unauthorized")
        return
    # ... reste du code

# Option 2 : Via message initial
@app.websocket("/ws/full")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    auth_message = await websocket.receive_json()
    if not verify_token(auth_message.get("token")):
        await websocket.close(code=1008, reason="Unauthorized")
        return
    # ... reste du code
```

**Bénéfices** :
- ✅ Sécurité supplémentaire pour WebSockets
- ✅ Contrôle d'accès aux streams temps réel

**Estimation** : 1-2h

---

### 5. ⚠️ Migrer Imports vers robot_factory.py (TERMINÉ - voir ci-dessus)

**Fichier** : `src/bbia_sim/robot_api.py` (ligne 283)

**Objectif** : Refactoring pour éviter imports circulaires et centraliser la création de robots

**Actions** :
1. Vérifier tous les imports de `RobotFactory` dans le projet
2. Remplacer `from robot_api import RobotFactory` par `from robot_factory import RobotFactory`
3. Mettre à jour les imports dans tous les fichiers concernés
4. Supprimer le code de compatibilité dans `robot_api.py`

**Fichiers à vérifier** :
```bash
grep -r "from.*robot_api import.*RobotFactory" src/
grep -r "from.*robot_api import.*Robot" src/
```

**Bénéfices** :
- ✅ Architecture plus propre
- ✅ Évite imports circulaires
- ✅ Meilleure séparation des responsabilités

**Estimation** : 2-3h

---

## 🟡 PRIORITÉ MOYENNE - Améliorations Performance

### 6. ⚠️ Métriques Performance (TERMINÉ - voir ci-dessus)

**Objectif** : Mesurer et exposer métriques de performance

**Actions** :

#### A. Ajouter métriques Prometheus (1-2h)
```python
# src/bbia_sim/daemon/app/routers/metrics.py
from prometheus_client import Counter, Histogram, Gauge

request_count = Counter('bbia_requests_total', 'Total requests')
request_latency = Histogram('bbia_request_duration_seconds', 'Request latency')
cpu_usage = Gauge('bbia_cpu_usage_percent', 'CPU usage')
memory_usage = Gauge('bbia_memory_usage_bytes', 'Memory usage')

@app.get("/metrics")
async def metrics():
    return Response(generate_latest(), media_type="text/plain")
```

**Métriques à ajouter** :
- Latence requêtes (p50, p95, p99)
- CPU/RAM usage
- FPS simulation
- Temps réponse watchdog
- Nombre de connexions WebSocket

#### B. Endpoints Health Checks (30min)
```python
@app.get("/healthz")
async def healthz():
    """Liveness probe - vérifie si le service est vivant"""
    return {"status": "ok"}

@app.get("/readyz")
async def readyz():
    """Readiness probe - vérifie si le service est prêt"""
    # Vérifier connexion robot, DB, etc.
    if robot.is_connected():
        return {"status": "ready"}
    return {"status": "not_ready"}, 503
```

#### C. Logs Structurés JSON (1h)
```python
import json
import logging

class JSONFormatter(logging.Formatter):
    def format(self, record):
        log_entry = {
            "timestamp": self.formatTime(record),
            "level": record.levelname,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
        }
        if hasattr(record, "extra"):
            log_entry.update(record.extra)
        return json.dumps(log_entry)
```

**Bénéfices** :
- ✅ Monitoring temps réel
- ✅ Détection de problèmes de performance
- ✅ Alertes automatiques possibles

**Estimation** : 3-4h total

---

### 7. ⚠️ Script Diagnostic Environnement (TERMINÉ - voir ci-dessus)

**Objectif** : Créer un script `bbia doctor` pour vérifier l'environnement

**Fichier** : `src/bbia_sim/__main__.py` ou `scripts/bbia_doctor.py`

**Vérifications** :
```python
def check_environment():
    checks = {
        "Python version": sys.version_info >= (3, 10),
        "Reachy Mini SDK": check_reachy_mini_installed(),
        "MuJoCo": check_mujoco_available(),
        "Audio libraries": check_audio_libraries(),
        "Camera access": check_camera_available(),
        "Network": check_network_connectivity(),
        "Permissions": check_file_permissions(),
    }
    return checks
```

**Bénéfices** :
- ✅ Diagnostic rapide des problèmes
- ✅ Aide au debugging
- ✅ Validation setup avant utilisation

**Estimation** : 1h

---

## 🟢 PRIORITÉ BASSE - Améliorations Optionnelles

### 5. ✅ Versionnement Schémas WebSocket (2h)

**Objectif** : Versionner les schémas de messages WebSocket pour compatibilité

**Implémentation** :
```python
# Messages WebSocket avec version
{
    "version": "1.0",
    "type": "state_update",
    "data": {...}
}

# Support multiple versions
@app.websocket("/ws/full")
async def websocket_endpoint(websocket: WebSocket, version: str = "1.0"):
    # Adapter schéma selon version
    if version == "1.0":
        # Format actuel
    elif version == "2.0":
        # Format futur
```

**Bénéfices** :
- ✅ Compatibilité avec clients multiples
- ✅ Évolution sans casser les clients existants

**Estimation** : 2h

---

### 6. ✅ Pagination et Filtres REST API (2-3h)

**Objectif** : Ajouter pagination et filtres aux endpoints REST

**Endpoints à améliorer** :
```python
@app.get("/api/datasets")
async def list_datasets(
    page: int = Query(1, ge=1),
    page_size: int = Query(20, ge=1, le=100),
    filter_type: str = Query(None),
    sort_by: str = Query("name"),
):
    # Pagination
    offset = (page - 1) * page_size
    datasets = get_datasets(offset=offset, limit=page_size, filter_type=filter_type)
    
    return {
        "items": datasets,
        "total": count_datasets(),
        "page": page,
        "page_size": page_size,
    }
```

**Bénéfices** :
- ✅ Performance avec grandes listes
- ✅ Meilleure UX pour les clients

**Estimation** : 2-3h

---

### 7. ✅ Extras Packaging (1h)

**Objectif** : Créer extras `lite`, `full`, `robot` pour installations différentes

**Fichier** : `pyproject.toml`

```toml
[project.optional-dependencies]
lite = [
    "bbia-sim[core]",
    # Pas de simulation, juste API
]
full = [
    "bbia-sim[core,audio,vision,ai]",
    "mujoco",
    "ultralytics",
    "whisper",
]
robot = [
    "bbia-sim[full]",
    "reachy-mini[mujoco]",
]
```

**Bénéfices** :
- ✅ Installations plus légères pour certains cas
- ✅ Meilleure organisation des dépendances

**Estimation** : 1h

---

### 8. ✅ Images Docker Multi-Architecture (2h)

**Objectif** : Créer images Docker pour CPU, GPU, Apple Silicon

**Fichiers** : `Dockerfile.cpu`, `Dockerfile.gpu`, `Dockerfile.mps`

**Bénéfices** :
- ✅ Support multi-plateforme
- ✅ Optimisations spécifiques par architecture

**Estimation** : 2h

---

### 9. ✅ CI/CD Améliorations (2-3h)

**Fichier** : `.github/workflows/ci.yml`

**Améliorations** :
1. **Matrice Python 3.11/3.12** (30min)
   ```yaml
   strategy:
     matrix:
       python-version: ["3.11", "3.12"]
   ```

2. **Pre-commit Hooks** (1h)
   ```yaml
   - uses: pre-commit/action@v3.0.0
   ```

3. **Sharding Tests** (1h)
   ```yaml
   - name: Run tests (shard ${{ matrix.shard }}/${{ matrix.total }})
     run: pytest tests/ --numprocesses=${{ matrix.shard }}
   ```

**Bénéfices** :
- ✅ Tests plus rapides
- ✅ Détection d'erreurs plus tôt
- ✅ Support multi-versions Python

**Estimation** : 2-3h

---

### 10. ✅ Sécurité Supplémentaire (2-3h)

**Objectif** : Améliorer la sécurité du projet

**Actions** :

#### A. CORS Strict (30min)
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://bbia.local"],  # Liste spécifique
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)
```

#### B. Rate Limiting (1h)
```python
from slowapi import Limiter
limiter = Limiter(key_func=get_remote_address)

@app.get("/api/endpoint")
@limiter.limit("10/minute")
async def endpoint():
    ...
```

#### C. SBOM Génération (30min)
```bash
# Générer SBOM avec CycloneDX
pip install cyclonedx-bom
cyclonedx-py -o sbom.json
```

#### D. Semgrep/Gitleaks CI (1h)
```yaml
- uses: returntocorp/semgrep-action@v1
- uses: gitleaks/gitleaks-action@v2
```

**Bénéfices** :
- ✅ Protection contre attaques
- ✅ Conformité sécurité
- ✅ Détection de secrets

**Estimation** : 2-3h

---

## 📚 PRIORITÉ BASSE - Documentation

### 11. ✅ Mettre à Jour FAQ (1h)

**Fichier** : `docs/guides_techniques/FAQ_TROUBLESHOOTING.md`

**Sections à ajouter** :
- Coverage tests actuel (99% vision_yolo, 92% voice_whisper)
- Comment utiliser les nouveaux endpoints
- Guide dépannage robot réel
- Questions fréquentes sur WebSocket

**Estimation** : 1h

---

### 12. ✅ Créer Guide Dashboard Advanced (1h)

**Fichier** : `docs/guides_techniques/GUIDE_DASHBOARD_ADVANCED.md`

**Contenu** :
- Utilisation de `dashboard_advanced.py`
- Configuration des widgets
- Personnalisation de l'interface
- Intégration avec autres modules

**Estimation** : 1h

---

### 13. ✅ Documenter Tests Coverage (30min)

**Fichier** : `tests/README.md`

**Contenu** :
- Comment lancer les tests
- Coverage actuel par module
- Comment ajouter de nouveaux tests
- Best practices

**Estimation** : 30min

---

## 🧹 PRIORITÉ BASSE - Nettoyage

### 14. ✅ Nettoyage Documentation Redondante (1-2h)

**Objectif** : Consolider les documents MD redondants

**Actions** :
1. Identifier les documents similaires
2. Fusionner les informations
3. Archiver les anciens documents
4. Créer un index consolidé

**Estimation** : 1-2h

---

### 15. ✅ Vérifier Liens MD Cassés (1h)

**Objectif** : Vérifier et corriger tous les liens MD

**Script existant** : Vérifier si `scripts/check_md_links.py` existe

**Actions** :
1. Lancer vérification liens
2. Corriger liens cassés
3. Mettre à jour liens obsolètes

**Estimation** : 1h

---

## 📊 RÉSUMÉ PAR PRIORITÉ

| Priorité | Tâche | Estimation | Impact |
|----------|-------|------------|--------|
| 🔴 Moyenne | Auth WebSocket | 1-2h | Sécurité |
| 🔴 Moyenne | Migration imports | 2-3h | Architecture |
| 🟡 Moyenne | Métriques performance | 3-4h | Monitoring |
| 🟡 Moyenne | Script diagnostic | 1h | Debugging |
| 🟢 Basse | Versionnement WS | 2h | Compatibilité |
| 🟢 Basse | Pagination REST | 2-3h | Performance |
| 🟢 Basse | Extras packaging | 1h | Organisation |
| 🟢 Basse | Docker multi-arch | 2h | Déploiement |
| 🟢 Basse | CI/CD améliorations | 2-3h | Qualité |
| 🟢 Basse | Sécurité supplémentaire | 2-3h | Sécurité |
| 🟢 Basse | Documentation | 2-3h | Documentation |
| 🟢 Basse | Nettoyage | 2-3h | Maintenance |

**Total estimé** : **20-30 heures** (optionnel, à faire selon besoins)

---

## 🎯 RECOMMANDATIONS

### À Faire Maintenant (Priorité) :
1. ✅ **Migration imports** (2-3h) - Architecture plus propre
2. ✅ **Métriques performance** (3-4h) - Monitoring essentiel
3. ✅ **Script diagnostic** (1h) - Aide debugging

### À Faire Plus Tard (Optionnel) :
- Auth WebSocket (si sécurité requise)
- Documentation (selon besoins utilisateurs)
- Nettoyage (maintenance continue)

---

## ✅ CONCLUSION

**15+ tâches réalisables maintenant** sans le robot réel.

**Toutes ces tâches sont optionnelles** et peuvent être faites selon les besoins prioritaires.

**Le projet est déjà prêt pour le robot réel** - ces améliorations sont des bonus pour la qualité, la sécurité et la maintenabilité.

---

**Dernière mise à jour** : Décembre 2025

