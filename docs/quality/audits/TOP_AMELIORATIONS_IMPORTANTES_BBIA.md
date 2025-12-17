# Améliorations importantes

**Date** : 17 Décembre 2025

---

## Fallback automatique sim → robot

**Fonctionnalité** : Détection automatique du robot disponible avec fallback vers la simulation.

**Utilisation** :
```python
from bbia_sim.robot_factory import RobotFactory

# Détection automatique + fallback transparent
robot = RobotFactory.create_backend('auto')  # Détecte robot, fallback sim si absent
```

**Bénéfices** :
- Plus besoin de configurer manuellement
- Fonctionne toujours (robot ou sim)
- Transition transparente sim → robot réel

---

## Heartbeat WebSocket adaptatif

**Fonctionnalité** : Ajustement automatique de l'intervalle heartbeat selon la latence réseau.

**Comportement** :
- Intervalle adaptatif : 10s-60s selon latence
- Détection déconnexions plus rapide
- Connexions plus stables

**Bénéfices** :
- Stabilité améliorée
- Performance adaptée à la latence réelle
- Dashboard plus réactif

---

## Découverte automatique robots

**Fonctionnalité** : Découverte automatique des robots disponibles sur le réseau.

**Utilisation** :
```python
from bbia_sim.robot_registry import RobotRegistry

robots = RobotRegistry().discover_robots()
# robots = [{"id": "robot-1", "hostname": "192.168.1.100", ...}]
robot = RobotFactory.create_backend('reachy_mini', robot_id='robot-1')
```

**Bénéfices** :
- Plus besoin de configurer IP/port manuellement
- Support plusieurs robots sur réseau
- Découverte automatique sur réseau local

**API** : Endpoint `/api/state/robots/list` pour lister robots découverts

---

## Lifespan context manager robuste

**Fonctionnalité** : Gestion robuste du démarrage avec retry automatique et fallback gracieux.

**Bénéfices** :
- Démarrage plus fiable
- Retry automatique si erreurs temporaires
- App démarre même si composants non disponibles

---

## Mode simplifié dashboard

**Fonctionnalité** : Interface simplifiée avec contrôles essentiels pour nouveaux utilisateurs.

**Bénéfices** :
- Accessibilité améliorée
- Progression naturelle : mode simplifié → avancé
- Adoption plus facile

---

**Dernière mise à jour** : 17 Décembre 2025
