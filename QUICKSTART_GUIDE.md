# 🚀 Guide de Démarrage Rapide BBIA Reachy Mini

## ⚡ Démarrage en 30 secondes

### 1. Vérification du projet
```bash
cd /Volumes/T7/bbia-reachy-sim
python scripts/verify_project.py
```
**Résultat attendu** : `🎉 TOUS LES COMPOSANTS SONT FONCTIONNELS !`

### 2. Lancement de la simulation 3D
```bash
mjpython scripts/launch_complete_robot.py --model reachy_mini_REAL_OFFICIAL.xml
```
**Résultat attendu** : Fenêtre MuJoCo avec robot Reachy Mini assemblé

### 3. Test des modules BBIA
```bash
python examples/demo_bbia_complete.py
```
**Résultat attendu** : Test de tous les modules BBIA (audio, vision, voice, emotions, behavior)

## 🎮 Contrôles de Base

### MuJoCo 3D Viewer
- **Souris** : Rotation de la vue
- **Molette** : Zoom in/out
- **Clic droit + glisser** : Déplacer la vue
- **Échap** : Fermer la fenêtre

### API REST
```bash
# Démarrer l'API
python scripts/start_api.py

# Tester l'API
curl http://localhost:8000/health
```

### WebSocket Télémétrie
```bash
# Démarrer la télémétrie
python scripts/test_api_complete.py
```

## 🔧 Scripts Utilitaires

### Gestion des processus
```bash
# Nettoyer les processus gourmands
./scripts/smart_process_cleanup.sh

# Gérer les processus BBIA
python scripts/process_manager.py status
```

### Téléchargement des assets
```bash
# Re-télécharger tous les STL officiels
python scripts/download_ALL_stl.py
```

## 🧪 Tests Rapides

### Tests unitaires
```bash
pytest tests/ -v
```

### Tests d'intégration
```bash
pytest tests/e2e/ -v
```

### Tests spécifiques
```bash
# Test audio
pytest tests/test_bbia_audio.py -v

# Test vision
pytest tests/test_bbia_vision.py -v

# Test API
pytest tests/test_api_integration.py -v
```

## 🎯 Exemples d'Utilisation

### Contrôle du robot
```python
from src.bbia_sim.daemon.simulation_service import SimulationService

# Démarrer la simulation
service = SimulationService()
await service.start_simulation()

# Contrôler les articulations
await service.set_joint_position("yaw_body", 0.5)
```

### Utilisation des modules BBIA
```python
from src.bbia_sim.bbia_emotions import BBIAEmotions
from src.bbia_sim.bbia_voice import dire_texte

# Émotions
emotions = BBIAEmotions()
emotions.set_emotion("happy", 0.8)

# Synthèse vocale
dire_texte("Bonjour, je suis BBIA !")
```

## 🚨 Résolution de Problèmes

### Problème : Robot en pièces détachées
**Solution** :
```bash
python scripts/download_ALL_stl.py
```

### Problème : Fenêtre 3D ne s'ouvre pas
**Solution** : Utiliser `mjpython` au lieu de `python`

### Problème : Erreur d'import
**Solution** :
```bash
pip install -r requirements.txt
```

### Problème : Tests qui échouent
**Solution** :
```bash
python scripts/verify_project.py
```

## 📊 Métriques de Performance

### Simulation
- **Fréquence** : 30Hz stable
- **Latence** : <100ms
- **Mémoire** : ~200MB

### API
- **Temps de réponse** : <50ms
- **Throughput** : 100 req/s
- **WebSocket** : 30Hz télémétrie

## 🎉 Prochaines Étapes

1. **Intégration BBIA ↔ Robot** : Connecter les modules BBIA au simulateur
2. **Mapping émotions → articulations** : Contrôler le robot selon les émotions
3. **Réactions visuelles** : Robot réagit aux détections
4. **Tests d'intégration** : Vérifier émotions → mouvements

---
*Guide mis à jour : 23 octobre 2025*
