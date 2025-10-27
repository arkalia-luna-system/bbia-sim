# 📚 Documentation BBIA - Index

## 🎯 **Vue d'ensemble**

Ce dossier contient toute la documentation du projet BBIA (Biological Brain Intelligence Agent) pour le robot Reachy Mini Wireless.

---

## 📁 **Structure de la documentation**

### 🎮 **Guides principaux**
- [Guide Reachy Mini Wireless](guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md)
- [Guide de simulation MuJoCo](simulations/MUJOCO_SIMULATION_GUIDE.md)

### 🔧 **Guides d'installation**
- [Installation audio](installation/AUDIO_SETUP.md)

### 🎮 **Guides Unity**
- [Guide Unity BBIA](unity/UNITY_BBIA_GUIDE.md)
- [Dépannage Unity](unity/UNITY_TROUBLESHOOTING.md)

### 📦 **Dépôts et ressources**
- Voir [PROJECT_HISTORY.md](./PROJECT_HISTORY.md) pour l'historique complet

---

## 🚀 **Navigation rapide**

### 📋 **Documentation principale**
- **Architecture** : [Architecture BBIA](ARCHITECTURE.md)
- **Tests** : [Guide des tests](TESTING_GUIDE.md)
- **Processus** : [Gestion des processus](PROCESS_MANAGEMENT.md)

### 🎯 **Missions accomplies**
- **Audit 3D** : [Audit 3D BBIA](audit/AUDIT_3D_BBIA_COMPLET.md)
- **Historique** : [PROJECT_HISTORY.md](./PROJECT_HISTORY.md)

---

## 📊 **État actuel du projet**

### ✅ **Fonctionnalités opérationnelles**
- **Simulation 3D** : Robot Reachy Mini parfaitement fonctionnel
- **Modules BBIA** : 8 émotions, vision, audio, comportements intégrés
- **API REST** : FastAPI + WebSocket opérationnels
- **Tests** : 531 tests collectés, 418 passent (79% réussite)
- **Coverage** : 76.70% de couverture de code
- **Documentation** : Complète et organisée

### 🎮 **Commandes principales**
```bash
# Activer l'environnement
source venv/bin/activate

# Voir le robot en 3D
mjpython examples/demo_viewer_bbia_corrected.py --duration 10 --joint yaw_body

# Tests automatiques
python -m pytest tests/ -v

# Qualité du code
ruff check . --fix
black src/ tests/ examples/ scripts/
mypy src/
```

---

## 🎯 **Prochaines étapes**

### 🚀 **Développement immédiat**
1. **Nouvelles émotions** : confusion, détermination, nostalgie, fierté
2. **Commandes vocales** : "tourne à gauche", "souris", "regarde-moi"
3. **Vision améliorée** : reconnaissance d'expressions humaines
4. **Tests de stabilité** : validation de tous les joints Stewart

### 🔧 **Intégrations**
1. **API avancée** : endpoints pour contrôle fin
2. **Interface web** : contrôle du robot via navigateur
3. **Intégration Unity** : synchronisation temps réel
4. **Scénarios interactifs** : robot qui réagit aux émotions

---

**🤖 BBIA Reachy Mini Simulation - Documentation complète et à jour ! ✨**

*Dernière mise à jour : Octobre 2025*