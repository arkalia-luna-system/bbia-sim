# 🏗️ Architecture BBIA-SIM

**Version:** 1.3.0  
**Date:** Octobre 2025

---

## 📋 Vue d'ensemble

Cette section documente l'architecture complète de BBIA-SIM v1.3.0, un moteur cognitif Python avancé pour robot Reachy Mini.

---

## 📚 Documentation Architecture

### **📖 ARCHITECTURE_OVERVIEW.md**
Vue d'ensemble complète de l'architecture BBIA-SIM v1.3.0:
- Objectifs architecturaux
- Architecture générale avec diagrammes Mermaid
- Couches du système (BBIA, RobotAPI, Backends)
- Tests et CI/CD
- Sécurité et limites
- Évolutivité

👉 [Lire ARCHITECTURE_OVERVIEW.md](./ARCHITECTURE_OVERVIEW.md)

---

### **📖 ARCHITECTURE_DETAILED.md**
Documentation détaillée technique:
- Détails techniques par couche
- Modules BBIA (émotions, vision, voix, comportements)
- RobotAPI Interface
- Backends (MuJoCo, Reachy)
- Simulation MuJoCo
- Tests et CI

👉 [Lire ARCHITECTURE_DETAILED.md](./ARCHITECTURE_DETAILED.md)

---

## 🎯 Points Clés

### **Conformité SDK**
- ✅ **21/21 méthodes SDK** officiel implémentées
- ✅ **100% conforme** au SDK Reachy-Mini
- ✅ Backend ReachyMini prêt pour robot physique

### **Innovation Technique**
- ✅ **RobotAPI Unifié** : Interface simulation ↔ robot réel
- ✅ **Modules BBIA** : 12 émotions, vision, voix, comportements
- ✅ **Bridge Zenoh/FastAPI** : Architecture distribuée

### **Qualité**
- ✅ **Tests** : 27 passent, 13 skippés
- ✅ **Coverage** : 63.37%
- ✅ **Outils** : Black, Ruff, MyPy, Bandit ✅

---

## 🚀 Quick Start

```bash
# Backend MuJoCo (simulation)
python -c "from bbia_sim.robot_api import RobotFactory; robot = RobotFactory.create_backend('mujoco')"

# Backend Reachy-Mini SDK (robot physique)
python -c "from bbia_sim.robot_api import RobotFactory; robot = RobotFactory.create_backend('reachy_mini')"
```

---

## 📞 Support

Pour toute question sur l'architecture:
- 📖 Consultez les guides détaillés ci-dessus
- 🐛 [Signaler un problème](https://github.com/arkalia-luna-system/bbia-sim/issues)

---

*Documentation BBIA-SIM v1.3.0 - Arkalia Luna System*

