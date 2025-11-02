---
**⚠️ ARCHIVE - DOCUMENT HISTORIQUE ⚠️**

Ce document a été archivé car il est devenu obsolète ou a été remplacé par une version plus récente.
Il est conservé à des fins de référence historique uniquement.

**Date d'archivage** : Oct / No2025025025025025
**Raison** : Document terminé/obsolète/remplacé
---

# 🏗️ Architecture BBIA-SIM

> Compatibilité Python et CI
>
> - Python requis: 3.11+
> - CI: `.github/workflows/ci.yml`
> - Setup rapide:
>   ```bash
>   pyenv install 3.11.9 && pyenv local 3.11.9
>   python -m pip install --upgrade pip
>   pip install -e .
>   ```

**Version:** 1.3.2
**Date:** Oct / No2025025025025025

---

## 📋 Vue d'ensemble

Cette section documente l'architecture complète de BBIA-SIM v1.3.2, un moteur cognitif Python avancé pour robot Reachy Mini.

---

## 📚 Documentation Architecture

> Référence état global
>
> Voir `docs/status.md` → "État par axe" pour l’état actuel (Observabilité, Performance, Sécurité, CI/CD, etc.) et les axes d’amélioration.

### **ARCHITECTURE_OVERVIEW.md**
Vue d'ensemble complète de l'architecture BBIA-SIM v1.3.2:
- Objectifs architecturaux
- Architecture générale avec diagrammes Mermaid
- Couches du système (BBIA, RobotAPI, Backends)
- Tests et CI/CD
- Sécurité et limites
- Évolutivité

👉 [Lire ARCHITECTURE_OVERVIEW.md](../../architecture/ARCHITECTURE_OVERVIEW.md)

---

### **ARCHITECTURE_DETAILED.md**
Documentation détaillée technique:
- Détails techniques par couche
- Modules BBIA (émotions, vision, voix, comportements)
- RobotAPI Interface
- Backends (MuJoCo, Reachy)
- Simulation MuJoCo
- Tests et CI

👉 [Lire ARCHITECTURE_DETAILED.md](../../architecture/ARCHITECTURE_DETAILED.md)

---

## Points clés

### Conformité SDK
- 21/21 méthodes SDK implémentées
- Conformité validée avec le SDK Reachy-Mini
- Backend ReachyMini prêt pour robot physique

### Innovation technique
- RobotAPI unifié : interface simulation ↔ robot réel
- Modules BBIA : 12 émotions, vision, voix, comportements
- Bridge Zenoh/FastAPI : architecture distribuée

### Qualité
- Tests : 27 passent, 13 skippés
- Couverture : 63.37%
- Outils : Black, Ruff, MyPy, Bandit

---

## Démarrage rapide

```bash
# Backend MuJoCo (simulation)
python -c "from bbia_sim.robot_factory import RobotFactory; robot = RobotFactory.create_backend('mujoco')"

# Backend Reachy-Mini SDK (robot physique)
python -c "from bbia_sim.robot_factory import RobotFactory; robot = RobotFactory.create_backend('reachy_mini')"
```

---

## 📞 Support

Pour toute question sur l'architecture:
- 📖 Consultez les guides détaillés ci-dessus
- 🐛 [Signaler un problème](https://github.com/arkalia-luna-system/bbia-sim/issues)

---

*Documentation BBIA-SIM v1.3.2 - Arkalia Luna System*

