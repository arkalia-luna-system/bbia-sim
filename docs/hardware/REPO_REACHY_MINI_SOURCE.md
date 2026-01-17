# 📁 Dossier `/Volumes/T7/reachy_mini` - Analyse

**Date d'analyse** : 6 Janvier 2026  
**Emplacement** : `/Volumes/T7/reachy_mini`

---

## 🎯 **QU'EST-CE QUE CE DOSSIER ?**

Ce dossier est le **dépôt source officiel du SDK Reachy Mini** cloné depuis GitHub.

**Source** : https://github.com/pollen-robotics/reachy_mini  
**Branche actuelle** : `develop`  
**Version** : v1.0.0-35-g2ba17f1 (35 commits après v1.0.0)

---

## 📊 **CONTENU DU DOSSIER**

### **Structure principale**

```
/Volumes/T7/reachy_mini/
├── src/reachy_mini/          # Code source du SDK Python
│   ├── daemon/               # Démon (service background)
│   ├── apps/                 # Applications
│   ├── kinematics/           # Cinématique inverse
│   ├── media/                # Gestion média (audio/vidéo)
│   ├── motion/               # Mouvements
│   └── reachy_mini.py        # Classe principale
├── examples/                 # Exemples d'utilisation
├── tests/                    # Tests unitaires
├── docs/                     # Documentation
├── tools/                    # Outils (setup moteurs, etc.)
├── pyproject.toml            # Configuration package Python
└── README.md                  # Documentation principale
```

### **Composants principaux**

1. **Le Démon (Daemon)** 😈
   - Service background qui gère la communication avec les moteurs
   - Peut tourner en simulation (MuJoCo) ou sur le robot réel
   - API REST via FastAPI sur `http://localhost:8000`

2. **Le SDK Python** 🐍
   - API Python pour contrôler le robot
   - Classes : `ReachyMini`, `Move`, etc.
   - Utilitaires : `create_head_pose()`, etc.

3. **L'API REST** 🕸️
   - API HTTP/WebSocket pour contrôler le robot
   - Documentation OpenAPI sur `/docs`
   - Accessible depuis n'importe quel langage

---

## 🔍 **À QUOI SERT CE DOSSIER ?**

### **Pour vous (BBIA-SIM)**

Ce dossier vous sert à :

1. **Consulter le code source officiel**
   - Comprendre comment fonctionne le SDK officiel
   - Voir les implémentations de référence
   - Comparer avec votre code BBIA

2. **Développer et tester**
   - Modifier le SDK si besoin (mode développement)
   - Tester des fonctionnalités avant qu'elles soient dans PyPI
   - Contribuer au projet officiel

3. **Documentation**
   - Exemples d'utilisation dans `/examples`
   - Documentation dans `/docs`
   - Tests de référence dans `/tests`

4. **Débogage**
   - Voir le code source pour comprendre les erreurs
   - Identifier les différences avec votre implémentation BBIA

---

## ⚠️ **IMPORTANT : Différence avec le package installé**

### **Ce dossier (source) vs Package installé**

| Aspect | Dossier source (`/Volumes/T7/reachy_mini`) | Package installé (`pip install reachy-mini`) |
|--------|--------------------------------------------|----------------------------------------------|
| **Emplacement** | `/Volumes/T7/reachy_mini` | Dans votre environnement Python (venv) |
| **Version** | `develop` (dernière version dev) | Version stable de PyPI (ex: v1.2.4) |
| **Utilisation** | Consultation, développement, tests | Utilisation normale dans vos scripts |
| **Mise à jour** | `git pull` | `pip install --upgrade reachy-mini` |

### **Quand utiliser quoi ?**

**Utiliser le package installé** (recommandé pour production) :
```python
# Dans vos scripts Python
from reachy_mini import ReachyMini  # ← Utilise le package installé
```

**Utiliser le dossier source** (pour développement) :
```bash
# Installer en mode développement
cd /Volumes/T7/reachy_mini
pip install -e .  # ← Installe depuis le dossier source
```

---

## 📦 **VERSION ACTUELLE**

**Version détectée** : `v1.0.0-35-g2ba17f1`
- Basé sur le tag `v1.0.0`
- 35 commits après ce tag
- Sur la branche `develop`

**Note** : Cette version est probablement **plus récente** que la version installée via PyPI (v1.2.4).

**Branches disponibles** :
- `develop` (actuelle) - Version de développement
- `main` - Version stable
- Plusieurs branches de features

---

## 🔧 **UTILISATION POUR BBIA-SIM**

### **1. Consultation du code source**

Vous pouvez consulter ce dossier pour :
- Voir comment Pollen implémente certaines fonctionnalités
- Comprendre les APIs officielles
- Identifier les différences avec votre implémentation BBIA

### **2. Développement**

Si vous voulez modifier ou tester le SDK officiel :

```bash
cd /Volumes/T7/reachy_mini
pip install -e .  # Installation en mode développement
```

**Attention** : Cela remplacera le package installé par la version du dossier source.

### **3. Mise à jour**

Pour mettre à jour le dépôt source :

```bash
cd /Volumes/T7/reachy_mini
git pull origin develop
```

**Note** : Cela met à jour le code source, mais **ne met pas à jour** le package installé dans votre environnement Python.

---

## 📚 **RESSOURCES UTILES DANS CE DOSSIER**

### **Exemples** (`/examples/`)

- `minimal_demo.py` - Exemple minimal
- `reachy_compliant_demo.py` - Démo de compliance
- `look_at_image.py` - Regarder une image
- `recorded_moves_example.py` - Mouvements enregistrés
- Et plus...

### **Documentation** (`/docs/`)

- `python-sdk.md` - Documentation SDK Python
- `rest-api.md` - Documentation API REST
- `awesome-apps.md` - Applications tierces
- `RPI.md` - Configuration Raspberry Pi

### **Tests** (`/tests/`)

- Tests unitaires de référence
- Exemples d'applications (`ok_app/`, `faulty_app/`)

---

## 🎯 **RECOMMANDATIONS**

### **Pour votre projet BBIA-SIM**

1. **Garder ce dossier** ✅
   - Utile pour consultation et référence
   - Ne prend pas beaucoup de place
   - Peut servir pour débogage

2. **Ne pas l'utiliser directement** ⚠️
   - Utiliser le package installé (`pip install reachy-mini`)
   - Plus stable et testé
   - Version contrôlée

3. **Mettre à jour régulièrement** 📅
   - `git pull` pour avoir les dernières versions
   - Surveiller les nouvelles fonctionnalités
   - Voir les corrections de bugs

4. **Documenter les différences** 📝
   - Si vous trouvez des différences avec votre code BBIA
   - Noter les améliorations possibles
   - Contribuer si vous trouvez des bugs

---

## 🔗 **LIENS UTILES**

- **GitHub officiel** : https://github.com/pollen-robotics/reachy_mini
- **Documentation BBIA** :
  - `REACHY_MINI_SDK_v1.2.4.md` - Version recommandée
  - `REACHY_MINI_SDK_v1.2.6_v1.2.7.md` - Nouvelles releases
  - `PROBLEME_MOTEURS_QC_BATCH_DEC2025.md` - Problèmes moteurs

---

## 📝 **NOTES**

- Ce dossier est un **clone local** du dépôt GitHub officiel
- Il contient le **code source complet** du SDK Reachy Mini
- Il est sur la branche `develop` (version de développement)
- Il peut être utilisé pour **développement et tests**
- Il ne remplace **pas** le package installé via PyPI

---

**En résumé** : Ce dossier est votre **référence locale** du SDK officiel. Gardez-le pour consultation et développement, mais utilisez le package installé pour la production ! 📦
