# 🧪 Tests BBIA

> **Tests et simulations pour BBIA - Brain-Based Interactive Agent**

## 🎯 **Tests Disponibles**

### 🎮 **Simulations Principales**
- **`test_bbia_reachy.py`** - Simulation BBIA de base
- **`demo_bbia_complete.py`** - Démonstration complète
- **`reachy_local_test.py`** - Test local Reachy
- **`reachy_test_sim.py`** - Test simulation Reachy
- **`reachy_websim_test.py`** - Test simulation web

---

## 🚀 **Utilisation Rapide**

### 🎮 **Simulation BBIA de Base**
```bash
python3 tests/test_bbia_reachy.py
```
**Fonctionnalités :**
- 🤖 6 émotions (neutral, happy, sad, angry, curious, excited)
- 🎤 4 microphones simulés avec reconnaissance vocale
- 📷 Caméra grand angle avec reconnaissance d'objets
- 🤖 Mouvements tête 6 DOF
- 📡 Animation des antennes selon l'émotion
- 🗣️ Reconnaissance vocale
- 🔋 Test de batterie

### 🎯 **Démonstration Complète**
```bash
python3 tests/demo_bbia_complete.py
```
**Fonctionnalités :**
- 🔍 Test de tous les composants installés
- 👁️ Démonstration pollen-vision
- 🎭 Toutes les émotions BBIA
- 🗣️ Reconnaissance vocale avancée
- 📚 Liste des composants disponibles

### 🧪 **Tests Reachy**
```bash
# Test local
python3 tests/reachy_local_test.py

# Test simulation
python3 tests/reachy_test_sim.py

# Test simulation web
python3 tests/reachy_websim_test.py
```

---

## 📁 **Structure des Tests**

```
tests/
├── 🎮 test_bbia_reachy.py            # Simulation BBIA de base
├── 🎯 demo_bbia_complete.py          # Démonstration complète
├── 🧪 reachy_local_test.py           # Test local Reachy
├── 🧪 reachy_test_sim.py             # Test simulation Reachy
├── 🧪 reachy_websim_test.py          # Test simulation web
└── 📖 README.md                      # Ce fichier
```

---

## 🎯 **Détails des Tests**

### 🎮 **test_bbia_reachy.py**
**Fonction :** Simulation BBIA de base avec toutes les fonctionnalités
**Émotions :** 6 émotions différentes avec animations
**Audio :** 4 microphones simulés avec reconnaissance vocale
**Vision :** Caméra grand angle avec reconnaissance d'objets
**Mouvements :** Tête 6 DOF + animation des antennes

### 🎯 **demo_bbia_complete.py**
**Fonction :** Démonstration complète de tous les composants
**Tests :** Vérification de tous les dépôts installés
**Vision :** Démonstration pollen-vision
**Émotions :** Toutes les émotions BBIA en détail
**Audio :** Reconnaissance vocale avancée

### 🧪 **reachy_local_test.py**
**Fonction :** Test local du robot Reachy
**Connexion :** Test de connexion au robot
**Mouvements :** Test des mouvements de base
**Capteurs :** Test des capteurs

### 🧪 **reachy_test_sim.py**
**Fonction :** Test de simulation Reachy
**Simulation :** Test de la simulation locale
**Interface :** Test de l'interface de simulation
**Fonctionnalités :** Test des fonctionnalités de base

### 🧪 **reachy_websim_test.py**
**Fonction :** Test de simulation web Reachy
**Web :** Test de la simulation web
**Interface :** Test de l'interface web
**Connexion :** Test de connexion web

---

## 🎯 **Commandes Rapides**

### 🚀 **Tests Principaux**
```bash
# Simulation BBIA de base
python3 tests/test_bbia_reachy.py

# Démonstration complète
python3 tests/demo_bbia_complete.py

# Menu interactif (recommandé)
./scripts/quick_start.sh
```

### 🧪 **Tests Reachy**
```bash
# Test local
python3 tests/reachy_local_test.py

# Test simulation
python3 tests/reachy_test_sim.py

# Test simulation web
python3 tests/reachy_websim_test.py
```

### 🔍 **Vérifications**
```bash
# Vérifier les dépôts
ls -la reachy_repos/

# Vérifier les packages
pip list | grep -i reachy
pip list | grep -i pollen

# Tester pollen-vision
python3 -c "import pollen_vision; print('✅ Vision OK')"
```

---

## 🎯 **Fonctionnalités Testées**

### 🤖 **BBIA Core**
- **6 émotions** : neutral, happy, sad, angry, curious, excited
- **4 microphones** simulés avec reconnaissance vocale
- **Caméra grand angle** avec reconnaissance d'objets
- **Mouvements tête 6 DOF** (6 degrés de liberté)
- **Animation des antennes** selon l'émotion
- **Test de batterie** simulé

### 👁️ **Vision par Ordinateur**
- **pollen-vision** installé et testé
- **Reconnaissance d'objets** en temps réel
- **Détection de visages** et expressions
- **Analyse de mouvements**
- **Suivi d'objets**

### 🎤 **Audio et Voix**
- **4 microphones** simulés
- **Reconnaissance vocale** en temps réel
- **Synthèse vocale** 5W
- **Phrases détectées** automatiquement

---

## 💡 **Conseils d'Utilisation**

1. **Commencez par BBIA** : `python3 tests/test_bbia_reachy.py`
2. **Explorez la démo** : `python3 tests/demo_bbia_complete.py`
3. **Utilisez le menu** : `./scripts/quick_start.sh`
4. **Testez Reachy** : `python3 tests/reachy_local_test.py`
5. **Vérifiez l'installation** : `python3 -c "import pollen_vision; print('✅ Vision OK')"`

---

## 🎤 **Tests Audio & Voix**

### bbia_audio.py
- Teste l’enregistrement, la lecture et la détection de son (mock)
- Nécessite : sounddevice, numpy, wave

### bbia_voice.py
- Teste la synthèse vocale (pyttsx3, voix Amélie fr_CA prioritaire)
- Teste la reconnaissance vocale (speech_recognition, PyAudio)
- Nécessite : pyttsx3, speech_recognition, pyaudio
- Conseil : Installe la voix Amélie (français Canada) dans Préférences Système > Accessibilité > Parole > Voix du système

### Dépannage rapide
- **Erreur PyAudio** :
  - Installe portaudio (`brew install portaudio`)
  - Puis `pip install pyaudio`
- **Aucune voix Amélie trouvée** :
  - Installe la voix Amélie (fr_CA) dans les réglages système
  - Relance le script

---

## 🎯 **Dépannage**

### ❌ **Problèmes Courants**
- **Module not found** : Lancez `./scripts/setup_reachy_environment.sh`
- **Dépôts manquants** : Lancez `./scripts/install_all_reachy_repos.sh`
- **Erreurs Unity** : Lancez `./scripts/fix_unity_warnings.sh`
- **Permissions** : `chmod +x scripts/*.sh`

### ✅ **Solutions**
- **Tous les tests** sont fonctionnels
- **Gestion d'erreurs** intégrée
- **Vérifications automatiques** après installation
- **Documentation** complète pour chaque test

---

## 🎯 **Résultats Attendus**

### 🎮 **test_bbia_reachy.py**
- Affichage des 6 émotions avec animations
- Test des microphones et reconnaissance vocale
- Test de la caméra et reconnaissance d'objets
- Test des mouvements de tête et antennes
- Test de la batterie

### 🎯 **demo_bbia_complete.py**
- Liste de tous les composants installés
- Test de pollen-vision
- Démonstration de toutes les émotions
- Test de reconnaissance vocale avancée
- Plan de développement Phase 2

---

**BBIA** - Brain-Based Interactive Agent  
*Tests et simulations* 🧪✨

**Version** : 2.0  
**Date** : 15 juillet 2024  
**Tests** : ✅ 5 tests fonctionnels  
**Simulations** : ✅ Complètes 