# 📱 Application Reachy Mini Control - Documentation

**Date** : 15 Décembre 2025  
**App** : Reachy Mini Control (officielle Pollen Robotics)  
**Version** : **0.7.18** (décembre 2025)  
**Bundle ID** : `com.pollen-robotics.reachy-mini`  
**Emplacement** : `/Volumes/Reachy Mini Control/Reachy Mini Control.app`  
**Système requis** : macOS 10.15+ (Catalina ou supérieur)

## 📥 Téléchargement - Quelle Version ?

### ✅ Versions Disponibles

| Plateforme | Architecture | Fichier | Statut |
|------------|--------------|---------|--------|
| **macOS** | Apple Silicon (M1/M2/M3) | `.dmg` | ✅ Disponible |
| **macOS** | Intel (x86_64) | `.dmg` | ✅ Disponible |
| **Windows** | 64 bits | `.msi` | ⚠️ À venir |
| **Linux** | Debian/Ubuntu | `.deb` | ⚠️ À venir |

### 🔍 Comment Savoir Quelle Version Télécharger ?

**Pour macOS** :

1. **Vérifier votre architecture** :
   ```bash
   uname -m
   ```
   - `arm64` → **Apple Silicon** (M1/M2/M3) → Télécharger "macOS Apple Silicon"
   - `x86_64` → **Intel** → Télécharger "macOS Intel"

2. **Ou vérifier dans "À propos de ce Mac"** :
   - **Apple Silicon** : "Chip Apple" (M1, M2, M3, etc.)
   - **Intel** : "Processeur Intel Core"

### ⚠️ Problème Actuel

Le script `mjpython` dans l'app pointe vers un chemin de build (`/Users/runner/work/...`) qui n'existe pas sur votre machine. Cela indique que :

- ✅ Vous avez probablement la **bonne version** (Apple Silicon ou Intel selon votre Mac)
- ❌ Mais l'app a un **problème de packaging** (chemins hardcodés)

**Solution** : Utiliser BBIA à la place (recommandé) ou attendre une mise à jour de l'app.

---

## 🎯 Qu'est-ce que Reachy Mini Control ?

**Reachy Mini Control** est l'application officielle de Pollen Robotics pour contrôler votre robot Reachy Mini depuis un appareil (Mac, Windows, iOS, Android).

### Informations Techniques

- **Version** : 0.7.18
- **Python** : 3.12 (intégré dans l'app)
- **Port daemon** : **8000** (HTTP) - ⚠️ **Important** : Port différent de BBIA (8080)
- **Protocole** : HTTP/HTTPS + Bonjour (découverte automatique)
- **Réseau** : Accès réseau local requis

### Fonctionnalités Principales

- ✅ **Contrôle des mouvements** : Tête, antennes, corps
- ✅ **Vision en direct** : Stream caméra du robot
- ✅ **Audio** : Microphone et haut-parleur
- ✅ **Émotions** : Contrôle des expressions (6 émotions officielles)
- ✅ **Configuration** : Paramètres réseau, Wi-Fi, etc.
- ✅ **Installation apps** : Gestion et installation d'applications Reachy Mini depuis Hugging Face Spaces
- ✅ **Découverte automatique** : Détection automatique du robot sur le réseau local (Bonjour)

---

## 🔗 Intégration avec BBIA

### ✅ Compatibilité

**BBIA est compatible avec Reachy Mini Control** car :

1. **Même SDK** : Les deux utilisent le SDK officiel `reachy_mini`
2. **API similaire** : Endpoints REST similaires mais ports différents
3. **Protocole** : Communication via HTTP (ports différents)

### ⚠️ Ports Différents

| Service | Port | Note |
|---------|------|------|
| **Reachy Mini Control** | **8000** | Port par défaut de l'app |
| **BBIA Dashboard** | **8000** | Port par défaut BBIA |
| **BBIA Daemon** | **8080** | Port alternatif BBIA |
| **Zenoh** | **7447** | Protocole sous-jacent (les deux) |

**⚠️ Attention** : Si BBIA et l'app tournent en même temps, ils peuvent entrer en conflit sur le port 8000. Utiliser des ports différents si nécessaire.

### ⚠️ Différences

| Aspect | Reachy Mini Control | BBIA |
|--------|---------------------|------|
| **Interface** | App graphique native | Dashboard web + API |
| **Fonctionnalités** | Contrôle basique | IA avancée (YOLO, Whisper, LLM) |
| **Émotions** | 6 émotions officielles | 12 émotions (6 officielles + 6 étendues) |
| **Vision** | Stream caméra | YOLO + MediaPipe + SmolVLM2 |
| **Voice** | Basique | Whisper STT + pyttsx3 TTS |

### 🎯 Utilisation Conjointe

**Vous pouvez utiliser les deux en même temps !**

1. **Reachy Mini Control** : Pour contrôle manuel et configuration
2. **BBIA** : Pour IA avancée et comportements autonomes

**Comment ?**

- Les deux se connectent au même robot via le même SDK
- BBIA peut fonctionner en arrière-plan pendant que vous utilisez l'app
- L'app peut être utilisée pour tester/calibrer avant d'utiliser BBIA

---

## 📋 Utilisation de l'App (Avant Réception)

### Préparation

1. **Installer l'app** : Déjà fait ✅ (`/Volumes/Reachy Mini Control/Reachy Mini Control.app`)

2. **Lire la documentation** : 
   - Guide d'utilisation dans l'app
   - Documentation Pollen : https://docs.pollen-robotics.com/

3. **Préparer connexion** :
   - Noter SSID Wi-Fi et mot de passe
   - Vérifier ports réseau (8080, 8081, 7447)

### Après Réception du Robot

1. **Démarrage initial** :
   - Allumer robot
   - Configurer Wi-Fi
   - Noter adresse IP du robot

2. **Connexion avec l'app** :
   - Ouvrir "Reachy Mini Control"
   - Entrer adresse IP du robot (ou laisser détection automatique)
   - Tester connexion

3. **Tests basiques** :
   - Contrôler mouvements tête
   - Tester caméra
   - Vérifier audio
   - Tester émotions

4. **Ensuite, tester BBIA** :
   - Lancer BBIA avec `localhost_only=False`
   - Vérifier que BBIA se connecte au robot
   - Comparer fonctionnalités

---

## 🔧 Configuration pour BBIA

### Utiliser l'App pour Configurer le Robot

L'app "Reachy Mini Control" peut être utilisée pour :

1. **Configuration réseau** :
   - Vérifier connexion Wi-Fi
   - Noter adresse IP
   - Tester ports réseau

2. **Calibration** :
   - Calibrer mouvements
   - Tester limites articulations
   - Vérifier caméra/audio

3. **Tests basiques** :
   - Vérifier que tout fonctionne
   - Identifier problèmes hardware
   - Préparer pour BBIA

### Ensuite, Utiliser BBIA

Une fois le robot configuré avec l'app :

```python
# BBIA se connecte au même robot
from bbia_sim.robot_factory import RobotFactory

robot = RobotFactory.create_backend(
    "reachy_mini",
    localhost_only=False,  # Pour version Wireless (défaut: True pour sécurité)
    use_sim=False  # Robot physique (défaut: True pour simulation)
)

# BBIA peut maintenant utiliser toutes ses fonctionnalités IA
```

**Note importante** :
- Par défaut, `RobotFactory.create_backend('reachy_mini')` utilise :
  - `localhost_only=True` (sécurité, connexion locale uniquement)
  - `use_sim=True` (mode simulation pour éviter timeout si pas de robot)
- Pour version Wireless, il faut explicitement :
  - `localhost_only=False` (permettre connexion réseau)
  - `use_sim=False` (chercher robot physique)
- **Important** : Vérifier firewall et réseau avant d'utiliser `localhost_only=False`

---

## 📝 Checklist Utilisation App

### Avant Réception
- [x] ✅ App téléchargée (`/Volumes/Reachy Mini Control/Reachy Mini Control.app`)
- [ ] 📚 Lire documentation app (si disponible)
- [ ] 🌐 Préparer réseau Wi-Fi

### Après Réception (Jour J)
- [ ] 🔌 Allumer robot
- [ ] 📡 Configurer Wi-Fi
- [ ] 📱 Ouvrir app "Reachy Mini Control"
- [ ] 🔗 Connecter app au robot (IP ou auto-détection)
- [ ] ✅ Tester connexion
- [ ] 🎮 Tester mouvements basiques
- [ ] 📷 Tester caméra
- [ ] 🔊 Tester audio
- [ ] 😊 Tester émotions
- [ ] 📝 Noter adresse IP pour BBIA

### Ensuite
- [ ] 🧠 Tester BBIA avec robot réel
- [ ] 🔄 Comparer fonctionnalités app vs BBIA
- [ ] 📊 Documenter différences et avantages

---

## 🎯 Avantages de BBIA vs App

### ✅ Ce que BBIA fait mieux

1. **IA Avancée** :
   - Vision : YOLO + MediaPipe + SmolVLM2 (vs stream basique)
   - Voice : Whisper STT + pyttsx3 TTS (vs basique)
   - LLM : Conversation intelligente (vs pas de LLM)

2. **Émotions Étendues** :
   - 12 émotions (vs 6 officielles)
   - Expressions plus nuancées

3. **Comportements Autonomes** :
   - Behaviors intelligents
   - Réactions contextuelles
   - Apprentissage adaptatif

4. **API Complète** :
   - REST + WebSocket
   - Intégration disponible
   - Extensible

### ✅ Ce que l'App fait mieux

1. **Interface Graphique** :
   - App native (vs web dashboard)
   - Contrôle tactile
   - Interface intuitive

2. **Simplicité** :
   - Plug & play
   - Pas de configuration complexe
   - Interface simplifiée

3. **Configuration** :
   - Setup réseau simplifié
   - Calibration guidée
   - Tests hardware

---

## 🔗 Liens Utiles

- **Documentation Pollen** : https://docs.pollen-robotics.com/
- **App Store** : (vérifier sur site Pollen)
- **GitHub SDK** : https://github.com/pollen-robotics/reachy_mini
- **BBIA Dashboard** : `http://localhost:8000` (après lancement BBIA)

---

## 📝 Notes Importantes

### ⚠️ Conflits Potentiels

**Les deux peuvent fonctionner en même temps** car :
- Même SDK sous-jacent
- Même protocole de communication
- Pas de verrous exclusifs

**Recommandation** :
- Utiliser l'app pour configuration/tests
- Utiliser BBIA pour IA/comportements avancés
- Éviter de contrôler les mêmes articulations simultanément

### ✅ Workflow Recommandé

1. **Jour 1** : Utiliser app pour setup et tests basiques
2. **Jour 2** : Tester BBIA avec robot réel
3. **Jour 3+** : Utiliser BBIA pour développement IA
4. **App** : Garder pour calibration et tests rapides

---

## 🔧 Dépannage

### ❌ Erreur : "Unable to spawn process: No such file or directory (os error 2)"

**Symptômes** :
```
❌ Error: Unable to spawn process: No such file or directory (os error 2)
⚠️  Warning: Python binary not properly signed in production!
```

**Causes possibles** :

1. **Permissions macOS** : L'app n'a pas les permissions pour exécuter des binaires non signés
2. **Binaire manquant** : Le wrapper `mjpython` ne trouve pas le binaire Python sous-jacent
3. **Chemin incorrect** : L'app a été déplacée et les chemins relatifs ne fonctionnent plus

**Solutions** :

#### Solution 1 : Vérifier les permissions macOS

1. **Ouvrir Préférences Système** → **Sécurité et confidentialité**
2. **Autoriser l'app** : Si l'app est bloquée, cliquer sur "Ouvrir quand même"
3. **Permissions réseau** : Vérifier que l'app a accès au réseau local

#### Solution 2 : Réinstaller l'app

1. **Supprimer l'app** de `/Applications/`
2. **Copier depuis le volume** `/Volumes/Reachy Mini Control/`
3. **Lancer depuis Applications** (pas depuis le volume)

#### Solution 3 : Utiliser BBIA à la place (Recommandé)

Si l'app ne fonctionne pas, **BBIA peut faire la même chose** :

```bash
# Lancer BBIA en mode simulation (comme l'app)
python -m bbia_sim.daemon.app.main --sim

# Ou utiliser le dashboard BBIA
python -m bbia_sim.dashboard
```

**Avantages BBIA** :
- ✅ Pas de problèmes de signature macOS
- ✅ Plus de fonctionnalités (IA avancée)
- ✅ Open source, pas de restrictions

#### Solution 4 : Contacter le support Pollen

Si le problème persiste :
- **Discord** : https://discord.gg/pollen-robotics
- **GitHub Issues** : https://github.com/pollen-robotics/reachy_mini/issues
- **Email** : support@pollen-robotics.com

### Mode Simulation

L'app essaie de lancer le daemon en **mode simulation** (MuJoCo) car :
- Vous n'avez pas encore le robot physique
- C'est normal et attendu

**Une fois le robot reçu** :
- L'app détectera automatiquement le robot sur le réseau
- Le mode simulation ne sera plus nécessaire
- L'erreur devrait disparaître

---

**Dernière mise à jour** : 15 Décembre 2025

