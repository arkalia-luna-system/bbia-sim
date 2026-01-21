# Guide Complet - Réception et Utilisation Reachy Mini Wireless

**Dernière mise à jour** : 20 Janvier 2026  
**Version BBIA** : 1.4.0  
**SDK Officiel** : v1.2.11 (Latest - 14 janvier 2026) - Voir `ANALYSE_REPO_OFFICIEL_JANVIER_2026.md`  
**SDK Actuel BBIA** : v1.2.4 - Voir `REACHY_MINI_SDK_v1.2.4.md` pour détails  
**Note** : Aucune nouvelle version SDK depuis le 17 janvier 2026

## 📦 STATUT RÉCEPTION

✅ **Robot reçu** : 18 Décembre 2025  
✅ **Montage effectué** : 20 Décembre 2025 (durée : 4 heures)  
✅ **Premiers tests** : 22 Décembre 2025  
✅ **IP Robot** : 192.168.129.64 (WiFi configuré)

### Problèmes rencontrés et résolus

- ✅ **Bug décembre 2025** : Moteurs avec paramètres d'usine incorrects (ID=1, baudrate 57,600) → **Reflash effectué** le 22 déc 2025
- ⚠️ **Tête penchée** : Correction logicielle effectuée, vérification câblage nécessaire
- 🔴 **Moteurs défectueux (batch QC 2544/2543)** : **Problème matériel identifié par Pollen** → Voir `PROBLEME_MOTEURS_QC_BATCH_DEC2025.md` et `REACHY_MINI_SDK_v1.2.4.md`
  - Moteur 1 (QC 2543) : Raide mécaniquement → ✅ **3 moteurs reçus le 17 janvier 2026**
  - Moteur 2 (QC 2544) : Raide + clignotement rouge → ✅ **En attente d'installation**
  - Moteur 4 (QC 2544) : Raide mécaniquement → ✅ **Voir GUIDE_PREVENTION_PROBLEMES_MOTEURS.md**
  - **Cause** : Moteurs non flashés correctement à l'usine (SDK v1.2.4+ va reflasher automatiquement)
  - **Statut** : ✅ Moteurs reçus (QC 2549 vérifié le 21 janvier 2026 - batch sain ✅), installation à prévoir selon guide de prévention

---

## Spécificités Version Wireless

**Avantages** :

- Connexion Wi-Fi (pas de câble USB)
- Batterie intégrée
- Raspberry Pi 5 intégré
- 4 microphones + haut-parleur 5W

**Différences vs Version Lite** :

| Aspect | Wireless | Lite |
| ------ | -------- | ---- |
| Connexion | Wi-Fi | USB |
| Alimentation | Batterie + USB-C | USB uniquement |
| Processeur | Raspberry Pi 5 intégré | Externe |
| Configuration | Wi-Fi requise | Plug & Play |

**Configuration Wi-Fi requise** :

- Robot et ordinateur sur le même réseau Wi-Fi
- Ports 8080 et 8081 accessibles (firewall)
- Adresse IP du robot à noter lors du premier démarrage

**À faire lors de la réception** :

- [ ] Configurer Wi-Fi (guide d'assemblage)
- [ ] Noter l'adresse IP
- [ ] Tester : `ping <IP_ROBOT>`

---

## Matériel et Outils

**Inclus dans le kit** :

- Robot Reachy Mini (composants mécaniques)
- Raspberry Pi 5 (intégré, OS pré-installé)
- Carte SD 64GB+ (OS pré-installé) - voir `CARTE_SD_REACHY_MINI.md`
- Batterie, microphones, haut-parleur, caméra, IMU
- Câbles, connecteurs, vis, guide d'assemblage

**À vérifier/prévoir** :

| Matériel | Statut | Si non inclus | Prix |
| -------- | ------ | ------------- | ---- |
| Carte SD | Incluse | Voir `CARTE_SD_REACHY_MINI.md` | - |
| Chargeur USB-C (5V/3A) | À vérifier | Amazon, Fnac | 10-15€ |
| Tournevis (petite taille) | Obligatoire | Quincaillerie, Amazon | 5-15€ |
| Pinces (petite taille) | Optionnel | Quincaillerie, Amazon | 5-10€ |

---

## Logiciel

**SDK Reachy Mini** :

- [x] ✅ Installé v1.2.4 : `pip install --upgrade "reachy-mini>=1.2.0"`
- [x] ✅ Compatibilité BBIA vérifiée et validée
- [ ] Vérifier changelog : <https://github.com/pollen-robotics/reachy_mini/releases>
- **Note** : Pour les fonctionnalités IA avancées (vision, LLM), utiliser `pip install "reachy-mini[full]"` (inclut transformers, tqdm, etc.)

**BBIA-SIM** :

- [ ] Vérifier installation : `pip install -e .`
- [ ] Tester simulation : `python examples/reachy_mini/minimal_demo.py`

**Configuration réseau (Wireless)** :

- [ ] Wi-Fi actif, SSID/mot de passe notés
- [ ] Ports 8080 et 8081 ouverts (firewall)
- [ ] Configuration BBIA : `localhost_only=False` (crucial pour Wireless)
  - **Note** : Par défaut, `RobotFactory.create_backend('reachy_mini')` utilise `localhost_only=True` (sécurité)
  - Pour version Wireless, il faut explicitement passer `localhost_only=False` pour permettre connexion réseau
  - **Important** : Vérifier firewall et réseau avant d'utiliser `localhost_only=False`

---

## Documentation

**Guides officiels Pollen** :

- Guide d'assemblage : <https://github.com/pollen-robotics/reachy_mini/blob/develop/docs/platforms/reachy_mini/get_started.md>
- Guide interactif : <https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide>
- Documentation SDK : <https://docs.pollen-robotics.com/>

**Guides BBIA** :

- `docs/guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md`
- `docs/hardware/CHECKLIST_VALIDATION_HARDWARE_DECEMBRE_2025.md`
- `docs/hardware/APP_REACHY_MINI_CONTROL.md`

**Communauté** : <https://discord.gg/pollen-robotics>

---

## État BBIA

**Conformité SDK** : 100% (21 méthodes, 37 tests passants)  
**Modules** : Vision (YOLO+MediaPipe), Audio (Whisper), Émotions (12), Mouvements, IA Conversation  
**Tests** : 1,362 tests, coverage 68.86%, CI/CD OK

---

## SDK Officiel

**Dernière version** : v1.2.11 (Latest - 14 janvier 2026)  
**BBIA utilise** : v1.2.4 ⚠️ (7 versions de retard, mise à jour recommandée après installation moteurs)

**Statut** :

- [x] ✅ Version installée : v1.2.4
- [x] ✅ Compatibilité : Testée et validée
- [x] ✅ Toutes les fonctionnalités SDK disponibles
- [ ] ⏳ Mise à jour vers v1.2.11 recommandée après installation des nouveaux moteurs

**Nouvelles fonctionnalités v1.2.5 → v1.2.11** :

- Intégration Hugging Face pour applications IA
- Applications : Hand Tracking, LLM Companion, Dance Dance Mini
- Firmware moteurs basé sur Rust (meilleure précision)
- Installation complète : `pip install "reachy-mini[full]"` pour dépendances IA

**Comparaison** : BBIA ~90-95% de parité + innovations (12 émotions vs 6, vision/audio avancés, RobotAPI unifié)

---

## Plan (15-18 Décembre)

**15 Décembre** :

- [ ] Commander chargeur USB-C si nécessaire
- [ ] Lire guide d'assemblage

**16 Décembre** :

- [ ] Installer SDK v1.2.0
- [ ] Tester compatibilité BBIA
- [ ] Rejoindre Discord Pollen

**17 Décembre** :

- [ ] Tester simulation BBIA
- [ ] Vérifier configuration Wi-Fi
- [ ] Préparer espace de travail

**18 Décembre - Réception** :

- [ ] Réception colis (vérifier contenu, photographier)
- [ ] Assemblage (2-3 heures, suivre guide)
- [ ] Premier démarrage (voir section détaillée ci-dessous)

---

## 🚀 Premier Démarrage avec Robot Physique

### Checklist de Connexion

**Avant de commencer** :

- [ ] Robot assemblé et allumé (LED verte)
- [ ] Robot connecté au WiFi (même réseau que votre ordinateur)
- [ ] IP du robot identifiée (voir méthodes ci-dessous)
- [ ] Ports réseau ouverts (8000 pour API, 7447 pour Zenoh)

### Étape 1 : Identifier l'IP du Robot

**Méthode 1 : Via Dashboard Robot**

```bash
# Le robot expose un hotspot WiFi temporaire au démarrage
# Connectez-vous au réseau "Reachy-Mini-XXXX"
# Ouvrez http://192.168.4.1 dans votre navigateur
# Configurez le WiFi et notez l'IP assignée
```

**Méthode 2 : Scan Réseau**

```bash
# Sur macOS/Linux
nmap -sn 192.168.1.0/24 | grep -B 2 "Reachy"

# Ou utiliser l'app Reachy Mini Control (iOS/Android)
```

**Méthode 3 : Via Router**

- Accédez à l'interface de votre routeur
- Cherchez l'appareil "Reachy-Mini" ou "pollen"
- Notez l'IP assignée

### Étape 2 : Vérifier la Connexion Réseau

```bash
# Test ping
ping <IP_ROBOT>

# Test API (si daemon lancé)
curl http://<IP_ROBOT>:8000/api/state/full

# Test Zenoh (si daemon lancé)
# Le port 7447 doit être accessible
```

### Étape 3 : Lancer le Backend Zenoh

**Option A : Via Dashboard (Recommandé)**

```bash
# Ouvrir dans navigateur
http://<IP_ROBOT>:8000

# Cliquer sur "Start" dans la section Daemon
```

**Option B : Via SSH (si accès disponible)**

```bash
# Se connecter au robot
ssh pollen@<IP_ROBOT>

# Lancer le daemon
reachy-mini-daemon
```

**Option C : Depuis votre Mac (si configuré)**

```bash
# Le daemon peut tourner sur votre Mac et se connecter au robot
# Voir docs/guides/DEMARRAGE_DAEMON.md
```

### Étape 4 : Test Connexion SDK

**Test SDK Officiel**

```python
from reachy_mini import ReachyMini
from reachy_mini.utils import create_head_pose

# Connexion au robot (localhost_only=False pour réseau)
robot = ReachyMini(
    localhost_only=False,  # ← CRITIQUE pour connexion réseau
    use_sim=False,
    timeout=30.0
)

with robot:
    # Test connexion
    pose = robot.head.head_pose
    print(f"✅ Robot connecté - Position: {pose}")
    
    # Test mouvement simple
    robot.goto_target(
        head=create_head_pose(roll=10, degrees=True),
        duration=2.0
    )
```

**Test BBIA-SIM**

```python
from bbia_sim.robot_factory import RobotFactory

# Option 1: Mode auto (détection automatique + fallback sim)
robot = RobotFactory.create_backend('auto')

# Option 2: Mode explicite (robot physique)
robot = RobotFactory.create_backend(
    'reachy_mini',
    localhost_only=False,  # ← CRITIQUE pour connexion réseau
    use_sim=False,
    timeout=30.0
)

# Option 3: Mode auto avec fallback
# Si robot non disponible, bascule automatiquement vers simulation
robot = RobotFactory.create_backend('auto')

if robot:
    robot.connect()
    if robot.is_connected:
        print("✅ Robot connecté via BBIA-SIM")
        # Utiliser robot...
```

### Étape 5 : Test Complet BBIA

```python
# Exemple complet avec BBIA
from bbia_sim.robot_factory import RobotFactory
from bbia_sim.bbia_emotions import BBIAEmotions

# Connexion
robot = RobotFactory.create_backend('auto')  # Auto-détection
robot.connect()

# Test émotions
emotions = BBIAEmotions()
emotions.set_emotion(robot, 'happy', intensity=0.8)

# Test mouvement
from reachy_mini.utils import create_head_pose
robot.goto_target(
    head=create_head_pose(roll=15, pitch=10, degrees=True),
    duration=2.0
)

print("✅ Tests complets réussis")
```

### Troubleshooting Commun

**Problème : Timeout de connexion**

```python
# Solution 1: Augmenter timeout
robot = ReachyMini(localhost_only=False, timeout=60.0)

# Solution 2: Vérifier que le daemon est lancé
# curl http://<IP_ROBOT>:8000/api/state/full

# Solution 3: Vérifier firewall
# Les ports 8000 et 7447 doivent être ouverts
```

**Problème : Robot non trouvé**

```python
# Utiliser mode auto avec fallback
robot = RobotFactory.create_backend('auto')
# Si robot non disponible, utilise automatiquement simulation
```

**Problème : Erreur Zenoh**

```bash
# Vérifier que Zenoh est installé
pip show eclipse-zenoh

# Tester connexion Zenoh locale
python -c "import zenoh; s = zenoh.open(); s.close(); print('Zenoh OK')"
```

### Checklist Finale

- [ ] ✅ Robot allumé et connecté au WiFi
- [ ] ✅ IP robot identifiée et accessible (ping OK)
- [ ] ✅ Backend Zenoh lancé (daemon actif)
- [ ] ✅ Test SDK officiel réussi
- [ ] ✅ Test BBIA-SIM réussi
- [ ] ✅ Mouvements de base fonctionnels
- [ ] ✅ Émotions BBIA applicables

**Une fois cette checklist complète, vous êtes prêt à utiliser BBIA-SIM avec le robot physique.**

---

## Notes

**SDK v1.2.0** : Vérifier changelog et tester compatibilité avant réception  
**BBIA** : 100% conforme SDK, 1,362 tests passants

---

## Références

- GitHub : <https://github.com/pollen-robotics/reachy_mini>
- Documentation : <https://docs.pollen-robotics.com/>
- Guide d'assemblage : <https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide>
- Discord : <https://discord.gg/pollen-robotics>

---

**Dernière mise à jour** : 20 Janvier 2026

