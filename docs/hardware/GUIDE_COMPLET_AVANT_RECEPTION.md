# Guide - Avant Réception Reachy Mini Wireless

**Date** : 15 Décembre 2025  
**Livraison prévue** : 18 Décembre 2025  
**Version** : Reachy Mini Wireless

---

## Spécificités Version Wireless

**Avantages** :
- Connexion Wi-Fi (pas de câble USB)
- Batterie intégrée
- Raspberry Pi 5 intégré
- 4 microphones + haut-parleur 5W

**Différences vs Version Lite** :

| Aspect | Wireless | Lite |
|--------|----------|------|
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
|----------|--------|---------------|------|
| Carte SD | Incluse | Voir `CARTE_SD_REACHY_MINI.md` | - |
| Chargeur USB-C (5V/3A) | À vérifier | Amazon, Fnac | 10-15€ |
| Tournevis (petite taille) | Obligatoire | Quincaillerie, Amazon | 5-15€ |
| Pinces (petite taille) | Optionnel | Quincaillerie, Amazon | 5-10€ |

---

## Logiciel

**SDK Reachy Mini** :
- [ ] Installer v1.2.0 : `pip install --upgrade "reachy-mini>=1.2.0"`
- [ ] Vérifier changelog : https://github.com/pollen-robotics/reachy_mini/releases/tag/v1.2.0
- [ ] Tester compatibilité BBIA

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
- Guide d'assemblage : https://github.com/pollen-robotics/reachy_mini/blob/develop/docs/platforms/reachy_mini/get_started.md
- Guide interactif : https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide
- Documentation SDK : https://docs.pollen-robotics.com/

**Guides BBIA** :
- `docs/guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md`
- `docs/hardware/CHECKLIST_VALIDATION_HARDWARE_DECEMBRE_2025.md`
- `docs/hardware/APP_REACHY_MINI_CONTROL.md`

**Communauté** : https://discord.gg/pollen-robotics

---

## État BBIA

**Conformité SDK** : 100% (21 méthodes, 37 tests passants)  
**Modules** : Vision (YOLO+MediaPipe), Audio (Whisper), Émotions (12), Mouvements, IA Conversation  
**Tests** : 1,362 tests, coverage 68.86%, CI/CD OK

---

## SDK Officiel

**Dernière version** : v1.2.0 (12 Décembre 2025)  
**BBIA utilise** : v1.1.3 (compatible, vérifier v1.2.0)

**Action requise** :
- [ ] Vérifier changelog v1.2.0 : https://github.com/pollen-robotics/reachy_mini/releases/tag/v1.2.0
- [ ] Tester compatibilité : `pip install --upgrade "reachy-mini>=1.2.0"`
- [ ] Mettre à jour si breaking changes

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
- [ ] Premier démarrage :
  - Allumer robot
  - Configurer Wi-Fi
  - Noter adresse IP
  - Tester : `ping <IP_ROBOT>`
- [ ] Tests connexion :
  ```python
  # SDK
  from reachy_mini import ReachyMini
  robot = ReachyMini(localhost_only=False, use_sim=False)
  
  # BBIA
  from bbia_sim.robot_factory import RobotFactory
  robot = RobotFactory.create_backend('reachy_mini', localhost_only=False, use_sim=False)
  ```
  
  **Note importante** :
  - Par défaut, `RobotFactory.create_backend('reachy_mini')` utilise `use_sim=True` (mode simulation)
  - Pour un robot physique, il faut **explicitement** passer `use_sim=False`
  - Pour version Wireless, il faut aussi `localhost_only=False` (par défaut `True` pour sécurité)

---

## Notes

**SDK v1.2.0** : Vérifier changelog et tester compatibilité avant réception  
**BBIA** : 100% conforme SDK, 1,362 tests passants

---

## Références

- GitHub : https://github.com/pollen-robotics/reachy_mini
- Documentation : https://docs.pollen-robotics.com/
- Guide d'assemblage : https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide
- Discord : https://discord.gg/pollen-robotics

---

**Dernière mise à jour** : 15 Décembre 2025

