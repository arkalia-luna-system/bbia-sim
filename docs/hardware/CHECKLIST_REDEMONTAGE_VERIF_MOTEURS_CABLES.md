# Checklist : redémonter Reachy pour vérifier moteurs / câbles

**Quand** : à faire quand tu reprends le robot (après ta pause).  
**But** : vérifier physiquement les moteurs et câbles, puis relancer le processus jusqu’à avoir le robot qui répond correctement.

Remplace `<ROBOT_IP>` par l’IP réelle de ton robot (ex. `192.168.x.x`) dans toutes les commandes.

---

## 1. Avant de démonter

- [ ] Robot **éteint** (débrancher l’alimentation).
- [ ] Noter l’état actuel : quels moteurs manquants / erreurs (ID 10, 11, 12, etc.) d’après le Testbench ou le dashboard.
- [ ] Avoir sous la main :
  - [Guide installation moteurs](GUIDE_INSTALLATION_MOTEURS_ETAPE_PAR_ETAPE.md)
  - [Description problème et composants](DESCRIPTION_PROBLEME_ET_COMPOSANTS_REACHY_MINI.md)
  - Tournevis, éventuellement câbles de secours (court/long entre moteurs).

---

## 2. Démonter et vérifier moteurs / câbles

- [ ] Démonter la **tête** (suivre le guide officiel Pollen / ton guide d’installation).
- [ ] **Câbles** :
  - [ ] Débrancher et rebrancher **tous** les câbles moteurs (bien enfoncer).
  - [ ] Priorité : **headboard → moteur 1**, puis **moteur 1 → 2** (câble court), **moteur 2 → 3** (long), **moteur 4 → 5** (long), **5 → 6** (court).
  - [ ] Si tu as des câbles de secours : tester à la place des câbles suspects (surtout entre slots 1–2 et 2–3 si Stewart 2 manquant, entre 4–5 si Stewart 4 manquant).
- [ ] **Moteurs** :
  - [ ] Vérifier que chaque moteur est bien **en place** dans son slot (1 à 6 Stewart + yaw_body + antennes).
  - [ ] Pas de vis desserrées, pas de connecteur qui tire.
- [ ] **Slots problématiques** (d’après tes diag : souvent ID 10 / 12 / 14) :
  - [ ] **ID 10 (Rotation du corps)** : connexion **base (Pi / power board) ↔ tête**.
  - [ ] **ID 12 (Stewart 2)** : câble **moteur 1 → 2** et connecteurs slot 1 et 2.
  - [ ] **ID 14 (Stewart 4)** : câble **moteur 4 → 5** et connecteurs slot 4 et 5.
- [ ] Remonter la tête proprement, sans forcer les câbles.

---

## 3. Après remontage – démarrage

- [ ] Rebrancher l’alimentation, allumer le robot.
- [ ] Attendre **1–2 minutes** (boot du Pi + daemon).
- [ ] Vérifier les **LEDs** des moteurs : pas de rouge persistant (après reflash elles s’éteignent).

---

## 4. Processus après remontage (dans l’ordre)

### 4.1 Diagnostic bus (sur le robot en SSH)

```bash
ssh pollen@<ROBOT_IP>
lsusb
```

- [ ] Le bus moteurs doit apparaître. Sinon → problème **tête ↔ Pi** (câble ou connecteur).

### 4.2 Reflash des moteurs (sur le robot)

```bash
# Toujours en SSH sur le robot
reachy-mini-reflash-motors
```

- Choisir **Wireless**.
- [ ] Tous les moteurs **10 à 18** doivent être détectés et reconfigurés. Si un ID manque → revérifier le câble de ce segment (voir § 2).

### 4.3 Redémarrer le daemon (si besoin)

```bash
sudo systemctl restart reachy-mini-daemon
```

- Attendre ~10 s.

### 4.4 Dashboard

- Sur ton Mac : ouvrir **http://\<ROBOT_IP\>:8000**.
- [ ] Vérifier qu’il n’y a plus « Missing motors » (ou noter lesquels manquent encore).
- Si le robot **se tord / clignote** au réveil : passer au § 4.5.

### 4.5 Script de déblocage (surcharge au réveil)

Depuis ton **Mac** (même WiFi que le robot) :

```bash
cd /Volumes/T7/bbia-reachy-sim
python3 examples/reachy_mini/fix_motors_1_2_overload_ssh.py --robot-ip <ROBOT_IP>
```

- [ ] Si timeout Zenoh : se connecter en SSH sur le robot, puis sur le robot lancer le script Python de déblocage avec `REACHY_ZENOH_CONNECT="tcp/127.0.0.1:7447"` (voir [DESCRIPTION_PROBLEME_ET_COMPOSANTS_REACHY_MINI.md](DESCRIPTION_PROBLEME_ET_COMPOSANTS_REACHY_MINI.md)).

### 4.6 Testbench (optionnel, pour scan détaillé)

- **Sur le robot** (SSH) : arrêter le daemon pour libérer le port série :  
  `sudo systemctl stop reachy-mini-daemon`  
  Puis lancer le Testbench (ex. `install_and_run_testbench_on_robot.sh`).
- **Sur le Mac** : ouvrir **http://\<ROBOT_IP\>:8042**.
- [ ] Scanner les moteurs (ID 10–18), vérifier positions / config. Puis redémarrer le daemon : `sudo systemctl start reachy-mini-daemon`.

### 4.7 Si un moteur reste manquant ou en erreur

- IDs consécutifs manquants → souvent **une** mauvaise connexion **en amont** du premier manquant (revérifier câbles § 2).
- Moteur en **config usine** (ID 1 @ 57 600) : sur le robot `reachy-mini-reflash-motors` ou `scan_motors_baudrate.py --serialport /dev/ttyAMA3 --auto-fix` (voir [MOTEURS_DIAGNOSTIC_ET_RECONFIG.md](../../examples/reachy_mini/MOTEURS_DIAGNOSTIC_ET_RECONFIG.md)).
- Clignotement / tête tordue persistants après reflash + script → **contacter Pollen** (calibration / offsets).

---

## 5. Résumé rapide

1. **Avant** : éteindre, noter l’état, avoir les guides.
2. **Démonter** : vérifier et rebrancher **tous** les câbles (surtout headboard → 1, 1→2, 4→5) ; vérifier slots moteurs.
3. **Remonter** : allumer, attendre 1–2 min.
4. **Après** : `lsusb` → `reachy-mini-reflash-motors` → redémarrer daemon si besoin → dashboard http://\<ROBOT_IP\>:8000 → si surcharge au réveil : script déblocage `fix_motors_1_2_overload_ssh.py --robot-ip <ROBOT_IP>`.

Pour plus de détail : [DESCRIPTION_PROBLEME_ET_COMPOSANTS_REACHY_MINI.md](DESCRIPTION_PROBLEME_ET_COMPOSANTS_REACHY_MINI.md), [GUIDE_INSTALLATION_MOTEURS_ETAPE_PAR_ETAPE.md](GUIDE_INSTALLATION_MOTEURS_ETAPE_PAR_ETAPE.md).
