# Description du problème et composants Reachy Mini – pour commande de pièces

**Date** : 28 Janvier 2026  
**Robot** : Reachy Mini Wireless (Pollen Robotics)  
**Objectif** : Décrire le problème et lister les composants (moteurs, câbles, connecteurs) pour pouvoir commander des pièces de rechange (AliExpress, Robotis, etc.).

> **Note** : Dans ce doc, `<ROBOT_IP>` est un placeholder. Remplace par l’IP réelle de ton robot (ex. `192.168.x.x`) quand tu exécutes les commandes. Ne pas versionner ton IP sur GitHub.

---

## Récap complet – ce qui s’est passé et pourquoi ça va ou ça ne va pas

### Ce qui s’est passé (chrono)

1. **Au début** : Daemon qui ne démarre pas, message *"Missing motors: stewart_2, stewart_4"* (moteurs 12 et 14 non détectés sur le bus).
2. **Remplacement moteurs** : Moteurs neufs (QC 2549 / 2548) en slots 1, 2 et 4. Câble entre slot 1 et 2 vérifié / changé.
3. **Reflash** : Au début le reflash échouait au moteur 12. Après démontage/remontage complet, **`reachy-mini-reflash-motors`** a réussi : **tous les moteurs 10 à 18** sont détectés et reconfigurés (offsets, limites, etc.) ✅
4. **Après reflash** : Au wakeup (allumage / ouverture dashboard), le robot **essaie de se lever**, **se tord**, **tous les moteurs clignotent une ou deux fois**, puis **plus rien** (plus de mouvement). Tu n’as pas encore pu le faire bouger correctement à la main.
5. **Script de déblocage** : On a un script (`fix_motors_1_2_overload_ssh.py`) qui doit désactiver les moteurs 5 s, réactiver, puis envoyer un mouvement très lent vers neutre. **Problème** : quand tu le lances depuis le Mac, il copie le script sur le robot et l’exécute en SSH, mais **sur le robot le script ne trouve pas le daemon** (connexion Zenoh timeout). Donc le script ne peut pas piloter le robot.
6. **Réseau** : **`reachy-mini.local`** peut ne pas se résoudre (mDNS). Utiliser **l’IP du robot** (ex. `192.168.x.x`). Avec cette IP : ping OK, SSH OK, daemon actif. *(Ne pas versionner ton IP réelle : utiliser `<ROBOT_IP>` dans les exemples.)*

### Ce qui va ✅

| Élément | Statut |
|--------|--------|
| **Robot allumé, même WiFi que le Mac** | OK (ping \<ROBOT_IP\> répond) |
| **SSH sur le robot** | OK (`ssh pollen@<ROBOT_IP>`, mot de passe utilisateur robot) |
| **Daemon Reachy Mini** | OK (`systemctl status reachy-mini-daemon` → active (running)) |
| **Reflash moteurs** | OK – tous les moteurs 10–18 détectés et configurés |
| **Bus moteurs (câbles, connecteurs)** | OK – sinon le reflash n’aurait pas vu tous les moteurs |

### Ce qui ne va pas (ou pas encore vérifié) ❌ / ⚠️

| Problème | Cause probable | Où on en est |
|----------|-----------------|--------------|
| **Au wakeup : robot se tord, moteurs clignotent, puis plus rien** | La **pose par défaut** (neutre / « look ahead ») envoyée au réveil ne correspond pas à la géométrie réelle (offsets pas parfaits après remplacement moteurs) → les moteurs forcent → **surcharge** → clignotement puis arrêt. | Pas résolu. Il faudrait soit une **calibration/offsets** (outil Pollen), soit réussir à envoyer un mouvement plus doux avant (notre script). |
| **Script de déblocage ne peut pas piloter le robot** | Le script s’exécute **sur le robot** et se connecte au **daemon via Zenoh**. Le **client Zenoh** peut ne pas trouver le **serveur Zenoh** du daemon (timeout). | Utiliser `localhost_only=False` sur le robot. **À retester** : `fix_motors_1_2_overload_ssh.py --robot-ip <ROBOT_IP>`. |
| **Depuis le Mac : timeout si on lance le script direct** | Le SDK (Zenoh) depuis le Mac ne trouve pas le robot sur le réseau. | Contournement : **version SSH** du script (exécution sur le robot). |
| **`reachy-mini.local` ne marche pas** | mDNS (Bonjour) ne résout pas ce nom. | Utiliser **l’IP du robot** partout (SSH, script, dashboard). |
| **Testbench (port 8042) affiche « DÉMON OFF »** | Sur **Reachy Wireless** le démon écoute sur l’**IP WiFi**, pas sur 127.0.0.1. | Script `install_and_run_testbench_on_robot.sh` : patch `localhost_only` → `network`. Démarrer le démon avant le Testbench. Sinon dashboard http://\<ROBOT_IP\>:8000. |
| **Testbench : scan / positions / check_all en 503** | Le démon garde le port série en exclusivité. | **Soit** démon actif (dashboard 8000), **soit** `sudo systemctl stop reachy-mini-daemon` puis Testbench pour diagnostic moteurs. |
| **`curl localhost:8000` ne retourne rien (sur le robot)** | Sur Reachy Wireless le démon écoute sur l’**IP WiFi**. | Depuis le robot : `curl -sS http://<ROBOT_IP>:8000/api/daemon/status`. Depuis le Mac : http://\<ROBOT_IP\>:8000. |
| **Après wakeup sur le 8000 : le robot n’est plus détecté** | Wakeup peut mettre le démon dans un mauvais état. | **Redémarrer** : `sudo systemctl restart reachy-mini-daemon`, attendre ~10 s. |
| **Testbench lancé sur le Mac au lieu du robot** | L’app doit tourner **sur le robot** en SSH. | **Mac** : `ssh pollen@<ROBOT_IP>`. **Robot** : `bash install_and_run_testbench_on_robot.sh`. **Mac** : http://\<ROBOT_IP\>:8042. |

### En résumé

- **Côté matériel / bus / reflash** : tout est OK (moteurs vus, reflash réussi).
- **Côté logiciel / utilisation** : au réveil le robot envoie une pose qui provoque surcharge et clignotement ; on n’a pas encore réussi à faire tourner le script de déblocage jusqu’au bout parce que la **connexion Zenoh** (script sur le robot → daemon) timeout. Prochaine étape logique : **retester le script** avec la dernière version (`localhost_only=False` sur le robot). Si ça timeout encore, il restera soit d’utiliser le **dashboard en HTTP** (http://\<ROBOT_IP\>:8000) pour envoyer à la main des mouvements doux, soit une procédure **calibration/offsets** côté Pollen.

---

## Que faire pour corriger – ordre des étapes

**Tu ne sais pas par où commencer ? Suis ça dans l’ordre.**

| Étape | Action | Où |
|-------|--------|-----|
| **1** | **Câbles** : débrancher / rebrancher **tous** les câbles moteurs, bien les enfoncer. Surtout **headboard → moteur 1**. Tester avec les **2 câbles de secours** si dispo. | § 10.1 |
| **2** | **Diagnostic bus** : `ssh pollen@reachy-mini.local` puis `lsusb`. Si le bus moteurs n’apparaît pas → problème **tête PCB ↔ Pi**. | § 10.2 |
| **3** | **Reflash** : sur le robot, `reachy-mini-reflash-motors` (choisir Wireless). Redémarrer le daemon si besoin. | § 9.3 |
| **4** | **Moteurs 1 et 2 clignotent / tête tordue** : lancer **une fois** le script de déblocage. Depuis le Mac (timeout possible) → utiliser la version SSH : `python3 examples/reachy_mini/fix_motors_1_2_overload_ssh.py --robot-ip reachy-mini.local` (ou l’IP du robot). | § 9.5 |
| **5** | **Toujours des moteurs manquants** : IDs consécutifs manquants = souvent **une** mauvaise connexion en amont (Stewart, foot/power board). Revérifier les câbles. | § 10.3 |
| **6** | **Moteur en config usine** (ID=1 @ 57 600) : sur le robot `reachy-mini-reflash-motors`, ou `scan_motors_baudrate.py --serialport /dev/ttyAMA3 --auto-fix`. Diagnostiquer : `scan_motors.py` ou `scan_motors_baudrate.py` (voir [MOTEURS_DIAGNOSTIC_ET_RECONFIG.md](../../examples/reachy_mini/MOTEURS_DIAGNOSTIC_ET_RECONFIG.md)). | § 10.4 |
| **7** | **Clignotement persistant après tout ça** : calibration / offsets → **contacter Pollen** (ils ont l’outil ou la procédure). | § 9.5 |

En résumé : **d’abord les câbles** (souvent mal branchés après réassemblage), puis `lsusb` + reflash, puis script déblocage si tête tordue, puis Pollen si ça bloque encore.

---

## 1. Résumé du problème

- **Symptôme** : Le daemon Reachy Mini ne démarre pas. Message d’erreur : *"No motor found on port /dev/ttyAMA3"* / *"Missing motors: stewart_2, stewart_4"*.
- **Diagnostic effectué** : Scan du bus moteurs (1 000 000 baud) sur le robot :
  - **Motor ID 10** (yaw_body) : OK  
  - **Motor ID 11** (stewart_1, slot 1) : OK  
  - **Motor ID 12** (stewart_2, **slot 2**) : **ne répond pas**  
  - **Motor ID 13** (stewart_3, slot 3) : OK  
  - **Motor ID 14** (stewart_4, **slot 4**) : **ne répond pas**  
  - **Motor ID 15 à 18** : OK  
- **Conclusion** : Seuls les moteurs en **slot 2** et **slot 4** (Motor ID 12 et 14) ne sont pas détectés sur le bus série. Les autres moteurs répondent. Le bus et le câblage global fonctionnent (le moteur 13 répond, donc la chaîne après le slot 2 est OK).
- **Déjà fait** : Remplacement des moteurs en slots 1, 2 et 4 par des moteurs neufs (QC 2549 / 2548). Vérification et changement du câble entre slot 1 et slot 2. Reflash logiciel (`reachy-mini-reflash-motors`) : échoue au moteur 12. Les moteurs 12 et 14 ne répondent ni à 1 000 000 baud ni à 57 600 baud.
- **Hypothèses** :  
  - Connecteurs défectueux ou mal en contact sur les **moteurs slot 2 et slot 4**.  
  - **Câbles** entre moteurs (court/long) défectueux ou mauvais type.  
  - **Moteurs** slot 2 et/ou slot 4 défectueux (ne répondent plus sur le bus).

**À faire en premier (Pollen Community Discord)** : revérifier que **tous les câbles sont bien enfoncés**, surtout **headboard → moteur 1** ; utiliser les câbles de secours. Souvent une **connexion mal serrée** après réassemblage, pas des moteurs HS. Voir **section 10**.

---

## 2. Robot et type de moteurs

- **Robot** : Reachy Mini Wireless (Pollen Robotics), tête à plateforme Stewart (6 moteurs dans la tête + 1 yaw_body + 2 antennes).
- **Fiche technique officielle** : voir [FICHE_TECHNIQUE_REACHY_MINI.md](FICHE_TECHNIQUE_REACHY_MINI.md).
- **Moteurs (spec officielle)** :  
  - **Base (yaw_body)** : 1× XC330-M288-PG (XC330-M288-T engrenage plastique).  
  - **Plateforme Stewart (tête)** : 6× **Dynamixel XL330-M288-T**.  
  - **Antennes** : 2× Dynamixel XL330-M077-T.
- **Communication** : Bus série TTL, 1 000 000 baud, IDs 10–18.
- **Documentation moteur** :  
  - [e-manual ROBOTIS XL330](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/)  
  - [ROBOTIS Dynamixel XL330](https://www.robotis.us/dynamixel-xl330-m077-t/)

---

## 3. Câblage et connecteurs

- **Type de câble** : Câble TTL 3 fils (Data+, Data-, GND), type utilisé pour Dynamixel XL330.
- **Connecteurs** : **JST** (côté moteur). Les câbles officiels ROBOTIS sont souvent appelés "Robot Cable-X3P" ou "TTL 3P Cable" avec connecteurs **JST-JST**.
- **Longueurs** :  
  - Câble **court** entre moteurs (ex. slot 1 ↔ slot 2).  
  - Câble **long** entre moteurs (ex. slot 2 ↔ slot 3, slot 4 ↔ slot 5).  
  Les références officielles mentionnent par exemple 180 mm pour un câble standard XL330.
- **Chaîne physique** :  
  - Moteur 1 (slot 1) → câble court → Moteur 2 (slot 2) → câble long → Moteur 3 (slot 3)  
  - Moteur 4 (slot 4) → câble long → Moteur 5 (slot 5) → câble court → Moteur 6 (slot 6)

**Moteurs concernés par le problème** :  
- **Slot 2** (Motor ID 12, stewart_2)  
- **Slot 4** (Motor ID 14, stewart_4)

---

## 4. Ce dont j’ai besoin pour commander moi-même

Je cherche à identifier et commander les bons composants (sur AliExpress, Robotis, ou autre) pour :

1. **Câbles de remplacement** pour Dynamixel XL330 / Reachy Mini :
   - Référence exacte ou type : "Dynamixel XL330 TTL 3P cable", "JST 3-pin cable XL330", "Robot Cable-X3P", etc.
   - Longueurs : court (environ 50–100 mm ?) et long (environ 150–200 mm ?).
   - Pas sûr du pas de connecteur JST (1.0 mm, 1.25 mm, 2.0 mm ?).

2. **Connecteurs JST** (si achetés séparément) :
   - Type exact (modèle JST, pas, nombre de broches) utilisés sur le XL330 et sur les câbles Reachy Mini.

3. **Moteurs de rechange** (si les moteurs slot 2 et 4 sont HS) :
   - Référence exacte : XL330-M077-T ou XL330-M288-T (ou autre variante utilisée par Pollen pour Reachy Mini).
   - Ou équivalent compatible (même protocole TTL, même connecteur).

4. **Toute pièce** susceptible de faire défaut sur la liaison électrique entre le Raspberry Pi et les moteurs 12 et 14 (ex. carte d’interface, adaptateur, etc.), si applicable.

---

## 5. Correspondance Motor ID ↔ Slot physique (pour dépannage)

### 5.1 Layout après démontage/remontage (1er février 2026)

| Slot | Réf. moteur | Marqué | Note |
|------|-------------|--------|------|
| 1 | 2549 | « 1 » | New |
| 2 | 2548 | « 2 » | |
| 3 | 2543 | « 3 » | |
| 4 | 2549 | « 4 » | Ancien du slot 1 (celui qui était détecté) |
| 5 | 2542 | | |
| 6 | 2542 | | |

**Important** : le moteur en slot 4 est celui qui était auparavant en slot 1 et qui répondait (il est encore configuré en **ID 11**). Le slot 4 doit avoir **ID 14**. Si le dashboard affiche encore des moteurs manquants ou un comportement bizarre, lancer `reachy-mini-reflash-motors` sur le robot pour reconfigurer les IDs (slot 4 → ID 14, etc.).

---

### 5.2 Table Motor ID ↔ Slot

| Motor ID | Nom logique   | Slot physique | Rôle              |
|----------|---------------|---------------|-------------------|
| 10       | yaw_body      | corps         | Rotation corps    |
| 11       | stewart_1     | Slot 1        | Tête Stewart      |
| 12       | stewart_2     | **Slot 2**    | Tête Stewart (manquant) |
| 13       | stewart_3     | Slot 3        | Tête Stewart      |
| 14       | stewart_4     | **Slot 4**    | Tête Stewart (manquant) |
| 15       | stewart_5     | Slot 5        | Tête Stewart      |
| 16       | stewart_6     | Slot 6        | Tête Stewart      |
| 17       | left_antenna  | antenne G     | Antenne gauche   |
| 18       | right_antenna | antenne D    | Antenne droite   |

---

## 6. Références utiles

- **Fiche technique locale** : [FICHE_TECHNIQUE_REACHY_MINI.md](FICHE_TECHNIQUE_REACHY_MINI.md) (spec officielle Reachy Mini).
- **Pollen Robotics** : https://www.pollen-robotics.com/reachy-mini/  
- **Documentation Reachy Mini (Hugging Face)** : guide installation, matériel, diagnostic moteurs, SSH (`pollen` / `root`).
- **Diagnostic moteurs (local)** : [DIAGNOSTIC_MOTEURS_OFFICIEL.md](DIAGNOSTIC_MOTEURS_OFFICIEL.md) – Testbench, reflash, inversion bras, lot QC 2544.
- **Documentation Pollen (hardware)** : https://docs.pollen-robotics.com/hardware-guide/specifications/motors-actuators  
- **ROBOTIS Dynamixel XL330** : https://www.robotis.us/dynamixel-xl330-m077-t/  
- **e-manual XL330** : https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/  
- **GitHub Reachy Mini** : https://github.com/pollen-robotics/reachy_mini  

---

## 7. Question pour une IA ou un vendeur

**À partir de cette description, pouvez-vous me donner :**

1. La référence exacte (ou des liens) des **câbles** compatibles Dynamixel XL330 / Reachy Mini (JST 3P, court et long), trouvables sur AliExpress ou ailleurs.  
2. Le **type exact de connecteur JST** (modèle, pas en mm) utilisé sur le XL330 et ses câbles.  
3. La **référence exacte du moteur** (XL330-M077-T, XL330-M288-T, ou autre) utilisée sur le Reachy Mini, si connue.  
4. Toute autre **pièce (connecteur, câble, interface)** susceptible d’expliquer que seuls les Motor ID 12 et 14 ne répondent pas sur le bus.

Merci.

---

## 8. Réponse reçue : références exactes et procédure (28 jan 2026)

*Section ajoutée après réponse d’une IA / vendeur : diagnostic détaillé, références de commande et procédure avant installation.*

### 8.1 Pourquoi ce sont très probablement les moteurs (et pas les câbles)

Dans une chaîne Dynamixel (daisy chain) : `Câble A → Moteur 12 → Câble B → Moteur 13`.

- Si le **Câble A** était coupé : 12 **et** 13 seraient muets.
- Si le **Câble B** était coupé : 12 répondrait, mais pas 13.
- Ici le **Moteur 13 répond** et le **Moteur 12 est muet** → le signal traverse bien le moteur 12 (pass-through OK), mais la **partie électronique du moteur 12** ne répond pas.

**Conclusion** : Très probablement **carte électronique interne du moteur 12 (et 14) endommagée** (fusible logique ou microcontrôleur). Les câbles sont peut-être bons, mais en commander en secours reste conseillé.

---

### 8.2 Références exactes pour commande

#### A. Moteurs (pièce critique)

- **Référence à commander** : **Dynamixel XL330-M288-T** (fort couple, ratio 288:1).
- Pollen utilise cette version pour la plateforme Stewart (bras de la tête) à cause du couple nécessaire.
- **Ne pas prendre** : XL330-**M077**-T (version « roue », moins de couple, utilisée uniquement pour les antennes).
- **Quantité** : 2 (slots 2 et 4), ou 3 avec un de rechange.
- **Mots-clés** : *"Dynamixel XL330-M288-T"* (AliExpress, Google, Robotis).

#### B. Câbles (Robot Cable-X3P)

- **Type** : **JST-PH**, pas **2.0 mm** (pas JST-SH 1.0 mm ni JST-XH 2.54 mm).
- **Court** : 100 mm (10 cm), JST-PH 2.0 mm 3 broches → 2 pièces (souvent fourni avec le moteur Robotis).
- **Long** : 180 mm ou 200 mm (standard « Robot Cable-X3P 180 mm ») → 2 pièces.
- **Mots-clés** : *"JST PH 2.0 3 pin cable 20cm"*, *"Dynamixel XL330 cable"*.

#### C. Connecteurs (réparation câbles)

- **Modèle** : **JST PHR-3** (femelle 3 broches), pas **2.00 mm**.

---

### 8.3 Tableau récapitulatif pour commande

| Composant | Référence | Quantité conseillée | Note |
|-----------|-----------|----------------------|------|
| **Moteur (Slot 2 & 4)** | **Dynamixel XL330-M288-T** | 2 (ou 3 en spare) | Ne pas prendre M077-T. |
| **Câble court** | JST-PH 2.0 mm 3 broches, 100 mm | 2 | Vérifier ordre des fils (GND/VDD/DATA). |
| **Câble long** | JST-PH 2.0 mm 3 broches, 180–200 mm (Robot Cable-X3P 180 mm) | 2 | Standard Robotis. |

---

### 8.4 Avant d’installer les nouveaux moteurs (important)

**Piège classique** : un moteur neuf sort d’usine en **ID 1** et **57 600 baud**. Le Reachy Mini utilise **1 000 000 baud** et les IDs **12** et **14**. Si on branche le moteur neuf directement dans le robot : (1) il ne sera pas vu (mauvaise vitesse), (2) risque de conflit d’ID.

**Procédure recommandée** :

1. Brancher le **nouveau moteur seul** (adaptateur USB type U2D2, ou via le Pi avec un seul moteur sur le bus).
2. Avec **Dynamixel Wizard 2.0** (gratuit) ou le script Pollen :
   - Passer le **Baud Rate** à **1 000 000** (registre souvent « 3 »).
   - Mettre l’**ID** à **12** (pour slot 2) ou **14** (pour slot 4).
3. Une fois configuré, l’installer dans le robot et relancer `reachy-mini-reflash-motors` si besoin.

**Résumé recherche** : *"Dynamixel XL330-M288-T"* ; *"JST PH 2.0 3 pin cable 20cm"*.

---

### 8.5 J’ai déjà mis les moteurs dans le robot sans les configurer – que faire ?

**Vous n’avez pas cassé les moteurs.** Ils sont simplement encore en réglage usine (ID 1, 57 600 baud), donc le robot ne les voit pas (il parle en 1 000 000 baud et cherche les ID 12 et 14).

**Procédure à suivre maintenant :**

1. **Démonter les moteurs** des slots 2 et 4 (sans les abîmer : dévisser, débrancher les câbles).
2. **En laisser un seul branché** sur le bus (par ex. sur le Raspberry Pi ou via un adaptateur USB U2D2) :
   - Brancher **un seul** des deux moteurs (slot 2 ou 4).
   - Ouvrir **Dynamixel Wizard 2.0**, scanner : il devrait trouver un moteur avec **ID 1**.
   - Mettre le **Baud Rate** à **1 000 000**.
   - Mettre l’**ID** à **12** (pour celui qui ira en slot 2) ou **14** (pour slot 4).
   - Sauvegarder, puis débrancher ce moteur.
3. **Faire pareil pour l’autre moteur** (le brancher seul, configurer ID 12 ou 14, baud 1 000 000, débrancher).
4. **Réinstaller les deux moteurs** dans le robot (slot 2 = ID 12, slot 4 = ID 14).
5. **Rallumer** et vérifier (dashboard, `reachy-mini-reflash-motors` si besoin).

En résumé : on fait *après* la configuration qu’on aurait dû faire *avant*. Les moteurs ne sont pas endommagés.

---

## 9. Rallumage et vérification (1er février 2026)

Après démontage/remontage (sans capot si besoin).

### 9.1 Rallumer le robot

1. **Brancher l’alimentation** (ou mettre la batterie / interrupteur ON).
2. **Attendre 30 s à 1 min** que le Pi démarre et que le Wi‑Fi soit prêt.
3. Le robot crée éventuellement un réseau « Reachy » ou se connecte à ton Wi‑Fi habituel.

### 9.2 Accéder au dashboard

1. **Même réseau** : ton Mac et le Reachy sur le **même Wi‑Fi**.
2. **Dans le navigateur** :
   - `http://reachy-mini.local:8000`  
   - ou `http://<ROBOT_IP>:8000` (remplace \<ROBOT_IP\> par l’IP réelle du robot).
3. **Dashboard** : tu devrais voir l’interface Reachy Mini (état, applications, etc.).

### 9.3 Vérifier que tout fonctionne

- **Pas d’erreur** type « Missing motors » ou « No motor found » → OK.
- **Si erreur** : noter le message (ex. quels moteurs manquants), puis en SSH sur le robot :
  ```bash
  ssh pollen@<ROBOT_IP>
  # mot de passe : root
  reachy-mini-reflash-motors
  ```
  Choisir **Wireless**, laisser le script reconfigurer les moteurs. Puis redémarrer le daemon si besoin :
  ```bash
  sudo systemctl start reachy-mini-daemon
  ```
- **Remettre le capot** quand tout est OK.

### 9.5 Moteurs 1 et 2 clignotent, tête tordue, 1 et 2 ne se lèvent pas – causes possibles

D’après la doc officielle et le dépôt :

1. **Bras (klaxons) mal orientés sur les slots 1 et 2**  
   Si **deux bras pointent vers le haut** et erreur de surcharge : les **deux repères** (moteur + bras) doivent être **alignés** quand le bras est « vers le haut ». Sinon, dévisser les **2 vis** du bras, remettre le bras en alignant les marques, revisser. Voir [DIAGNOSTIC_MOTEURS_OFFICIEL.md](DIAGNOSTIC_MOTEURS_OFFICIEL.md).

2. **Câble dans la tête trop tendu**  
   Pas assez de mou → la tête ne peut pas monter librement → les moteurs forcent → surcharge, clignotement. Laisser **du mou** au câble.

3. **Butée / blocage mécanique**  
   Un bras ou la tête qui bute contre une pièce ou le capot → surcharge. Vérifier qu’aucun câble ni capot ne bloque.

4. **Script de déblocage moteurs 1 et 2 (à lancer en premier)**  
   - **Depuis le Mac (WiFi)** : `python3 examples/reachy_mini/fix_motors_1_2_overload.py`  
     À lancer **une fois** après avoir allumé le robot (daemon déjà démarré, même réseau WiFi).  
     Il désactive les moteurs 5 s (détente, réinit erreur surcharge), réactive avec compensation gravité si dispo, puis envoie **un seul** mouvement très lent (6 s) vers la position neutre. Pas d’angle fort, pas de mouvement brusque.  
   - **Autre option (sûre)** : `python3 examples/reachy_mini/force_head_straight.py`  
     Désactive 5 s, puis petits mouvements ±5°.  
   - **À éviter** : `fix_head_tilted.py` (corrections très fortes, peut aggraver la surcharge).

**Si timeout « Timeout while waiting for connection with the server »** : utiliser la version SSH (script exécuté sur le robot) : `python3 examples/reachy_mini/fix_motors_1_2_overload_ssh.py --robot-ip <IP>` (ex. `--robot-ip <ROBOT_IP>` ou `reachy-mini.local`).

**Ordre conseillé** : 1) Lancer **une fois** `fix_motors_1_2_overload.py` ou en cas de timeout `fix_motors_1_2_overload_ssh.py --robot-ip <IP>`. 2) Rouvrir le dashboard et tester. **Si les moteurs 1 et 2 clignotent encore** : cause très probablement **calibration / offsets côté Pollen** (ils ont l'outil ou la procédure). Voir **§ Où trouver la calibration Pollen** ci-dessous.

### Où trouver la calibration Pollen (tête tordue / offsets moteurs)

- **Discord Pollen Community** (le plus réactif) : [discord.gg/pollen-robotics](https://discord.gg/pollen-robotics) → canal **#support** ou canal Reachy Mini. Demander : *« Calibration / offsets pour Reachy Mini – tête tordue, moteurs 1 et 2 en surcharge après reflash, avez-vous une procédure ou un outil ? »*. Caroline et l’équipe Pollen répondent souvent rapidement.
- **GitHub** : [github.com/pollen-robotics/reachy_mini](https://github.com/pollen-robotics/reachy_mini) → onglet **Issues** (chercher « calibration », « offset », « head ») ou ouvrir une issue. La doc **docs/troubleshooting.md** peut contenir des infos.
- **Documentation** : [docs.pollen-robotics.com](https://docs.pollen-robotics.com), [huggingface.co/docs/reachy_mini](https://huggingface.co/docs/reachy_mini).
- **Email** : **sales@pollen-robotics.com** (pour demande formelle ou si tu as déjà un contact).

---

### 9.4 Limite du script officiel `reachy-mini-reflash-motors`

**Le script ne change pas l’ID des moteurs.** Il parcourt les IDs attendus (10, 11, 12, …, 18), **cherche un moteur qui a déjà cet ID** sur le bus, puis met à jour ses paramètres (offset, limites, etc.). Il ne réassigne pas un ID en fonction de la position physique dans la chaîne.

- Si un moteur en **slot 4** a encore l’**ID 11** (parce qu’il venait du slot 1), le script ne le « passera » pas en ID 14. Il cherche un moteur ID 14, ne le trouve pas (ou trouve un autre), et ne corrige pas le décalage.
- Pour que les mouvements soient corrects, il faut que **chaque position physique** ait le **bon ID** : slot 1 → 11, slot 2 → 12, slot 3 → 13, slot 4 → 14, etc.
- **Si tu as déplacé des moteurs** (ex. moteur ID 11 mis en slot 4), il faut soit :
  1. **Remettre les moteurs aux bons emplacements** (celui qui a l’ID 11 en slot 1, celui qui a l’ID 14 en slot 4), soit  
  2. **Changer l’ID du moteur** actuellement en slot 4 (11 → 14) et celui en slot 1 (ex. 1 → 11) avec un outil qui le permet (Dynamixel Wizard, ou script `fix_motor_config_december_bug.py` **un moteur à la fois**, en débranchant les autres).

---

## 10. Solutions Pollen Community (Discord)

Référence : [Pollen Community Discord](https://discord.com/channels/519098054377340948/1462500170850766900/1467248540949676195) – plusieurs devs (mini-group-0-2) ont eu les mêmes problèmes ; **souvent c’était une connexion mal serrée après réassemblage, pas des moteurs défectueux**.

### 10.1 Problème de connexion câble (le plus courant)

**Tom Mulder et Caroline (Pollen Team)** : *"The problem is most likely in the connection from the headboard to the first motor, considering that all of the motors appear as missing"*.

**Actions :**
- Revérifier que **tous** les câbles moteurs sont **complètement enfoncés** (fully seated).
- Vérifier surtout la connexion **headboard → moteur 1**.
- Utiliser les **2 câbles de secours** fournis avec le robot.

### 10.2 Diagnostic avec `lsusb`

**Caroline (Pollen)** recommande :
- SSH : `ssh pollen@reachy-mini.local`
- Lancer : `lsusb` pour voir si le bus moteurs est détecté.
- **Si le bus est absent** = problème de connexion **tête PCB ↔ Raspberry Pi**.

### 10.3 IDs manquants consécutifs = une mauvaise connexion en amont

**Hunar Jain** : *"Consecutive missing IDs usually mean one bad connection upstream in the chain, not multiple separate failures"*.

- **Une seule** connexion lâche peut faire disparaître **plusieurs** moteurs.
- Chercher d’abord la **Stewart platform** et la connexion **foot/power board**.

### 10.4 Moteurs en config usine (mauvais paramètres)

**squirrel** a résolu : moteur encore en config factory (**ID=1 @ 57 600 baud**) au lieu de (**ID=13 @ 1 000 000 baud**).

**Dans bbia-reachy-sim – fichiers à utiliser :**

| But | Fichier / commande |
|-----|---------------------|
| **Diagnostiquer** (équivalent Pollen `scan_motors.py`) | `python3 examples/reachy_mini/scan_motors_baudrate.py` ou `python3 examples/reachy_mini/scan_motors.py` – à lancer **sur le robot** en SSH avec `--serialport /dev/ttyAMA3` (daemon arrêté). |
| **Reconfigurer** (équivalent `reachy_mini.tools.setup_motor`) | Sur le robot : **`reachy-mini-reflash-motors`** (recommandé), ou **`scan_motors_baudrate.py --serialport /dev/ttyAMA3 --auto-fix`** (utilise setup_motor si le SDK est installé). |
| **Un moteur précis** | Sur le robot : `python3 .../fix_motor_config_december_bug.py --serialport /dev/ttyAMA3 --motor-id <ID>`. |

**Guide détaillé** : [examples/reachy_mini/MOTEURS_DIAGNOSTIC_ET_RECONFIG.md](../../examples/reachy_mini/MOTEURS_DIAGNOSTIC_ET_RECONFIG.md) – ordre des étapes, commandes exactes, copie des scripts sur le robot.
