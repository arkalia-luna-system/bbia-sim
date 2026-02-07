# Guide de diagnostic et de dépannage des moteurs (officiel)

**Source** : Documentation officielle Reachy Mini (Hugging Face / Pollen Robotics).

Ce document décrit comment diagnostiquer et résoudre les problèmes courants liés aux moteurs du robot Reachy Mini.

---

## Application Reachy Mini Testbench

Pour faciliter le diagnostic, Pollen fournit l’**application Reachy Mini Testbench**. Elle permet de :

- Tester chaque moteur individuellement
- Vérifier son état
- Identifier les problèmes potentiels

**À faire** : l’installer sur le robot avant de l’utiliser.

**Version Lite** : exécuter le daemon sans démarrage automatique dans un terminal et accéder au tableau de bord comme indiqué dans la doc officielle.

---

## Quand faire un diagnostic ?

Procédure typique si vous avez **l’un de ces symptômes** :

- Les moteurs **clignotent en rouge** et ne répondent pas / ne bougent pas
- **Aucun message d’erreur**, mais les moteurs ne répondent pas aux commandes
- Moteurs avec erreurs du type **« Erreur de surcharge »**
- **Moteurs manquants** : *« Aucun moteur trouvé sur le port »*, *« Moteur manquant »*, etc.

---

## Étapes de diagnostic

1. **Mettre le robot sous tension**
2. **Ouvrir l’application Reachy Mini Testbench**
3. **Lancer un diagnostic moteur** : cliquer sur **« Scanner les moteurs »**

---

## Résultats possibles

### 1. Tous les moteurs sont détectés

→ Tous les moteurs sont physiquement connectés et répondent.

- Si le problème persiste : cliquer sur **« Vérifier tous les moteurs »** pour s’assurer que la configuration (ID, baudrate) correspond aux valeurs attendues.
- En cas d’anomalie, le daemon **reprogrammera les moteurs** au redémarrage.

**Vérifications supplémentaires si ça bloque encore :**

#### Inversion des moteurs

- **Symptôme** : *« Erreurs matérielles moteur : ['Erreur de surcharge'] »*, LED qui clignotent quelques secondes après le premier démarrage, **deux bras moteurs pointent vers le haut**.
- **Cause probable** : moteurs mal installés (ex. moteur 1 dans l’emplacement 2).
- **Action** : Vérifier l’orientation du bras (klaxon) du moteur : retirer le moteur, placer le bras vers le haut, vérifier que les **deux repères** sont alignés. Si non, retirer les deux vis du bras et le remettre en place en alignant les deux lignes.

#### Longueur du câble USB dans la tête

- Si le câble USB est **trop long**, pas assez de jeu en dessous → la tête ne peut pas bouger librement → contraintes excessives sur les moteurs, risque de dommage.
- **Action** : Laisser du **mou au câble USB** pour que la tête puisse bouger librement jusqu’à sa hauteur maximale.

#### Moteur défectueux (lot limité)

- Problème identifié sur un **lot de production limité** (lot de moteurs Dynamixel défectueux).
- Souvent : **moteur numéro 4** ou moteur avec **étiquette QC n°2544**.
- **Symptôme** : moteur clignote en rouge, **anormalement difficile à déplacer à la main quand le robot est éteint** (ex. vidéo sur la doc officielle), et vous êtes certain que le moteur est au bon emplacement.
- **Action** : Mettre à jour le logiciel du robot puis redémarrer (reprogrammation des moteurs). Si le problème persiste, contacter le support Pollen.

---

### 2. Certains moteurs ne sont pas détectés

#### Un seul moteur manquant

- Vérifier le **raccordement physique** (instructions de montage).
- Si le câblage est correct : le moteur n’a peut‑être **pas été correctement flashé** → voir section **« Problème de flashage »** ci‑dessous.

#### Plusieurs moteurs avec IDs successifs manquants

- Ex. moteurs **1-2-3**, ou **4-5-6**, ou **17-18** manquants.
- **Action** : Vérifier les **connexions physiques** entre la carte d’alimentation (pied) et les moteurs 3 et 4. Si le problème est sur 17-18, vérifier la connexion entre le moteur « R » et la carte dans la tête.

#### Deux moteurs manquants, connexions OK

- Possibilité : **deux moteurs avec le même identifiant** (conflit).
- Très improbable d’en avoir trois. → Voir section **« Problème de clignotement »** ci‑dessous.

---

## Problème de clignotement (ID / baudrate)

Si un ou plusieurs moteurs ne sont pas détectés, cela peut venir d’un **mauvais flash** : **identifiants** ou **débit en bauds** incorrects.

**À faire :**

1. Dans l’application Testbench : cliquer sur **« Analyser tous les débits en bauds »**.
2. Les moteurs doivent être détectés à **1 000 000 bauds**, avec des **ID entre 10 et 18**, sans doublon ni valeur manquante.
3. Si un moteur a un baudrate et/ou un ID incorrects → le **reprogrammer** (voir ci‑dessous).

---

## Procédure de reprogrammation d’un moteur (Testbench)

1. Dans la section **« Reprogrammation du moteur »** :
   - Choisir le **moteur** dont l’ID ou le baudrate est incorrect.
   - Choisir le **préréglage** du moteur (ex. moteur 10 pour la rotation de la base).
   - Cliquer sur **« Reprogrammer le moteur »**.

**Si plusieurs moteurs sont manquants** et que les connexions sont correctes : reprogrammer **tous les moteurs défectueux un par un**, en **débranchant les autres moteurs défectueux** à chaque fois pour éviter les conflits d’ID.

---

## Résumé

| Situation | Action |
|-----------|--------|
| Tous détectés mais problème persiste | « Vérifier tous les moteurs », puis redémarrer le daemon |
| Erreur de surcharge + 2 bras vers le haut | Vérifier orientation des bras (repères alignés) |
| Câble USB tête trop tendu | Laisser du mou |
| Un moteur clignote rouge, très dur à la main, bon emplacement | Mise à jour logiciel + redémarrage ; sinon support Pollen |
| Un moteur manquant | Vérifier câblage ; sinon reflash (ID + baudrate) |
| IDs successifs manquants (ex. 3-4-5) | Vérifier connexions carte alimentation ↔ moteurs 3 et 4 (ou 17-18) |
| Deux moteurs manquants, câblage OK | Probable conflit d’ID → reprogrammer un par un en débranchant les autres |

---

*Document basé sur le guide officiel Reachy Mini (diagnostic des moteurs), janvier 2026.*
