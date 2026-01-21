# ⚠️ Problème Moteurs - Batch QC Décembre 2025

**Date** : Décembre 2025  
**Statut** : 🔴 **PROBLÈME CRITIQUE IDENTIFIÉ PAR POLLEN**  
**SDK Fix** : v1.2.4 (reflash automatique)

---

## 🚨 **PROBLÈME IDENTIFIÉ PAR POLLEN ROBOTICS**

### **Cause racine découverte** (Décembre 2025)

1. **Moteur 4 (QC 2544)** : **Non flashé correctement à l'usine**
   - Le SDK v1.2.4 va **automatiquement reflasher** les moteurs lors de la connexion et du démarrage
   - **Malheureusement**, si le moteur est déjà endommagé, il faudra le remplacer quand même

2. **Moteurs 1 et 2** : Si les deux ont un problème, c'est souvent dû à une **inversion entre leurs slots**
   - Vérifier que le moteur 1 est bien sur le slot n°1
   - Vérifier que le moteur 2 est bien sur le slot n°2

3. **Problème mécanique** : Le problème identifié semble **"brûler" le moteur lors de la première utilisation**
   - Le moteur était **OK pendant l'assemblage**
   - Le moteur devient **raide après la première utilisation du robot**
   - Si le moteur est raide **même débranché**, c'est un problème matériel irréversible

---

## 📊 **BATCHES QC DÉFECTUEUX**

### **Batch QC 2544** 🔴 **DÉFECTUEUX**
- **Problème** : Non flashé correctement à l'usine
- **Moteurs affectés** : Moteur 2 (stewart_2) et Moteur 4 (stewart_4)
- **Symptômes** :
  - Moteur raide mécaniquement (même débranché)
  - Clignotement rouge
  - Ne bouge pas
- **Solution** : Remplacement nécessaire (formulaire rempli)

### **Batch QC 2543** ⚠️ **PROBLÉMATIQUE**
- **Moteurs affectés** : Moteur 1 (stewart_1)
- **Symptômes** : Moteur raide mécaniquement (même débranché)
- **Solution** : Remplacement nécessaire (formulaire rempli)

### **Batch QC 2542** ⚠️ **À SURVEILLER**
- **Moteurs** : Moteur 5 (stewart_5) et Moteur 6 (stewart_6)
- **Statut actuel** : ✅ Smooth pour l'instant
- **Recommandation** : Surveiller, remplacer préventivement si possible pour éviter de démonter la tête plusieurs fois

---

## 🔍 **DIAGNOSTIC**

### **Comment identifier un moteur défectueux**

1. **Test mécanique (moteur débranché)** :
   - Débrancher complètement le moteur
   - Essayer de le tourner à la main
   - Si le moteur est **raide** même débranché → **Problème matériel irréversible**

2. **Test visuel** :
   - Clignotement rouge au démarrage
   - Moteur ne bouge pas lors des commandes

3. **Vérification QC batch** :
   - Regarder le sticker QC sur le moteur
   - Batch QC 2544 → **Défectueux confirmé**
   - Batch QC 2543 → **Problématique**

### **Vérification des slots (moteurs 1 et 2)**

Si les **moteurs 1 et 2** ont tous les deux un problème :

1. Vérifier que le moteur 1 est bien sur le **slot n°1**
2. Vérifier que le moteur 2 est bien sur le **slot n°2**
3. Vérifier le câblage :
   - Motor 1 → short → Motor 2 → long → Motor 3
   - Motor 4 → long → Motor 5 → short → Motor 6

---

## ✅ **SOLUTIONS**

### **1. Mise à jour SDK v1.2.4** (Automatique)

Le SDK v1.2.4 va **automatiquement reflasher** les moteurs lors de :
- La connexion au robot
- Le démarrage du robot
- L'ouverture du dashboard

**Action requise** : Mettre à jour le SDK via le dashboard (Settings → Update SDK → v1.2.4)

### **2. Remplacement des moteurs défectueux**

**Formulaires remplis** (Décembre 2025) :
- ✅ Moteur 1 (QC 2543 - raide)
- ✅ Moteur 2 (QC 2544 - raide + clignotement rouge)
- ✅ Moteur 4 (QC 2544 - raide)

**⚠️ PROBLÈME** : Aucun email de confirmation reçu après les formulaires

**Email envoyé le 6 janvier 2026** :
- Contact direct avec Pollen Robotics
- Demande de remplacement pour :
  - Moteur 1 (stewart_1) - nouveau batch sain
  - Moteur 2 (stewart_2) - nouveau batch sain
  - Moteur 3 (stewart_3) - remplacement préventif (pour éviter de redémonter la tête)
  - Moteur 4 (stewart_4) - nouveau batch sain
  - Moteur 5 (stewart_5) - remplacement préventif (optionnel, QC 2542)
  - Moteur 6 (stewart_6) - remplacement préventif (optionnel, QC 2542)
- Informations de commande incluses : REACHYMINI-2213 (14 juillet 2025)

**Email envoyé avec succès le 6 janvier 2026, 15h08** :
- ✅ Email envoyé à **sales@pollen-robotics.com**
- ✅ Demande complète pour les 6 moteurs (1, 2, 3, 4, 5, 6)
- ✅ Documentation fournie (QC batches, SDK v1.2.4, reflash effectué)
- ⏳ **En attente de réponse** de Pollen Robotics

**Autres utilisateurs en attente aussi** :
- **Hala** attend toujours sa résolution depuis le 2-3 janvier (problème motor 4)
- **Plusieurs utilisateurs** sur Discord ont des problèmes similaires (batches QC 2542, 2543, 2544)

**✅ MOTEURS REÇUS** : **17 Janvier 2026**
- ✅ **3 moteurs reçus** (moteurs 1, 2, 4)
- ✅ **Numéro QC vérifié** : **QC 2549** (21 janvier 2026)
  - ✅ **Excellent** : Batch QC 2549 n'est PAS dans les batches problématiques (2542/2543/2544)
  - ✅ **Sécurisé** : Moteurs d'un batch sain, pas de risque connu
- 📦 **Statut** : Moteurs reçus et vérifiés, en attente d'installation et de tests
- ⏳ **Prochaines étapes** : Installation des moteurs, tests unitaires, tests globaux

### **3. Remplacement préventif (recommandé)**

Si vous avez des moteurs QC 2542 (moteurs 5 et 6) :
- **Recommandation** : Les remplacer préventivement pour éviter de démonter la tête plusieurs fois
- **Raison** : Si les moteurs 5 et 6 tombent en panne plus tard, il faudra démonter la tête à nouveau

---

## 📝 **CHECKLIST DE VÉRIFICATION**

### **Avant première utilisation**

- [ ] Vérifier les numéros QC de tous les moteurs
- [ ] Vérifier que le SDK est à jour (v1.2.4 minimum)
- [ ] Vérifier le câblage (slots corrects)
- [ ] Tester chaque moteur manuellement (débranché) pour vérifier qu'il n'est pas raide

### **Après première utilisation**

- [ ] Vérifier qu'aucun moteur ne clignote en rouge
- [ ] Tester les mouvements de la tête
- [ ] Si un moteur clignote ou est raide → **Arrêter immédiatement** et remplir le formulaire de remplacement

### **Si problème détecté**

- [ ] Remplir le formulaire de remplacement Dynamixel
- [ ] Noter le numéro QC du moteur
- [ ] Prendre des photos (moteur, QC sticker, tête)
- [ ] Décrire tous les symptômes (raideur, clignotement, etc.)

---

## 🔗 **RESSOURCES**

- **Formulaire de remplacement** : Google Form "Dynamixel motor replacement request" (https://forms.gle/JdhMzadeCnbynw7Q6)
  - ⚠️ **Note** : Aucune confirmation automatique reçue après soumission
- **SDK v1.2.4** : Mise à jour automatique via dashboard
- **Support Pollen** : 
  - Contact via Discord #support
  - Email direct (recommandé si pas de réponse aux formulaires)
- **Documentation BBIA** : `docs/hardware/REACHY_MINI_SDK_v1.2.4.md`

## 📧 **INFORMATIONS DE COMMANDE (pour référence)**

- **Facture** : REACHYMINI-2213
- **Date** : 14 juillet 2025
- **Email** : siwekathalia@gmail.com
- **Nom** : Siwek
- **Adresse** : Rue Dieudonné Randaxhe 1, 4602 Cheratte, Belgique
- **Téléphone** : +32 472 87 56 94
- **Modèle** : Reachy Mini (with Onboard Compute and battery)

---

## 📅 **HISTORIQUE**

- **Décembre 2025** : Problème identifié par Pollen Robotics
- **22 Décembre 2025** : Reflash effectué (tous les moteurs détectés)
- **Décembre 2025** : Formulaires de remplacement remplis (moteurs 1, 2, 4) - **Aucune confirmation reçue**
- **Décembre 2025** : SDK v1.2.4 annoncé avec reflash automatique
- **6 Janvier 2026, 15h08** : Email envoyé avec succès à sales@pollen-robotics.com
  - Demande de remplacement pour moteurs 1, 2, 3, 4 (et 5, 6 en préventif)
  - Informations de commande : REACHYMINI-2213 (14 juillet 2025)
  - Email professionnel et complet avec tous les détails
- **17 Janvier 2026** : ✅ **3 moteurs reçus** (moteurs 1, 2, 4)
  - Moteurs de remplacement livrés
  - En attente d'installation et de tests
- **21 Janvier 2026** : ✅ **Numéro QC vérifié** : **QC 2549**
  - ✅ Batch sain (pas dans les batches problématiques 2542/2543/2544)
  - ✅ Moteurs sécurisés pour installation

---

## ⚠️ **IMPORTANT**

- **Si un moteur est raide même débranché** → **Problème matériel irréversible** → Remplacement nécessaire
- **Le reflash automatique du SDK v1.2.4** ne peut pas réparer un moteur déjà endommagé
- **Le reflash automatique** protège les moteurs qui ne sont pas encore endommagés
- **Si vous avez des moteurs QC 2542** → Surveiller ou remplacer préventivement

