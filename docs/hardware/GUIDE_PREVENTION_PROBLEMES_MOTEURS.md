# 🛡️ Guide de Prévention - Problèmes Moteurs Reachy Mini

**Dernière mise à jour** : 17 Janvier 2026  
**Objectif** : Éviter que les problèmes de moteurs (batches QC 2542, 2543, 2544) ne se reproduisent

---

## 🎯 **OBJECTIF**

Ce guide fournit une **checklist complète** et des **procédures de prévention** pour garantir que les moteurs restent fiables et fonctionnels après remplacement.

---

## ✅ **CHECKLIST AVANT INSTALLATION DES NOUVEAUX MOTEURS**

### **1. Vérification des nouveaux moteurs (AVANT installation)**

- [ ] **Vérifier les numéros QC** de chaque nouveau moteur
  - Noter les numéros de série/QC
  - Vérifier qu'ils ne sont PAS des batches QC 2542, 2543, 2544
  - Prendre des photos des stickers QC

- [ ] **Test mécanique (moteur débranché)**
  - Débrancher complètement chaque moteur
  - Essayer de le tourner à la main
  - ✅ Le moteur doit tourner **smooth** (pas raide)
  - ❌ Si raide → **NE PAS INSTALLER**, contacter Pollen immédiatement

- [ ] **Vérification visuelle**
  - Vérifier l'état physique (pas de dommages)
  - Vérifier les connecteurs (pas de pliage, pas de corrosion)
  - Vérifier les câbles fournis (si inclus)

---

## 🔧 **CHECKLIST PENDANT L'INSTALLATION**

### **1. Vérification du câblage**

- [ ] **Slots corrects**
  - Moteur 1 → Slot n°1
  - Moteur 2 → Slot n°2
  - Moteur 3 → Slot n°3
  - Moteur 4 → Slot n°4
  - Moteur 5 → Slot n°5
  - Moteur 6 → Slot n°6

- [ ] **Câblage correct**
  - Motor 1 → short → Motor 2 → long → Motor 3
  - Motor 4 → long → Motor 5 → short → Motor 6
  - Vérifier que tous les câbles sont bien branchés
  - Vérifier qu'il n'y a pas de torsions ni pincements

- [ ] **Pas de dommages aux câbles**
  - Vérifier l'isolation (pas de déchirures)
  - Vérifier les connecteurs (bien enfoncés)
  - Vérifier qu'il n'y a pas de courts-circuits visibles

### **2. Vérification mécanique**

- [ ] **Alignement mécanique**
  - Vérifier que les moteurs sont bien montés
  - Vérifier l'alignement selon le guide de montage
  - Vérifier qu'il n'y a pas de contraintes mécaniques

- [ ] **Pas de surcharge**
  - Vérifier que les moteurs ne sont pas en butée
  - Vérifier qu'il n'y a pas de résistance mécanique anormale

---

## 📋 **CHECKLIST APRÈS INSTALLATION**

### **1. Vérification logicielle**

- [ ] **SDK à jour**
  - Vérifier la version SDK : `pip show reachy-mini`
  - **Recommandé** : v1.2.11 (dernière version stable)
  - **Minimum** : v1.2.4 (reflash automatique)

- [ ] **Reflash automatique**
  - Le SDK v1.2.4+ va automatiquement reflasher les moteurs
  - Vérifier que le reflash s'est bien passé (logs)
  - Vérifier que les LEDs des moteurs sont éteintes après reflash

- [ ] **Limites logicielles**
  - Vérifier les limites de rotation/position dans les réglages
  - S'assurer que les limites sont correctes pour éviter les butées

### **2. Tests unitaires (chaque moteur individuellement)**

- [ ] **Test moteur 1 (stewart_1)**
  - Démarrer le robot
  - Vérifier que le moteur répond
  - Tester un mouvement simple
  - ✅ Pas de clignotement rouge
  - ✅ Mouvement fluide

- [ ] **Test moteur 2 (stewart_2)**
  - Même procédure que moteur 1
  - ✅ Pas de clignotement rouge
  - ✅ Mouvement fluide

- [ ] **Test moteur 4 (stewart_4)**
  - Même procédure que moteur 1
  - ✅ Pas de clignotement rouge
  - ✅ Mouvement fluide

- [ ] **Test tous les autres moteurs**
  - Moteur 3, 5, 6 (si toujours en place)
  - Vérifier qu'ils fonctionnent toujours correctement

### **3. Tests globaux**

- [ ] **Test synchronisé (tête complète)**
  - Tester des mouvements de tête complexes
  - Vérifier qu'il n'y a pas de bruit anormal
  - Vérifier la fluidité des mouvements
  - Vérifier qu'il n'y a pas de vibrations

- [ ] **Test de stress (durée)**
  - Laisser fonctionner les moteurs pendant 30-60 minutes
  - Surveiller la température (si possible)
  - Vérifier qu'il n'y a pas de surchauffe
  - Vérifier qu'il n'y a pas de perte de performance

- [ ] **Test de charge**
  - Tester des mouvements rapides
  - Tester des mouvements lents
  - Tester des mouvements répétitifs
  - Vérifier qu'il n'y a pas d'erreurs

---

## 🔍 **SURVEILLANCE CONTINUE**

### **1. Surveillance quotidienne**

- [ ] **Vérifier les LEDs des moteurs**
  - Au démarrage : Pas de clignotement rouge
  - Pendant fonctionnement : Pas de LED rouge
  - Si LED rouge → **Arrêter immédiatement** et diagnostiquer

- [ ] **Surveiller les logs**
  - Vérifier les logs du daemon
  - Chercher les erreurs "Overload Error"
  - Chercher les erreurs "Input Voltage Error"
  - Chercher les alertes matérielles

- [ ] **Surveiller les performances**
  - Vérifier que les mouvements restent fluides
  - Vérifier qu'il n'y a pas de bruit anormal
  - Vérifier qu'il n'y a pas de vibrations

### **2. Surveillance hebdomadaire**

- [ ] **Test de diagnostic complet**
  - Utiliser le script de scan des moteurs
  - Vérifier les baudrates et IDs
  - Vérifier qu'il n'y a pas de moteurs manquants

- [ ] **Vérification mécanique**
  - Vérifier que les moteurs ne sont pas devenus raides
  - Tester manuellement (débranché) si possible
  - Vérifier qu'il n'y a pas de contraintes mécaniques

### **3. Surveillance mensuelle**

- [ ] **Mise à jour SDK**
  - Vérifier les nouvelles releases
  - Lire les release notes
  - Mettre à jour si nécessaire (après tests)

- [ ] **Documentation**
  - Noter tout problème rencontré
  - Noter les numéros de série des moteurs
  - Noter les dates d'installation

---

## 🚨 **PROCÉDURE EN CAS DE PROBLÈME**

### **1. Problème détecté**

**Si un moteur montre des signes de problème** :

1. **Arrêter immédiatement** le robot
2. **Noter les symptômes** :
   - LED rouge ?
   - Moteur raide ?
   - Moteur ne bouge pas ?
   - Bruit anormal ?
   - Surchauffe ?

3. **Test mécanique** :
   - Débrancher le moteur
   - Tester manuellement
   - Si raide même débranché → **Problème matériel irréversible**

4. **Documenter** :
   - Prendre des photos
   - Noter le numéro QC
   - Noter tous les symptômes

5. **Contacter Pollen** :
   - Remplir le formulaire de remplacement
   - Envoyer un email avec toutes les informations
   - Mentionner le numéro de facture (REACHYMINI-XXXX)

### **2. Diagnostic**

**Utiliser les outils disponibles** :

- Script de scan des moteurs
- Page de diagnostic dans le dashboard
- Logs du daemon
- Test mécanique (débranché)

---

## 📝 **DOCUMENTATION À CONSERVER**

### **Informations importantes**

- **Numéros QC des nouveaux moteurs**
- **Dates d'installation**
- **Version SDK utilisée**
- **Résultats des tests**
- **Tout problème rencontré**

### **Fichiers à créer**

- `MOTEURS_INSTALLES_17JAN2026.md` - Liste des moteurs installés
- `TESTS_MOTEURS_17JAN2026.md` - Résultats des tests
- `SURVEILLANCE_MOTEURS.md` - Log de surveillance continue

---

## ✅ **CHECKLIST FINALE**

### **Avant première utilisation après installation**

- [ ] Tous les moteurs testés individuellement ✅
- [ ] Tests globaux effectués ✅
- [ ] Pas de LED rouge ✅
- [ ] Mouvements fluides ✅
- [ ] SDK à jour (v1.2.11 recommandé) ✅
- [ ] Documentation complète ✅

### **Après première utilisation**

- [ ] Aucun problème détecté ✅
- [ ] Moteurs fonctionnent correctement ✅
- [ ] Pas de surchauffe ✅
- [ ] Performance normale ✅

---

## 🎯 **RÈGLES D'OR**

1. **Toujours vérifier les numéros QC** avant installation
2. **Toujours tester mécaniquement** (débranché) avant installation
3. **Toujours vérifier le câblage** avant démarrage
4. **Toujours surveiller les LEDs** au démarrage
5. **Toujours documenter** tout problème
6. **Toujours arrêter immédiatement** si problème détecté
7. **Toujours contacter Pollen** si problème matériel

---

## 🔗 **RESSOURCES**

- **Formulaire de remplacement** : <https://forms.gle/JdhMzadeCnbynw7Q6>
- **Support Pollen** : <mailto:sales@pollen-robotics.com>
- **Discord Pollen** : #support
- **Documentation BBIA** :
  - `PROBLEME_MOTEURS_QC_BATCH_DEC2025.md`
  - `ANALYSE_REPO_OFFICIEL_JANVIER_2026.md`
  - `REACHY_MINI_SDK_v1.2.4.md`

---

**En suivant ce guide, vous minimisez les risques de problèmes futurs avec les moteurs !** 🛡️🤖
