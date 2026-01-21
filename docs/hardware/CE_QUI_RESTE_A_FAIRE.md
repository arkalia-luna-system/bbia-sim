# 📋 Ce Qui Reste À Faire - 20 Janvier 2026

**Date** : 21 Janvier 2026  
**Dernière mise à jour** : 21 Janvier 2026  
**Statut** : ⏳ **EN ATTENTE DE LA COMMANDE DU 22 JANVIER**  
**Note** : ✅ Numéro QC des 3 moteurs reçus vérifié : **QC 2549** (batch sain)

---

## 🎯 **SITUATION ACTUELLE**

### ✅ **Ce qui est fait**

- ✅ **3 moteurs reçus** le 17 janvier 2026 (moteurs 1, 2, 4)
- ✅ **Numéro QC vérifié** : **QC 2549** (21 janvier 2026) - Batch sain ✅
- ✅ **Documentation complète** et à jour (20 janvier 2026)
- ✅ **Scripts de validation** créés et testés
- ✅ **Guides d'installation** complets
- ✅ **Tests** créés et passants
- ✅ **Analyse du repo officiel** complète
- ✅ **Erreurs lint** corrigées (13 warnings mineurs acceptables)
- ✅ **Black** : 321 fichiers OK
- ✅ **Git** : Tout poussé sur develop

### ⏳ **En attente**

- ⏳ **Réception de la commande du 22 janvier 2026** (contenu inconnu pour l'instant)
- ⏳ **Vérification du contenu** de la commande
- ⏳ **Installation des moteurs** (après vérification)

---

## 📦 **PROCHAINES ÉTAPES IMMÉDIATES**

### **1. Réception de la commande (22 janvier 2026)**

**À faire** :

- [ ] **Ouvrir la commande** et vérifier le contenu
- [ ] **Identifier ce qui a été envoyé** :
  - Moteurs supplémentaires ?
  - Câbles de rechange ?
  - Autres pièces ?
- [ ] **Vérifier les numéros QC** de tous les moteurs reçus
- [ ] **Documenter le contenu** dans `SUIVI_COMMUNICATION_POLLEN.md`

**Important** : Ne pas démonter Reachy avant d'avoir vérifié le contenu complet de la commande.

---

### **2. Vérification des moteurs reçus**

**Checklist de vérification** :

- [x] ✅ **Numéros QC vérifiés** (21 janvier 2026) :
  - ✅ **QC 2549** - Batch sain (pas dans les batches problématiques 2542/2543/2544)
  - ✅ Tous les 3 moteurs (1, 2, 4) ont le même numéro QC 2549
  - ✅ Moteurs sécurisés pour installation
- [ ] **Test mécanique** :
  - Chaque moteur doit tourner smooth (pas de résistance)
  - Pas de bruit anormal
  - Pas de dommages visibles
- [ ] **Quantité** :
  - 3 moteurs minimum (1, 2, 4)
  - Vérifier si d'autres moteurs sont inclus (3, 5, 6 pour remplacement préventif)

---

### **3. Préparation avant installation**

**Avant de démonter Reachy** :

- [ ] **Exécuter le script de vérification** :

  ```bash
  python examples/reachy_mini/check_before_motor_installation.py
  ```

- [ ] **Lire les guides** :
  - `docs/hardware/GUIDE_INSTALLATION_MOTEURS_ETAPE_PAR_ETAPE.md`
  - `docs/hardware/GUIDE_PREVENTION_PROBLEMES_MOTEURS.md`

- [ ] **Préparer les outils** :
  - Tournevis (petite taille)
  - Documentation imprimée ou accessible
  - Appareil photo (pour photos du câblage)

- [ ] **Prendre des photos** :
  - Photos du câblage actuel (pour référence)
  - Photos des numéros QC des anciens moteurs

---

### **4. Installation des moteurs**

**Procédure** :

1. **Éteindre le robot**
2. **Démonter la tête** (suivre guide officiel Pollen)
3. **Remplacer les moteurs** (1, 2, 4)
4. **Vérifier le câblage**
5. **Remonter la tête**

**Détails** : Voir `GUIDE_INSTALLATION_MOTEURS_ETAPE_PAR_ETAPE.md`

---

### **5. Rallumage et validation**

**Après installation** :

- [ ] **Allumer le robot**
- [ ] **Attendre le démarrage complet**
- [ ] **Exécuter le script de validation** :

  ```bash
  python examples/reachy_mini/validate_motor_installation.py
  ```

- [ ] **Vérifier les logs** :

  ```bash
  journalctl -u reachy-mini-daemon -f
  ```

- [ ] **Effectuer les tests manuels** (voir guide)

---

## 📚 **RESSOURCES DISPONIBLES**

### **Scripts**

1. **`check_before_motor_installation.py`**
   - Vérification avant installation
   - Checklist de préparation

2. **`validate_motor_installation.py`**
   - Validation complète après installation
   - Rapport détaillé

3. **`scan_motors_baudrate.py`**
   - Scan des moteurs
   - Détection des problèmes de baudrate

### **Guides**

1. **`GUIDE_INSTALLATION_MOTEURS_ETAPE_PAR_ETAPE.md`**
   - Guide complet d'installation
   - Procédures détaillées

2. **`GUIDE_PREVENTION_PROBLEMES_MOTEURS.md`**
   - Guide de prévention
   - Surveillance continue

3. **`PROBLEME_MOTEURS_QC_BATCH_DEC2025.md`**
   - Historique des problèmes
   - Solutions

### **Documentation**

- Tous les fichiers MD dans `docs/hardware/`
- Analyse complète du repo officiel
- Vérification complète effectuée

---

## ⚠️ **IMPORTANT**

### **Ne pas démonter Reachy avant**

- ✅ Avoir reçu et vérifié le contenu complet de la commande du 22 janvier
- ✅ Avoir vérifié les numéros QC des nouveaux moteurs
- ✅ Avoir effectué les tests mécaniques
- ✅ Avoir lu les guides d'installation
- ✅ Avoir préparé les outils

### **Points d'attention**

- **Numéros QC** : Vérifier que les nouveaux moteurs ne sont PAS QC 2542/2543/2544
- **Test mécanique** : Chaque moteur doit tourner smooth avant installation
- **Câblage** : Vérifier attentivement le câblage lors de l'installation
- **Documentation** : Mettre à jour `SUIVI_COMMUNICATION_POLLEN.md` après réception

---

## 🎯 **RÉSUMÉ**

### **Actions immédiates (22 janvier)**

1. ⏳ Réception de la commande
2. ⏳ Vérification du contenu
3. ⏳ Vérification des numéros QC
4. ⏳ Tests mécaniques

### **Actions après vérification**

1. ⏳ Préparation (scripts, guides, outils)
2. ⏳ Installation des moteurs
3. ⏳ Rallumage et validation
4. ⏳ Tests et surveillance

---

## ✅ **TOUT EST PRÊT !**

**Vous avez maintenant** :

- ✅ Tous les scripts nécessaires
- ✅ Tous les guides détaillés
- ✅ Toute la documentation
- ✅ Tous les tests
- ✅ Toute l'analyse

**Il ne reste plus qu'à :**

1. ⏳ Recevoir et vérifier la commande du 22 janvier
2. ⏳ Installer les moteurs
3. ⏳ Valider l'installation

**Tout est nickel !** 🎉

---

**Dernière mise à jour** : 21 Janvier 2026
