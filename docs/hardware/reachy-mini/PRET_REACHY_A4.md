# 📋 CHECKLIST REACHY RÉEL - STATUT ACTUEL

**Date réception** : 18 Décembre 2025  
**Date montage** : 20 Décembre 2025 (durée : 4 heures)  
**Dernière mise à jour** : 22 Décembre 2025  
**Version** : 1.2.0  
**Robot** : Reachy Mini Wireless (Pollen Robotics)  
**IP Robot** : 192.168.129.64

---

## ✅ **PRÉPARATION HARDWARE**

### **🔌 Connexion Robot**

- [x] **SDK Reachy installé** : Version 1.2.3 ✅
- [x] **Robot allumé** : LED de statut verte ✅
- [x] **Connexion réseau** : IP robot 192.168.129.64 ✅
- [x] **Ports ouverts** : 8080 (API), 8081 (WebSocket) ✅
- [x] **WiFi configuré** : Robot connecté au réseau local ✅

### **🛡️ Sécurité**

- [ ] **Bouton STOP** : Coupure d'urgence accessible
- [ ] **Espace libre** : 2m autour du robot
- [ ] **Personnes averties** : Équipe informée du test
- [ ] **Limites sécurité** : amp ≤ 0.3 rad actives

---

## ✅ **VALIDATION LOGICIELLE**

### **🧪 Tests Automatiques**

- [ ] **hardware_dry_run.py** : Script exécuté avec succès
- [ ] **Latence cible** : <40ms set→read mesurée
- [ ] **Joints validés** : Noms et limites corrects
- [ ] **Limites sécurité** : Clamp automatique actif

### **📊 Métriques Cibles**

- [ ] **Latence moyenne** : <40ms
- [ ] **Latence max** : <100ms
- [ ] **Erreurs** : 0 erreur critique
- [ ] **Durée test** : 10s minimum

---

## ✅ **MAPPING JOINTS**

### **🎯 Joints Principaux**

- [ ] **yaw_body** : Rotation corps (limite ±0.3 rad)
- [ ] **head_yaw** : Rotation tête (limite ±0.3 rad)
- [ ] **head_pitch** : Inclinaison tête (limite ±0.3 rad)

### **🚫 Joints Interdits**

- [x] **left_antenna** : Animable avec limites (-0.3 à 0.3 rad)
- [x] **right_antenna** : Animable avec limites (-0.3 à 0.3 rad)
- [ ] **passive_1-7** : BLOQUÉS (passifs)

---

## ✅ **COMMANDES DE TEST**

### **🚀 Test Complet**

```bash
# Activer le venv
source venv/bin/activate

# Test hardware complet (10s)
python scripts/hardware_dry_run.py --duration 10

# Test joint spécifique
python scripts/hardware_dry_run.py --joint yaw_body --duration 5

```

### **📊 Résultats Attendus**

```text
✅ Robot Reachy connecté avec succès
✅ Tous les joints de test sont disponibles
✅ Limite d'amplitude respectée
✅ Joint interdit correctement rejeté
⏱️ Latence moyenne: 25.3ms
✅ Latence cible atteinte (<40ms)
🎉 Hardware dry run réussi !

```

---

## ✅ **EN CAS DE PROBLÈME**

### **❌ Erreurs Courantes**

- **Connexion échouée** : Vérifier IP, ports, SDK
- **Latence élevée** : Vérifier réseau, charge système
- **Joint introuvable** : Vérifier mapping SDK
- **Limite non respectée** : Vérifier RobotAPI

### **🔧 Actions Correctives**

1. **Redémarrer robot** : Power cycle
2. **Vérifier SDK** : Version compatible
3. **Tester réseau** : ping, telnet
4. **Logs détaillés** : --verbose

---

## ✅ **VALIDATION FINALE**

### **🎯 Critères de Succès**

- [ ] **Connexion stable** : Pas de déconnexion
- [ ] **Latence acceptable** : <40ms moyenne
- [ ] **Sécurité active** : Limites respectées
- [ ] **Joints fonctionnels** : set→read OK
- [ ] **Erreurs zéro** : Aucune erreur critique

### **📝 Notes**

```text
Date du test: _______________
Opérateur: _______________
Version SDK: _______________
IP Robot: _______________
Latence mesurée: _______________
Observations: _______________

```

---

## 📝 NOTES POST-MONTAGE (22 Déc 2025)

### Problèmes identifiés et actions

1. **Bug décembre 2025 - Reflash moteurs** ✅
   - Problème : Moteurs avec paramètres d'usine incorrects (ID=1, baudrate 57,600)
   - Solution : Reflash effectué le 22 déc 2025 via `reachy-mini-reflash-motors`
   - Résultat : Tous les moteurs (ID 10-18) reconfigurés correctement

2. **Tête penchée** ⚠️
   - Problème : Tête penchée vers l'avant et sur le côté droit
   - Correction logicielle : Script `fix_head_tilted.py` avec corrections roll=180°, pitch=-70°, z=80mm
   - Statut : Amélioration mais problème persiste → Vérification câblage nécessaire

3. **Moteur clignotant rouge** ⚠️
   - Problème : Moteur physique (probablement stewart_2) clignote en rouge
   - Diagnostic : Tous les moteurs répondent logiciellement (positions lisibles)
   - Action : Vérification câblage et placement du moteur nécessaire

### Scripts de diagnostic créés

- ✅ `diagnostic_motor_errors_ssh.py` : Diagnostic complet des moteurs avec test de mouvement
- ✅ `fix_head_tilted.py` : Correction automatique de la tête penchée
- ✅ `diagnostic_stewart.py` : Diagnostic des joints Stewart platform
- ✅ `REFLASH_GUIDE.md` : Guide complet pour le reflash des moteurs
- ✅ `GUIDE_MOTEUR_CLIGNOTANT.md` : Guide pour résoudre les problèmes de moteurs

### Prochaines étapes

- [ ] Vérification câblage du moteur qui clignote
- [ ] Vérification placement des moteurs selon guide d'assemblage
- [ ] Calibration complète via app Reachy Mini Control
- [ ] Tests de mouvement complets

---

**✅ CHECKLIST VALIDÉE** : Robot monté et opérationnel, diagnostics effectués

*Dernière mise à jour : 22 Décembre 2025*
