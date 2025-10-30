# 📋 CHECKLIST PRÉ-REACHY RÉEL (A4)

**Date** : Octobre 2025
**Version** : 1.1.1
**Robot** : Reachy Mini Wireless (Pollen Robotics)

---

## ✅ **PRÉPARATION HARDWARE**

### **🔌 Connexion Robot**
- [ ] **SDK Reachy installé** : Version notée _______________
- [ ] **Robot allumé** : LED de statut verte
- [ ] **Connexion réseau** : IP robot _______________
- [ ] **Ports ouverts** : 8080 (API), 8081 (WebSocket)

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
- [ ] **left_antenna** : BLOQUÉ (passif)
- [ ] **right_antenna** : BLOQUÉ (passif)
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
```
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
```
Date du test: _______________
Opérateur: _______________
Version SDK: _______________
IP Robot: _______________
Latence mesurée: _______________
Observations: _______________
```

---

**✅ CHECKLIST VALIDÉE** : Robot prêt pour les démos BBIA !

*Dernière mise à jour : Octobre 2025*
