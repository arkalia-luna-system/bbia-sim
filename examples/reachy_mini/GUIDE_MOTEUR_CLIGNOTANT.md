# 🔴 Guide: Moteur qui clignote en rouge

## ❌ **C'EST ANORMAL !**

Un moteur qui clignote en rouge indique une **erreur matérielle** :
- Erreur de surcharge
- Surchauffe
- Problème de connexion
- Moteur en butée mécanique

---

## 🔍 **Identifier le moteur problématique**

### **1. Correspondance Motor ID ↔ Moteur physique**

| Motor ID | Nom physique | Emplacement |
|----------|--------------|-------------|
| **10** | Base | Rotation du corps |
| **11** | stewart_1 | Tête (moteur 1) |
| **12** | stewart_2 | Tête (moteur 2) ← **Si c'est celui qui clignote !** |
| **13** | stewart_3 | Tête (moteur 3) |
| **14** | stewart_4 | Tête (moteur 4) |
| **15** | stewart_5 | Tête (moteur 5) |
| **16** | stewart_6 | Tête (moteur 6) |
| **17** | Antenne gauche | Antenne |
| **18** | Antenne droite | Antenne |

### **2. Diagnostic automatique**

```bash
# Depuis votre Mac
python examples/reachy_mini/diagnostic_motor_errors.py
```

### **3. Vérification visuelle**

1. **Éteignez le robot** (interrupteur OFF)
2. **Enlevez le capot** de la tête
3. **Rallumez le robot** (interrupteur ON)
4. **Observez** quel moteur clignote en rouge
5. **Notez** le numéro du moteur (1 à 6 pour la tête)

---

## ✅ **Solutions selon le problème**

### **Problème 1: Moteur mal placé**

**Symptôme:** Le moteur clignote dès le démarrage

**Solution:**
1. Consultez le guide d'assemblage officiel :
   - [Reachy Mini Wireless - Guide étape par étape](https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide)
   - [Reachy Mini LITE - Guide](https://github.com/pollen-robotics/reachy_mini/blob/develop/docs/platforms/reachy_mini_lite/get_started.md)
2. Vérifiez que chaque moteur est dans le **bon emplacement**
3. Vérifiez que les **câbles sont branchés dans le bon ordre** (daisy-chain)

### **Problème 2: Câble mal branché**

**Symptôme:** Le moteur clignote de manière intermittente

**Solution:**
1. **Éteignez le robot** (interrupteur OFF)
2. Vérifiez que le câble est **bien enfoncé** dans le connecteur
3. Vérifiez qu'**aucun câble n'est plié ou coincé**
4. Vérifiez que le **câble n'est pas endommagé** (fils visibles, isolation cassée)
5. **Rallumez** et testez

### **Problème 3: Moteur en butée mécanique**

**Symptôme:** Le moteur clignote quand il essaie de bouger

**Solution:**
1. **Éteignez le robot**
2. **Déplacez manuellement** le moteur pour vérifier qu'il n'est pas bloqué
3. Vérifiez qu'**aucun câble ne bloque** le mouvement
4. Vérifiez que le **moteur n'est pas en butée** (position extrême)
5. **Rallumez** et testez

### **Problème 4: Moteur défectueux**

**Symptôme:** Le moteur clignote même après vérification de tout

**Solution:**
1. Contactez le **support Pollen Robotics** sur Discord
2. Fournissez:
   - Photo/vidéo du moteur qui clignote
   - Numéro du moteur (ID et position physique)
   - Logs du diagnostic (`diagnostic_motor_errors.py`)

---

## 🔧 **Vérification étape par étape**

### **Étape 1: Diagnostic automatique**

```bash
python examples/reachy_mini/diagnostic_motor_errors.py
```

### **Étape 2: Vérification visuelle**

1. Robot **éteint**
2. **Capot enlevé**
3. Robot **allumé**
4. **Observer** quel moteur clignote
5. **Noter** le numéro

### **Étape 3: Vérification câblage**

1. Robot **éteint**
2. Vérifier **chaque câble**:
   - Bien enfoncé
   - Pas plié
   - Pas endommagé
   - Dans le bon ordre (daisy-chain)
3. Consulter le guide d'assemblage pour l'ordre correct

### **Étape 4: Test manuel**

1. Robot **éteint**
2. **Déplacer manuellement** chaque moteur de la tête
3. Vérifier qu'**aucun n'est bloqué**
4. Robot **allumé**
5. Tester avec un mouvement simple

### **Étape 5: Si problème persiste**

1. **Redémarrer** complètement le robot (OFF/ON)
2. Relancer le **diagnostic**
3. Si toujours en erreur → **Support Pollen Robotics**

---

## 📋 **Checklist rapide**

- [ ] Robot éteint
- [ ] Capot enlevé
- [ ] Robot allumé
- [ ] Moteur qui clignote identifié (numéro noté)
- [ ] Câbles vérifiés (bien branchés, pas pliés, pas endommagés)
- [ ] Moteur testé manuellement (pas bloqué)
- [ ] Robot redémarré
- [ ] Diagnostic relancé
- [ ] Si toujours en erreur → Support contacté

---

## 🆘 **Support**

- **Discord Pollen Robotics:** [Lien Discord](https://discord.gg/pollen-robotics)
- **GitHub Issues:** [reachy_mini/issues](https://github.com/pollen-robotics/reachy_mini/issues)
- **Documentation officielle:** [Troubleshooting](https://github.com/pollen-robotics/reachy_mini/blob/develop/docs/troubleshooting.md)

---

## 📝 **Informations à fournir au support**

Si le problème persiste, fournissez:

1. **Numéro du moteur** (ID et position physique)
2. **Photo/vidéo** du moteur qui clignote
3. **Résultat du diagnostic** (`diagnostic_motor_errors.py`)
4. **Date de réception** du robot
5. **Version du firmware** (résultat du reflash)
6. **Description du problème** (quand ça clignote, depuis quand, etc.)

---

**💡 Astuce:** Gardez le robot **éteint** pendant que vous vérifiez le câblage pour éviter tout risque.

