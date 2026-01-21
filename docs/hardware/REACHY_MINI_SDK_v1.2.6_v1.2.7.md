# SDK Reachy Mini v1.2.6 et v1.2.7 - Documentation Complète

**Dernière mise à jour** : 6 Janvier 2026  
**Version SDK actuelle BBIA** : v1.2.4 (recommandé)  
**Dernière version stable** : v1.2.6 (3 janvier 2026)  
**Dernière version pré-release** : v1.2.7rc2 (6 janvier 2026, 12h51)

---

## 📦 **RELEASES RÉCENTES**

### **v1.2.7rc2** (Pré-lancement) - 6 janvier 2026, 12h51

**⚠️ VERSION PRÉ-RELEASE - NE PAS INSTALLER EN PRODUCTION**

**Corrections apportées** :
- Faire fonctionner `get_ip_address` sous Windows
- Ajout profil H264 configurable pour compatibilité multiplateforme
- Amélioration documentation module multimédia
- Réduction utilisation intégration continue

**Impact BBIA** : 
- ✅ Compatible Mac (pas de changement nécessaire)
- ⚠️ Version pré-release avec potentiels bugs - **Ne pas mettre à jour**

---

### **v1.2.7rc1** (Pré-lancement) - 5 janvier 2026

**⚠️ VERSION PRÉ-RELEASE - NE PAS INSTALLER EN PRODUCTION**

**Corrections apportées** :
- Correction radio saccadée et lente

**Impact BBIA** : 
- ✅ Compatible Mac
- ⚠️ Version pré-release - **Ne pas mettre à jour**

---

### **v1.2.6** (Stable) - 3 janvier 2026

**✅ VERSION STABLE - MAIS PROBLÈMES CONNUS**

**Plus de 20 corrections et améliorations** :

1. **Correction Windows Lite**
2. **Fix mode sleep/stop avec WebRTC**
3. **Mode --mockup-sim** (simulation légère)
4. **Amélioration détection Lite vs Wireless**
5. **Binding IMU pour wireless**
6. **Fix applications 100% CPU**
7. Et bien plus...

**⚠️ PROBLÈMES CONNUS AVEC v1.2.6** :

**Rapportés par la communauté Discord (6 janvier 2026)** :

1. **Crashes du démon robot** avec Rust panic
   - **Utilisateur** : Damien (00:27)
   - **Symptômes** : Démon robot crash avec erreur Rust panic
   - **Impact** : Robot devient inutilisable

2. **Erreurs IK (Inverse Kinematics)**
   - **Symptômes** : "Collision detected or head pose not achievable"
   - **Impact** : Mouvements de tête impossibles

3. **Pertes de connexion entre app et daemon**
   - **Symptômes** : Connexion se perd régulièrement
   - **Impact** : Contrôle du robot intermittent

4. **Erreurs frontend 404s**
   - **Symptômes** : Erreurs 404 sur `/status` endpoint
   - **Impact** : Dashboard ne fonctionne pas correctement

**Caroline de Pollen** a créé plusieurs fils de discussion sur ces problèmes :
- "1.2.6 issue"
- "1.2.6 upgrade"
- "Lost connection with the server"

**L'équipe Pollen travaille activement sur les corrections** (v1.2.7rc2 sortie il y a 2h!)

**Impact BBIA** :
- ✅ Compatible Mac (pas de changement nécessaire)
- ⚠️ **RECOMMANDATION** : Rester sur SDK v1.2.4 jusqu'à ce que v1.2.7 stable soit disponible
- ⚠️ **NE PAS METTRE À JOUR** vers v1.2.6 si vous avez un robot fonctionnel

---

## 🖥️ **COMPATIBILITÉ MAC**

### **Mac Mini (votre configuration)**

✅ **Toutes les versions SDK sont compatibles Mac** :
- v1.2.4 : ✅ Compatible
- v1.2.6 : ✅ Compatible (mais problèmes connus)
- v1.2.7rc1/rc2 : ✅ Compatible (mais pré-release)

**Aucun problème spécifique Mac identifié** dans les releases récentes.

**Note** : Les problèmes rapportés concernent principalement :
- Le démon robot (côté robot, pas Mac)
- Les connexions WebSocket (réseau)
- Les erreurs IK (calculs côté robot)

**Votre Mac Mini n'est pas concerné** par ces problèmes.

---

## 💬 **ACTIVITÉ DISCORD - 6 JANVIER 2026**

### **Messages importants du jour**

1. **Damien** (00:27) - Problèmes avec SDK v1.2.6 et v1.2.7.rc1 :
   - Crashes du démon robot avec Rust panic
   - Erreurs IK "Collision detected or head pose not achievable"
   - Pertes de connexion entre app et daemon
   - Frontend erreurs 404s sur /status endpoint

2. **RReitsma** (23:36-23:37) - **FIX POUR WINDOWS** :
   - A posté un fix pour faire fonctionner Reachy mini control sur Windows
   - Problème avec le module `pwd` non disponible sur Windows
   - Solution: import conditionnel du module pwd
   - **Impact Mac** : Aucun (module `pwd` disponible sur macOS)

### **Problèmes de moteurs rapportés**

- **Hala** (5 janvier 08:01) - Plusieurs personnes rapportent des problèmes avec **motor 4** et demandent résolution
- **Plusieurs utilisateurs** ont des moteurs du lot QC 2542, 2543, 2544 défectueux
- **Voir** : `PROBLEME_MOTEURS_QC_BATCH_DEC2025.md` pour détails complets

### **Problèmes techniques courants**

- Apps qui ne fonctionnent qu'en version wired (problème WebSocket)
- Erreurs de shutdown via webui (`'NoneType' object has no attribute 'should_stop'`)
- Problèmes de connexion wireless intermittents
- Camera "Coming Soon" dans dashboard (normal - feature pas encore implémentée)

---

## ✅ **RECOMMANDATIONS POUR BBIA**

### **Court terme (immédiat)**

1. **Rester sur SDK v1.2.4** (version stable que vous avez)
2. **Ne pas mettre à jour** vers v1.2.6 (problèmes connus)
3. **Ne pas mettre à jour** vers v1.2.7rc1/rc2 (versions pré-release)

### **Moyen terme (quand v1.2.7 stable sortira)**

1. **Attendre la version stable v1.2.7** (pas de rc)
2. **Tester en environnement de développement** avant mise à jour production
3. **Vérifier que les problèmes v1.2.6 sont corrigés**

### **Surveillance**

1. **Surveiller le Discord** #support pour annonce v1.2.7 stable
2. **Vérifier GitHub releases** : https://github.com/pollen-robotics/reachy_mini/releases
3. **Documenter** tout nouveau problème si vous mettez à jour

---

## 📊 **COMPARAISON DES VERSIONS**

| Version | Date | Statut | Recommandation BBIA | Problèmes connus |
|--------|------|--------|---------------------|------------------|
| **v1.2.4** | Décembre 2025 | ✅ Stable | ✅ **RECOMMANDÉ** | Aucun |
| **v1.2.6** | 3 janvier 2026 | ⚠️ Stable mais bugs | ❌ **NE PAS INSTALLER** | Crashes, IK errors, connexions |
| **v1.2.7rc1** | 5 janvier 2026 | ⚠️ Pré-release | ❌ **NE PAS INSTALLER** | Version de test |
| **v1.2.7rc2** | 6 janvier 2026 | ⚠️ Pré-release | ❌ **NE PAS INSTALLER** | Version de test |

---

## 🔗 **RESSOURCES**

- **GitHub Releases** : https://github.com/pollen-robotics/reachy_mini/releases
- **Discord Pollen** : #support (Caroline et équipe actives)
- **Documentation BBIA** : 
  - `REACHY_MINI_SDK_v1.2.4.md` - Version recommandée
  - `PROBLEME_MOTEURS_QC_BATCH_DEC2025.md` - Problèmes moteurs
- **Support Pollen** : sales@pollen-robotics.com

---

## 📅 **HISTORIQUE**

- **3 Janvier 2026** : Release v1.2.6 (stable mais problèmes connus)
- **5 Janvier 2026** : Release v1.2.7rc1 (pré-release)
- **6 Janvier 2026** : Release v1.2.7rc2 (pré-release, 12h51)
- **6 Janvier 2026** : Problèmes v1.2.6 rapportés sur Discord
- **6 Janvier 2026** : Documentation créée pour BBIA

---

## ⚠️ **IMPORTANT**

- **Votre Mac Mini n'est pas concerné** par les problèmes spécifiques Windows/Linux
- **Rester sur SDK v1.2.4** jusqu'à ce que v1.2.7 stable soit disponible
- **Surveiller Discord** pour annonce de la version stable
- **Tester en dev** avant toute mise à jour production

