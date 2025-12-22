# 📦 Mise à jour SDK Reachy Mini v1.2.4

**Date** : Décembre 2025  
**Version SDK** : v1.2.4 (nouvelle release)  
**Version précédente** : v1.2.3

## 🎉 Corrections apportées dans v1.2.4

Selon l'annonce officielle, cette release corrige :

### ✅ 1. Gestion média avec version wireless
- **Problème** : Problèmes de gestion média sur la version wireless
- **Impact BBIA** : Nous utilisons déjà `robot.media` avec fallbacks gracieux dans :
  - `bbia_vision.py` → `robot.media.camera`
  - `bbia_audio.py` → `robot.media.microphone` et `robot.media.record_audio()`
  - `bbia_voice.py` → `robot.media.speaker` et `robot.media.play_audio()`
- **Action** : ✅ Aucune action requise - notre code gère déjà les fallbacks

### ✅ 2. Problèmes de moteurs (wrong operating mode)
- **Problème** : Moteurs avec mauvais mode opératoire après `enable_motors()`
- **Impact BBIA** : Nous avons un workaround dans `reachy_mini_backend.py` (lignes 1439-1451) :
  ```python
  # Issue #323: S'assurer que le mode est position controlled après enable
  if hasattr(self.robot, "set_operating_mode"):
      self.robot.set_operating_mode("position")
  ```
- **Action** : ⚠️ **À vérifier** : Si le SDK corrige ce bug, nous pourrions simplifier ce code (mais garder le fallback pour compatibilité)

### ✅ 3. Problèmes de propriété venv sur version wireless
- **Problème** : Problèmes de propriété venv sur la version wireless
- **Impact BBIA** : Pas d'impact direct sur notre code (problème système)
- **Action** : ✅ Aucune action requise

### ✅ 4. Problème du moteur 4 (QC 2544) - **CAUSE IDENTIFIÉE**
- **Problème** : **Moteur 4 non flashé correctement à l'usine** (batch QC 2544)
- **Solution SDK v1.2.4** : **Reflash automatique** lors de la connexion et du démarrage du robot
- **⚠️ IMPORTANT** : Si le moteur est **déjà endommagé** (raide mécaniquement), le reflash ne peut pas le réparer → **Remplacement nécessaire**
- **Impact BBIA** : Le moteur 4 (stewart_4) est référencé dans plusieurs endroits :
  - `reachy_mini_backend.py` : Joint stewart_4 (ID 14)
  - Scripts de diagnostic : `diagnose_motor_2_issue.py`, `diagnostic_motor_errors.py`
  - Documentation : `SUPPORT_POLLEN_INFO.md`, `PROBLEME_CALIBRATION.md`
- **Action** : ✅ **SDK v1.2.4 va reflasher automatiquement** - Voir `PROBLEME_MOTEURS_QC_BATCH_DEC2025.md` pour détails complets

### ✅ 5. Et plus encore
- **Action** : ⚠️ **À tester** : Tester toutes les fonctionnalités après mise à jour

---

## 🔧 Actions recommandées

### 1. Mettre à jour le SDK
```bash
# Via le dashboard (recommandé)
# Settings → Update SDK → v1.2.4

# Ou manuellement
pip install --upgrade reachy-mini
```

### 2. Vérifier les workarounds
- [ ] Tester si `set_operating_mode("position")` est toujours nécessaire après `enable_motors()`
- [ ] Vérifier si les problèmes de calibration stewart_4 sont résolus
- [ ] Tester la gestion média sur version wireless

### 3. Mettre à jour la documentation
- [ ] Mettre à jour `GUIDE_COMPLET_AVANT_RECEPTION.md` : v1.2.3 → v1.2.4
- [ ] Documenter les changements si des workarounds peuvent être supprimés

---

## 📝 Notes techniques

### Workarounds actuels qui pourraient être simplifiés

1. **`set_operating_mode("position")`** (ligne 1439-1451 de `reachy_mini_backend.py`)
   - Si le SDK corrige le bug, ce code pourrait être simplifié
   - **Recommandation** : Garder le code mais ajouter un commentaire indiquant que c'est pour compatibilité avec anciennes versions

2. **Gestion média avec fallbacks**
   - Notre code gère déjà gracieusement l'absence de `robot.media`
   - Aucun changement nécessaire

3. **Problèmes de calibration stewart_4**
   - Si le SDK corrige les problèmes de calibration, nos scripts de diagnostic pourraient être simplifiés
   - **Recommandation** : Tester après mise à jour

---

## ✅ Checklist de mise à jour

- [ ] Mettre à jour le SDK à v1.2.4
- [ ] Tester la connexion au robot
- [ ] Tester les fonctionnalités média (camera, microphone, speaker)
- [ ] Tester les mouvements de la tête (stewart joints)
- [ ] Vérifier que `enable_motors()` fonctionne correctement
- [ ] Tester sur version wireless si disponible
- [ ] Mettre à jour la documentation si nécessaire

---

## 🔗 Références

- **SDK Officiel** : <https://github.com/pollen-robotics/reachy_mini>
- **Release Notes** : Voir l'annonce officielle pour détails complets
- **Documentation BBIA** : 
  - `docs/hardware/GUIDE_COMPLET_AVANT_RECEPTION.md`
  - `docs/hardware/PROBLEME_MOTEURS_QC_BATCH_DEC2025.md` ⚠️ **NOUVEAU - Problème batch QC identifié**

