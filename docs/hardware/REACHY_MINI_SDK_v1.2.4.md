# SDK Reachy Mini v1.2.4 - Documentation Complète

**Dernière mise à jour** : 17 Janvier 2026  
**Date** : Décembre 2025  
**Version SDK** : v1.2.4  
**Version précédente** : v1.2.3  
**Dernière version disponible** : v1.2.11 (14 janvier 2026) - Voir `ANALYSE_REPO_OFFICIEL_JANVIER_2026.md`

---

## Vue d'ensemble

Cette release corrige plusieurs problèmes critiques identifiés dans les versions précédentes, notamment le problème de batch QC 2544 affectant certains moteurs non flashés correctement à l'usine.

---

## Corrections apportées

### 1. Gestion média avec version wireless

**Problème** : Problèmes de gestion média sur la version wireless  
**Impact BBIA** : Aucun - notre code gère déjà `robot.media` avec fallbacks gracieux dans :
- `bbia_vision.py` → `robot.media.camera`
- `bbia_audio.py` → `robot.media.microphone` et `robot.media.record_audio()`
- `bbia_voice.py` → `robot.media.speaker` et `robot.media.play_audio()`

**Action** : ✅ Aucune action requise

---

### 2. Problèmes de moteurs (wrong operating mode)

**Problème** : Moteurs avec mauvais mode opératoire après `enable_motors()`  
**Impact BBIA** : Workaround existant dans `reachy_mini_backend.py` (lignes 1439-1451) :

```python
# Issue #323: S'assurer que le mode est position controlled après enable
if hasattr(self.robot, "set_operating_mode"):
    self.robot.set_operating_mode("position")
```

**Action** : ⚠️ À vérifier si le SDK corrige ce bug. Le workaround est conservé pour compatibilité avec anciennes versions.

---

### 3. Problèmes de propriété venv sur version wireless

**Problème** : Problèmes de propriété venv sur la version wireless  
**Impact BBIA** : Aucun impact direct (problème système)  
**Action** : ✅ Aucune action requise

---

### 4. Problème du moteur 4 (QC 2544) - Cause identifiée

**Problème** : Moteur 4 non flashé correctement à l'usine (batch QC 2544)  
**Solution SDK v1.2.4** : Reflash automatique lors de la connexion et du démarrage du robot

**⚠️ Important** : Si le moteur est déjà endommagé (raide mécaniquement), le reflash ne peut pas le réparer → Remplacement nécessaire.

**Impact BBIA** : Le moteur 4 (stewart_4) est référencé dans :
- `reachy_mini_backend.py` : Joint stewart_4 (ID 14)
- Scripts de diagnostic : `diagnose_motor_2_issue.py`, `diagnostic_motor_errors.py`
- Documentation : `examples/reachy_mini/GUIDE_DEPANNAGE_REACHY_MINI.md` (guide consolidé)

**Action** : ✅ SDK v1.2.4 va reflasher automatiquement - Voir `PROBLEME_MOTEURS_QC_BATCH_DEC2025.md` pour détails complets

---

## Actions recommandées

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

## Statut des actions

### Actions complétées

#### Documentation créée
- [x] `REACHY_MINI_SDK_v1.2.4_UPDATE.md` - Analyse complète de la release
- [x] `PROBLEME_MOTEURS_QC_BATCH_DEC2025.md` - Documentation problème batch QC
- [x] `SDK_v1.2.4_ACTIONS_STATUS.md` - Suivi des actions (fusionné dans ce document)

#### Code mis à jour
- [x] **`reachy_mini_backend.py`** :
  - [x] Ajout commentaire sur reflash automatique SDK v1.2.4+ dans `_try_connect_robot()`
  - [x] Mise à jour commentaire `enable_motors()` pour clarifier compatibilité SDK v1.2.4+
  - [x] Workaround `set_operating_mode("position")` conservé pour compatibilité anciennes versions

**Modifications effectuées** :

```python
# Méthode _try_connect_robot() (ligne ~252)
def _try_connect_robot(self) -> bool:
    """Tente de se connecter au robot physique.

    Note: Avec SDK v1.2.4+, le reflash automatique des moteurs se fait
    lors de la connexion et du démarrage du robot. Cela protège les futurs
    moteurs contre le problème de batch QC 2544 (moteurs non flashés à l'usine).
    """
    # ...
    logger.debug(
        "SDK v1.2.4+ : Reflash automatique des moteurs activé (protection batch QC 2544)"
    )
```

```python
# Méthode enable_motors() (ligne ~1427)
def enable_motors(self) -> None:
    """Active les moteurs du robot.

    Issue #323: S'assure que enable_motors définit le mode position controlled.

    Note: Avec SDK v1.2.4+, le SDK gère automatiquement le mode position.
    Ce workaround est conservé pour compatibilité avec anciennes versions SDK (< v1.2.4).
    """
    # ...
    # NOTE: SDK v1.2.4+ gère automatiquement, mais on garde ce code pour
    # compatibilité avec anciennes versions (< v1.2.4) où le bug existait
```

---

### Actions en attente (requièrent robot physique)

#### 1. Mettre à jour le SDK via le dashboard
- [ ] **Action** : Settings → Update SDK → v1.2.4
- [ ] **Statut** : ⏳ En attente (robot actuellement démonté pour remplacement moteurs)
- [ ] **Note** : Le reflash automatique se fera lors de la prochaine connexion

#### 2. Tester si le workaround `set_operating_mode("position")` est encore nécessaire
- [ ] **Action** : Tester avec SDK v1.2.4 si le workaround est toujours nécessaire
- [ ] **Statut** : ⏳ En attente (robot actuellement démonté)
- [ ] **Note** : Le code est déjà compatible (workaround conservé mais inoffensif pour v1.2.4+)

#### 3. Vérifier si les problèmes de calibration stewart_4 sont résolus
- [ ] **Action** : Tester les mouvements de la tête après remplacement moteurs
- [ ] **Statut** : ⏳ En attente (remplacement moteurs 1, 2, 4 en cours)
- [ ] **Note** : Les nouveaux moteurs devraient être correctement flashés

#### 4. Tester la gestion média sur version wireless
- [ ] **Action** : Tester camera, microphone, speaker après mise à jour SDK
- [ ] **Statut** : ⏳ En attente (robot actuellement démonté)
- [ ] **Note** : Notre code gère déjà les fallbacks gracieux

---

## Protection des futurs moteurs

### Ce qui est en place

1. **Reflash automatique SDK v1.2.4+** :
   - Le SDK v1.2.4 reflash automatiquement les moteurs lors de la connexion
   - Cela protège les futurs moteurs contre le problème batch QC 2544
   - **Aucune action code requise** - le SDK le fait automatiquement

2. **Workaround `set_operating_mode("position")`** :
   - Conservé pour compatibilité avec anciennes versions SDK
   - Inoffensif pour SDK v1.2.4+ (le SDK gère déjà automatiquement)
   - **Protection** : Si quelqu'un utilise une ancienne version SDK, le workaround protège

3. **Gestion d'erreurs robuste** :
   - Le code gère gracieusement les erreurs de connexion
   - Fallback en mode simulation si robot non disponible
   - **Protection** : Ne casse pas si le SDK n'est pas à jour

### Ce qui nécessite action utilisateur

1. **Mettre à jour le SDK** :
   - Via dashboard : Settings → Update SDK → v1.2.4
   - **Important** : Le reflash automatique ne fonctionne qu'avec SDK v1.2.4+

2. **Vérifier les numéros QC des nouveaux moteurs** :
   - S'assurer que les nouveaux moteurs ne sont pas du batch QC 2544
   - Vérifier qu'ils ne sont pas raides mécaniquement avant installation

---

## Notes techniques

### Reflash automatique SDK v1.2.4

Le SDK v1.2.4 effectue automatiquement un reflash des moteurs lors de :
- La connexion au robot (`ReachyMini.__init__()`)
- Le démarrage du robot
- L'ouverture du dashboard

**Important** :
- ✅ Protège les **futurs moteurs** (pas encore endommagés)
- ❌ Ne peut **pas réparer** les moteurs déjà endommagés (raides mécaniquement)
- ⚠️ Si un moteur est raide même débranché → **Remplacement nécessaire**

### Workaround `set_operating_mode("position")`

**Pourquoi on le garde** :
- Compatibilité avec anciennes versions SDK (< v1.2.4)
- Inoffensif pour SDK v1.2.4+ (le SDK gère déjà)
- Protection si quelqu'un utilise une ancienne version

**Quand on pourra le simplifier** :
- Quand on sera sûr que tous les utilisateurs ont SDK v1.2.4+
- Ou après tests confirmant que le SDK v1.2.4 gère bien automatiquement

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

## Checklist de mise à jour

- [ ] Mettre à jour le SDK à v1.2.4
- [ ] Tester la connexion au robot
- [ ] Tester les fonctionnalités média (camera, microphone, speaker)
- [ ] Tester les mouvements de la tête (stewart joints)
- [ ] Vérifier que `enable_motors()` fonctionne correctement
- [ ] Tester sur version wireless si disponible
- [ ] Mettre à jour la documentation si nécessaire

---

## Prochaines étapes

1. **En attente remplacement moteurs** (moteurs 1, 2, 4)
2. **Après remplacement** :
   - Réassembler la tête
   - Mettre à jour SDK via dashboard → v1.2.4
   - Tester la connexion (reflash automatique se fera)
   - Tester les mouvements de la tête
   - Tester la gestion média
3. **Vérifier** :
   - Que les nouveaux moteurs ne sont pas raides
   - Que le reflash automatique fonctionne
   - Que les mouvements sont corrects

---

## Références

- **SDK Officiel** : <https://github.com/pollen-robotics/reachy_mini>
- **Release Notes** : Voir l'annonce officielle pour détails complets
- **Documentation BBIA** :
  - `docs/hardware/GUIDE_COMPLET_AVANT_RECEPTION.md`
  - `docs/hardware/PROBLEME_MOTEURS_QC_BATCH_DEC2025.md` ⚠️ **NOUVEAU - Problème batch QC identifié**

---

*Document consolidé - Décembre 2025*

