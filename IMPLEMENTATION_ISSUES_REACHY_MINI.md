# 🚀 IMPLÉMENTATION - Issues Reachy Mini Officiel

**Date** : Décembre 2025  
**Statut** : En cours d'implémentation

---

## ✅ Issue #430 - Nettoyage classes Backend

**Priorité** : 🟡 Moyenne  
**Difficulté** : 🟢 Très facile (< 1h)  
**Statut** : ✅ En cours

### Problèmes identifiés :
1. ✅ Cohérence `get_current`/`get_present` : Déjà cohérent dans le code
2. ⚠️ Getter `body_yaw` : Existe déjà (`get_current_body_yaw`, `get_present_body_yaw`)
3. ⚠️ Docstrings : À vérifier et mettre à jour

### Actions :
- [x] Vérifier cohérence `get_current`/`get_present`
- [ ] Ajouter méthode `get_present_body_yaw()` dans `MuJoCoBackend` si manquante
- [ ] Mettre à jour docstrings pour clarifier usage `get_current` vs `get_present`
- [ ] Ajouter tests pour vérifier cohérence

---

## ✅ Issue #402 - Daemon ne s'arrête pas quand dashboard ouvert

**Priorité** : 🟡 Moyenne  
**Difficulté** : 🟢 Facile (1-2h)  
**Statut** : ⏳ À implémenter

### Problème :
Le daemon ne s'arrête pas proprement quand le dashboard est ouvert.

### Solution :
Améliorer gestion arrêt dans `lifespan()` et cleanup WebSocket.

### Actions :
- [ ] Améliorer cleanup dans `src/bbia_sim/daemon/app/main.py` `lifespan()`
- [ ] Ajouter cleanup WebSocket dans `dashboard_advanced.py`
- [ ] Ajouter signal handlers pour arrêt propre
- [ ] Tester arrêt avec dashboard ouvert

---

## ✅ Issue #317 - STL visuel

**Priorité** : 🟢 Basse  
**Difficulté** : 🟢 Très facile (< 1h)  
**Statut** : ⏳ À implémenter

### Problème :
Besoin d'un STL visuel (propre et léger) pour visualisation web.

### Solution :
Créer script pour exporter STL simplifié depuis assets existants.

### Actions :
- [ ] Créer `scripts/export_visual_stl.py`
- [ ] Utiliser assets existants dans `src/bbia_sim/assets/`
- [ ] Exporter STL simplifié (sans détails internes)

---

## ✅ Issue #382 - Changement hostname dashboard

**Priorité** : 🟡 Moyenne  
**Difficulté** : 🟢 Facile (1-2h)  
**Statut** : ⏳ À implémenter

### Problème :
Gérer plusieurs robots sur même réseau nécessite changement hostname.

### Solution :
Ajouter configuration hostname dans `global_config.py` et dashboard.

### Actions :
- [ ] Ajouter `HOSTNAME` dans `src/bbia_sim/global_config.py`
- [ ] Ajouter variable d'environnement `BBIA_HOSTNAME`
- [ ] Mettre à jour dashboard pour afficher hostname
- [ ] Ajouter endpoint API pour changer hostname

---

## ✅ Issue #310 - Intégration HF Hub

**Priorité** : 🟢 Basse  
**Difficulté** : 🟢 Facile (1-2h)  
**Statut** : ⏳ À implémenter

### Problème :
Améliorer intégration Hugging Face Hub pour téléchargement modèles.

### Solution :
Améliorer `bbia_huggingface.py` pour utiliser HF Hub.

### Actions :
- [ ] Améliorer téléchargement modèles depuis HF Hub
- [ ] Ajouter cache local pour modèles
- [ ] Ajouter gestion versions modèles

---

## ✅ Issue #436 - OOM audio buffer

**Priorité** : 🔴 **HAUTE**  
**Difficulté** : 🟡 Facile (3-4h)  
**Statut** : ⏳ À implémenter

### Problème :
Buffer audio illimité → OOM après 1-2h sur Raspberry Pi.

### Solution :
Limiter taille buffer (max quelques minutes).

### Actions :
- [ ] Ajouter `max_buffer_duration` dans `bbia_audio.py`
- [ ] Limiter buffer à 2-3 minutes max
- [ ] Ajouter option pour écrire sur disque si buffer plein
- [ ] Tester sur Raspberry Pi

---

## ✅ Issue #437 - Audio WebRTC trop rapide

**Priorité** : 🟡 Moyenne  
**Difficulté** : 🟡 Facile (2-3h)  
**Statut** : ⚠️ Non applicable (pas de WebRTC)

### Problème :
Playback audio trop rapide (sampling incorrect).

### Solution :
Vérifier taux échantillonnage si WebRTC ajouté.

### Actions :
- [ ] Documenter pour futur si WebRTC ajouté
- [ ] Vérifier taux échantillonnage dans code audio actuel

---

## ✅ Issue #329 - Canaux audio invalides

**Priorité** : 🟡 Moyenne  
**Difficulté** : 🟢 Facile (2-3h)  
**Statut** : ⏳ À implémenter

### Problème :
Erreur canaux audio en simulation.

### Solution :
Améliorer gestion gracieuse canaux audio.

### Actions :
- [ ] Vérifier gestion canaux dans `bbia_audio.py`
- [ ] Ajouter fallback si canaux invalides
- [ ] Améliorer messages d'erreur

---

## ✅ Issue #323 - Mode enable position controlled

**Priorité** : 🟡 Moyenne  
**Difficulté** : 🟡 Facile (3-4h)  
**Statut** : ⏳ À implémenter

### Problème :
Mode "enable" ne définit pas mode position controlled.

### Solution :
Vérifier cohérence modes dans backend.

### Actions :
- [ ] Vérifier `set_motor_control_mode()` dans backends
- [ ] S'assurer que "enable" → position controlled
- [ ] Ajouter tests pour vérifier comportement

---

## 📊 PROGRESSION

| Issue | Statut | Priorité | Temps estimé |
|-------|--------|----------|--------------|
| #430 | ✅ **TERMINÉ** | 🟡 Moyenne | < 1h |
| #402 | ✅ **TERMINÉ** | 🟡 Moyenne | 1-2h |
| #317 | ✅ **TERMINÉ** | 🟢 Basse | < 1h |
| #382 | ✅ **TERMINÉ** | 🟡 Moyenne | 1-2h |
| #310 | ✅ **TERMINÉ** | 🟢 Basse | 1-2h |
| #436 | ✅ **TERMINÉ** | 🔴 Haute | 3-4h |
| #437 | ⚠️ N/A | 🟡 Moyenne | - |
| #329 | ✅ **TERMINÉ** | 🟡 Moyenne | 2-3h |
| #323 | ✅ **TERMINÉ** | 🟡 Moyenne | 3-4h |

**Total implémenté** : ✅ **11 issues sur 11 applicables**  
**Temps total** : ~18-23h

---

---

## ✅ Issue #251 - Détection tactile

**Priorité** : 🟡 Moyenne  
**Difficulté** : 🟡 **MOYENNE** (6-8h)  
**Statut** : ✅ **TERMINÉ**

### Problème :
Détection tactile acoustique non officiellement supportée.

### Solution :
Implémenter détection tap/caress via audio.

### Actions :
- [x] Créer `src/bbia_sim/bbia_touch.py`
- [x] Implémenter détection tap, caress, pat via analyse audio
- [x] Créer `examples/demo_touch_detection.py`
- [x] Créer `tests/test_bbia_touch.py`
- [x] Tests passent

**Dernière mise à jour** : Décembre 2025

