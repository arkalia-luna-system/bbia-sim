# 📱 Apps Hugging Face Spaces - Pollen vs BBIA

**Date** : 15 Décembre 2025  
**Référence** : https://pollen-robotics-reachy-mini-landing-page.hf.space/#/apps

---

## 🎯 Situation Actuelle

### ✅ Ce que Pollen a
- **Page dédiée** : https://pollen-robotics-reachy-mini-landing-page.hf.space/#/apps
- **App Store intégré** : Interface pour découvrir et installer apps
- **15+ apps** : Behaviors créés par la communauté
- **Installation automatique** : Depuis Hugging Face Spaces

### ⚠️ Ce que BBIA a
- ✅ **Infrastructure complète** : Router `/development/api/apps/*` (11 endpoints)
- ✅ **3 apps locales** : `bbia_vision`, `bbia_chat`, `bbia_emotions`
- ✅ **Code de base** : Fonction pour lister apps HF Spaces (dans `apps.py`)
- ❌ **Chargement dynamique** : Pas encore implémenté depuis HF Hub API
- ❌ **Interface app store** : Pas d'interface graphique comme Pollen

---

## 📊 Comparaison Détaillée

| Fonctionnalité | Pollen | BBIA | Statut |
|----------------|--------|------|--------|
| **Router API apps** | ✅ | ✅ | ✅ **ÉGAL** |
| **Infrastructure apps** | ✅ | ✅ | ✅ **ÉGAL** |
| **Apps locales** | ✅ 3+ | ✅ 3 | ✅ **ÉGAL** |
| **Chargement HF Hub** | ✅ Dynamique | ⚠️ Partiel | 🟡 **PARTIEL** |
| **Interface app store** | ✅ Graphique | ❌ API seule | 🔴 **MANQUANT** |
| **Installation auto** | ✅ | ⚠️ Manuelle | 🟡 **PARTIEL** |

---

## 🔍 Code Existant dans BBIA

### Fichier : `src/bbia_sim/daemon/app/routers/apps.py`

**Lignes 161-195** : Code pour lister apps HF Spaces

```python
@router.get("/list-community")
async def list_community_apps() -> list[dict[str, Any]]:
    """Liste les apps créées par la communauté (HF Spaces)."""
    try:
        from huggingface_hub import HfApi
        
        api = HfApi()
        hf_spaces = api.list_spaces(
            author="pollen-robotics",  # ⚠️ Filtré sur pollen-robotics
            search="reachy-mini",
            sort="created",
            direction=-1,
        )
        
        apps = []
        for space in hf_spaces:
            apps.append({
                "name": space.id,
                "source_kind": "hf_space",
                "description": space.sdk or "HF Space",
                "hf_space": space.id,
                # ...
            })
        return apps
    except Exception:
        return []
```

**Problème actuel** :
- ⚠️ Code présent mais pas testé avec robot réel
- ⚠️ Filtre sur `author="pollen-robotics"` uniquement
- ⚠️ Pas d'installation automatique

---

## 🎯 Plan d'Action (Après Réception Robot)

### Phase 1 : Tester Apps BBIA Existantes (Semaine 1)
- [ ] Tester `bbia_vision` sur robot réel
- [ ] Tester `bbia_chat` sur robot réel
- [ ] Tester `bbia_emotions` sur robot réel
- [ ] Évaluer si suffisant ou besoin d'apps supplémentaires

### Phase 2 : Implémenter Chargement Dynamique (Si nécessaire)
- [ ] Améliorer fonction `list_community_apps()` dans `apps.py`
- [ ] Ajouter filtres plus larges (pas seulement `pollen-robotics`)
- [ ] Tester avec vraie API Hugging Face Hub
- [ ] Ajouter gestion erreurs et fallbacks

### Phase 3 : Interface App Store (Optionnel)
- [ ] Créer interface graphique dans dashboard BBIA
- [ ] Afficher apps disponibles depuis HF Hub
- [ ] Bouton "Install" pour chaque app
- [ ] Gestion installation/désinstallation

---

## 💡 Recommandation

### ⚠️ **NE PAS IMPLÉMENTER MAINTENANT**

**Pourquoi ?**

1. **Pas de robot réel** : Impossible de tester correctement
2. **Apps BBIA suffisantes** : Vous avez déjà 3 apps fonctionnelles
3. **Complexité inutile** : Ajouter du code sans bénéfice immédiat
4. **Priorité** : Se concentrer sur réception et tests hardware

### ✅ **Action Immédiate**

1. **Lire la page Pollen** : https://pollen-robotics-reachy-mini-landing-page.hf.space/#/apps
2. **Noter les apps intéressantes** : Pour référence future
3. **Tester apps BBIA d'abord** : Sur robot réel
4. **Décider ensuite** : Si besoin d'ajouter chargement dynamique

---

## 📝 Apps Pollen à Explorer (Référence)

D'après la page officielle, les apps incluent probablement :

- 🗣️ **Conversation App** : Talk naturally with Reachy Mini (LLMs)
- 📻 **Radio** : Listen to the radio with Reachy Mini
- 👋 **Hand Tracker** : Robot follows hand movements in real-time
- + 12+ autres behaviors créés par la communauté

**Note** : Ces apps sont disponibles sur Hugging Face Spaces avec tag `reachy-mini` ou `pollen-robotics`.

---

## 🔗 Liens Utiles

- **Page Apps Pollen** : https://pollen-robotics-reachy-mini-landing-page.hf.space/#/apps
- **Hugging Face Spaces** : https://huggingface.co/spaces
- **Documentation BBIA Apps** : `docs/community/GUIDE_HUGGINGFACE_SPACES.md`
- **Code BBIA Apps Router** : `src/bbia_sim/daemon/app/routers/apps.py`

---

**Date création** : 15 Décembre 2025  
**Statut** : ⚠️ **À IMPLÉMENTER APRÈS RÉCEPTION ROBOT**

