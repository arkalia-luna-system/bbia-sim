# 🎭 PLAN DÉTAILLÉ : Comportements Avancés

**Date** : 19 Novembre 2025  
**Priorité** : 🟡 **MOYENNE**  
**Durée estimée** : 6 semaines  
**Objectif** : Créer 15+ comportements aussi avancés que l'officiel, mais avec l'identité BBIA  
**Statut** : ✅ **TERMINÉ - 15/15 comportements créés (100%)**

---

## 🎯 Objectif

Créer une **bibliothèque de 15+ comportements intelligents** qui :
- ✅ Utilisent l'IA BBIA (vision, voice, émotions)
- ✅ Sont plus avancés que de simples démos
- ✅ Intègrent l'expressivité BBIA (12 émotions)
- ✅ Sont modulaires et extensibles

---

## 📊 ÉTAT ACTUEL

### ✅ Comportements Existants

**Fichier :** `src/bbia_sim/bbia_behavior.py`

**Comportements actuels :**
1. ✅ `WakeUpBehavior` - Réveil robot
2. ✅ `SleepBehavior` - Endormissement
3. ✅ `FollowFaceBehavior` - Suivre visage
4. ✅ `VisionTrackingBehavior` - Suivre objet
5. ✅ `ConversationBehavior` - Conversation (basique)

**Total : 5 comportements** (vs 15+ officiel)

---

## 🚀 COMPORTEMENTS À CRÉER

### Bibliothèque Complète (15 Comportements)

#### 1. **FollowFace** ✅ (FAIT)

**Description :** Suivre visage avec mouvements fluides

**Améliorations :**
- ✅ Détection visage MediaPipe (déjà fait)
- ✅ Améliorer précision suivi
- ✅ Ajouter émotions selon distance visage
- ✅ Réaction si visage disparaît

**Fichier :** `src/bbia_sim/behaviors/follow_face.py` ✅ **CRÉÉ**

---

#### 2. **FollowObject** ✅ (FAIT)

**Description :** Suivre objet détecté avec YOLO

**Améliorations :**
- ✅ Détection YOLO (déjà fait)
- ✅ Suivi multi-objets
- ✅ Priorisation objets (personne > objet)
- ✅ Réaction si objet perdu

**Fichier :** `src/bbia_sim/behaviors/follow_object.py` ✅ **CRÉÉ**

---

#### 3. **Conversation** ✅ (FAIT)

**Description :** Conversation naturelle avec LLM

**Améliorations :**
- ✅ Intégrer LLM (BBIAHuggingFace si disponible)
- ✅ Mouvements expressifs selon émotions
- ✅ Réactions visuelles (hochement tête)

**Fichier :** `src/bbia_sim/behaviors/conversation.py` ✅ **CRÉÉ**

---

#### 4. **Dance** ✅ (FAIT)

**Description :** Danses synchronisées avec musique

**Fonctionnalités :**
- ✅ Détection rythme audio
- ✅ Mouvements chorégraphiés
- ✅ Synchronisation musique
- ✅ Émotions selon type musique

**Fichier :** `src/bbia_sim/behaviors/dance.py` ✅ **CRÉÉ**

**Code structure :**
```python
class DanceBehavior(BBIABehavior):
    def __init__(self, robot_api):
        super().__init__("dance", "Danse synchronisée avec musique")
        self.dance_routines = {
            "happy": self._dance_happy,
            "calm": self._dance_calm,
            "energetic": self._dance_energetic,
        }
    
    def execute(self, context):
        # Détecter musique
        music_type = self._detect_music_type()
        
        # Choisir routine
        routine = self.dance_routines.get(music_type, self._dance_default)
        
        # Exécuter danse
        routine()
```

---

#### 5. **EmotionShow** ✅ (FAIT)

**Description :** Démonstration des 12 émotions BBIA

**Fonctionnalités :**
- ✅ Parcourir toutes les émotions
- ✅ Transitions fluides entre émotions
- ✅ Explications vocales ("Maintenant je suis heureux")
- ✅ Durée adaptative selon émotion

**Fichier :** `src/bbia_sim/behaviors/emotion_show.py` ✅ **CRÉÉ**

---

#### 6. **Storytelling** ✅ (FAIT)

**Description :** Raconter histoires avec mouvements expressifs

**Fonctionnalités :**
- ✅ Histoires pré-enregistrées
- ✅ Mouvements synchronisés avec narration
- ✅ Émotions selon scènes histoire
- ✅ Interaction utilisateur (questions)

**Fichier :** `src/bbia_sim/behaviors/storytelling.py` ✅ **CRÉÉ**

**Exemple :**
```python
class StorytellingBehavior(BBIABehavior):
    def __init__(self):
        super().__init__("storytelling", "Raconter histoires avec mouvements")
        self.stories = {
            "petit_chaperon_rouge": self._story_little_red_riding_hood,
            "trois_petits_cochons": self._story_three_little_pigs,
        }
    
    def execute(self, context):
        story_name = context.get("story", "petit_chaperon_rouge")
        story_func = self.stories.get(story_name)
        if story_func:
            story_func()
```

---

#### 7. **Teaching** ✅ (FAIT)

**Description :** Mode éducatif interactif

**Fonctionnalités :**
- ✅ Leçons pré-définies (maths, sciences, etc.)
- ✅ Mouvements explicatifs (pointer, montrer)
- ✅ Questions/réponses interactives
- ✅ Encouragements selon performance

**Fichier :** `src/bbia_sim/behaviors/teaching.py` ✅ **CRÉÉ**

---

#### 8. **Meditation** ✅ (FAIT)

**Description :** Guide méditation avec mouvements lents

**Fonctionnalités :**
- ✅ Séances méditation guidées
- ✅ Mouvements lents et fluides
- ✅ Voix calme et apaisante
- ✅ Respiration synchronisée

**Fichier :** `src/bbia_sim/behaviors/meditation.py` ✅ **CRÉÉ**

---

#### 9. **Exercise** ✅ (FAIT)

**Description :** Guide exercices physiques

**Fonctionnalités :**
- ✅ Exercices pré-définis (étirements, etc.)
- ✅ Mouvements démonstratifs
- ✅ Comptage répétitions
- ✅ Encouragements

**Fichier :** `src/bbia_sim/behaviors/exercise.py` ✅ **CRÉÉ**

---

#### 10. **MusicReaction** ✅ (FAIT)

**Description :** Réagir à la musique avec mouvements

**Fonctionnalités :**
- ✅ Détection genre musical
- ✅ Mouvements selon rythme
- ✅ Émotions selon musique (happy pour pop, calm pour classique)
- ✅ Synchronisation audio

**Fichier :** `src/bbia_sim/behaviors/music_reaction.py` ✅ **CRÉÉ**

---

#### 11. **PhotoBooth** ✅ (FAIT)

**Description :** Mode photo avec poses expressives

**Fonctionnalités :**
- ✅ Poses pré-définies (happy, cool, etc.)
- ✅ Détection visage pour cadrage
- ✅ Compte à rebours ("3, 2, 1, souriez !")
- ✅ Capture photo automatique

**Fichier :** `src/bbia_sim/behaviors/photo_booth.py` ✅ **CRÉÉ**

---

#### 12. **AlarmClock** ✅ (FAIT)

**Description :** Réveil intelligent avec interactions

**Fonctionnalités :**
- ✅ Réveil à heure définie
- ✅ Séquence réveil progressive (mouvements + voix)
- ✅ Détection si utilisateur se réveille
- ✅ Mode snooze

**Fichier :** `src/bbia_sim/behaviors/alarm_clock.py` ✅ **CRÉÉ**

---

#### 13. **WeatherReport** ✅ (FAIT)

**Description :** Rapport météo avec gestes expressifs

**Fonctionnalités :**
- ✅ Récupération météo (API)
- ✅ Mouvements selon météo (soleil = happy, pluie = triste)
- ✅ Narration météo
- ✅ Recommandations (parapluie, etc.)

**Fichier :** `src/bbia_sim/behaviors/weather_report.py` ✅ **CRÉÉ**

---

#### 14. **NewsReader** ✅ (FAIT)

**Description :** Lecture actualités avec réactions

**Fonctionnalités :**
- ✅ Récupération actualités (RSS/API)
- ✅ Narration actualités
- ✅ Réactions émotionnelles selon contenu
- ✅ Résumé actualités

**Fichier :** `src/bbia_sim/behaviors/news_reader.py` ✅ **CRÉÉ**

---

#### 15. **Game** ✅ (FAIT)

**Description :** Jeux interactifs (pierre-papier-ciseaux, etc.)

**Fonctionnalités :**
- ✅ Jeux pré-définis
- ✅ Détection gestes utilisateur (via vision)
- ✅ Réactions selon résultat
- ✅ Score et statistiques

**Fichier :** `src/bbia_sim/behaviors/game.py` ✅ **CRÉÉ**

---

## 📋 STRUCTURE ORGANISATIONNELLE

### Architecture Fichiers

```
src/bbia_sim/behaviors/
├── __init__.py              # Exports tous comportements
├── base.py                  # BBIABehavior (déjà existant)
├── follow_face.py           # ✅ Existant - Améliorer
├── follow_object.py         # ✅ Existant - Améliorer
├── conversation.py          # ⚠️ Existant - Transformer
├── dance.py                 # 🆕 Nouveau
├── emotion_show.py          # 🆕 Nouveau
├── storytelling.py          # 🆕 Nouveau
├── teaching.py              # 🆕 Nouveau
├── meditation.py            # 🆕 Nouveau
├── exercise.py              # 🆕 Nouveau
├── music_reaction.py        # 🆕 Nouveau
├── photo_booth.py           # 🆕 Nouveau
├── alarm_clock.py           # 🆕 Nouveau
├── weather_report.py        # 🆕 Nouveau
├── news_reader.py          # 🆕 Nouveau
└── game.py                  # 🆕 Nouveau
```

---

## 🗓️ CALENDRIER D'IMPLÉMENTATION

### Semaine 1-2 : Comportements Expressifs

- [x] **EmotionShow** - Démonstration émotions ✅
- [x] **Dance** - Danses synchronisées ✅
- [x] **PhotoBooth** - Mode photo ✅

### Semaine 3-4 : Comportements Interactifs

- [x] **Storytelling** - Histoires avec mouvements ✅
- [x] **Teaching** - Mode éducatif ✅
- [x] **Game** - Jeux interactifs ✅

### Semaine 5-6 : Comportements Utilitaires

- [x] **Meditation** - Guide méditation ✅
- [x] **Exercise** - Guide exercices ✅
- [x] **AlarmClock** - Réveil intelligent ✅
- [x] **WeatherReport** - Météo avec gestes ✅
- [x] **NewsReader** - Actualités ✅
- [x] **MusicReaction** - Réaction musique ✅

### Semaine 7-8 : Amélioration & Intégration

- [x] Améliorer comportements existants ✅
- [ ] Intégration Hugging Face Hub 🟡 (Optionnel)
- [x] Tests tous comportements ✅ **FAIT**
- [x] Documentation complète ✅ **FAIT**

---

## 🧪 TESTS

### Tests Unitaires

**Fichier :** `tests/test_behaviors_advanced.py`

```python
def test_emotion_show():
    """Test démonstration émotions."""
    behavior = EmotionShowBehavior(robot_api=mock_robot)
    result = behavior.execute({})
    assert result is True
    assert behavior.is_active is False  # Se termine automatiquement

def test_dance():
    """Test danse synchronisée."""
    behavior = DanceBehavior(robot_api=mock_robot)
    # Simuler musique
    context = {"music_type": "happy"}
    result = behavior.execute(context)
    assert result is True

def test_storytelling():
    """Test narration histoire."""
    behavior = StorytellingBehavior(robot_api=mock_robot)
    context = {"story": "petit_chaperon_rouge"}
    result = behavior.execute(context)
    assert result is True
```

---

## 📚 DOCUMENTATION

### Guide Comportements

**Fichier :** `docs/guides/GUIDE_COMPORTEMENTS.md`

**Contenu :**
- Liste tous comportements
- Utilisation basique
- Configuration avancée
- Création nouveaux comportements

---

## ✅ CHECKLIST FINALE

### Comportements à Créer

- [x] **Dance** - Danses synchronisées ✅
- [x] **EmotionShow** - Démonstration émotions ✅
- [x] **Storytelling** - Histoires avec mouvements ✅
- [x] **Teaching** - Mode éducatif ✅
- [x] **Meditation** - Guide méditation ✅
- [x] **Exercise** - Guide exercices ✅
- [x] **MusicReaction** - Réaction musique ✅
- [x] **PhotoBooth** - Mode photo ✅
- [x] **AlarmClock** - Réveil intelligent ✅
- [x] **WeatherReport** - Météo avec gestes ✅
- [x] **NewsReader** - Actualités ✅ **CRÉÉ**
- [x] **Game** - Jeux interactifs ✅ **CRÉÉ**

### Améliorations

- [x] Améliorer **FollowFace** ✅
- [x] Améliorer **FollowObject** ✅
- [x] Transformer **Conversation** (avec LLM) ✅

### Infrastructure

- [x] Créer structure `behaviors/` ✅
- [ ] Intégration Hugging Face Hub 🟡 (Optionnel)
- [x] Tests tous comportements ✅ **FAIT**
- [x] Documentation complète ✅ **FAIT**

---

## 🎯 MÉTRIQUES DE SUCCÈS

| Métrique | Actuel | Objectif | Statut |
|----------|--------|----------|--------|
| **Comportements** | 15 | 15+ | ✅ **100%** |
| **Comportements Avancés** | 15 | 15+ | ✅ **100%** |
| **Intégration IA** | Complète | Complète | ✅ **100%** |
| **Tests** | Complets | Complets | ✅ **100%** |

---

## 🎉 RÉSULTAT ATTENDU

Après 8 semaines, BBIA aura :
- ✅ **15+ comportements intelligents**
- ✅ **Intégration IA complète** (vision, voice, émotions)
- ✅ **Expressivité BBIA** (12 émotions)
- ✅ **Modularité** (facile d'ajouter nouveaux comportements)

**BBIA aura une bibliothèque de comportements aussi riche que l'officiel, mais avec l'identité unique BBIA !**

---

**Document créé le :** Novembre 2024  
**Dernière mise à jour :** 19 Novembre 2025  
**Version BBIA :** 1.3.2  
**Auteur :** Arkalia Luna System  
**Statut :** ✅ **15/15 comportements créés (100%) - TERMINÉ**

