# 🎭 Guide des Comportements BBIA

**Date** : 8 Décembre 2025
**Version BBIA** : 1.4.0
**Objectif** : Guide complet pour utiliser et créer des comportements BBIA

---

## 📋 Liste des Comportements Disponibles

BBIA dispose de **15 comportements avancés** organisés en catégories :

### 🎯 Comportements de Suivi

#### 1. **FollowFace** - Suivi de visage

**Description :** Suit un visage détecté avec MediaPipe, ajuste les émotions selon la distance, et réagit si le visage disparaît.

**Utilisation :**
```python
from bbia_sim.behaviors import FollowFaceBehavior
from bbia_sim.bbia_vision import BBIAVision

vision = BBIAVision(robot_api=robot_api)
behavior = FollowFaceBehavior(vision=vision, robot_api=robot_api)

# Exécuter pour 30 secondes
context = {"duration": 30.0}
behavior.execute(context)
```

**Paramètres :**
- `duration` (float) : Durée du suivi en secondes (défaut: 30.0)

---

#### 2. **FollowObject** - Suivi d'objet

**Description :** Suit des objets détectés avec YOLO, priorise les personnes, et réagit si l'objet est perdu.

**Utilisation :**
```python
from bbia_sim.behaviors import FollowObjectBehavior

behavior = FollowObjectBehavior(vision=vision, robot_api=robot_api)

# Suivre un objet spécifique
context = {"target_object": "person", "duration": 30.0}
behavior.execute(context)
```

**Paramètres :**
- `target_object` (str, optionnel) : Nom de l'objet à suivre spécifiquement
- `duration` (float) : Durée du suivi en secondes (défaut: 30.0)

---

### 💬 Comportements de Communication

#### 3. **Conversation** - Conversation intelligente

**Description :** Conversation naturelle avec LLM (BBIAHuggingFace si disponible), mouvements expressifs, et réactions visuelles.

**Utilisation :**
```python
from bbia_sim.behaviors import ConversationBehavior

behavior = ConversationBehavior(robot_api=robot_api)
behavior.execute({})
```

**Fonctionnalités :**
- Intégration LLM automatique si disponible
- Mouvements expressifs (hochement de tête)
- Détection d'émotions depuis le texte
- Réponses enrichies en fallback

---

### 🎨 Comportements Expressifs

#### 4. **EmotionShow** - Démonstration des émotions

**Description :** Parcourt toutes les 12 émotions BBIA avec transitions fluides et explications vocales.

**Utilisation :**
```python
from bbia_sim.behaviors import EmotionShowBehavior

behavior = EmotionShowBehavior(robot_api=robot_api)

# Toutes les émotions
behavior.execute({})

# Émotions spécifiques
behavior.execute({"emotions_list": ["happy", "sad", "excited"]})
```

**Paramètres :**
- `emotions_list` (list, optionnel) : Liste d'émotions à démontrer (défaut: toutes)

**Émotions disponibles :**
- `neutral`, `happy`, `sad`, `angry`, `curious`, `excited`
- `surprised`, `fearful`, `confused`, `determined`, `nostalgic`, `proud`

---

#### 5. **Dance** - Danse synchronisée

**Description :** Danse synchronisée avec musique, détection de rythme, et émotions selon le type de musique.

**Utilisation :**
```python
from bbia_sim.behaviors import DanceBehavior

behavior = DanceBehavior(robot_api=robot_api)

# Danse joyeuse
behavior.execute({"music_type": "happy", "duration": 30.0})

# Danse calme
behavior.execute({"music_type": "calm", "duration": 20.0})

# Danse énergique
behavior.execute({"music_type": "energetic", "duration": 30.0})
```

**Paramètres :**
- `music_type` (str) : Type de musique (`happy`, `calm`, `energetic`)
- `duration` (float) : Durée de la danse en secondes (défaut: 30.0)
- `audio_file` (str, optionnel) : Fichier audio à analyser

---

#### 6. **PhotoBooth** - Mode photo

**Description :** Prend des photos avec poses expressives, détection de visage pour cadrage, et compte à rebours.

**Utilisation :**
```python
from bbia_sim.behaviors import PhotoBoothBehavior

behavior = PhotoBoothBehavior(vision=vision, robot_api=robot_api)

# Une photo avec pose joyeuse
behavior.execute({"pose": "happy", "countdown": True})

# Plusieurs photos
behavior.execute({
    "pose": "cool",
    "num_photos": 3,
    "countdown": True,
    "auto_capture": True
})
```

**Paramètres :**
- `pose` (str) : Pose à prendre (`happy`, `cool`, `surprised`, `proud`)
- `countdown` (bool) : Activer compte à rebours (défaut: True)
- `auto_capture` (bool) : Capture automatique (défaut: True)
- `num_photos` (int) : Nombre de photos à prendre (défaut: 1)

---

### 📚 Comportements Interactifs

#### 7. **Storytelling** - Raconter des histoires

**Description :** Raconte des histoires avec mouvements expressifs, émotions selon les scènes, et interaction utilisateur.

**Utilisation :**
```python
from bbia_sim.behaviors import StorytellingBehavior

behavior = StorytellingBehavior(robot_api=robot_api)

# Histoire du Petit Chaperon Rouge
behavior.execute({"story": "petit_chaperon_rouge"})

# Histoire des Trois Petits Cochons
behavior.execute({"story": "trois_petits_cochons"})
```

**Histoires disponibles :**
- `petit_chaperon_rouge`
- `trois_petits_cochons`

---

#### 8. **Teaching** - Mode éducatif

**Description :** Mode éducatif interactif avec leçons pré-définies, mouvements explicatifs, et questions/réponses.

**Utilisation :**
```python
from bbia_sim.behaviors import TeachingBehavior

behavior = TeachingBehavior(robot_api=robot_api)

# Leçon de maths niveau débutant
behavior.execute({"subject": "maths", "level": "beginner"})

# Leçon de sciences niveau avancé
behavior.execute({"subject": "sciences", "level": "advanced"})
```

**Paramètres :**
- `subject` (str) : Matière (`maths`, `sciences`, etc.)
- `level` (str) : Niveau (`beginner`, `intermediate`, `advanced`)

---

#### 9. **Game** - Jeux interactifs

**Description :** Jeux interactifs (pierre-papier-ciseaux, devine le nombre, mémoire) avec réactions selon le résultat.

**Utilisation :**
```python
from bbia_sim.behaviors import GameBehavior

behavior = GameBehavior(robot_api=robot_api)

# Pierre-papier-ciseaux
behavior.execute({"game": "rock_paper_scissors", "rounds": 3})

# Devine le nombre
behavior.execute({"game": "guess_number", "max_number": 100})

# Jeu de mémoire
behavior.execute({"game": "memory", "difficulty": "easy"})
```

**Jeux disponibles :**
- `rock_paper_scissors` : Pierre-papier-ciseaux
- `guess_number` : Devine le nombre
- `memory` : Jeu de mémoire

**Paramètres :**
- `game` (str) : Nom du jeu
- `rounds` (int, optionnel) : Nombre de rounds
- `max_number` (int, optionnel) : Nombre maximum pour devine le nombre
- `difficulty` (str, optionnel) : Difficulté (`easy`, `medium`, `hard`)

---

### 🧘 Comportements Utilitaires

#### 10. **Meditation** - Guide méditation

**Description :** Guide méditation avec mouvements lents, voix calme, et respiration synchronisée.

**Utilisation :**
```python
from bbia_sim.behaviors import MeditationBehavior

behavior = MeditationBehavior(robot_api=robot_api)

# Séance de 10 minutes
behavior.execute({"duration": 600})
```

**Paramètres :**
- `duration` (int) : Durée de la séance en secondes (défaut: 300)

---

#### 11. **Exercise** - Guide exercices

**Description :** Guide exercices physiques avec mouvements démonstratifs, comptage de répétitions, et encouragements.

**Utilisation :**
```python
from bbia_sim.behaviors import ExerciseBehavior

behavior = ExerciseBehavior(robot_api=robot_api)

# Rotation de tête
behavior.execute({"exercise": "head_rotation", "repetitions": 5})

# Étirements
behavior.execute({"exercise": "stretching", "repetitions": 3})
```

**Paramètres :**
- `exercise` (str) : Type d'exercice (`head_rotation`, `stretching`, etc.)
- `repetitions` (int) : Nombre de répétitions

---

#### 12. **AlarmClock** - Réveil intelligent

**Description :** Réveil intelligent avec séquence progressive, détection si l'utilisateur se réveille, et mode snooze.

**Utilisation :**
```python
from bbia_sim.behaviors import AlarmClockBehavior

behavior = AlarmClockBehavior(robot_api=robot_api)

# Réveil à 7h30
behavior.execute({"hour": 7, "minute": 30})

# Avec snooze activé
behavior.execute({"hour": 7, "minute": 30, "snooze": True})
```

**Paramètres :**
- `hour` (int) : Heure du réveil (0-23)
- `minute` (int) : Minute du réveil (0-59)
- `snooze` (bool) : Activer mode snooze (défaut: False)

---

#### 13. **WeatherReport** - Rapport météo

**Description :** Rapport météo avec mouvements expressifs selon les conditions et recommandations.

**Utilisation :**
```python
from bbia_sim.behaviors import WeatherReportBehavior

behavior = WeatherReportBehavior(robot_api=robot_api)

# Météo pour Paris
behavior.execute({"city": "Paris"})

# Avec coordonnées GPS
behavior.execute({"lat": 48.8566, "lon": 2.3522})
```

**Paramètres :**
- `city` (str, optionnel) : Nom de la ville
- `lat` (float, optionnel) : Latitude GPS
- `lon` (float, optionnel) : Longitude GPS

---

#### 14. **NewsReader** - Lecture actualités

**Description :** Lecture d'actualités avec réactions émotionnelles selon le contenu et résumé.

**Utilisation :**
```python
from bbia_sim.behaviors import NewsReaderBehavior

behavior = NewsReaderBehavior(robot_api=robot_api)

# Lire 5 actualités
behavior.execute({"max_items": 5})

# Actualités d'une catégorie spécifique
behavior.execute({"category": "technology", "max_items": 3})
```

**Paramètres :**
- `max_items` (int) : Nombre maximum d'actualités à lire (défaut: 5)
- `category` (str, optionnel) : Catégorie d'actualités

---

#### 15. **MusicReaction** - Réaction à la musique

**Description :** Réagit à la musique avec mouvements synchronisés, détection de genre, et émotions adaptatives.

**Utilisation :**
```python
from bbia_sim.behaviors import MusicReactionBehavior

behavior = MusicReactionBehavior(robot_api=robot_api)

# Réaction à la pop
behavior.execute({"genre": "pop", "duration": 30})

# Réaction à la musique classique
behavior.execute({"genre": "classical", "duration": 20})
```

**Paramètres :**
- `genre` (str) : Genre musical (`pop`, `rock`, `classical`, `jazz`, etc.)
- `duration` (float) : Durée de la réaction en secondes (défaut: 30.0)

---

## 🚀 Utilisation Basique

### Via BBIABehaviorManager

```python
from bbia_sim.bbia_behavior import BBIABehaviorManager
from bbia_sim.robot_factory import create_robot

# Créer le robot
robot = create_robot("mujoco")

# Créer le gestionnaire de comportements
manager = BBIABehaviorManager(robot_api=robot)

# Exécuter un comportement
manager.execute_behavior("dance", {"music_type": "happy", "duration": 30})

# Lister tous les comportements disponibles
behaviors = manager.get_available_behaviors()
for behavior in behaviors:
    print(f"{behavior['name']}: {behavior['description']}")
```

### Directement

```python
from bbia_sim.behaviors import DanceBehavior

behavior = DanceBehavior(robot_api=robot)
behavior.execute({"music_type": "happy", "duration": 30})
```

---

## ⚙️ Configuration Avancée

### Priorités des Comportements

Les comportements ont des priorités (1-10, 10 étant le plus prioritaire) :

```python
behavior.priority = 8  # Comportement prioritaire
```

### Vérification d'Exécution

Avant d'exécuter, vérifier si le comportement peut être exécuté :

```python
if behavior.can_execute(context):
    behavior.execute(context)
```

### Arrêt d'un Comportement

```python
behavior.stop()  # Arrête le comportement
```

---

## 🛠️ Création de Nouveaux Comportements

### Structure de Base

Tous les comportements héritent de `BBIABehavior` :

```python
from bbia_sim.behaviors.base import BBIABehavior
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from ..robot_api import RobotAPI

class MonComportement(BBIABehavior):
    """Description du comportement."""

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        super().__init__(
            name="mon_comportement",
            description="Description du comportement",
            robot_api=robot_api,
        )
        self.priority = 5  # Priorité 1-10

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté."""
        return self.robot_api is not None

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le comportement."""
        if not self.robot_api:
            return False

        # Votre logique ici
        # ...

        return True

    def stop(self) -> None:
        """Arrête le comportement."""
        super().stop()
        # Nettoyage si nécessaire
```

### Exemple Complet

```python
#!/usr/bin/env python3
"""Comportement exemple pour BBIA."""

from bbia_sim.behaviors.base import BBIABehavior
from typing import TYPE_CHECKING, Any
import logging

if TYPE_CHECKING:
    from ..robot_api import RobotAPI

logger = logging.getLogger("BBIA")

class ExempleBehavior(BBIABehavior):
    """Comportement exemple."""

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        super().__init__(
            name="exemple",
            description="Comportement exemple",
            robot_api=robot_api,
        )
        self.priority = 5

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté."""
        return self.robot_api is not None

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le comportement."""
        if not self.robot_api:
            logger.error("Robot API non disponible")
            return False

        logger.info("Exécution du comportement exemple")

        # Appliquer une émotion
        if hasattr(self.robot_api, "set_emotion"):
            self.robot_api.set_emotion("happy", 0.7)

        # Mouvement
        if hasattr(self.robot_api, "goto_target"):
            self.robot_api.goto_target(
                body_yaw=0.1,
                duration=0.5,
                method="minjerk",
            )

        return True
```

### Enregistrer le Comportement

Ajouter dans `src/bbia_sim/behaviors/__init__.py` :

```python
from .exemple import ExempleBehavior

__all__ = [
    # ...
    "ExempleBehavior",
]
```

Et dans `BBIABehaviorManager._register_default_behaviors()` :

```python
from .behaviors import ExempleBehavior

self.register_behavior(ExempleBehavior(robot_api=self.robot_api))
```

---

## 📝 Bonnes Pratiques

1. **Gestion d'erreurs** : Toujours utiliser try/except avec fallback gracieux
2. **Validation** : Valider tous les inputs (heures, volumes, etc.)
3. **Logging** : Utiliser le logger BBIA pour toutes les opérations
4. **Type hints** : Utiliser typing pour tous les paramètres et retours
5. **Documentation** : Ajouter docstrings pour toutes les fonctions
6. **Tests** : Créer des tests unitaires pour chaque comportement

---

## 🧪 Tests

Tous les comportements sont testés dans `tests/test_behaviors_advanced.py` :

```bash
pytest tests/test_behaviors_advanced.py -v
```

---

## 📚 Références

- **Plan détaillé** : `docs/quality/audits/PLAN_COMPORTEMENTS_AVANCES.md`
- **Architecture** : `docs/development/architecture/ARCHITECTURE_DETAILED.md`
- **Code source** : `src/bbia_sim/behaviors/`

---

**Document créé le :** 8 Décembre 2025
**Version BBIA :** 1.4.0
**Dernière mise à jour :** 8 Décembre 2025
**Auteur :** Arkalia Luna System

