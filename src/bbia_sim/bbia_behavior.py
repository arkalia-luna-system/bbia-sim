#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
BBIA Behavior Manager - Module de gestion des comportements pour Reachy Mini Wireless
Comportements personnalisés, réactions automatiques, intégration avec les émotions
"""

import time
import random
import threading
from typing import Dict, List, Optional, Callable, Any
from datetime import datetime
from queue import Queue

try:
    from .bbia_emotions import BBIAEmotions
    from .bbia_vision import BBIAVision
    from .bbia_voice import dire_texte, reconnaitre_parole
    from .bbia_audio import enregistrer_audio, lire_audio
except ImportError:
    # Pour les tests directs
    from bbia_emotions import BBIAEmotions
    from bbia_vision import BBIAVision
    from bbia_voice import dire_texte, reconnaitre_parole
    from bbia_audio import enregistrer_audio, lire_audio


class BBIABehavior:
    """Comportement de base pour BBIA"""

    def __init__(self, name: str, description: str):
        self.name = name
        self.description = description
        self.is_active = False
        self.priority = 1  # 1-10, 10 étant le plus prioritaire

    def can_execute(self, context: Dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté"""
        return True

    def execute(self, context: Dict[str, Any]) -> bool:
        """Exécute le comportement"""
        print(f"🎭 Exécution du comportement : {self.name}")
        return True

    def stop(self):
        """Arrête le comportement"""
        self.is_active = False


class WakeUpBehavior(BBIABehavior):
    """Comportement de réveil de BBIA"""

    def __init__(self):
        super().__init__("wake_up", "Séquence de réveil complète de BBIA")
        self.priority = 10

    def execute(self, context: Dict[str, Any]) -> bool:
        print("\n✨ [BBIA] Séquence de réveil...")

        # 1. Initialisation
        print("💡 Lumière blanche faible...")
        time.sleep(1)

        # 2. Intensification
        print("💡 Lumière qui s'intensifie doucement...")
        time.sleep(1)

        # 3. Halo bleu
        print("💙 Halo bleu : BBIA s'éveille.")
        time.sleep(1)

        # 4. Respiration simulée
        print("🫧 Respiration simulée : inspiration...")
        time.sleep(1)
        print("🫧 Respiration simulée : expiration...")
        time.sleep(1)

        # 5. Son de démarrage
        print("🔊 Léger son de démarrage...")
        time.sleep(1)

        # 6. Mouvements de tête
        print("🤖 Mouvements de tête lents...")
        time.sleep(2)

        # 7. Expression
        print("😊 Expression : sourire doux.")
        time.sleep(1)

        # 8. Première parole
        print("🗣️ Première pensée : 'Je suis là, Athalia.'")
        dire_texte("Je suis là, Athalia.")

        print("✨ BBIA est complètement réveillé et prêt !\n")
        return True


class GreetingBehavior(BBIABehavior):
    """Comportement de salutation"""

    def __init__(self):
        super().__init__("greeting", "Salutation personnalisée")
        self.greetings = [
            "Bonjour ! Comment allez-vous ?",
            "Salut ! Ravi de vous voir !",
            "Hello ! Belle journée, n'est-ce pas ?",
            "Bonjour ! Je suis BBIA, enchanté !",
        ]

    def execute(self, context: Dict[str, Any]) -> bool:
        greeting = random.choice(self.greetings)
        print(f"👋 {greeting}")
        dire_texte(greeting)
        return True


class EmotionalResponseBehavior(BBIABehavior):
    """Comportement de réponse émotionnelle"""

    def __init__(self, emotions: BBIAEmotions):
        super().__init__("emotional_response", "Réponse émotionnelle automatique")
        self.emotions = emotions
        self.priority = 8

    def execute(self, context: Dict[str, Any]) -> bool:
        stimulus = context.get("stimulus", "")
        if stimulus:
            emotion = self.emotions.emotional_response(stimulus)
            print(f"🎭 Réponse émotionnelle : {emotion}")
            return True
        return False


class VisionTrackingBehavior(BBIABehavior):
    """Comportement de suivi visuel"""

    def __init__(self, vision: BBIAVision):
        super().__init__("vision_tracking", "Suivi visuel d'objets")
        self.vision = vision
        self.priority = 6

    def execute(self, context: Dict[str, Any]) -> bool:
        print("👁️ Activation du suivi visuel...")

        # Scan de l'environnement
        result = self.vision.scan_environment()

        # Suivi du premier objet détecté
        if result["objects"]:
            first_object = result["objects"][0]
            print(f"🎯 Suivi de l'objet : {first_object['name']}")
            self.vision.track_object(first_object["name"])
            return True

        return False


class ConversationBehavior(BBIABehavior):
    """Comportement de conversation"""

    def __init__(self):
        super().__init__("conversation", "Conversation interactive")
        self.priority = 7

    def execute(self, context: Dict[str, Any]) -> bool:
        print("🗣️ Mode conversation activé...")
        dire_texte("Je vous écoute.")

        # Écoute pendant 5 secondes
        texte = reconnaitre_parole(duree=5)

        if texte:
            print(f"👤 Vous avez dit : {texte}")
            dire_texte(f"J'ai entendu : {texte}")

            # Réponse simple basée sur le contenu
            if "bonjour" in texte.lower():
                dire_texte("Bonjour à vous aussi !")
            elif "comment" in texte.lower():
                dire_texte("Je vais très bien, merci !")
            elif "au revoir" in texte.lower():
                dire_texte("Au revoir ! À bientôt !")
            else:
                dire_texte("C'est intéressant, dites-moi en plus.")
        else:
            dire_texte("Je n'ai rien entendu. Pouvez-vous répéter ?")

        return True


class AntennaAnimationBehavior(BBIABehavior):
    """Comportement d'animation des antennes"""

    def __init__(self):
        super().__init__("antenna_animation", "Animation des antennes selon l'émotion")
        self.priority = 5

    def execute(self, context: Dict[str, Any]) -> bool:
        emotion = context.get("emotion", "neutral")

        animations = {
            "happy": "📡 Antennes légèrement relevées et vibrantes",
            "sad": "📡 Antennes tombantes et immobiles",
            "angry": "📡 Antennes rigides et dressées",
            "curious": "📡 Antennes frémissantes et orientées",
            "excited": "📡 Antennes très vibrantes et animées",
            "neutral": "📡 Antennes droites et calmes",
        }

        animation = animations.get(emotion, animations["neutral"])
        print(animation)
        time.sleep(2)
        return True


class BBIABehaviorManager:
    """Gestionnaire de comportements pour BBIA"""

    def __init__(self):
        self.behaviors: Dict[str, BBIABehavior] = {}
        self.active_behaviors: List[str] = []
        self.behavior_queue = Queue()
        self.is_running = False
        self.worker_thread = None

        # Initialiser les modules BBIA
        self.emotions = BBIAEmotions()
        self.vision = BBIAVision()

        # Enregistrer les comportements par défaut
        self._register_default_behaviors()

        print("🎭 BBIA Behavior Manager initialisé")
        print(f"   • Comportements disponibles : {len(self.behaviors)}")

    def _register_default_behaviors(self):
        """Enregistre les comportements par défaut"""
        self.register_behavior(WakeUpBehavior())
        self.register_behavior(GreetingBehavior())
        self.register_behavior(EmotionalResponseBehavior(self.emotions))
        self.register_behavior(VisionTrackingBehavior(self.vision))
        self.register_behavior(ConversationBehavior())
        self.register_behavior(AntennaAnimationBehavior())

    def register_behavior(self, behavior: BBIABehavior):
        """Enregistre un nouveau comportement"""
        self.behaviors[behavior.name] = behavior
        print(f"✅ Comportement enregistré : {behavior.name}")

    def execute_behavior(
        self, behavior_name: str, context: Optional[Dict[str, Any]] = None
    ) -> bool:
        """Exécute un comportement spécifique"""
        if behavior_name not in self.behaviors:
            print(f"❌ Comportement inconnu : {behavior_name}")
            return False

        behavior = self.behaviors[behavior_name]
        context = context or {}

        if behavior.can_execute(context):
            return behavior.execute(context)
        else:
            print(f"❌ Comportement {behavior_name} ne peut pas être exécuté")
            return False

    def add_to_queue(
        self, behavior_name: str, context: Optional[Dict[str, Any]] = None
    ):
        """Ajoute un comportement à la queue d'exécution"""
        self.behavior_queue.put((behavior_name, context or {}))

    def start_behavior_worker(self):
        """Démarre le worker de comportements"""
        if self.is_running:
            return

        self.is_running = True
        self.worker_thread = threading.Thread(target=self._behavior_worker, daemon=True)
        self.worker_thread.start()
        print("🎭 Worker de comportements démarré")

    def stop_behavior_worker(self):
        """Arrête le worker de comportements"""
        self.is_running = False
        if self.worker_thread:
            self.worker_thread.join()
        print("🎭 Worker de comportements arrêté")

    def _behavior_worker(self):
        """Worker qui traite la queue de comportements"""
        while self.is_running:
            try:
                if not self.behavior_queue.empty():
                    behavior_name, context = self.behavior_queue.get(timeout=1)
                    self.execute_behavior(behavior_name, context)
                else:
                    time.sleep(0.1)
            except Exception as e:
                print(f"❌ Erreur dans le worker de comportements : {e}")

    def get_available_behaviors(self) -> List[Dict[str, Any]]:
        """Retourne la liste des comportements disponibles"""
        return [
            {
                "name": behavior.name,
                "description": behavior.description,
                "priority": behavior.priority,
                "is_active": behavior.is_active,
            }
            for behavior in self.behaviors.values()
        ]

    def get_behavior_stats(self) -> Dict[str, Any]:
        """Retourne les statistiques des comportements"""
        return {
            "total_behaviors": len(self.behaviors),
            "active_behaviors": len(self.active_behaviors),
            "queue_size": self.behavior_queue.qsize(),
            "worker_running": self.is_running,
            "available_behaviors": self.get_available_behaviors(),
        }


def main():
    """Test du module BBIA Behavior Manager"""
    print("🧪 Test du module BBIA Behavior Manager")
    print("=" * 50)

    # Créer l'instance
    manager = BBIABehaviorManager()

    # Test 1 : Comportement de réveil
    print("\n1️⃣ Test comportement de réveil")
    manager.execute_behavior("wake_up")

    # Test 2 : Salutation
    print("\n2️⃣ Test comportement de salutation")
    manager.execute_behavior("greeting")

    # Test 3 : Animation des antennes
    print("\n3️⃣ Test animation des antennes")
    context = {"emotion": "happy"}
    manager.execute_behavior("antenna_animation", context)

    # Test 4 : Réponse émotionnelle
    print("\n4️⃣ Test réponse émotionnelle")
    context = {"stimulus": "compliment"}
    manager.execute_behavior("emotional_response", context)

    # Test 5 : Suivi visuel
    print("\n5️⃣ Test suivi visuel")
    manager.execute_behavior("vision_tracking")

    # Test 6 : Statistiques
    print("\n6️⃣ Test statistiques")
    stats = manager.get_behavior_stats()
    print(f"Statistiques : {stats}")

    print("\n✅ Test BBIA Behavior Manager terminé")


if __name__ == "__main__":
    main()
