#!/usr/bin/env python3

"""
BBIA Behavior Manager - Module de gestion des comportements pour Reachy Mini Wireless
Comportements personnalisés, réactions automatiques, intégration avec les émotions
"""

import logging
import os
import random
import threading
import time
from queue import Queue
from typing import Any, Optional

try:
    from .bbia_emotions import BBIAEmotions
    from .bbia_vision import BBIAVision
    from .bbia_voice import dire_texte, reconnaitre_parole
except ImportError:
    # Pour les tests directs
    pass

# Création du dossier logs si besoin
os.makedirs("logs", exist_ok=True)
# Logger BBIA strictement comme le test minimal

logger = logging.getLogger("BBIA")
# Supprime tous les handlers existants
for h in list(logger.handlers):
    logger.removeHandler(h)
handler = logging.FileHandler("logs/bbia.log", mode="a", encoding="utf-8")
formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)
logger.setLevel(logging.INFO)
logger.info("--- Initialisation du logger BBIA ---")


class BBIABehavior:
    """Comportement de base pour BBIA"""

    def __init__(self, name: str, description: str):
        self.name = name
        self.description = description
        self.is_active = False
        self.priority = 1  # 1-10, 10 étant le plus prioritaire

    def can_execute(self, context: dict[str, Any]) -> bool:
        logger.info(f"Vérification d'exécution du comportement : {self.name}")
        """Vérifie si le comportement peut être exécuté"""
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        logger.info(f"Exécution du comportement : {self.name}")
        print(f"🎭 Exécution du comportement : {self.name}")
        """Exécute le comportement"""
        return True

    def stop(self):
        logger.info(f"Arrêt du comportement : {self.name}")
        """Arrête le comportement"""
        self.is_active = False


class WakeUpBehavior(BBIABehavior):
    """Comportement de réveil de BBIA"""

    def __init__(self):
        super().__init__("wake_up", "Séquence de réveil complète de BBIA")
        self.priority = 10

    def execute(self, context: dict[str, Any]) -> bool:
        logger.info("Début de la séquence de réveil BBIA")
        print("\n✨ [BBIA] Séquence de réveil...")

        print("💡 Lumière blanche faible...")
        logger.info("Étape : Lumière blanche faible")
        time.sleep(1)

        print("💡 Lumière qui s'intensifie doucement...")
        logger.info("Étape : Lumière qui s'intensifie doucement")
        time.sleep(1)

        print("💙 Halo bleu : BBIA s'éveille.")
        logger.info("Étape : Halo bleu d'éveil")
        time.sleep(1)

        print("🫧 Respiration simulée : inspiration...")
        logger.info("Étape : Respiration simulée (inspiration)")
        time.sleep(1)
        print("🫧 Respiration simulée : expiration...")
        logger.info("Étape : Respiration simulée (expiration)")
        time.sleep(1)

        print("🔊 Léger son de démarrage...")
        logger.info("Étape : Son de démarrage")
        time.sleep(1)

        print("🤖 Mouvements de tête lents...")
        logger.info("Étape : Mouvements de tête lents")
        time.sleep(2)

        print("😊 Expression : sourire doux.")
        logger.info("Étape : Expression sourire doux")
        time.sleep(1)

        print("🗣️ Première pensée : 'Je suis là, Athalia.'")
        logger.info("Étape : Première parole d'éveil")
        dire_texte("Je suis là, Athalia.")
        logger.info("Synthèse vocale : Je suis là, Athalia.")

        print("✨ BBIA est complètement réveillé et prêt !\n")
        logger.info("Fin de la séquence de réveil BBIA")
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

    def execute(self, context: dict[str, Any]) -> bool:
        greeting = random.choice(self.greetings)
        logger.info(f"Salutation choisie : {greeting}")
        print(f"👋 {greeting}")
        dire_texte(greeting)
        logger.info(f"Synthèse vocale : {greeting}")
        return True


class EmotionalResponseBehavior(BBIABehavior):
    """Comportement de réponse émotionnelle"""

    def __init__(self, emotions: BBIAEmotions):
        super().__init__("emotional_response", "Réponse émotionnelle automatique")
        self.emotions = emotions
        self.priority = 8

    def execute(self, context: dict[str, Any]) -> bool:
        stimulus = context.get("stimulus", "")
        logger.info(f"Stimulus reçu pour réponse émotionnelle : {stimulus}")
        if stimulus:
            emotion = self.emotions.emotional_response(stimulus)
            logger.info(f"Réponse émotionnelle générée : {emotion}")
            print(f"🎭 Réponse émotionnelle : {emotion}")
            return True
        logger.info("Aucun stimulus fourni pour la réponse émotionnelle")
        return False


class VisionTrackingBehavior(BBIABehavior):
    """Comportement de suivi visuel"""

    def __init__(self, vision: BBIAVision):
        super().__init__("vision_tracking", "Suivi visuel d'objets")
        self.vision = vision
        self.priority = 6

    def execute(self, context: dict[str, Any]) -> bool:
        logger.info("Activation du suivi visuel")
        print("👁️ Activation du suivi visuel...")

        result = self.vision.scan_environment()
        logger.info(f"Résultat du scan environnement : {result}")

        if result["objects"]:
            first_object = result["objects"][0]
            logger.info(f"Suivi de l'objet : {first_object['name']}")
            print(f"🎯 Suivi de l'objet : {first_object['name']}")
            self.vision.track_object(first_object["name"])
            return True

        logger.info("Aucun objet détecté pour le suivi visuel")
        return False


class ConversationBehavior(BBIABehavior):
    """Comportement de conversation"""

    def __init__(self):
        super().__init__("conversation", "Conversation interactive")
        self.priority = 7

    def execute(self, context: dict[str, Any]) -> bool:
        logger.info("Activation du mode conversation")
        print("🗣️ Mode conversation activé...")
        dire_texte("Je vous écoute.")
        logger.info("Synthèse vocale : Je vous écoute.")

        texte = reconnaitre_parole(duree=5)
        logger.info(f"Texte reconnu : {texte}")

        if texte:
            print(f"👤 Vous avez dit : {texte}")
            dire_texte(f"J'ai entendu : {texte}")
            logger.info(f"Synthèse vocale : J'ai entendu : {texte}")

            if "bonjour" in texte.lower():
                dire_texte("Bonjour à vous aussi !")
                logger.info("Synthèse vocale : Bonjour à vous aussi !")
            elif "comment" in texte.lower():
                dire_texte("Je vais très bien, merci !")
                logger.info("Synthèse vocale : Je vais très bien, merci !")
            elif "au revoir" in texte.lower():
                dire_texte("Au revoir ! À bientôt !")
                logger.info("Synthèse vocale : Au revoir ! À bientôt !")
            else:
                dire_texte("C'est intéressant, dites-moi en plus.")
                logger.info("Synthèse vocale : C'est intéressant, dites-moi en plus.")
        else:
            dire_texte("Je n'ai rien entendu. Pouvez-vous répéter ?")
            logger.info("Synthèse vocale : Je n'ai rien entendu. Pouvez-vous répéter ?")

        return True


class AntennaAnimationBehavior(BBIABehavior):
    """Comportement d'animation des antennes"""

    def __init__(self):
        super().__init__("antenna_animation", "Animation des antennes selon l'émotion")
        self.priority = 5

    def execute(self, context: dict[str, Any]) -> bool:
        emotion = context.get("emotion", "neutral")
        logger.info(f"Animation des antennes pour l'émotion : {emotion}")

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
        logger.info(f"Étape animation antennes : {animation}")
        time.sleep(2)
        return True


class HideBehavior(BBIABehavior):
    """Comportement 'se cacher' (hide) pour Reachy Mini : tête baissée, antennes repliées, yeux fermés."""

    def __init__(self):
        super().__init__(
            "hide", "Se cacher : tête baissée, antennes repliées, yeux fermés"
        )
        self.priority = 9

    def execute(self, context: dict[str, Any]) -> bool:
        logger.info("Début de la séquence 'se cacher'")
        print("\n🙈 [BBIA] Séquence 'se cacher'...")
        print("🤖 Tête qui s'abaisse lentement...")
        logger.info("Étape : Tête qui s'abaisse lentement")
        time.sleep(1.5)
        print("📡 Antennes qui se replient devant le visage...")
        logger.info("Étape : Antennes qui se replient devant le visage")
        time.sleep(1.2)
        print("👁️ Yeux qui se ferment (ou s'éteignent)...")
        logger.info("Étape : Yeux qui se ferment (ou s'éteignent)")
        time.sleep(1)
        print("💤 BBIA se cache et devient silencieux.")
        logger.info("Étape : BBIA se cache et devient silencieux")
        dire_texte("Je me cache... Chut !")
        logger.info("Synthèse vocale : Je me cache... Chut !")
        time.sleep(1)
        print("(BBIA attend discrètement...)")
        logger.info("Fin de la séquence 'se cacher'")
        return True


class BBIABehaviorManager:
    """Gestionnaire de comportements pour BBIA"""

    def __init__(self):
        self.behaviors: dict[str, BBIABehavior] = {}
        self.active_behaviors: list[str] = []
        self.behavior_queue: Queue[tuple[str, dict[str, Any]]] = Queue()
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
        self.register_behavior(HideBehavior())

    def register_behavior(self, behavior: BBIABehavior):
        """Enregistre un nouveau comportement"""
        self.behaviors[behavior.name] = behavior
        print(f"✅ Comportement enregistré : {behavior.name}")

    def execute_behavior(
        self, behavior_name: str, context: Optional[dict[str, Any]] = None
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
        self, behavior_name: str, context: Optional[dict[str, Any]] = None
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

    def get_available_behaviors(self) -> list[dict[str, Any]]:
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

    def get_behavior_stats(self) -> dict[str, Any]:
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
    # Flush et fermeture explicite du handler pour garantir l'écriture du fichier de log
    for h in logger.handlers:
        h.flush()
        h.close()
