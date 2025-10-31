#!/usr/bin/env python3
"""
Module mémoire persistante BBIA - Sauvegarde conversation et préférences
Permet à BBIA de se souvenir entre les sessions
"""

import json
import logging
from datetime import datetime
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class BBIAMemory:
    """Module mémoire persistante pour BBIA.

    Sauvegarde :
    - Historique conversation
    - Préférences utilisateur
    - Apprentissages (patterns détectés)
    """

    def __init__(self, memory_dir: str = "bbia_memory"):
        """Initialise le module mémoire.

        Args:
            memory_dir: Répertoire pour stocker les fichiers mémoire
        """
        self.memory_dir = Path(memory_dir)
        self.memory_dir.mkdir(parents=True, exist_ok=True)

        # Fichiers mémoire
        self.conversation_file = self.memory_dir / "conversation_history.json"
        self.preferences_file = self.memory_dir / "preferences.json"
        self.learnings_file = self.memory_dir / "learnings.json"

        logger.info(f"💾 BBIAMemory initialisé (dir: {self.memory_dir})")

    def save_conversation(self, conversation_history: list[dict[str, Any]]) -> bool:
        """Sauvegarde l'historique conversation dans JSON.

        Args:
            conversation_history: Liste des conversations (format BBIAHuggingFace)

        Returns:
            True si sauvegarde réussie
        """
        try:
            # Sauvegarder avec timestamp
            data = {
                "last_updated": datetime.now().isoformat(),
                "conversation_count": len(conversation_history),
                "history": conversation_history,
            }

            with open(self.conversation_file, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)

            logger.debug(
                f"💾 Conversation sauvegardée ({len(conversation_history)} messages)"
            )
            return True
        except Exception as e:
            logger.error(f"❌ Erreur sauvegarde conversation: {e}")
            return False

    def load_conversation(self) -> list[dict[str, Any]]:
        """Charge l'historique conversation depuis JSON.

        Returns:
            Liste des conversations ou liste vide si erreur
        """
        try:
            if not self.conversation_file.exists():
                return []

            with open(self.conversation_file, encoding="utf-8") as f:
                data = json.load(f)

            history = data.get("history", [])
            logger.debug(f"💾 Conversation chargée ({len(history)} messages)")
            return history
        except Exception as e:
            logger.warning(f"⚠️ Erreur chargement conversation: {e}")
            return []

    def remember_preference(self, key: str, value: Any) -> bool:
        """Sauvegarde une préférence utilisateur.

        Args:
            key: Clé de la préférence (ex: "voix_preferee", "emotion_defaut")
            value: Valeur de la préférence

        Returns:
            True si sauvegarde réussie
        """
        try:
            # Charger préférences existantes
            preferences = self.load_preferences()

            # Ajouter/modifier préférence
            preferences[key] = {
                "value": value,
                "last_updated": datetime.now().isoformat(),
            }

            # Sauvegarder
            with open(self.preferences_file, "w", encoding="utf-8") as f:
                json.dump(preferences, f, indent=2, ensure_ascii=False)

            logger.debug(f"💾 Préférence sauvegardée: {key} = {value}")
            return True
        except Exception as e:
            logger.error(f"❌ Erreur sauvegarde préférence: {e}")
            return False

    def load_preferences(self) -> dict[str, Any]:
        """Charge les préférences utilisateur.

        Returns:
            Dictionnaire des préférences ou dict vide si erreur
        """
        try:
            if not self.preferences_file.exists():
                return {}

            with open(self.preferences_file, encoding="utf-8") as f:
                preferences = json.load(f)

            return preferences
        except Exception as e:
            logger.warning(f"⚠️ Erreur chargement préférences: {e}")
            return {}

    def get_preference(self, key: str, default: Any = None) -> Any:
        """Récupère une préférence spécifique.

        Args:
            key: Clé de la préférence
            default: Valeur par défaut si préférence non trouvée

        Returns:
            Valeur de la préférence ou default
        """
        preferences = self.load_preferences()
        if key in preferences:
            return preferences[key].get("value", default)
        return default

    def remember_learning(self, pattern: str, response: str) -> bool:
        """Sauvegarde un apprentissage (pattern détecté).

        Exemple : "Quand je dis 'salut', BBIA me reconnaît"

        Args:
            pattern: Pattern détecté (ex: "user_says_salut")
            response: Réponse/action associée (ex: "recognize_user")

        Returns:
            True si sauvegarde réussie
        """
        try:
            # Charger apprentissages existants
            learnings = self.load_learnings()

            # Ajouter/modifier apprentissage
            learnings[pattern] = {
                "response": response,
                "last_updated": datetime.now().isoformat(),
                "count": learnings.get(pattern, {}).get("count", 0) + 1,
            }

            # Sauvegarder
            with open(self.learnings_file, "w", encoding="utf-8") as f:
                json.dump(learnings, f, indent=2, ensure_ascii=False)

            logger.debug(f"💾 Apprentissage sauvegardé: {pattern} → {response}")
            return True
        except Exception as e:
            logger.error(f"❌ Erreur sauvegarde apprentissage: {e}")
            return False

    def load_learnings(self) -> dict[str, Any]:
        """Charge les apprentissages.

        Returns:
            Dictionnaire des apprentissages ou dict vide si erreur
        """
        try:
            if not self.learnings_file.exists():
                return {}

            with open(self.learnings_file, encoding="utf-8") as f:
                learnings = json.load(f)

            return learnings
        except Exception as e:
            logger.warning(f"⚠️ Erreur chargement apprentissages: {e}")
            return {}

    def get_learning(self, pattern: str) -> str | None:
        """Récupère un apprentissage spécifique.

        Args:
            pattern: Pattern à rechercher

        Returns:
            Réponse associée ou None si non trouvé
        """
        learnings = self.load_learnings()
        if pattern in learnings:
            return learnings[pattern].get("response")
        return None

    def clear_memory(self) -> bool:
        """Efface toute la mémoire (conversation + préférences + apprentissages).

        Returns:
            True si effacement réussi
        """
        try:
            for file in [
                self.conversation_file,
                self.preferences_file,
                self.learnings_file,
            ]:
                if file.exists():
                    file.unlink()

            logger.info("💾 Mémoire effacée")
            return True
        except Exception as e:
            logger.error(f"❌ Erreur effacement mémoire: {e}")
            return False


# Fonction utilitaire pour intégration avec BBIAHuggingFace
def load_conversation_from_memory(
    memory_dir: str = "bbia_memory",
) -> list[dict[str, Any]]:
    """Charge la conversation depuis la mémoire persistante.

    Args:
        memory_dir: Répertoire mémoire

    Returns:
        Liste des conversations
    """
    memory = BBIAMemory(memory_dir=memory_dir)
    return memory.load_conversation()


def save_conversation_to_memory(
    conversation_history: list[dict[str, Any]], memory_dir: str = "bbia_memory"
) -> bool:
    """Sauvegarde la conversation dans la mémoire persistante.

    Args:
        conversation_history: Historique conversation
        memory_dir: Répertoire mémoire

    Returns:
        True si sauvegarde réussie
    """
    memory = BBIAMemory(memory_dir=memory_dir)
    return memory.save_conversation(conversation_history)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    # Test rapide
    memory = BBIAMemory()
    memory.remember_preference("voix_preferee", "aurelie")
    memory.remember_learning("user_says_salut", "recognize_user")
    print(f"Préférence voix: {memory.get_preference('voix_preferee')}")
    print(f"Apprentissage: {memory.get_learning('user_says_salut')}")
