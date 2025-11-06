#!/usr/bin/env python3
"""
Tests pour bbia_memory.py - Amélioration coverage 0% → 70%+
"""

import json
import tempfile
from pathlib import Path

import pytest

# IMPORTANT: Import direct (pas dans try/except) pour que coverage le détecte
import bbia_sim.bbia_memory  # noqa: F401

from bbia_sim.bbia_memory import (
    BBIAMemory,
    load_conversation_from_memory,
    save_conversation_to_memory,
)


@pytest.mark.unit
@pytest.mark.fast
class TestBBIAMemory:
    """Tests pour module mémoire persistante."""

    def test_save_conversation_to_memory_success(self):
        """Test sauvegarde conversation réussie."""
        conversation = [
            {
                "user": "Bonjour",
                "bbia": "Salut !",
                "timestamp": "2025-01-31T10:00:00",
            },
            {
                "user": "Comment vas-tu ?",
                "bbia": "Je vais bien, merci !",
                "timestamp": "2025-01-31T10:01:00",
            },
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            # Utiliser memory_dir au lieu de memory_file
            result = save_conversation_to_memory(conversation, memory_dir=str(tmpdir))

            assert result is True

            # Vérifier fichier créé
            memory_file = Path(tmpdir) / "conversation_history.json"
            assert memory_file.exists()

            # Vérifier contenu
            with open(memory_file) as f:
                data = json.load(f)
                assert "history" in data
                assert len(data["history"]) == 2
                assert data["history"][0]["user"] == "Bonjour"

    def test_load_conversation_from_memory_success(self):
        """Test chargement conversation réussi."""
        conversation = [
            {
                "user": "Hello",
                "bbia": "Hi!",
                "timestamp": "2025-01-31T10:00:00",
            },
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            # Créer fichier au bon format
            memory_file = Path(tmpdir) / "conversation_history.json"
            data = {
                "last_updated": "2025-01-31T10:00:00",
                "conversation_count": 1,
                "history": conversation,
            }
            with open(memory_file, "w") as f:
                json.dump(data, f)

            # Charger avec memory_dir
            loaded = load_conversation_from_memory(memory_dir=str(tmpdir))

            assert loaded is not None
            assert len(loaded) == 1
            assert loaded[0]["user"] == "Hello"

    def test_load_conversation_from_memory_not_exists(self):
        """Test chargement fichier inexistant."""
        with tempfile.TemporaryDirectory() as tmpdir:
            loaded = load_conversation_from_memory(memory_dir=str(tmpdir))

            assert loaded == []

    def test_save_conversation_empty_list(self):
        """Test sauvegarde liste vide."""
        with tempfile.TemporaryDirectory() as tmpdir:
            result = save_conversation_to_memory([], memory_dir=str(tmpdir))

            assert result is True

            memory_file = Path(tmpdir) / "conversation_history.json"
            assert memory_file.exists()

            # Vérifier contenu vide
            with open(memory_file) as f:
                data = json.load(f)
                assert data["history"] == []

    def test_load_conversation_corrupted_file(self):
        """Test chargement fichier corrompu."""
        with tempfile.TemporaryDirectory() as tmpdir:
            memory_file = Path(tmpdir) / "conversation_history.json"

            # Créer fichier corrompu
            with open(memory_file, "w") as f:
                f.write("invalid json {")

            # Charger devrait gérer l'erreur
            loaded = load_conversation_from_memory(memory_dir=str(tmpdir))

            # Devrait retourner liste vide ou gérer erreur
            assert isinstance(loaded, list)

    def test_save_conversation_with_sentiment(self):
        """Test sauvegarde avec sentiment."""
        conversation = [
            {
                "user": "Je suis content !",
                "bbia": "Super !",
                "sentiment": {"sentiment": "POSITIVE", "score": 0.9},
                "timestamp": "2025-01-31T10:00:00",
            },
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            result = save_conversation_to_memory(conversation, memory_dir=str(tmpdir))

            assert result is True

            # Vérifier sentiment sauvegardé
            memory_file = Path(tmpdir) / "conversation_history.json"
            with open(memory_file) as f:
                data = json.load(f)
                assert "sentiment" in data["history"][0]
                assert data["history"][0]["sentiment"]["sentiment"] == "POSITIVE"

    def test_load_conversation_preserves_structure(self):
        """Test que structure conversation préservée."""
        conversation = [
            {
                "user": "Test",
                "bbia": "Réponse",
                "timestamp": "2025-01-31T10:00:00",
                "extra_field": "value",
            },
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            save_conversation_to_memory(conversation, memory_dir=str(tmpdir))
            loaded = load_conversation_from_memory(memory_dir=str(tmpdir))

            assert loaded[0]["extra_field"] == "value"

    def test_bbia_memory_init(self):
        """Test initialisation BBIAMemory."""
        with tempfile.TemporaryDirectory() as tmpdir:
            memory = BBIAMemory(memory_dir=str(tmpdir))

            assert memory.memory_dir.exists()
            assert memory.conversation_file == memory.memory_dir / "conversation_history.json"
            assert memory.preferences_file == memory.memory_dir / "preferences.json"
            assert memory.learnings_file == memory.memory_dir / "learnings.json"

    def test_remember_preference(self):
        """Test sauvegarde préférence."""
        with tempfile.TemporaryDirectory() as tmpdir:
            memory = BBIAMemory(memory_dir=str(tmpdir))

            result = memory.remember_preference("voix_preferee", "aurelie")
            assert result is True

            # Vérifier préférence sauvegardée
            value = memory.get_preference("voix_preferee")
            assert value == "aurelie"

    def test_load_preferences_empty(self):
        """Test chargement préférences vide."""
        with tempfile.TemporaryDirectory() as tmpdir:
            memory = BBIAMemory(memory_dir=str(tmpdir))

            prefs = memory.load_preferences()
            assert prefs == {}

    def test_remember_learning(self):
        """Test sauvegarde apprentissage."""
        with tempfile.TemporaryDirectory() as tmpdir:
            memory = BBIAMemory(memory_dir=str(tmpdir))

            result = memory.remember_learning("user_says_salut", "recognize_user")
            assert result is True

            # Vérifier apprentissage sauvegardé
            response = memory.get_learning("user_says_salut")
            assert response == "recognize_user"

    def test_clear_memory(self):
        """Test effacement mémoire."""
        with tempfile.TemporaryDirectory() as tmpdir:
            memory = BBIAMemory(memory_dir=str(tmpdir))

            # Créer quelques fichiers
            memory.remember_preference("test", "value")
            memory.remember_learning("pattern", "response")

            # Effacer
            result = memory.clear_memory()
            assert result is True

            # Vérifier fichiers supprimés
            assert not memory.conversation_file.exists()
            assert not memory.preferences_file.exists()
            assert not memory.learnings_file.exists()
