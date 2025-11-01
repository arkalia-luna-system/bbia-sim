#!/usr/bin/env python3
"""
Tests pour bbia_memory.py - Amélioration coverage 0% → 70%+
"""

import json
import tempfile
from pathlib import Path
from unittest.mock import patch

import pytest

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
            result = save_conversation_to_memory(
                conversation, memory_dir=str(tmpdir)
            )

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

    @patch("builtins.open", side_effect=PermissionError("Permission denied"))
    def test_save_conversation_permission_error(self, mock_open):
        """Test sauvegarde erreur permissions."""
        conversation = [{"user": "Test", "bbia": "Response"}]

        result = save_conversation_to_memory(
            conversation, memory_file="/fake/path/memory.json"
        )

        # Devrait gérer erreur gracieusement
        assert result is False or result is None

    @patch("json.dump", side_effect=Exception("JSON error"))
    def test_save_conversation_json_error(self, mock_json):
        """Test sauvegarde erreur JSON."""
        conversation = [{"user": "Test", "bbia": "Response"}]

        with tempfile.TemporaryDirectory() as tmpdir:
            memory_file = Path(tmpdir) / "bbia_memory.json"

            result = save_conversation_to_memory(conversation, memory_file=str(memory_file))

            # Devrait gérer erreur gracieusement
            assert result is False or result is None

