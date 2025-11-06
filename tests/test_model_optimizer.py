#!/usr/bin/env python3
"""Tests pour model_optimizer.py - Cache et lazy loading de modèles."""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

# S'assurer que src est dans le path pour coverage
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# Importer le module directement - coverage doit le détecter
# IMPORTANT: Import direct (pas dans try/except) pour que coverage le détecte
import bbia_sim.model_optimizer  # noqa: F401
from bbia_sim.model_optimizer import (
    clear_model_cache,
    get_cache_size,
    get_cached_model,
    lazy_load_model,
)


class TestModelOptimizer:
    """Tests pour le module model_optimizer."""

    def setup_method(self):
        """Nettoyer le cache avant chaque test."""
        clear_model_cache()

    def test_get_cached_model_first_load(self):
        """Test chargement initial d'un modèle (pas de cache)."""
        mock_loader = MagicMock(return_value="model_data")
        result = get_cached_model("test_model", mock_loader, "arg1", kwarg1="value1")

        assert result == "model_data"
        mock_loader.assert_called_once_with("arg1", kwarg1="value1")
        assert get_cache_size() == 1

    def test_get_cached_model_from_cache(self):
        """Test récupération d'un modèle depuis le cache."""
        mock_loader = MagicMock(return_value="model_data")
        # Premier chargement
        result1 = get_cached_model("test_model", mock_loader)
        assert result1 == "model_data"
        assert mock_loader.call_count == 1

        # Deuxième chargement (depuis cache)
        result2 = get_cached_model("test_model", mock_loader)
        assert result2 == "model_data"
        assert mock_loader.call_count == 1  # Pas de nouveau chargement
        assert get_cache_size() == 1

    def test_get_cached_model_multiple_models(self):
        """Test cache avec plusieurs modèles."""
        loader1 = MagicMock(return_value="model1")
        loader2 = MagicMock(return_value="model2")

        result1 = get_cached_model("model1", loader1)
        result2 = get_cached_model("model2", loader2)

        assert result1 == "model1"
        assert result2 == "model2"
        assert get_cache_size() == 2

    def test_clear_model_cache(self):
        """Test effacement du cache."""
        mock_loader = MagicMock(return_value="model_data")
        get_cached_model("test_model", mock_loader)
        assert get_cache_size() == 1

        clear_model_cache()
        assert get_cache_size() == 0

    def test_get_cache_size(self):
        """Test récupération de la taille du cache."""
        assert get_cache_size() == 0

        mock_loader = MagicMock(return_value="model_data")
        get_cached_model("model1", mock_loader)
        assert get_cache_size() == 1

        get_cached_model("model2", mock_loader)
        assert get_cache_size() == 2

    def test_lazy_load_model(self):
        """Test lazy loader (retourne fonction qui charge à la demande)."""
        mock_loader = MagicMock(return_value="lazy_model")

        lazy_func = lazy_load_model("lazy_model", mock_loader)

        # La fonction n'est pas encore appelée
        assert mock_loader.call_count == 0

        # Appel de la fonction lazy
        result = lazy_func("arg1", kwarg1="value1")

        assert result == "lazy_model"
        mock_loader.assert_called_once_with("arg1", kwarg1="value1")

    def test_lazy_load_model_cached(self):
        """Test lazy loader avec cache (deux appels = un seul chargement)."""
        mock_loader = MagicMock(return_value="lazy_model")

        lazy_func = lazy_load_model("lazy_model", mock_loader)

        # Premier appel
        result1 = lazy_func()
        assert result1 == "lazy_model"
        assert mock_loader.call_count == 1

        # Deuxième appel (depuis cache)
        result2 = lazy_func()
        assert result2 == "lazy_model"
        assert mock_loader.call_count == 1  # Pas de nouveau chargement

    @patch("bbia_sim.model_optimizer.logger")
    def test_get_cached_model_logging(self, mock_logger):
        """Test logging lors du chargement d'un modèle."""
        mock_loader = MagicMock(return_value="model_data")

        # Premier chargement (info)
        get_cached_model("test_model", mock_loader)
        assert mock_logger.info.call_count >= 1

        # Deuxième chargement depuis cache (debug)
        get_cached_model("test_model", mock_loader)
        assert mock_logger.debug.call_count >= 1

    @patch("bbia_sim.model_optimizer.logger")
    def test_clear_model_cache_logging(self, mock_logger):
        """Test logging lors de l'effacement du cache."""
        clear_model_cache()
        mock_logger.info.assert_called_with("🧹 Cache modèles effacé")
