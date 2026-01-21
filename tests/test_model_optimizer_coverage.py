#!/usr/bin/env python3
"""Tests pour le module model_optimizer.py - Couverture complète."""

import pytest

from bbia_sim.model_optimizer import (
    clear_model_cache,
    get_cache_size,
    get_cached_model,
    lazy_load_model,
)


class TestModelOptimizer:
    """Tests pour le cache de modèles."""

    def setup_method(self) -> None:
        """Nettoie le cache avant chaque test."""
        clear_model_cache()

    def teardown_method(self) -> None:
        """Nettoie le cache après chaque test."""
        clear_model_cache()

    def test_get_cached_model_loads_new_model(self) -> None:
        """Test que get_cached_model charge un nouveau modèle."""
        loader_called = [False]

        def mock_loader() -> str:
            loader_called[0] = True
            return "test_model"

        result = get_cached_model("test_key", mock_loader)

        assert result == "test_model"
        assert loader_called[0] is True
        assert get_cache_size() == 1

    def test_get_cached_model_returns_cached(self) -> None:
        """Test que get_cached_model retourne le modèle depuis le cache."""
        load_count = [0]

        def mock_loader() -> str:
            load_count[0] += 1
            return f"model_{load_count[0]}"

        # Premier appel: charge le modèle
        result1 = get_cached_model("test_key", mock_loader)
        assert result1 == "model_1"
        assert load_count[0] == 1

        # Second appel: utilise le cache
        result2 = get_cached_model("test_key", mock_loader)
        assert result2 == "model_1"  # Même modèle
        assert load_count[0] == 1  # Loader pas appelé à nouveau

    def test_get_cached_model_with_args(self) -> None:
        """Test que get_cached_model passe les arguments au loader."""
        received_args: list[tuple] = []
        received_kwargs: list[dict] = []

        def mock_loader(*args, **kwargs) -> str:
            received_args.append(args)
            received_kwargs.append(kwargs)
            return "model_with_args"

        result = get_cached_model(
            "key_args", mock_loader, "arg1", "arg2", kwarg1="val1"
        )

        assert result == "model_with_args"
        assert received_args[-1] == ("arg1", "arg2")
        assert received_kwargs[-1] == {"kwarg1": "val1"}

    def test_cache_lru_eviction(self) -> None:
        """Test que le cache évince les modèles LRU quand plein."""
        # Remplir le cache (max 10 par défaut)
        for i in range(12):
            get_cached_model(f"model_{i}", lambda i=i: f"value_{i}")

        # Le cache doit avoir au maximum 10 éléments
        assert get_cache_size() <= 10

    def test_clear_model_cache(self) -> None:
        """Test que clear_model_cache vide le cache."""
        # Ajouter quelques modèles
        get_cached_model("key1", lambda: "model1")
        get_cached_model("key2", lambda: "model2")
        assert get_cache_size() == 2

        # Vider le cache
        clear_model_cache()
        assert get_cache_size() == 0

    def test_get_cache_size(self) -> None:
        """Test que get_cache_size retourne la bonne taille."""
        assert get_cache_size() == 0

        get_cached_model("key1", lambda: "model1")
        assert get_cache_size() == 1

        get_cached_model("key2", lambda: "model2")
        assert get_cache_size() == 2

    def test_lazy_load_model_returns_function(self) -> None:
        """Test que lazy_load_model retourne une fonction."""
        loader = lazy_load_model("lazy_key", lambda: "lazy_model")

        assert callable(loader)
        assert get_cache_size() == 0  # Pas encore chargé

    def test_lazy_load_model_loads_on_call(self) -> None:
        """Test que lazy_load_model charge à l'appel."""
        load_count = [0]

        def mock_loader() -> str:
            load_count[0] += 1
            return "lazy_loaded"

        lazy_loader = lazy_load_model("lazy_key2", mock_loader)

        # Avant appel: rien chargé
        assert load_count[0] == 0

        # Premier appel: charge
        result1 = lazy_loader()
        assert result1 == "lazy_loaded"
        assert load_count[0] == 1

        # Second appel: depuis cache
        result2 = lazy_loader()
        assert result2 == "lazy_loaded"
        assert load_count[0] == 1  # Pas rechargé
