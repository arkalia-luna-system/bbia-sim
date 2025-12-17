#!/usr/bin/env python3
"""
🧪 TESTS CACHE MODÈLES MUJOCO
Tests pour garantir le bon fonctionnement du cache LRU pour modèles MuJoCo.
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.mujoco_model_cache import (
    clear_mujoco_cache,
    get_cache_stats,
    get_cached_mujoco_model,
    set_cache_max_size,
)


class TestMujocoModelCache:
    """Tests pour le cache LRU de modèles MuJoCo."""

    def setup_method(self) -> None:
        """Nettoyer le cache avant chaque test."""
        clear_mujoco_cache()

    def teardown_method(self) -> None:
        """Nettoyer le cache après chaque test."""
        clear_mujoco_cache()

    def test_cache_hit_same_model(self):
        """Test que le cache retourne le même modèle pour le même chemin."""
        model_path = "src/bbia_sim/sim/models/reachy_mini.xml"

        with patch("pathlib.Path.exists", return_value=True):
            with patch("mujoco.MjModel.from_xml_path") as mock_load:
                mock_model = MagicMock()
                mock_load.return_value = mock_model

                # Premier chargement
                model1 = get_cached_mujoco_model(model_path)
                assert model1 is mock_model
                assert mock_load.call_count == 1

                # Deuxième chargement (devrait être depuis cache)
                model2 = get_cached_mujoco_model(model_path)
                assert model2 is mock_model
                assert model2 is model1  # Même instance
                assert mock_load.call_count == 1  # Pas de rechargement

        print("✅ Cache hit fonctionne correctement.")

    def test_cache_miss_different_models(self):
        """Test que différents modèles sont chargés séparément."""
        model_path1 = "src/bbia_sim/sim/models/reachy_mini.xml"
        model_path2 = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"

        with patch("pathlib.Path.exists", return_value=True):
            with patch("mujoco.MjModel.from_xml_path") as mock_load:
                mock_model1 = MagicMock()
                mock_model2 = MagicMock()
                mock_load.side_effect = [mock_model1, mock_model2]

                model1 = get_cached_mujoco_model(model_path1)
                model2 = get_cached_mujoco_model(model_path2)

                assert model1 is mock_model1
                assert model2 is mock_model2
                assert model1 is not model2
                assert mock_load.call_count == 2

        print("✅ Cache miss fonctionne correctement.")

    def test_cache_lru_eviction(self):
        """Test que l'éviction LRU fonctionne quand le cache est plein."""
        # Réduire la taille du cache pour le test
        set_cache_max_size(2)

        model_paths = [
            "src/bbia_sim/sim/models/model1.xml",
            "src/bbia_sim/sim/models/model2.xml",
            "src/bbia_sim/sim/models/model3.xml",
        ]

        with patch("pathlib.Path.exists", return_value=True):
            with patch("mujoco.MjModel.from_xml_path") as mock_load:
                # Créer des mocks uniques pour chaque appel
                mock_model1 = MagicMock()
                mock_model2 = MagicMock()
                mock_model3 = MagicMock()
                mock_model1_reload = MagicMock()
                mock_load.side_effect = [
                    mock_model1,
                    mock_model2,
                    mock_model3,
                    mock_model1_reload,
                ]

                # Charger 2 modèles (cache plein)
                model1 = get_cached_mujoco_model(model_paths[0])
                model2 = get_cached_mujoco_model(model_paths[1])
                assert mock_load.call_count == 2
                assert model1 is mock_model1
                assert model2 is mock_model2

                # Charger un 3ème modèle (devrait évincer le 1er)
                model3 = get_cached_mujoco_model(model_paths[2])
                assert mock_load.call_count == 3
                assert model3 is mock_model3

                # Recharger le 1er modèle (devrait recharger car évincé)
                model1_again = get_cached_mujoco_model(model_paths[0])
                assert mock_load.call_count == 4
                assert model1_again is not model1  # Nouvelle instance
                assert model1_again is mock_model1_reload

                # Le 2ème modèle a été évincé quand on a rechargé le 1er
                # (car le 2ème était le moins récemment utilisé après le chargement du 3)
                # Vérifier que le cache contient maintenant modèle 3 et modèle 1 (pas modèle 2)
                stats = get_cache_stats()
                assert len(stats["cached_models"]) == 2
                # Vérifier que le modèle 2 n'est plus dans le cache
                cached_paths = [Path(p).name for p in stats["cached_models"]]
                assert "model2.xml" not in " ".join(cached_paths)
                assert "model3.xml" in " ".join(cached_paths)
                assert "model1.xml" in " ".join(cached_paths)

        # Restaurer taille par défaut
        set_cache_max_size(5)
        print("✅ Éviction LRU fonctionne correctement.")

    def test_cache_lru_recently_used(self):
        """Test que le modèle récemment utilisé n'est pas évincé."""
        set_cache_max_size(2)

        model_paths = [
            "src/bbia_sim/sim/models/model1.xml",
            "src/bbia_sim/sim/models/model2.xml",
            "src/bbia_sim/sim/models/model3.xml",
        ]

        with patch("pathlib.Path.exists", return_value=True):
            with patch("mujoco.MjModel.from_xml_path") as mock_load:
                mock_models = [MagicMock() for _ in range(3)]
                mock_load.side_effect = mock_models

                # Charger modèle 1
                model1 = get_cached_mujoco_model(model_paths[0])
                # Charger modèle 2
                model2 = get_cached_mujoco_model(model_paths[1])
                assert model2 is mock_models[1]
                # Recharger modèle 1 (devient récemment utilisé)
                model1_again = get_cached_mujoco_model(model_paths[0])
                assert model1_again is model1

                # Charger modèle 3 (devrait évincer modèle 2, pas modèle 1)
                model3 = get_cached_mujoco_model(model_paths[2])
                assert mock_load.call_count == 3
                assert model3 is mock_models[2]

                # Modèle 1 devrait toujours être en cache
                model1_final = get_cached_mujoco_model(model_paths[0])
                assert mock_load.call_count == 3  # Pas de rechargement
                assert model1_final is model1

        set_cache_max_size(5)
        print("✅ LRU récemment utilisé fonctionne correctement.")

    def test_cache_stats(self):
        """Test que les statistiques du cache sont correctes."""
        # Restaurer taille par défaut au cas où un test précédent l'aurait modifiée
        set_cache_max_size(5)
        model_path = "src/bbia_sim/sim/models/reachy_mini.xml"

        with patch("pathlib.Path.exists", return_value=True):
            with patch("mujoco.MjModel.from_xml_path"):
                stats_before = get_cache_stats()
                assert stats_before["size"] == 0
                assert stats_before["max_size"] == 5

                get_cached_mujoco_model(model_path)

                stats_after = get_cache_stats()
                assert stats_after["size"] == 1
                assert stats_after["max_size"] == 5
                assert len(stats_after["cached_models"]) == 1
                assert model_path in stats_after["cached_models"][0]

        print("✅ Statistiques cache fonctionnent correctement.")

    def test_clear_cache(self):
        """Test que le cache peut être vidé."""
        model_path = "src/bbia_sim/sim/models/reachy_mini.xml"

        with patch("pathlib.Path.exists", return_value=True):
            with patch("mujoco.MjModel.from_xml_path"):
                get_cached_mujoco_model(model_path)
                assert get_cache_stats()["size"] == 1

                clear_mujoco_cache()
                assert get_cache_stats()["size"] == 0

        print("✅ Nettoyage cache fonctionne correctement.")

    def test_set_cache_max_size(self):
        """Test que la taille maximale du cache peut être modifiée."""
        original_max = get_cache_stats()["max_size"]

        set_cache_max_size(10)
        assert get_cache_stats()["max_size"] == 10

        set_cache_max_size(original_max)
        assert get_cache_stats()["max_size"] == original_max

        print("✅ Modification taille cache fonctionne correctement.")

    def test_set_cache_max_size_eviction(self):
        """Test que la réduction de la taille du cache évince les modèles."""
        set_cache_max_size(3)

        model_paths = [
            "src/bbia_sim/sim/models/model1.xml",
            "src/bbia_sim/sim/models/model2.xml",
            "src/bbia_sim/sim/models/model3.xml",
        ]

        with patch("pathlib.Path.exists", return_value=True):
            with patch("mujoco.MjModel.from_xml_path") as mock_load:
                mock_models = [MagicMock() for _ in range(3)]
                mock_load.side_effect = mock_models

                # Charger 3 modèles
                for path in model_paths:
                    get_cached_mujoco_model(path)
                assert get_cache_stats()["size"] == 3

                # Réduire la taille à 1
                set_cache_max_size(1)
                assert get_cache_stats()["size"] == 1

                # Le dernier modèle chargé devrait être conservé
                stats = get_cache_stats()
                assert len(stats["cached_models"]) == 1
                assert "model3.xml" in stats["cached_models"][0]

        set_cache_max_size(5)
        print("✅ Éviction lors réduction taille cache fonctionne.")

    def test_cache_file_not_found(self):
        """Test que FileNotFoundError est levée si le fichier n'existe pas."""
        with patch("pathlib.Path.exists", return_value=False):
            with pytest.raises(FileNotFoundError):
                get_cached_mujoco_model("nonexistent.xml")

        print("✅ Gestion erreur fichier introuvable fonctionne.")

    def test_cache_invalid_model_path(self):
        """Test que le cache gère correctement les chemins invalides."""
        with patch("pathlib.Path.exists", return_value=True):
            with patch("mujoco.MjModel.from_xml_path") as mock_load:
                mock_load.side_effect = Exception("Invalid model")

                with pytest.raises(Exception, match="Invalid model"):
                    get_cached_mujoco_model("invalid.xml")

                # Le modèle ne devrait pas être en cache après erreur
                assert get_cache_stats()["size"] == 0

        print("✅ Gestion erreur modèle invalide fonctionne.")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
