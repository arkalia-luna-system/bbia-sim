#!/usr/bin/env python3
"""
🧪 TESTS CACHE ASSETS STL
Tests pour garantir le bon fonctionnement du cache lazy pour les fichiers STL.
"""

import sys
from pathlib import Path
from unittest.mock import patch

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.stl_asset_cache import (
    clear_stl_cache,
    get_cached_stl,
    get_stl_cache_stats,
    set_stl_cache_max_size,
)


class TestSTLAssetCache:
    """Tests pour le cache d'assets STL."""

    @pytest.fixture(autouse=True)
    def setup_and_teardown(self):
        """Nettoie le cache avant et après chaque test."""
        clear_stl_cache()
        set_stl_cache_max_size(max_files=20, max_size_mb=50)
        yield
        clear_stl_cache()
        set_stl_cache_max_size(max_files=20, max_size_mb=50)

    def test_cache_hit_same_file(self, tmp_path):
        """Test que le même fichier STL est retourné depuis le cache."""
        stl_file = tmp_path / "test.stl"
        stl_file.write_bytes(b"fake stl content")

        # Premier chargement
        content1 = get_cached_stl(stl_file)
        assert content1 == b"fake stl content"

        # Deuxième chargement (devrait venir du cache)
        with patch("builtins.open", wraps=open) as mock_open:
            content2 = get_cached_stl(stl_file)
            assert content2 == content1
            # Le fichier ne devrait être ouvert qu'une fois (premier chargement)
            # Le deuxième appel devrait utiliser le cache
            # Note: Le cache est vérifié avant d'ouvrir le fichier
            assert mock_open.call_count == 0  # Cache hit, pas d'ouverture

        print("✅ Cache hit fonctionne correctement.")

    def test_cache_miss_different_files(self, tmp_path):
        """Test que différents fichiers STL sont chargés et mis en cache."""
        stl_file1 = tmp_path / "test1.stl"
        stl_file2 = tmp_path / "test2.stl"
        stl_file1.write_bytes(b"content1")
        stl_file2.write_bytes(b"content2")

        content1 = get_cached_stl(stl_file1)
        content2 = get_cached_stl(stl_file2)

        assert content1 == b"content1"
        assert content2 == b"content2"
        assert content1 != content2

        stats = get_stl_cache_stats()
        assert stats["file_count"] == 2

        print("✅ Cache miss fonctionne correctement.")

    def test_cache_lru_eviction(self, tmp_path):
        """Test que l'éviction LRU fonctionne lorsque le cache est plein."""
        set_stl_cache_max_size(max_files=2, max_size_mb=50)

        stl_file1 = tmp_path / "test1.stl"
        stl_file2 = tmp_path / "test2.stl"
        stl_file3 = tmp_path / "test3.stl"
        stl_file1.write_bytes(b"content1")
        stl_file2.write_bytes(b"content2")
        stl_file3.write_bytes(b"content3")

        # Charger 2 fichiers (cache plein)
        get_cached_stl(stl_file1)
        get_cached_stl(stl_file2)
        assert get_stl_cache_stats()["file_count"] == 2

        # Charger un 3ème fichier (devrait évincer le 1er)
        get_cached_stl(stl_file3)
        stats = get_stl_cache_stats()
        assert stats["file_count"] == 2

        # Le fichier 1 devrait être évincé, donc rechargé depuis le disque
        with patch("builtins.open", wraps=open) as mock_open:
            get_cached_stl(stl_file1)
            # Devrait être rechargé depuis le disque (évincé)
            assert mock_open.call_count > 0

        print("✅ Éviction LRU fonctionne correctement.")

    def test_cache_size_limit(self, tmp_path):
        """Test que le cache respecte la limite de taille."""
        # Limite très basse (1 MB)
        set_stl_cache_max_size(max_files=100, max_size_mb=1)

        stl_file1 = tmp_path / "test1.stl"
        stl_file2 = tmp_path / "test2.stl"
        # Créer des fichiers de 600 KB chacun
        stl_file1.write_bytes(b"x" * (600 * 1024))
        stl_file2.write_bytes(b"x" * (600 * 1024))

        # Charger le premier fichier (600 KB < 1 MB, OK)
        get_cached_stl(stl_file1)
        stats = get_stl_cache_stats()
        assert stats["file_count"] == 1

        # Charger le deuxième fichier (1200 KB > 1 MB, devrait évincer le 1er)
        get_cached_stl(stl_file2)
        stats = get_stl_cache_stats()
        # Le cache devrait contenir seulement le 2ème fichier
        assert stats["file_count"] == 1
        assert stats["total_size_mb"] < 1.0

        print("✅ Limite de taille cache fonctionne correctement.")

    def test_cache_stats(self, tmp_path):
        """Test que les statistiques du cache sont correctes."""
        stl_file = tmp_path / "test.stl"
        stl_file.write_bytes(b"test content")

        stats_before = get_stl_cache_stats()
        assert stats_before["file_count"] == 0

        get_cached_stl(stl_file)

        stats_after = get_stl_cache_stats()
        assert stats_after["file_count"] == 1
        assert stats_after["total_size_bytes"] > 0
        assert len(stats_after["cached_files"]) == 1

        print("✅ Statistiques cache fonctionnent correctement.")

    def test_clear_cache(self, tmp_path):
        """Test que le cache peut être vidé."""
        stl_file = tmp_path / "test.stl"
        stl_file.write_bytes(b"test content")

        get_cached_stl(stl_file)
        assert get_stl_cache_stats()["file_count"] == 1

        clear_stl_cache()
        assert get_stl_cache_stats()["file_count"] == 0

        print("✅ Nettoyage cache fonctionne correctement.")

    def test_set_cache_max_size(self, tmp_path):
        """Test que la taille maximale du cache peut être modifiée."""
        original_stats = get_stl_cache_stats()
        original_max_files = original_stats["max_file_count"]

        set_stl_cache_max_size(max_files=10, max_size_mb=20)
        stats = get_stl_cache_stats()
        assert stats["max_file_count"] == 10
        assert stats["max_size_mb"] == 20.0

        set_stl_cache_max_size(max_files=original_max_files, max_size_mb=50)
        stats = get_stl_cache_stats()
        assert stats["max_file_count"] == original_max_files

        print("✅ Modification taille cache fonctionne correctement.")

    def test_cache_file_not_found(self, tmp_path):
        """Test que FileNotFoundError est levée si le fichier n'existe pas."""
        non_existent_file = tmp_path / "nonexistent.stl"

        with pytest.raises(FileNotFoundError):
            get_cached_stl(non_existent_file)

        print("✅ Gestion erreur fichier introuvable fonctionne.")

    def test_cache_lru_recently_used(self, tmp_path):
        """Test que le fichier récemment utilisé n'est pas évincé."""
        set_stl_cache_max_size(max_files=2, max_size_mb=50)

        stl_file1 = tmp_path / "test1.stl"
        stl_file2 = tmp_path / "test2.stl"
        stl_file3 = tmp_path / "test3.stl"
        stl_file1.write_bytes(b"content1")
        stl_file2.write_bytes(b"content2")
        stl_file3.write_bytes(b"content3")

        # Charger fichier 1
        content1 = get_cached_stl(stl_file1)
        # Charger fichier 2
        get_cached_stl(stl_file2)
        # Recharger fichier 1 (devient récemment utilisé)
        content1_again = get_cached_stl(stl_file1)
        assert content1_again == content1

        # Charger fichier 3 (devrait évincer fichier 2, pas fichier 1)
        get_cached_stl(stl_file3)

        # Fichier 1 devrait toujours être en cache
        with patch("builtins.open", wraps=open) as mock_open:
            get_cached_stl(stl_file1)
            # Devrait venir du cache (pas d'ouverture)
            assert mock_open.call_count == 0

        print("✅ LRU récemment utilisé fonctionne correctement.")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

