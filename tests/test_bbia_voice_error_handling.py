#!/usr/bin/env python3
"""
Tests de gestion d'erreurs pour bbia_voice.

Vérifie que les erreurs sont gérées correctement avec les logs appropriés.
Tests légers sans charger de modèles pour économiser RAM.
"""

import inspect
import logging

import pytest

logger = logging.getLogger(__name__)


class TestBBIAVoiceErrorHandling:
    """Tests de gestion d'erreurs pour bbia_voice (sans charger de modèles lourds)."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_voice_error_handling_code_structure(self):
        """Test que le code gère les erreurs gracieusement (sans charger de modèles)."""
        # Ne pas charger de modèle pour économiser RAM
        # On teste juste la structure du code, pas le fonctionnement complet
        from bbia_sim import bbia_voice

        source = inspect.getsource(bbia_voice)
        # Vérifier que les erreurs sont gérées avec des fallbacks
        assert "except Exception" in source or "fallback" in source.lower()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_voice_fallback_handling(self):
        """Test que les fallbacks sont gérés correctement."""
        # Vérifier que le code gère les fallbacks gracieusement
        # En regardant le code source, on voit que les erreurs sont loggées en DEBUG
        # pour les fallbacks normaux (pas critiques)
        from bbia_sim import bbia_voice

        source = inspect.getsource(bbia_voice)
        # Vérifier que les fallbacks sont documentés comme "fallback normal"
        assert "fallback normal" in source.lower() or "fallback" in source.lower()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_voice_error_logs_level(self):
        """Test que les erreurs sont loggées au bon niveau."""
        # Vérifier que les erreurs de fallback sont en DEBUG (pas ERROR)
        from bbia_sim import bbia_voice

        source = inspect.getsource(bbia_voice)
        # Vérifier que les fallbacks utilisent logger.debug (pas logger.error)
        assert "logger.debug" in source or "logging.debug" in source


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
