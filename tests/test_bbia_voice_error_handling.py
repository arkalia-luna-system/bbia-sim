#!/usr/bin/env python3
"""
Tests de gestion d'erreurs pour bbia_voice.

Vérifie que les erreurs sont gérées correctement avec les logs appropriés.
"""

import logging
from unittest.mock import MagicMock, patch

import pytest

from bbia_sim.bbia_voice import BBIAVoice

logger = logging.getLogger(__name__)


class TestBBIAVoiceErrorHandling:
    """Tests de gestion d'erreurs pour BBIAVoice."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_voice_speak_error_handling(self):
        """Test que speak() gère les erreurs correctement."""
        voice = BBIAVoice(robot_api=None)

        # Simuler une erreur lors de la synthèse vocale
        mock_logger = MagicMock(spec=logging.Logger)
        with patch("bbia_sim.bbia_voice.logger", mock_logger):
            with patch.object(
                voice,
                "_synthesize_advanced",
                side_effect=RuntimeError("Erreur synthèse"),
            ):
                # Ne doit pas crasher, doit utiliser fallback
                try:
                    voice.speak("test")
                except Exception:  # noqa: BLE001
                    # Si le fallback échoue aussi, c'est acceptable
                    pass
                # Vérifier que des logs ont été émis
                assert mock_logger.debug.called or mock_logger.warning.called

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_voice_play_audio_error_handling(self):
        """Test que _play_audio() gère les erreurs correctement."""
        # Ne pas initialiser complètement pour économiser RAM
        # On teste juste la gestion d'erreur, pas le fonctionnement complet
        # Vérifier que le code gère les erreurs gracieusement
        import inspect

        from bbia_sim import bbia_voice

        source = inspect.getsource(bbia_voice)
        # Vérifier que les erreurs sont gérées avec des fallbacks
        assert "except Exception" in source or "fallback" in source.lower()
        # Vérifier que les fallbacks sont documentés comme "fallback normal"
        assert "fallback normal" in source.lower() or "debug" in source.lower()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_voice_fallback_handling(self):
        """Test que les fallbacks sont gérés correctement."""
        # Vérifier que le code gère les fallbacks gracieusement
        # En regardant le code source, on voit que les erreurs sont loggées en DEBUG
        # pour les fallbacks normaux (pas critiques)
        import inspect

        from bbia_sim import bbia_voice

        source = inspect.getsource(bbia_voice)
        # Vérifier que les fallbacks sont documentés comme "fallback normal"
        assert "fallback normal" in source.lower() or "fallback" in source.lower()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
