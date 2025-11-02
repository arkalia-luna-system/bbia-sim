"""Tests unitaires pour le module __main__."""

from unittest.mock import patch


def test_setup_logging():
    """Test configuration logging."""
    from src.bbia_sim import __main__

    with patch("src.bbia_sim.__main__.logging.basicConfig") as mock_config:
        __main__.setup_logging(verbose=True)
        mock_config.assert_called_once()


def test_run_awake_sequence():
    """Test séquence awake."""
    from src.bbia_sim import __main__

    with patch("bbia_sim.bbia_awake.start_bbia_sim") as mock_start:
        __main__.run_awake_sequence()
        mock_start.assert_called_once()


def test_run_voice_synthesis():
    """Test synthèse vocale."""
    from src.bbia_sim import __main__

    with patch("bbia_sim.bbia_voice.dire_texte") as mock_dire:
        __main__.run_voice_synthesis("Hello")
        mock_dire.assert_called_once_with("Hello")


def test_run_voice_recognition():
    """Test reconnaissance vocale."""
    from src.bbia_sim import __main__

    with patch("bbia_sim.bbia_voice.reconnaitre_parole") as mock_reconnaitre:
        mock_reconnaitre.return_value = "Hello world"
        __main__.run_voice_recognition()
        mock_reconnaitre.assert_called_once_with(duree=5)
