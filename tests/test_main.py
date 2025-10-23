"""Tests unitaires pour le module __main__."""

from unittest.mock import Mock, patch


def test_setup_logging():
    """Test configuration logging."""
    from src.bbia_sim import __main__

    with patch("src.bbia_sim.__main__.logging.basicConfig") as mock_config:
        __main__.setup_logging(verbose=True)
        mock_config.assert_called_once()


def test_run_awake_sequence():
    """Test séquence awake."""
    from src.bbia_sim import __main__

    with patch("src.bbia_sim.__main__.importlib.import_module") as mock_import:
        mock_module = Mock()
        mock_import.return_value = mock_module

        __main__.run_awake_sequence()

        mock_import.assert_called_once_with("bbia_sim.bbia_awake")


def test_run_voice_synthesis():
    """Test synthèse vocale."""
    from src.bbia_sim import __main__

    with patch("src.bbia_sim.__main__.importlib.import_module") as mock_import:
        mock_module = Mock()
        mock_import.return_value = mock_module

        __main__.run_voice_synthesis("Hello")

        mock_import.assert_called_once_with("bbia_sim.bbia_voice")


def test_run_voice_recognition():
    """Test reconnaissance vocale."""
    from src.bbia_sim import __main__

    with patch("src.bbia_sim.__main__.importlib.import_module") as mock_import:
        mock_module = Mock()
        mock_import.return_value = mock_module

        __main__.run_voice_recognition()

        mock_import.assert_called_once_with("bbia_sim.bbia_voice")