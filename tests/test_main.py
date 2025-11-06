"""Tests unitaires pour le module __main__."""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

# S'assurer que src est dans le path pour coverage
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# Importer le module directement - coverage doit le détecter
# IMPORTANT: Import direct (pas dans try/except) pour que coverage le détecte
import bbia_sim.__main__  # noqa: F401
from bbia_sim import __main__


def test_setup_logging():
    """Test configuration logging."""
    with patch("bbia_sim.__main__.logging.basicConfig") as mock_config:
        __main__.setup_logging(verbose=True)
        mock_config.assert_called_once()


def test_setup_logging_not_verbose():
    """Test configuration logging sans verbose."""
    with patch("bbia_sim.__main__.logging.basicConfig") as mock_config:
        __main__.setup_logging(verbose=False)
        mock_config.assert_called_once()


def test_run_awake_sequence():
    """Test séquence awake."""
    with patch("bbia_sim.bbia_awake.start_bbia_sim") as mock_start:
        __main__.run_awake_sequence()
        mock_start.assert_called_once()


def test_run_voice_synthesis():
    """Test synthèse vocale."""
    with patch("bbia_sim.bbia_voice.dire_texte") as mock_dire:
        __main__.run_voice_synthesis("Hello")
        mock_dire.assert_called_once_with("Hello")


def test_run_voice_recognition():
    """Test reconnaissance vocale."""
    with patch("bbia_sim.bbia_voice.reconnaitre_parole") as mock_reconnaitre:
        mock_reconnaitre.return_value = "Hello world"
        __main__.run_voice_recognition()
        mock_reconnaitre.assert_called_once_with(duree=5)


@patch("bbia_sim.__main__.sys.argv", ["bbia_sim", "--awake"])
@patch("bbia_sim.__main__.run_awake_sequence")
@patch("bbia_sim.__main__.setup_logging")
def test_main_awake_option(mock_setup, mock_awake):
    """Test main() avec option --awake."""
    with patch("bbia_sim.__main__.sys.exit"):
        with patch("bbia_sim.bbia_awake.start_bbia_sim"):
            __main__.main()
            mock_setup.assert_called_once()
            mock_awake.assert_called_once()


@patch("bbia_sim.__main__.sys.argv", ["bbia_sim", "--voice", "Hello"])
@patch("bbia_sim.__main__.run_voice_synthesis")
@patch("bbia_sim.__main__.setup_logging")
def test_main_voice_option(mock_setup, mock_voice):
    """Test main() avec option --voice."""
    with patch("bbia_sim.__main__.sys.exit"):
        with patch("bbia_sim.bbia_voice.dire_texte"):
            __main__.main()
            mock_setup.assert_called_once()
            mock_voice.assert_called_once_with("Hello")


@patch("bbia_sim.__main__.sys.argv", ["bbia_sim", "--listen"])
@patch("bbia_sim.__main__.run_voice_recognition")
@patch("bbia_sim.__main__.setup_logging")
def test_main_listen_option(mock_setup, mock_listen):
    """Test main() avec option --listen."""
    with patch("bbia_sim.__main__.sys.exit"):
        with patch("bbia_sim.bbia_voice.reconnaitre_parole", return_value="test"):
            __main__.main()
            mock_setup.assert_called_once()
            mock_listen.assert_called_once()


@patch("bbia_sim.__main__.sys.argv", ["bbia_sim"])
@patch("bbia_sim.__main__.argparse.ArgumentParser")
def test_main_no_options(mock_parser_class):
    """Test main() sans options (affiche l'aide)."""
    mock_parser = MagicMock()
    mock_parser.parse_args.return_value = MagicMock(
        sim=False, awake=False, voice=None, listen=False, doctor=False, verbose=False
    )
    mock_parser_class.return_value = mock_parser

    with patch("bbia_sim.__main__.sys.exit"):
        __main__.main()
        mock_parser.print_help.assert_called_once()


@patch("bbia_sim.__main__.sys.argv", ["bbia_sim", "--sim"])
@patch("bbia_sim.__main__.run_simulation")
@patch("bbia_sim.__main__.setup_logging")
def test_main_sim_option(mock_setup, mock_sim):
    """Test main() avec option --sim."""
    mock_args = MagicMock()
    mock_args.sim = True
    mock_args.awake = False
    mock_args.voice = None
    mock_args.listen = False
    mock_args.doctor = False
    mock_args.verbose = False

    with patch("bbia_sim.__main__.argparse.ArgumentParser") as mock_parser_class:
        mock_parser = MagicMock()
        mock_parser.parse_args.return_value = mock_args
        mock_parser_class.return_value = mock_parser

        with patch("bbia_sim.__main__.sys.exit"):
            __main__.main()
            mock_setup.assert_called_once()
            mock_sim.assert_called_once()


@patch("bbia_sim.__main__.sys.argv", ["bbia_sim", "--doctor"])
@patch("bbia_sim.__main__.run_doctor")
@patch("bbia_sim.__main__.setup_logging")
def test_main_doctor_option(mock_setup, mock_doctor):
    """Test main() avec option --doctor."""
    with patch("bbia_sim.__main__.sys.exit"):
        __main__.main()
        mock_setup.assert_called_once()
        mock_doctor.assert_called_once()


@patch("bbia_sim.__main__.sys.argv", ["bbia_sim", "--awake"])
@patch("bbia_sim.__main__.run_awake_sequence")
@patch("bbia_sim.__main__.setup_logging")
@patch("bbia_sim.__main__.logger")
def test_main_keyboard_interrupt(mock_logger, mock_setup, mock_awake):
    """Test main() avec KeyboardInterrupt."""
    mock_awake.side_effect = KeyboardInterrupt()

    with patch("bbia_sim.__main__.sys.exit") as mock_exit:
        __main__.main()
        mock_logger.info.assert_called_with("Arrêt demandé par l'utilisateur")
        mock_exit.assert_called_with(0)


@patch("bbia_sim.__main__.sys.argv", ["bbia_sim", "--awake"])
@patch("bbia_sim.__main__.run_awake_sequence")
@patch("bbia_sim.__main__.setup_logging")
@patch("bbia_sim.__main__.logger")
def test_main_exception(mock_logger, mock_setup, mock_awake):
    """Test main() avec exception."""
    mock_awake.side_effect = Exception("Test error")

    with patch("bbia_sim.__main__.sys.exit") as mock_exit:
        __main__.main()
        mock_logger.error.assert_called()
        mock_exit.assert_called_with(1)
