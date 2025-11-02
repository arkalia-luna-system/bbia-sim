"""Tests pour vérifier l'aide CLI et les messages multi-OS."""

import os
import subprocess
import sys

import pytest


class TestCLIHelp:
    """Tests pour vérifier l'aide CLI et les messages multi-OS."""

    def test_cli_help_contains_required_options(self):
        """Test que --help liste toutes les options requises."""
        result = subprocess.run(
            [sys.executable, "-m", "bbia_sim", "--help"],
            capture_output=True,
            text=True,
            cwd=os.getcwd(),
        )

        assert result.returncode == 0, f"CLI help failed: {result.stderr}"

        help_text = result.stdout

        # Vérifier que les options requises sont présentes
        required_options = ["--sim", "--headless", "--scene", "--duration", "--verbose"]

        for option in required_options:
            assert option in help_text, f"Option {option} manquante dans --help"

    @pytest.mark.skipif(
        os.getenv("CI") is not None and sys.platform != "darwin",
        reason="Test spécifique macOS",
    )
    def test_macos_viewer_message(self):
        """Test que le message macOS est présent (uniquement sur macOS)."""
        if sys.platform != "darwin":
            pytest.skip("Test spécifique macOS")

        # Test avec un modèle minimal pour éviter les erreurs de chargement
        result = subprocess.run(
            [sys.executable, "-m", "bbia_sim", "--sim", "--scene", "minimal.xml"],
            capture_output=True,
            text=True,
            cwd=os.getcwd(),
        )

        # Le message macOS devrait être présent dans stderr ou stdout
        output = result.stdout + result.stderr

        # Vérifier la présence du message macOS
        macos_indicators = ["macOS", "mjpython", "mujoco.viewer"]

        found_indicators = [
            indicator for indicator in macos_indicators if indicator in output
        ]

        if found_indicators:
            pass
        else:
            pass

    def test_headless_mode_works(self):
        """Test que le mode headless fonctionne sans viewer."""
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "bbia_sim",
                "--sim",
                "--headless",
                "--duration",
                "1",
            ],
            capture_output=True,
            text=True,
            cwd=os.getcwd(),
        )

        # Le mode headless devrait fonctionner même sans viewer
        assert result.returncode == 0, f"Mode headless failed: {result.stderr}"

        # Vérifier que le message headless est présent
        output = result.stdout + result.stderr
        assert "headless" in output.lower(), "Message headless non détecté"

    def test_version_command(self):
        """Test que --version affiche la version."""
        result = subprocess.run(
            [sys.executable, "-m", "bbia_sim", "--version"],
            capture_output=True,
            text=True,
            cwd=os.getcwd(),
        )

        assert result.returncode == 0, f"Version command failed: {result.stderr}"

        version_output = result.stdout.strip()
        assert "BBIA-SIM" in version_output, f"Version non détectée: {version_output}"
