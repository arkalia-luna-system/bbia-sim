#!/usr/bin/env python3
"""
Tests pour les 4 vertical slices BBIA
Tests headless pour validation automatique
"""

import os
import subprocess
import sys
import time

import pytest


@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (subprocess, lance scripts réels)
class TestVerticalSlices:
    """Tests pour les 4 vertical slices BBIA."""

    @pytest.fixture
    def demo_scripts(self):
        """Retourne les chemins des scripts de démo."""
        return {
            "emotion": "examples/demo_emotion_ok.py",
            "voice": "examples/demo_voice_ok.py",
            "vision": "examples/demo_vision_ok.py",
            "behavior": "examples/demo_behavior_ok.py",
        }

    def test_demo_emotion_headless(self, demo_scripts):
        """Test de la démo émotion en mode headless."""
        # Skip en CI car trop lent (subprocess lourd)
        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (trop lent avec subprocess)")

        script = demo_scripts["emotion"]

        # Test avec une seule émotion pour éviter timeout
        emotions = ["happy"]

        for emotion in emotions:
            result = subprocess.run(
                [
                    sys.executable,
                    script,
                    "--emotion",
                    emotion,
                    "--intensity",
                    "0.5",
                    "--duration",
                    "1",
                    "--headless",
                    "--joint",
                    "yaw_body",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )

            assert (
                result.returncode == 0
            ), f"Erreur avec émotion {emotion}: {result.stderr}"
            assert (
                "Animation terminée" in result.stdout
                or "Animation headless terminée" in result.stdout
            )
            assert f"• Émotion : {emotion}" in result.stdout

    def test_demo_voice_headless(self, demo_scripts):
        """Test de la démo voix en mode headless."""
        # Skip en CI car trop lent (subprocess lourd)
        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (trop lent avec subprocess)")

        script = demo_scripts["voice"]

        # Test avec une seule commande pour éviter timeout
        commands = ["regarde-moi"]

        for command in commands:
            result = subprocess.run(
                [
                    sys.executable,
                    script,
                    "--command",
                    command,
                    "--duration",
                    "1",
                    "--headless",
                    "--joint",
                    "yaw_body",
                    "--no-sound",  # Désactiver audio pour éviter threads audio
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )

            assert (
                result.returncode == 0
            ), f"Erreur avec commande {command}: {result.stderr}"
            assert (
                "Animation terminée" in result.stdout
                or "Animation headless terminée" in result.stdout
            )
            assert f"• Commande : '{command}'" in result.stdout

    def test_demo_vision_headless(self, demo_scripts):
        """Test de la démo vision en mode headless."""
        # Skip en CI car trop lent (subprocess lourd)
        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (trop lent avec subprocess)")

        script = demo_scripts["vision"]

        result = subprocess.run(
            [
                sys.executable,
                script,
                "--duration",
                "1",
                "--headless",
                "--joint",
                "yaw_body",
                "--target-speed",
                "0.01",
                "--tracking-gain",
                "0.3",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

        assert result.returncode == 0, f"Erreur démo vision: {result.stderr}"
        assert "Animation headless terminée" in result.stdout
        assert "Cible virtuelle" in result.stdout
        assert "BBIA Vision initialisé" in result.stdout

    def test_demo_behavior_headless(self, demo_scripts):
        """Test de la démo comportement en mode headless."""
        # Skip en CI car trop lent (subprocess lourd)
        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (trop lent avec subprocess)")

        script = demo_scripts["behavior"]

        # Test avec un seul comportement pour éviter timeout
        behaviors = ["wake_up"]

        for behavior in behaviors:
            result = subprocess.run(
                [
                    sys.executable,
                    script,
                    "--behavior",
                    behavior,
                    "--duration",
                    "1",
                    "--headless",
                    "--joint",
                    "yaw_body",
                    "--intensity",
                    "0.8",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )

            assert (
                result.returncode == 0
            ), f"Erreur avec comportement {behavior}: {result.stderr}"
            assert (
                "Animation terminée" in result.stdout
                or "Animation headless terminée" in result.stdout
            )
            assert f"• Comportement : {behavior}" in result.stdout

    def test_demo_performance(self, demo_scripts):
        """Test de performance des démos."""
        # Skip en CI car trop lent (subprocess lourd)
        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (trop lent avec subprocess)")

        script = demo_scripts["emotion"]

        start_time = time.time()
        result = subprocess.run(
            [
                sys.executable,
                script,
                "--emotion",
                "happy",
                "--intensity",
                "0.5",
                "--duration",
                "1",
                "--headless",
                "--joint",
                "yaw_body",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

        execution_time = time.time() - start_time

        assert result.returncode == 0
        assert execution_time < 10, f"Performance trop lente: {execution_time:.2f}s"

    def test_demo_error_handling(self, demo_scripts):
        """Test de gestion d'erreurs des démos."""
        # Skip en CI car peut être lent avec subprocess
        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (peut être lent avec subprocess)")

        script = demo_scripts["emotion"]

        # Test avec émotion invalide
        result = subprocess.run(
            [
                sys.executable,
                script,
                "--emotion",
                "invalid_emotion",
                "--duration",
                "1",
                "--headless",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )

        assert result.returncode == 1
        assert "non supportée" in result.stdout

    def test_demo_joint_validation(self, demo_scripts):
        """Test de validation des joints."""
        # Skip en CI car peut être lent avec subprocess
        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (peut être lent avec subprocess)")

        script = demo_scripts["emotion"]

        # Test avec joint invalide
        result = subprocess.run(
            [
                sys.executable,
                script,
                "--emotion",
                "happy",
                "--duration",
                "1",
                "--headless",
                "--joint",
                "invalid_joint",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )

        assert result.returncode == 1
        assert "introuvable" in result.stdout

    def test_demo_intensity_validation(self, demo_scripts):
        """Test de validation de l'intensité."""
        # Skip en CI car peut être lent avec subprocess
        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (peut être lent avec subprocess)")

        script = demo_scripts["emotion"]

        # Test avec intensité invalide
        result = subprocess.run(
            [
                sys.executable,
                script,
                "--emotion",
                "happy",
                "--intensity",
                "2.5",  # Trop élevé
                "--duration",
                "1",
                "--headless",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )

        assert result.returncode == 1
        assert "Intensité doit être entre 0.0 et 1.0" in result.stdout

    def test_all_demos_smoke(self, demo_scripts):
        """Test smoke de toutes les démos."""
        # Skip en CI car trop lent (subprocess lourd)
        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (trop lent avec subprocess)")

        for demo_name, script in demo_scripts.items():
            result = subprocess.run(
                [sys.executable, script, "--duration", "1", "--headless"],
                capture_output=True,
                text=True,
                timeout=10,
            )

            assert (
                result.returncode == 0
            ), f"Smoke test échoué pour {demo_name}: {result.stderr}"
            assert "terminée avec succès" in result.stdout
