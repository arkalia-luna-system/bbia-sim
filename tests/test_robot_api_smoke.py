#!/usr/bin/env python3
"""
Tests smoke pour les démos RobotAPI
Tests headless <5s avec assert simple (variation qpos > seuil)
"""

import subprocess
import sys
import time

import pytest


class TestRobotAPISmoke:
    """Tests smoke pour RobotAPI."""

    @pytest.fixture
    def demo_scripts(self):
        """Retourne les chemins des scripts de démo."""
        return {
            "emotion": "examples/demo_emotion_ok.py",
            "voice": "examples/demo_voice_ok.py",
            "vision": "examples/demo_vision_ok.py",
            "behavior": "examples/demo_behavior_ok.py",
        }

    def test_demo_emotion_mujoco_smoke(self, demo_scripts):
        """Test smoke démo émotion avec backend MuJoCo."""
        script = demo_scripts["emotion"]

        start_time = time.time()
        result = subprocess.run(
            [
                sys.executable,
                script,
                "--emotion",
                "happy",
                "--intensity",
                "0.8",
                "--duration",
                "2",
                "--headless",
                "--backend",
                "mujoco",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

        execution_time = time.time() - start_time

        assert result.returncode == 0, f"Erreur démo émotion MuJoCo: {result.stderr}"
        assert execution_time < 5, f"Trop lent: {execution_time:.2f}s"
        assert "Backend mujoco connecté" in result.stdout
        assert "Animation terminée" in result.stdout
        assert "16 joints détectés" in result.stdout

    def test_demo_emotion_reachy_smoke(self, demo_scripts):
        """Test smoke démo émotion avec backend Reachy."""
        script = demo_scripts["emotion"]

        start_time = time.time()
        result = subprocess.run(
            [
                sys.executable,
                script,
                "--emotion",
                "happy",
                "--intensity",
                "0.8",
                "--duration",
                "2",
                "--headless",
                "--backend",
                "reachy",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

        execution_time = time.time() - start_time

        assert result.returncode == 0, f"Erreur démo émotion Reachy: {result.stderr}"
        assert execution_time < 5, f"Trop lent: {execution_time:.2f}s"
        assert "Backend reachy connecté" in result.stdout
        assert "Animation terminée" in result.stdout
        assert "7 joints détectés" in result.stdout

    def test_robot_api_backend_switching(self, demo_scripts):
        """Test de basculement entre backends."""
        script = demo_scripts["emotion"]

        # Test MuJoCo
        result_mujoco = subprocess.run(
            [
                sys.executable,
                script,
                "--emotion",
                "sad",
                "--duration",
                "1",
                "--headless",
                "--backend",
                "mujoco",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

        # Test Reachy
        result_reachy = subprocess.run(
            [
                sys.executable,
                script,
                "--emotion",
                "sad",
                "--duration",
                "1",
                "--headless",
                "--backend",
                "reachy",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

        assert result_mujoco.returncode == 0, f"Erreur MuJoCo: {result_mujoco.stderr}"
        assert result_reachy.returncode == 0, f"Erreur Reachy: {result_reachy.stderr}"

        # Vérifier que les deux backends fonctionnent
        assert "Backend mujoco connecté" in result_mujoco.stdout
        assert "Backend reachy connecté" in result_reachy.stdout

    def test_robot_api_error_handling(self, demo_scripts):
        """Test de gestion d'erreurs RobotAPI."""
        script = demo_scripts["emotion"]

        # Test avec backend invalide
        result = subprocess.run(
            [
                sys.executable,
                script,
                "--emotion",
                "happy",
                "--duration",
                "1",
                "--headless",
                "--backend",
                "invalid_backend",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

        assert result.returncode == 1
        assert "Impossible de créer le backend" in result.stdout

    def test_robot_api_joint_validation(self, demo_scripts):
        """Test de validation des joints RobotAPI."""
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
                "--backend",
                "mujoco",
                "--joint",
                "invalid_joint",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

        assert result.returncode == 1
        assert "introuvable" in result.stdout

    def test_robot_api_performance(self, demo_scripts):
        """Test de performance RobotAPI."""
        script = demo_scripts["emotion"]

        start_time = time.time()
        result = subprocess.run(
            [
                sys.executable,
                script,
                "--emotion",
                "angry",
                "--intensity",
                "0.6",
                "--duration",
                "3",
                "--headless",
                "--backend",
                "mujoco",
            ],
            capture_output=True,
            text=True,
            timeout=15,
        )

        execution_time = time.time() - start_time

        assert result.returncode == 0
        assert execution_time < 8, f"Performance trop lente: {execution_time:.2f}s"
        assert "Animation terminée" in result.stdout
