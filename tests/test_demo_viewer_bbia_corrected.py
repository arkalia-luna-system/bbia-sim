#!/usr/bin/env python3
"""
Tests pour la démo 3D BBIA corrigée
Tests headless de la nouvelle démonstration
"""

import subprocess
import sys
from pathlib import Path

import pytest

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

import mujoco


class TestDemoViewerBBIA:
    """Tests pour la démo 3D BBIA corrigée."""

    @pytest.fixture
    def model_path(self):
        """Chemin vers le modèle MuJoCo."""
        return "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"

    @pytest.fixture
    def demo_script(self):
        """Chemin vers le script de démo."""
        return "examples/demo_viewer_bbia_corrected.py"

    def test_model_loading(self, model_path):
        """Test le chargement du modèle MuJoCo."""
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)

        assert model is not None
        assert data is not None
        assert model.njnt == 16  # 16 joints dans le modèle Reachy Mini

    def test_safe_joints_detection(self, model_path):
        """Test la détection des joints sûrs."""
        model = mujoco.MjModel.from_xml_path(model_path)

        safe_joints = []
        problematic_joints = []
        blocked_joints = []

        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            joint_range = model.jnt_range[i]

            if joint_range[0] == joint_range[1]:  # Joint bloqué
                blocked_joints.append(name)
            elif name == "yaw_body":  # Joint très sûr
                safe_joints.append((name, joint_range, "TRÈS SÛR"))
            elif "stewart" in name:  # Joints Stewart - risqués
                range_size = joint_range[1] - joint_range[0]
                if range_size > 2.0:
                    problematic_joints.append((name, joint_range, "PROBLÉMATIQUE"))
                else:
                    safe_joints.append((name, joint_range, "SÛR"))
            else:
                safe_joints.append((name, joint_range, "SÛR"))

        # Vérifications
        assert len(safe_joints) >= 1, "Au moins un joint sûr doit être détecté"
        assert len(blocked_joints) >= 8, "Au moins 8 joints bloqués attendus"
        assert len(problematic_joints) >= 6, "Au moins 6 joints Stewart problématiques"

        # Vérifier que yaw_body est dans les joints sûrs
        safe_joint_names = [name for name, _, _ in safe_joints]
        assert "yaw_body" in safe_joint_names, "yaw_body doit être dans les joints sûrs"

    def test_demo_list_joints(self, demo_script):
        """Test la commande --list-joints de la démo."""
        result = subprocess.run(
            [sys.executable, demo_script, "--list-joints"],
            capture_output=True,
            text=True,
            timeout=30,
        )

        assert result.returncode == 0, f"Erreur: {result.stderr}"
        assert "JOINTS SÛRS" in result.stdout
        assert "JOINTS PROBLÉMATIQUES" in result.stdout
        assert "JOINTS BLOQUÉS" in result.stdout
        assert "yaw_body" in result.stdout

    def test_demo_headless_animation(self, demo_script):
        """Test l'animation headless de la démo."""
        result = subprocess.run(
            [
                sys.executable,
                demo_script,
                "--headless",
                "--duration",
                "2",
                "--joint",
                "yaw_body",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )

        assert result.returncode == 0, f"Erreur: {result.stderr}"
        assert "Animation headless terminée" in result.stdout
        assert "yaw_body" in result.stdout

    def test_demo_invalid_joint(self, demo_script):
        """Test la gestion d'un joint invalide."""
        result = subprocess.run(
            [
                sys.executable,
                demo_script,
                "--headless",
                "--duration",
                "1",
                "--joint",
                "invalid_joint",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )

        assert (
            result.returncode == 1
        ), "Le script doit retourner une erreur pour un joint invalide"
        assert "non trouvé" in result.stdout or "non sûr" in result.stdout

    def test_demo_amplitude_clamping(self, demo_script):
        """Test le clamping automatique de l'amplitude."""
        result = subprocess.run(
            [
                sys.executable,
                demo_script,
                "--headless",
                "--duration",
                "1",
                "--joint",
                "yaw_body",
                "--amplitude",
                "10.0",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )

        assert result.returncode == 0, f"Erreur: {result.stderr}"
        assert "Amplitude" in result.stdout and "ajustée" in result.stdout

    def test_demo_frequency_and_amplitude(self, demo_script):
        """Test différentes fréquences et amplitudes."""
        test_cases = [
            ("0.1", "0.1"),  # Très lent, petite amplitude
            ("1.0", "0.2"),  # Normal, amplitude modérée
            ("2.0", "0.05"),  # Rapide, très petite amplitude
        ]

        for frequency, amplitude in test_cases:
            result = subprocess.run(
                [
                    sys.executable,
                    demo_script,
                    "--headless",
                    "--duration",
                    "1",
                    "--joint",
                    "yaw_body",
                    "--frequency",
                    frequency,
                    "--amplitude",
                    amplitude,
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )

            assert (
                result.returncode == 0
            ), f"Erreur pour freq={frequency}, amp={amplitude}: {result.stderr}"
            assert "Animation headless terminée" in result.stdout

    def test_demo_xml_path_validation(self, demo_script):
        """Test la validation du chemin XML."""
        result = subprocess.run(
            [
                sys.executable,
                demo_script,
                "--headless",
                "--xml",
                "nonexistent.xml",
                "--duration",
                "1",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )

        assert (
            result.returncode == 1
        ), "Le script doit retourner une erreur pour un XML inexistant"
        assert "introuvable" in result.stdout or "No such file" in result.stdout

    def test_demo_performance(self, demo_script):
        """Test les performances de la démo."""
        import time

        start_time = time.time()
        result = subprocess.run(
            [
                sys.executable,
                demo_script,
                "--headless",
                "--duration",
                "3",
                "--joint",
                "yaw_body",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )
        end_time = time.time()

        assert result.returncode == 0, f"Erreur: {result.stderr}"

        # La démo doit s'exécuter en moins de 10 secondes pour 3 secondes d'animation
        execution_time = end_time - start_time
        assert execution_time < 10.0, f"Trop lent: {execution_time:.2f}s"

        # Vérifier qu'il y a eu des steps
        assert "steps" in result.stdout

    def test_demo_step_count_consistency(self, demo_script):
        """Test la cohérence du nombre de steps."""
        result = subprocess.run(
            [
                sys.executable,
                demo_script,
                "--headless",
                "--duration",
                "2",
                "--joint",
                "yaw_body",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )

        assert result.returncode == 0, f"Erreur: {result.stderr}"

        # Extraire le nombre de steps du output
        lines = result.stdout.split("\n")
        steps_line = [line for line in lines if "steps" in line and "terminée" in line]
        assert len(steps_line) > 0, "Ligne avec nombre de steps non trouvée"

        # Le nombre de steps doit être raisonnable (environ 200 pour 2s à 100Hz)
        steps_text = steps_line[0]
        import re

        steps_match = re.search(r"(\d+) steps", steps_text)
        assert steps_match is not None, "Nombre de steps non trouvé"

        steps_count = int(steps_match.group(1))
        assert 150 <= steps_count <= 250, f"Nombre de steps incohérent: {steps_count}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
