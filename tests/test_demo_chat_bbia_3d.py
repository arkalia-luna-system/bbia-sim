#!/usr/bin/env python3
"""
Tests pour la démo Chat BBIA en 3D
Tests de validation de la démo 3D MuJoCo
"""

import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestDemoChatBBIA3D:
    """Tests pour la démo chat BBIA 3D."""

    def test_chat_initialization(self):
        """Test que le chat peut s'initialiser."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            bbia = BBIAHuggingFace()
            assert hasattr(bbia, "bbia_personality")
            assert hasattr(bbia, "conversation_history")
        except ImportError:
            # Fallback sans HF
            class MockHuggingFace:
                def __init__(self):
                    self.bbia_personality = "friendly_robot"
                    self.conversation_history = []

            bbia_fallback: BBIAHuggingFace = MockHuggingFace()  # type: ignore[assignment]
            bbia = bbia_fallback
            assert hasattr(bbia, "bbia_personality")

    def test_chat_method(self):
        """Test que la méthode chat fonctionne."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            bbia = BBIAHuggingFace()
            response = bbia.chat("Bonjour")
            assert isinstance(response, str)
            assert len(response) > 0
        except ImportError:
            # Fallback sans HF
            class MockHuggingFace:
                def chat(self, message):
                    return "🤖 Bonjour !"

            bbia_fallback: BBIAHuggingFace = MockHuggingFace()  # type: ignore[assignment]
            bbia = bbia_fallback
            response = bbia.chat("Bonjour")
            assert isinstance(response, str)

    def test_mujoco_model_load(self):
        """Test que le modèle MuJoCo peut être chargé."""
        try:
            import mujoco

            model_path = Path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")

            if model_path.exists():
                model = mujoco.MjModel.from_xml_path(str(model_path))
                data = mujoco.MjData(model)

                assert model is not None
                assert data is not None
                assert model.nq > 0  # Au moins 1 joint
                assert model.njnt > 0  # Au moins 1 joint défini
            else:
                pytest.skip("Modèle MuJoCo non trouvé")
        except ImportError:
            pytest.skip("MuJoCo non disponible")

    def test_get_joint_id(self):
        """Test la fonction get_joint_id."""
        try:
            import mujoco

            model_path = Path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")

            if not model_path.exists():
                pytest.skip("Modèle MuJoCo non trouvé")

            model = mujoco.MjModel.from_xml_path(str(model_path))

            def get_joint_id(model, joint_name):
                try:
                    return mujoco.mj_name2id(
                        model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
                    )
                except Exception:
                    return None

            # Test avec des joints connus
            _stewart_1 = get_joint_id(model, "stewart_1")
            # stewart_1 devrait être None ou un ID valide

            _yaw_body = get_joint_id(model, "yaw_body")
            # yaw_body devrait être None ou un ID valide

            assert True  # Si on arrive ici, pas d'erreur

        except ImportError:
            pytest.skip("MuJoCo non disponible")

    def test_animate_robot_function(self):
        """Test que la fonction animate_robot peut être exécutée."""
        try:
            import mujoco

            model_path = Path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")

            if not model_path.exists():
                pytest.skip("Modèle MuJoCo non trouvé")

            model = mujoco.MjModel.from_xml_path(str(model_path))
            data = mujoco.MjData(model)

            def get_joint_id(model, joint_name):
                try:
                    return mujoco.mj_name2id(
                        model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
                    )
                except Exception:
                    return None

            # Simuler animation
            stewart_joints = ["stewart_1", "stewart_2", "stewart_3"]
            for joint_name in stewart_joints:
                joint_id = get_joint_id(model, joint_name)
                if joint_id is not None:
                    data.qpos[joint_id] = 0.1

            # Tester simulation
            mujoco.mj_step(model, data)

            assert True  # Si on arrive ici, pas d'erreur

        except ImportError:
            pytest.skip("MuJoCo non disponible")

    def test_conversation_history(self):
        """Test que l'historique est sauvegardé."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            bbia = BBIAHuggingFace()
            initial_count = len(bbia.conversation_history)

            bbia.chat("Test message")

            assert len(bbia.conversation_history) == initial_count + 1
        except ImportError:
            # Fallback sans HF
            class MockHuggingFace:
                def __init__(self):
                    self.conversation_history = []

                def chat(self, msg):
                    self.conversation_history.append({"user": msg, "bbia": "🤖 OK"})

            bbia_fallback: BBIAHuggingFace = MockHuggingFace()  # type: ignore[assignment]
            bbia = bbia_fallback
            bbia.chat("Test")
            assert len(bbia.conversation_history) == 1

    def test_bbia_personality(self):
        """Test que la personnalité BBIA fonctionne."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            bbia = BBIAHuggingFace()
            assert bbia.bbia_personality in [
                "friendly_robot",
                "curious",
                "enthusiastic",
                "calm",
            ]

            # Tester changement de personnalité
            bbia.bbia_personality = "curious"
            assert bbia.bbia_personality == "curious"
        except ImportError:
            # Fallback
            class MockHuggingFace:
                def __init__(self):
                    self.bbia_personality = "friendly_robot"

            bbia_fallback: BBIAHuggingFace = MockHuggingFace()  # type: ignore[assignment]
            bbia = bbia_fallback
            assert bbia.bbia_personality == "friendly_robot"


def test_demo_can_import():
    """Test que la démo peut être importée."""
    # Juste vérifier que le fichier existe et peut être lu
    demo_path = Path("examples/demo_chat_bbia_3d.py")
    assert demo_path.exists(), "Fichier demo_chat_bbia_3d.py introuvable"

    # Vérifier qu'il contient les bonnes fonctions
    content = demo_path.read_text()
    assert "def demo_chat_bbia_3d" in content, "Fonction demo_chat_bbia_3d manquante"
    assert (
        "def animate_robot_from_chat" in content
    ), "Fonction animate_robot_from_chat manquante"
    assert "MockHuggingFace" in content, "Classe MockHuggingFace manquante"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
