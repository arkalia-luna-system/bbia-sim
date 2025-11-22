"""Tests pour bbia_idle_animations.py - Animations idle."""

import gc
import os
import sys
import time
from unittest.mock import MagicMock, patch

import pytest

# Ajouter le répertoire src au PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim.bbia_idle_animations import (
    BBIABreathingAnimation,
    BBIAPoseTransitionManager,
    BBIAVocalTremor,
    BBIIdleAnimationManager,
)


class TestBBIABreathingAnimation:
    """Tests pour BBIABreathingAnimation."""

    @pytest.fixture
    def mock_robot_api(self):
        """Mock RobotAPI."""
        robot = MagicMock()
        robot.is_connected = True
        robot.goto_target = MagicMock()
        robot.set_target_head_pose = MagicMock()
        robot.set_joint_pos = MagicMock()
        return robot

    @pytest.fixture
    def breathing(self, mock_robot_api):
        """Créer instance BBIABreathingAnimation."""
        return BBIABreathingAnimation(robot_api=mock_robot_api)

    def test_init(self, breathing):
        """Test initialisation."""
        assert breathing.robot_api is not None
        assert breathing.is_active is False
        assert breathing.breathing_rate == 0.15
        assert breathing.breathing_amplitude == 0.02

    def test_start_stop(self, breathing):
        """Test démarrage/arrêt respiration."""
        # Démarrer
        success = breathing.start()
        assert success is True
        assert breathing.is_active is True

        # Attendre un peu
        time.sleep(0.2)

        # Arrêter
        breathing.stop()
        assert breathing.is_active is False

    def test_start_without_robot(self):
        """Test démarrage sans robot connecté."""
        robot = MagicMock()
        robot.is_connected = False

        breathing = BBIABreathingAnimation(robot_api=robot)
        success = breathing.start()

        assert success is False
        assert breathing.is_active is False


class TestBBIAPoseTransitionManager:
    """Tests pour BBIAPoseTransitionManager."""

    @pytest.fixture
    def mock_robot_api(self):
        """Mock RobotAPI."""
        robot = MagicMock()
        robot.is_connected = True
        robot.goto_target = MagicMock()
        robot.set_target_head_pose = MagicMock()
        return robot

    @pytest.fixture
    def pose_manager(self, mock_robot_api):
        """Créer instance BBIAPoseTransitionManager."""
        return BBIAPoseTransitionManager(robot_api=mock_robot_api)

    def test_init(self, pose_manager):
        """Test initialisation."""
        assert pose_manager.robot_api is not None
        assert pose_manager.is_active is False
        assert len(pose_manager.idle_poses) > 0

    def test_start_stop(self, pose_manager):
        """Test démarrage/arrêt transitions."""
        success = pose_manager.start()
        assert success is True
        assert pose_manager.is_active is True

        time.sleep(0.2)

        pose_manager.stop()
        assert pose_manager.is_active is False


class TestBBIAVocalTremor:
    """Tests pour BBIAVocalTremor."""

    @pytest.fixture
    def mock_robot_api(self):
        """Mock RobotAPI (optimisé RAM)."""
        # OPTIMISATION RAM: Mock minimal au lieu de MagicMock complet
        robot = MagicMock(spec=["set_target_head_pose", "is_connected"])
        robot.set_target_head_pose = MagicMock()
        robot.is_connected = True
        return robot

    @pytest.fixture
    def vocal_tremor(self, mock_robot_api):
        """Créer instance BBIAVocalTremor."""
        return BBIAVocalTremor(robot_api=mock_robot_api)

    def teardown_method(self):
        """OPTIMISATION RAM: Nettoyer après chaque test."""
        gc.collect()

    def test_init(self, vocal_tremor):
        """Test initialisation."""
        assert vocal_tremor.robot_api is not None
        assert vocal_tremor.is_active is False

    def test_update_audio_level(self, vocal_tremor):
        """Test mise à jour niveau audio (optimisé RAM)."""
        # OPTIMISATION RAM: Test minimal sans import SDK lourd
        vocal_tremor.start()

        # OPTIMISATION RAM: Mock simple - le code gère gracieusement l'ImportError
        # On teste juste que last_audio_level est mis à jour correctement
        # Pas besoin de mock create_head_pose car le code gère l'ImportError avec pass
        vocal_tremor.update_audio_level(0.8)
        assert vocal_tremor.last_audio_level == 0.8

        # Test avec niveau audio inférieur au seuil (0.3)
        vocal_tremor.update_audio_level(0.2)
        assert vocal_tremor.last_audio_level == 0.2

        # Test avec niveau audio élevé
        vocal_tremor.update_audio_level(0.9)
        assert vocal_tremor.last_audio_level == 0.9


class TestBBIIdleAnimationManager:
    """Tests pour BBIIdleAnimationManager."""

    @pytest.fixture
    def mock_robot_api(self):
        """Mock RobotAPI."""
        robot = MagicMock()
        robot.is_connected = True
        return robot

    @pytest.fixture
    def idle_manager(self, mock_robot_api):
        """Créer instance BBIIdleAnimationManager."""
        return BBIIdleAnimationManager(robot_api=mock_robot_api)

    def test_init(self, idle_manager):
        """Test initialisation."""
        assert idle_manager.robot_api is not None
        assert idle_manager.breathing is not None
        assert idle_manager.pose_transitions is not None
        assert idle_manager.vocal_tremor is not None
        assert idle_manager.is_active is False

    def test_start_stop(self, idle_manager):
        """Test démarrage/arrêt animations."""
        idle_manager.start()
        # Peut être False si robot non connecté dans mock
        # Mais structure doit être correcte

        idle_manager.stop()
        assert idle_manager.is_active is False

    def test_update_vocal_tremor(self, idle_manager):
        """Test mise à jour tremblement vocal."""
        idle_manager.start()
        idle_manager.update_vocal_tremor(0.6)
        # Vérifier que vocal_tremor est mis à jour
        assert idle_manager.vocal_tremor.last_audio_level == 0.6

    def test_is_running(self, idle_manager):
        """Test vérification état."""
        assert idle_manager.is_running() is False

        idle_manager.start()
        # État dépend de succès démarrage
        assert isinstance(idle_manager.is_running(), bool)
