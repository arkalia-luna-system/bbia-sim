#!/usr/bin/env python3
"""Tests d'intégration pour les comportements BBIA.

Tests des interactions entre comportements, gestion des ressources,
timeout, et arrêt propre.
"""

import pytest
import time
from unittest.mock import MagicMock, patch

from bbia_sim.behaviors.emotion_show import EmotionShowBehavior
from bbia_sim.behaviors.exercise import ExerciseBehavior
from bbia_sim.behaviors.meditation import MeditationBehavior


class TestBehaviorsIntegration:
    """Tests d'intégration entre comportements."""

    @pytest.fixture
    def mock_robot(self):
        """Crée un mock robot pour les tests."""
        return MagicMock()

    @patch("bbia_sim.behaviors.exercise.dire_texte")
    @patch("bbia_sim.behaviors.emotion_show.dire_texte")
    def test_multiple_behaviors_sequential(
        self, mock_dire_emotion, mock_dire_exercise, mock_robot
    ):
        """Test exécution séquentielle de plusieurs comportements."""
        # Mock dire_texte pour éviter les appels réels qui prennent du temps
        mock_dire_emotion.return_value = None
        mock_dire_exercise.return_value = None

        # Exécuter comportement 1
        behavior1 = EmotionShowBehavior(robot_api=mock_robot)
        behavior1.execute({})
        assert behavior1.is_active is False

        # Exécuter comportement 2
        behavior2 = ExerciseBehavior(robot_api=mock_robot)
        behavior2.execute({"exercise": "head_rotation", "repetitions": 2})
        assert behavior2.is_active is False

        # Vérifier qu'ils ne se chevauchent pas
        assert not (behavior1.is_active and behavior2.is_active)

    def test_behavior_stop_during_execution(self, mock_robot):
        """Test arrêt d'un comportement pendant exécution."""
        behavior = MeditationBehavior(robot_api=mock_robot)
        behavior.is_active = True

        # Arrêter pendant exécution
        behavior.stop()
        assert behavior.is_active is False

    def test_behavior_timeout(self, mock_robot):
        """Test timeout d'un comportement (max 5 minutes)."""
        behavior = ExerciseBehavior(robot_api=mock_robot)
        start_time = time.time()

        # Exécuter avec beaucoup de répétitions (simulation timeout)
        behavior.execute({"exercise": "head_rotation", "repetitions": 1000})

        elapsed = time.time() - start_time
        # Vérifier que comportement se termine (pas de boucle infinie)
        assert behavior.is_active is False
        # Vérifier qu'il ne prend pas trop de temps (max 30s pour test)
        assert elapsed < 30.0

    def test_behavior_resource_cleanup(self, mock_robot):
        """Test nettoyage ressources après arrêt."""
        behavior = MeditationBehavior(robot_api=mock_robot)
        behavior.execute({"duration": 1})
        behavior.stop()

        # Vérifier que ressources sont nettoyées
        assert behavior.is_active is False
        assert behavior.current_phase == 0

    def test_behavior_priority(self, mock_robot):
        """Test gestion priorité entre comportements."""
        behavior1 = EmotionShowBehavior(robot_api=mock_robot)
        behavior2 = ExerciseBehavior(robot_api=mock_robot)

        # Vérifier que priorité est définie
        assert hasattr(behavior1, "priority")
        assert hasattr(behavior2, "priority")
        assert isinstance(behavior1.priority, int)
        assert isinstance(behavior2.priority, int)
