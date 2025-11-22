#!/usr/bin/env python3
"""Tests d'intégration pour les comportements BBIA.

Tests des interactions entre comportements, gestion des ressources,
timeout, et arrêt propre.
"""

import time
from unittest.mock import MagicMock, patch

import pytest

from bbia_sim.behaviors.emotion_show import EmotionShowBehavior
from bbia_sim.behaviors.exercise import ExerciseBehavior
from bbia_sim.behaviors.meditation import MeditationBehavior


class TestBehaviorsIntegration:
    """Tests d'intégration entre comportements."""

    @pytest.fixture
    def mock_robot(self):
        """Crée un mock robot pour les tests."""
        return MagicMock()

    @patch("bbia_sim.bbia_voice.dire_texte")
    @patch("time.sleep")  # Mock sleep pour éviter timeout
    def test_multiple_behaviors_sequential(
        self, mock_sleep, mock_dire_texte, mock_robot
    ):
        """Test exécution séquentielle de plusieurs comportements."""
        # Mock dire_texte pour éviter les appels réels qui prennent du temps
        mock_dire_texte.return_value = None
        # Mock sleep pour éviter timeout dans les tests
        mock_sleep.return_value = None

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

    @patch("time.sleep")
    def test_behavior_timeout(self, mock_sleep, mock_robot):
        """Test timeout d'un comportement (max 5 minutes)."""
        # Mock time.sleep pour accélérer le test et éviter timeout
        mock_sleep.return_value = None

        behavior = ExerciseBehavior(robot_api=mock_robot)
        
        # Mock time.time pour contrôler le temps écoulé
        mock_time_values = [100.0, 100.5, 101.5]  # 0s, 0.5s, 1.5s
        time_index = [0]
        
        def mock_time():
            idx = time_index[0]
            time_index[0] = min(idx + 1, len(mock_time_values) - 1)
            return mock_time_values[idx]
        
        with patch("time.time", side_effect=mock_time):
            # Exécuter avec beaucoup de répétitions (simulation timeout)
            behavior.execute({"exercise": "head_rotation", "repetitions": 1000})

        # Vérifier que comportement se termine (pas de boucle infinie)
        assert behavior.is_active is False

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
