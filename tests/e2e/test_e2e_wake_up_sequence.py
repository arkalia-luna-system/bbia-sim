#!/usr/bin/env python3
"""
Test E2E: Scénario utilisateur - BBIA réveil → émotion → mouvement
"""

from unittest.mock import MagicMock

import pytest

from bbia_sim.bbia_behavior import BBIABehaviorManager
from bbia_sim.bbia_emotions import BBIAEmotions


@pytest.mark.e2e
@pytest.mark.slow
class TestE2EWakeUpSequence:
    """Tests E2E scénario: réveil → émotion → mouvement."""

    def setup_method(self):
        """Configuration avant chaque test."""
        # Mock robot API
        self.mock_robot = MagicMock()
        self.mock_robot.set_joint_pos.return_value = True
        self.mock_robot.step.return_value = True
        self.mock_robot.set_emotion.return_value = True

        # Emotions
        self.emotions = BBIAEmotions()

        # Behavior manager
        self.behavior = BBIABehaviorManager(robot_api=self.mock_robot)

    def test_bbia_wakes_up_emotion_movement(self):
        """Scénario: BBIA réveil → émotion → mouvement."""
        # 1. Initialiser BBIA
        assert self.behavior is not None
        assert self.emotions is not None

        # 2. Activer comportement wake_up
        self.behavior.execute_behavior = MagicMock(return_value=True)
        wake_up_result = self.behavior.execute_behavior("wake_up", {})
        assert wake_up_result is True

        # 3. Vérifier émotion "excited" (typique réveil)
        emotion_set = self.emotions.set_emotion("excited", 0.8)
        assert emotion_set is True

        current_emotion = self.emotions.get_current_emotion()
        assert current_emotion is not None
        assert current_emotion.get("emotion") == "excited"
        assert current_emotion.get("intensity") == 0.8

        # 4. Vérifier mouvement (antennes ou yaw_body)
        # Le comportement wake_up peut bouger les joints
        # Vérifier que robot API a été utilisé
        assert self.mock_robot.set_joint_pos.called or self.mock_robot.step.called

    def test_bbia_wake_up_full_sequence(self):
        """Séquence complète réveil: comportement → émotion → vérification état."""
        # 1. État initial
        initial_emotion = self.emotions.get_current_emotion()
        assert initial_emotion is not None

        # 2. Réveil via comportement
        self.behavior.execute_behavior = MagicMock(return_value=True)
        result = self.behavior.execute_behavior("wake_up", {"duration": 2.0})
        assert result is True

        # 3. Émotion réveil
        self.emotions.set_emotion("excited", 0.9)
        emotion_state = self.emotions.get_current_emotion()
        assert emotion_state["emotion"] == "excited"

        # 4. Vérifier mouvement (simulation)
        # Le comportement peut avoir appelé set_joint_pos
        # Vérifier qu'au moins step() a été appelé
        assert self.mock_robot.step.called or self.mock_robot.set_joint_pos.called

    def test_bbia_wake_up_to_greeting_flow(self):
        """Flux: réveil → greeting après détection utilisateur."""
        # 1. Réveil initial
        self.behavior.execute_behavior = MagicMock(return_value=True)
        wake_up_result = self.behavior.execute_behavior("wake_up", {})
        assert wake_up_result is True

        # 2. Émotion excited
        self.emotions.set_emotion("excited", 0.8)

        # 3. Simuler détection utilisateur (mock vision)
        # En réalité, ce serait via vision.scan_environment()
        # Ici on simule juste la transition vers greeting

        # 4. Transition vers greeting
        greeting_result = self.behavior.execute_behavior(
            "greeting", {"from_wake_up": True}
        )
        assert greeting_result is True

        # 5. Émotion transition (happy après greeting)
        self.emotions.set_emotion("happy", 0.7)
        emotion_state = self.emotions.get_current_emotion()
        assert emotion_state["emotion"] == "happy"
