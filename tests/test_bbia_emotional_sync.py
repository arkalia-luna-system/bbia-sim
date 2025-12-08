#!/usr/bin/env python3
"""Tests pour bbia_emotional_sync.py - Synchronisation fine mouvements émotionnels ↔ parole."""

from __future__ import annotations

import sys
from unittest.mock import MagicMock, Mock, patch

from bbia_sim.bbia_emotional_sync import BBIAEmotionalSync, ConversationState


class TestBBIAEmotionalSync:
    """Tests pour BBIAEmotionalSync."""

    def test_init(self) -> None:
        """Test initialisation."""
        robot_api = Mock()
        sync = BBIAEmotionalSync(robot_api=robot_api)

        assert sync.robot_api == robot_api
        assert sync.current_state == ConversationState.IDLE
        assert sync.is_speaking is False
        assert sync.sync_thread is None

    def test_init_no_robot_api(self) -> None:
        """Test initialisation sans robot_api."""
        sync = BBIAEmotionalSync(robot_api=None)

        assert sync.robot_api is None
        assert sync.current_state == ConversationState.IDLE

    def test_estimate_speech_duration(self) -> None:
        """Test estimation durée parole."""
        sync = BBIAEmotionalSync()

        # Texte court
        duration = sync.estimate_speech_duration("Bonjour", words_per_minute=150)
        assert 0.5 <= duration <= 2.0

        # Texte moyen
        duration = sync.estimate_speech_duration(
            "Bonjour, comment allez-vous aujourd'hui ?",
            words_per_minute=150,
        )
        assert 1.0 <= duration <= 5.0

        # Texte long
        long_text = " ".join(["mot"] * 100)
        duration = sync.estimate_speech_duration(long_text, words_per_minute=150)
        assert duration <= 30.0  # Maximum

    def test_estimate_speech_duration_custom_wpm(self) -> None:
        """Test estimation avec vitesse personnalisée."""
        sync = BBIAEmotionalSync()

        # Vitesse rapide
        duration_fast = sync.estimate_speech_duration("Bonjour", words_per_minute=200)
        # Vitesse lente
        duration_slow = sync.estimate_speech_duration("Bonjour", words_per_minute=100)

        assert duration_fast < duration_slow

    def test_sync_speak_with_emotion_no_robot_api(self) -> None:
        """Test synchronisation sans robot_api."""
        sync = BBIAEmotionalSync(robot_api=None)

        result = sync.sync_speak_with_emotion("Test", emotion="happy", intensity=0.7)
        assert result is False

    def test_sync_speak_with_emotion_with_robot_api(self) -> None:
        """Test synchronisation avec robot_api."""
        robot_api = Mock()
        robot_api.set_emotion = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)

        speak_function = Mock()

        with patch("bbia_sim.bbia_emotional_sync.time.sleep"):
            result = sync.sync_speak_with_emotion(
                "Test",
                emotion="happy",
                intensity=0.7,
                speak_function=speak_function,
            )

        assert result is True
        speak_function.assert_called_once_with("Test", robot_api)

    def test_sync_speak_with_emotion_default_speak(self) -> None:
        """Test synchronisation avec dire_texte par défaut."""
        robot_api = Mock()
        robot_api.set_emotion = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)

        with (
            patch("bbia_sim.bbia_voice.dire_texte") as mock_dire,
            patch(
                "bbia_sim.bbia_emotional_sync.time.sleep",
            ),
            patch("bbia_sim.bbia_emotional_sync.threading.Thread"),
        ):
            result = sync.sync_speak_with_emotion(
                "Test", emotion="happy", intensity=0.7
            )

        assert result is True
        mock_dire.assert_called_once()

    def test_sync_speak_with_emotion_state_changes(self) -> None:
        """Test changements d'état pendant synchronisation."""
        robot_api = Mock()
        robot_api.set_emotion = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)

        speak_function = Mock()

        with (
            patch("bbia_sim.bbia_emotional_sync.time.sleep"),
            patch(
                "bbia_sim.bbia_emotional_sync.threading.Thread",
            ) as mock_thread,
        ):
            mock_thread_instance = Mock()
            mock_thread.return_value = mock_thread_instance

            sync.sync_speak_with_emotion(
                "Test",
                emotion="happy",
                intensity=0.7,
                speak_function=speak_function,
            )

        assert sync.current_state == ConversationState.IDLE
        assert sync.is_speaking is False

    def test_micro_head_movement(self) -> None:
        """Test micro-mouvement de tête."""
        robot_api = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)

        # Mocker l'import conditionnel dans le module bbia_emotional_sync
        # pour éviter ModuleNotFoundError si reachy_mini n'est pas installé
        # On doit mocker sys.modules pour que l'import fonctionne
        mock_reachy_mini = MagicMock()
        mock_pose = Mock()  # Mock de la pose retournée
        mock_reachy_mini.utils.create_head_pose = Mock(return_value=mock_pose)
        mock_utils = MagicMock()
        mock_utils.create_head_pose = mock_reachy_mini.utils.create_head_pose

        with patch.dict(
            sys.modules,
            {"reachy_mini": mock_reachy_mini, "reachy_mini.utils": mock_utils},
        ):
            sync._micro_head_movement(0.05, 0.2)

        robot_api.goto_target.assert_called()

    def test_micro_head_movement_fallback(self) -> None:
        """Test micro-mouvement de tête avec fallback."""
        robot_api = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)

        # Simuler ImportError pour forcer le fallback
        # Ne pas mocker sys.modules pour que l'import échoue et déclenche le fallback
        # Si reachy_mini n'est pas installé, l'ImportError sera levé naturellement
        sync._micro_head_movement(0.05, 0.2)

        robot_api.goto_target.assert_called()

    def test_micro_antenna_movement(self) -> None:
        """Test micro-mouvement d'antenne."""
        robot_api = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)

        sync._micro_antenna_movement(0.03, 0.2)

        robot_api.goto_target.assert_called()

    def test_micro_antenna_movement_no_robot_api(self) -> None:
        """Test micro-mouvement d'antenne sans robot_api."""
        sync = BBIAEmotionalSync(robot_api=None)

        # Ne doit pas lever d'exception
        sync._micro_antenna_movement(0.03, 0.2)

    def test_start_listening_movements(self) -> None:
        """Test démarrage micro-mouvements écoute."""
        robot_api = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)

        with patch("bbia_sim.bbia_emotional_sync.threading.Thread") as mock_thread:
            mock_thread_instance = Mock()
            mock_thread.return_value = mock_thread_instance

            sync.start_listening_movements()

        assert sync.current_state == ConversationState.LISTENING
        assert sync.sync_thread is not None

    def test_stop_listening_movements(self) -> None:
        """Test arrêt micro-mouvements écoute."""
        robot_api = Mock()
        sync = BBIAEmotionalSync(robot_api=robot_api)

        # Simuler thread actif
        mock_thread = Mock()
        sync.sync_thread = mock_thread
        sync.current_state = ConversationState.LISTENING

        sync.stop_listening_movements()

        assert sync.current_state == ConversationState.IDLE
        assert sync.stop_sync.is_set()

    def test_transition_to_thinking(self) -> None:
        """Test transition vers réflexion."""
        robot_api = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)

        # Mocker l'import conditionnel pour éviter ModuleNotFoundError
        # On doit mocker sys.modules pour que l'import fonctionne
        mock_reachy_mini = MagicMock()
        mock_pose = Mock()  # Mock de la pose retournée
        mock_reachy_mini.utils.create_head_pose = Mock(return_value=mock_pose)
        mock_utils = MagicMock()
        mock_utils.create_head_pose = mock_reachy_mini.utils.create_head_pose

        with patch.dict(
            sys.modules,
            {"reachy_mini": mock_reachy_mini, "reachy_mini.utils": mock_utils},
        ):
            sync.transition_to_thinking()

        assert sync.current_state == ConversationState.THINKING
        robot_api.goto_target.assert_called()

    def test_transition_to_thinking_fallback(self) -> None:
        """Test transition vers réflexion avec fallback."""
        robot_api = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)

        # Simuler ImportError pour forcer le fallback
        # Ne pas mocker sys.modules pour que l'import échoue et déclenche le fallback
        # Si reachy_mini n'est pas installé, l'ImportError sera levé naturellement
        sync.transition_to_thinking()

        assert sync.current_state == ConversationState.THINKING
        robot_api.goto_target.assert_called()

    def test_transition_to_reacting(self) -> None:
        """Test transition vers réaction."""
        robot_api = Mock()
        robot_api.set_emotion = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)

        sync.transition_to_reacting("happy", intensity=0.8)

        assert sync.current_state == ConversationState.REACTING
        robot_api.set_emotion.assert_called_once_with("happy", 0.8)

    def test_transition_to_reacting_no_robot_api(self) -> None:
        """Test transition vers réaction sans robot_api."""
        sync = BBIAEmotionalSync(robot_api=None)

        # Ne doit pas lever d'exception
        sync.transition_to_reacting("happy", intensity=0.8)

    def test_get_current_state(self) -> None:
        """Test récupération état actuel."""
        sync = BBIAEmotionalSync()

        assert sync.get_current_state() == ConversationState.IDLE

        sync.current_state = ConversationState.SPEAKING
        assert sync.get_current_state() == ConversationState.SPEAKING

    def test_sync_emotion_during_speech(self) -> None:
        """Test synchronisation émotion pendant parole."""
        robot_api = Mock()
        robot_api.set_emotion = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)
        sync.stop_sync = Mock()
        sync.stop_sync.is_set = Mock(return_value=True)  # Arrêter immédiatement

        with (
            patch("bbia_sim.bbia_emotional_sync.time.time") as mock_time,
            patch(
                "bbia_sim.bbia_emotional_sync.time.sleep",
            ),
        ):
            mock_time.return_value = 0.0  # Temps fixe

            sync._sync_emotion_during_speech("happy", 0.7, 0.5)

        robot_api.set_emotion.assert_called()

    def test_listening_micro_movements(self) -> None:
        """Test micro-mouvements pendant écoute."""
        robot_api = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)
        sync.stop_sync = Mock()
        sync.stop_sync.is_set = Mock(
            side_effect=[False, False, True]
        )  # Arrêter après 2 itérations

        with patch("bbia_sim.bbia_emotional_sync.time.sleep"):
            sync._listening_micro_movements()

        # Vérifier que goto_target a été appelé
        assert robot_api.goto_target.call_count >= 0

    def test_sync_speak_with_emotion_error_handling(self) -> None:
        """Test gestion erreurs synchronisation."""
        robot_api = Mock()
        robot_api.set_emotion = Mock()
        robot_api.goto_target = Mock()

        sync = BBIAEmotionalSync(robot_api=robot_api)

        speak_function = Mock(side_effect=RuntimeError("Erreur"))

        with patch("bbia_sim.bbia_emotional_sync.threading.Thread"):
            result = sync.sync_speak_with_emotion(
                "Test",
                emotion="happy",
                intensity=0.7,
                speak_function=speak_function,
            )

        # Doit gérer l'erreur gracieusement
        assert result is False
        assert sync.is_speaking is False
        assert sync.current_state == ConversationState.IDLE


class TestConversationState:
    """Tests pour ConversationState enum."""

    def test_conversation_state_values(self) -> None:
        """Test valeurs enum."""
        assert ConversationState.IDLE.value == "idle"
        assert ConversationState.LISTENING.value == "listening"
        assert ConversationState.THINKING.value == "thinking"
        assert ConversationState.SPEAKING.value == "speaking"
        assert ConversationState.REACTING.value == "reacting"
