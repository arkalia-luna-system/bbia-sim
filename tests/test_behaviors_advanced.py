#!/usr/bin/env python3
"""Tests pour les comportements avancés BBIA.

Tests de tous les comportements intelligents (15 comportements au total).
"""

import pytest
from unittest.mock import MagicMock, patch

from bbia_sim.behaviors.alarm_clock import AlarmClockBehavior
from bbia_sim.behaviors.conversation import ConversationBehavior
from bbia_sim.behaviors.dance import DanceBehavior
from bbia_sim.behaviors.emotion_show import EmotionShowBehavior
from bbia_sim.behaviors.exercise import ExerciseBehavior
from bbia_sim.behaviors.follow_face import FollowFaceBehavior
from bbia_sim.behaviors.follow_object import FollowObjectBehavior
from bbia_sim.behaviors.game import GameBehavior
from bbia_sim.behaviors.meditation import MeditationBehavior
from bbia_sim.behaviors.music_reaction import MusicReactionBehavior
from bbia_sim.behaviors.news_reader import NewsReaderBehavior
from bbia_sim.behaviors.photo_booth import PhotoBoothBehavior
from bbia_sim.behaviors.storytelling import StorytellingBehavior
from bbia_sim.behaviors.teaching import TeachingBehavior
from bbia_sim.behaviors.weather_report import WeatherReportBehavior


class TestStorytellingBehavior:
    """Tests pour le comportement storytelling."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance StorytellingBehavior."""
        mock_robot = MagicMock()
        return StorytellingBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_story(self, behavior):
        """Test exécution d'une histoire."""
        result = behavior.execute({"story": "petit_chaperon_rouge"})
        assert result is True
        assert behavior.is_active is False  # Se termine automatiquement

    def test_execute_invalid_story(self, behavior):
        """Test exécution avec histoire invalide."""
        result = behavior.execute({"story": "invalid_story"})
        assert result is False

    def test_stop(self, behavior):
        """Test arrêt du comportement."""
        behavior.is_active = True
        behavior.stop()
        assert behavior.is_active is False


class TestTeachingBehavior:
    """Tests pour le comportement teaching."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance TeachingBehavior."""
        mock_robot = MagicMock()
        return TeachingBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_lesson(self, behavior):
        """Test exécution d'une leçon."""
        result = behavior.execute({"subject": "maths", "level": "beginner"})
        assert result is True
        assert behavior.is_active is False

    def test_execute_invalid_subject(self, behavior):
        """Test exécution avec matière invalide."""
        result = behavior.execute({"subject": "invalid_subject"})
        assert result is False


class TestMeditationBehavior:
    """Tests pour le comportement meditation."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance MeditationBehavior."""
        mock_robot = MagicMock()
        return MeditationBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_meditation(self, behavior):
        """Test exécution méditation."""
        result = behavior.execute({"duration": 5})
        assert result is True
        assert behavior.is_active is False


class TestExerciseBehavior:
    """Tests pour le comportement exercise."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance ExerciseBehavior."""
        mock_robot = MagicMock()
        return ExerciseBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_exercise(self, behavior):
        """Test exécution exercice."""
        result = behavior.execute({"exercise": "head_rotation", "repetitions": 3})
        assert result is True
        assert behavior.is_active is False

    def test_execute_invalid_exercise(self, behavior):
        """Test exécution avec exercice invalide."""
        result = behavior.execute({"exercise": "invalid_exercise"})
        assert result is False


class TestMusicReactionBehavior:
    """Tests pour le comportement music_reaction."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance MusicReactionBehavior."""
        mock_robot = MagicMock()
        return MusicReactionBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_music_reaction(self, behavior):
        """Test exécution réaction musique."""
        result = behavior.execute({"genre": "pop", "duration": 10})
        assert result is True
        assert behavior.is_active is False

    def test_execute_invalid_genre(self, behavior):
        """Test exécution avec genre invalide."""
        result = behavior.execute({"genre": "invalid_genre"})
        assert result is False


class TestAlarmClockBehavior:
    """Tests pour le comportement alarm_clock."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance AlarmClockBehavior."""
        mock_robot = MagicMock()
        return AlarmClockBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_alarm(self, behavior):
        """Test exécution réveil."""
        from datetime import datetime, time as dt_time
        import threading

        # Utiliser une heure dans le passé pour déclencher immédiatement
        now = datetime.now()
        past_minute = (now.minute - 1) % 60
        past_hour = now.hour if past_minute < now.minute else (now.hour - 1) % 24
        alarm_time = dt_time(past_hour, past_minute)

        # Créer un mock datetime.now() qui retourne une heure après l'alarme
        # Le code appelle datetime.now().time(), donc on doit retourner un datetime
        # dont .time() retourne une heure >= alarm_time
        future_minute = (past_minute + 2) % 60
        future_hour = (
            past_hour if future_minute > past_minute else (past_hour + 1) % 24
        )
        future_time = dt_time(future_hour, future_minute)

        def mock_datetime_now():
            real_now = datetime.now()
            # Retourner un datetime avec l'heure après l'alarme
            return datetime.combine(real_now.date(), future_time)

        # Mocker time.sleep pour éviter les attentes réelles
        sleep_calls = []

        def mock_sleep(seconds):
            sleep_calls.append(seconds)
            # Limiter le nombre de sleeps pour éviter les boucles infinies
            if len(sleep_calls) > 10:
                behavior.stop()

        with (
            patch("bbia_sim.behaviors.alarm_clock.time.sleep", side_effect=mock_sleep),
            patch("bbia_sim.behaviors.alarm_clock.datetime") as mock_datetime,
        ):
            mock_datetime.now = mock_datetime_now
            mock_datetime.combine = datetime.combine

            # Arrêter le comportement dans un thread après un court délai
            def stop_behavior():
                import time as real_time

                real_time.sleep(0.2)
                behavior.stop()

            stop_thread = threading.Thread(target=stop_behavior, daemon=True)
            stop_thread.start()

            # Exécuter avec une heure passée pour déclencher immédiatement
            result = behavior.execute(
                {"hour": alarm_time.hour, "minute": alarm_time.minute}
            )

            # Attendre que le thread d'arrêt se termine
            stop_thread.join(timeout=2.0)

            assert behavior.is_active is False


class TestWeatherReportBehavior:
    """Tests pour le comportement weather_report."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance WeatherReportBehavior."""
        mock_robot = MagicMock()
        return WeatherReportBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_weather_report(self, behavior):
        """Test exécution rapport météo."""
        result = behavior.execute({"city": "Paris"})
        assert result is True
        assert behavior.is_active is False


class TestNewsReaderBehavior:
    """Tests pour le comportement news_reader."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance NewsReaderBehavior."""
        mock_robot = MagicMock()
        return NewsReaderBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_news_reader(self, behavior):
        """Test exécution lecture actualités."""
        result = behavior.execute({"max_items": 3})
        assert result is True
        assert behavior.is_active is False


class TestGameBehavior:
    """Tests pour le comportement game."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance GameBehavior."""
        mock_robot = MagicMock()
        return GameBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_game(self, behavior):
        """Test exécution jeu."""
        result = behavior.execute({"game": "rock_paper_scissors", "rounds": 2})
        assert result is True
        assert behavior.is_active is False

    def test_execute_invalid_game(self, behavior):
        """Test exécution avec jeu invalide."""
        result = behavior.execute({"game": "invalid_game"})
        assert result is False


class TestFollowFaceBehavior:
    """Tests pour le comportement follow_face."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance FollowFaceBehavior."""
        mock_robot = MagicMock()
        mock_vision = MagicMock()
        mock_vision.detect_faces.return_value = []
        return FollowFaceBehavior(vision=mock_vision, robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_follow_face(self, behavior):
        """Test exécution suivi visage."""
        result = behavior.execute({"duration": 1.0})
        assert result is True


class TestFollowObjectBehavior:
    """Tests pour le comportement follow_object."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance FollowObjectBehavior."""
        mock_robot = MagicMock()
        mock_vision = MagicMock()
        mock_vision.scan_environment.return_value = {"objects": []}
        return FollowObjectBehavior(vision=mock_vision, robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_follow_object(self, behavior):
        """Test exécution suivi objet."""
        result = behavior.execute({"duration": 1.0})
        assert result is True


class TestConversationBehavior:
    """Tests pour le comportement conversation."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance ConversationBehavior."""
        mock_robot = MagicMock()
        return ConversationBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_conversation(self, behavior):
        """Test exécution conversation."""
        with patch(
            "bbia_sim.behaviors.conversation.reconnaitre_parole", return_value=None
        ):
            result = behavior.execute({})
            assert result is True


class TestDanceBehavior:
    """Tests pour le comportement dance."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance DanceBehavior."""
        mock_robot = MagicMock()
        return DanceBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_dance(self, behavior):
        """Test exécution danse."""
        result = behavior.execute({"music_type": "happy", "duration": 1.0})
        assert result is True


class TestEmotionShowBehavior:
    """Tests pour le comportement emotion_show."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance EmotionShowBehavior."""
        mock_robot = MagicMock()
        return EmotionShowBehavior(robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_emotion_show(self, behavior):
        """Test exécution démonstration émotions."""
        result = behavior.execute({"emotions_list": ["happy", "sad"]})
        assert result is True


class TestPhotoBoothBehavior:
    """Tests pour le comportement photo_booth."""

    @pytest.fixture
    def behavior(self):
        """Crée une instance PhotoBoothBehavior."""
        mock_robot = MagicMock()
        mock_vision = MagicMock()
        mock_vision.detect_faces.return_value = []
        return PhotoBoothBehavior(vision=mock_vision, robot_api=mock_robot)

    def test_can_execute(self, behavior):
        """Test que le comportement peut être exécuté."""
        assert behavior.can_execute({}) is True

    def test_execute_photo_booth(self, behavior):
        """Test exécution photo booth."""
        result = behavior.execute({"pose": "happy", "countdown": False})
        assert result is True
