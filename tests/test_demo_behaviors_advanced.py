#!/usr/bin/env python3
"""Tests pour les nouveaux exemples de comportements avancés.

Tests unitaires pour vérifier que les exemples fonctionnent correctement.
"""

import sys
from pathlib import Path
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestDemoBehaviorsAdvanced:
    """Tests pour les exemples de comportements avancés."""

    @patch("examples.demo_dance.MuJoCoBackend")
    def test_demo_dance_import(self, mock_backend):
        """Test import demo_dance."""
        import examples.demo_dance as demo_dance

        assert hasattr(demo_dance, "main")

    @patch("examples.demo_emotion_show.MuJoCoBackend")
    def test_demo_emotion_show_import(self, mock_backend):
        """Test import demo_emotion_show."""
        import examples.demo_emotion_show as demo_emotion_show

        assert hasattr(demo_emotion_show, "main")

    @patch("examples.demo_photo_booth.MuJoCoBackend")
    @patch("examples.demo_photo_booth.BBIAVision")
    def test_demo_photo_booth_import(self, mock_vision, mock_backend):
        """Test import demo_photo_booth."""
        import examples.demo_photo_booth as demo_photo_booth

        assert hasattr(demo_photo_booth, "main")

    @patch("examples.demo_storytelling.MuJoCoBackend")
    def test_demo_storytelling_import(self, mock_backend):
        """Test import demo_storytelling."""
        import examples.demo_storytelling as demo_storytelling

        assert hasattr(demo_storytelling, "main")

    @patch("examples.demo_teaching.MuJoCoBackend")
    def test_demo_teaching_import(self, mock_backend):
        """Test import demo_teaching."""
        import examples.demo_teaching as demo_teaching

        assert hasattr(demo_teaching, "main")

    @patch("examples.demo_meditation.MuJoCoBackend")
    def test_demo_meditation_import(self, mock_backend):
        """Test import demo_meditation."""
        import examples.demo_meditation as demo_meditation

        assert hasattr(demo_meditation, "main")

    @patch("examples.demo_exercise.MuJoCoBackend")
    def test_demo_exercise_import(self, mock_backend):
        """Test import demo_exercise."""
        import examples.demo_exercise as demo_exercise

        assert hasattr(demo_exercise, "main")

    @patch("examples.demo_music_reaction.MuJoCoBackend")
    def test_demo_music_reaction_import(self, mock_backend):
        """Test import demo_music_reaction."""
        import examples.demo_music_reaction as demo_music_reaction

        assert hasattr(demo_music_reaction, "main")

    @patch("examples.demo_alarm_clock.MuJoCoBackend")
    def test_demo_alarm_clock_import(self, mock_backend):
        """Test import demo_alarm_clock."""
        import examples.demo_alarm_clock as demo_alarm_clock

        assert hasattr(demo_alarm_clock, "main")

    @patch("examples.demo_weather_report.MuJoCoBackend")
    def test_demo_weather_report_import(self, mock_backend):
        """Test import demo_weather_report."""
        import examples.demo_weather_report as demo_weather_report

        assert hasattr(demo_weather_report, "main")

    @patch("examples.demo_news_reader.MuJoCoBackend")
    def test_demo_news_reader_import(self, mock_backend):
        """Test import demo_news_reader."""
        import examples.demo_news_reader as demo_news_reader

        assert hasattr(demo_news_reader, "main")

    @patch("examples.demo_game.MuJoCoBackend")
    def test_demo_game_import(self, mock_backend):
        """Test import demo_game."""
        import examples.demo_game as demo_game

        assert hasattr(demo_game, "main")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
