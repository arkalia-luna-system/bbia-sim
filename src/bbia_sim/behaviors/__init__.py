#!/usr/bin/env python3
"""Module de comportements avancés pour BBIA.

Ce module contient tous les comportements robotiques intelligents,
organisés de manière modulaire et extensible.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .base import BBIABehavior

# Import de la classe de base
# Import des comportements existants améliorés
from .alarm_clock import AlarmClockBehavior
from .base import BBIABehavior
from .conversation import ConversationBehavior
from .dance import DanceBehavior
from .emotion_show import EmotionShowBehavior
from .exercise import ExerciseBehavior
from .follow_face import FollowFaceBehavior
from .follow_object import FollowObjectBehavior
from .game import GameBehavior
from .meditation import MeditationBehavior
from .music_reaction import MusicReactionBehavior
from .news_reader import NewsReaderBehavior
from .photo_booth import PhotoBoothBehavior
from .storytelling import StorytellingBehavior
from .teaching import TeachingBehavior
from .weather_report import WeatherReportBehavior

__all__ = [
    "AlarmClockBehavior",
    "BBIABehavior",
    "ConversationBehavior",
    "DanceBehavior",
    "EmotionShowBehavior",
    "ExerciseBehavior",
    "FollowFaceBehavior",
    "FollowObjectBehavior",
    "GameBehavior",
    "MeditationBehavior",
    "MusicReactionBehavior",
    "NewsReaderBehavior",
    "PhotoBoothBehavior",
    "StorytellingBehavior",
    "TeachingBehavior",
    "WeatherReportBehavior",
]
