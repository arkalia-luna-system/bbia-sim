#!/usr/bin/env python3
"""Tests pour les démos supplémentaires (follow_object, memory, adaptive_behavior, awake, sanity).

Tests unitaires pour vérifier que les nouvelles démos fonctionnent correctement.
"""

import sys
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestDemoFollowObject:
    """Tests pour demo_follow_object.py."""

    @patch("examples.demo_follow_object.MuJoCoBackend")
    @patch("examples.demo_follow_object.BBIAVision")
    @patch("examples.demo_follow_object.RobotFactory")
    def test_demo_follow_object_import(self, mock_factory, mock_vision, mock_backend):
        """Test que demo_follow_object peut être importé."""
        import examples.demo_follow_object as demo_follow_object

        assert hasattr(demo_follow_object, "main")

    @patch("examples.demo_follow_object.MuJoCoBackend")
    @patch("examples.demo_follow_object.BBIAVision")
    @patch("examples.demo_follow_object.RobotFactory")
    def test_demo_follow_object_has_main(self, mock_factory, mock_vision, mock_backend):
        """Test que demo_follow_object a une fonction main."""
        import examples.demo_follow_object as demo_follow_object

        assert callable(demo_follow_object.main)


class TestDemoMemory:
    """Tests pour demo_memory.py."""

    def test_demo_memory_import(self):
        """Test que demo_memory peut être importé."""
        import examples.demo_memory as demo_memory

        assert hasattr(demo_memory, "main")

    def test_demo_memory_has_main(self):
        """Test que demo_memory a une fonction main."""
        import examples.demo_memory as demo_memory

        assert callable(demo_memory.main)


class TestDemoAdaptiveBehavior:
    """Tests pour demo_adaptive_behavior.py."""

    @patch("examples.demo_adaptive_behavior.MuJoCoBackend")
    @patch("examples.demo_adaptive_behavior.RobotFactory")
    def test_demo_adaptive_behavior_import(self, mock_factory, mock_backend):
        """Test que demo_adaptive_behavior peut être importé."""
        import examples.demo_adaptive_behavior as demo_adaptive_behavior

        assert hasattr(demo_adaptive_behavior, "main")

    @patch("examples.demo_adaptive_behavior.MuJoCoBackend")
    @patch("examples.demo_adaptive_behavior.RobotFactory")
    def test_demo_adaptive_behavior_has_main(self, mock_factory, mock_backend):
        """Test que demo_adaptive_behavior a une fonction main."""
        import examples.demo_adaptive_behavior as demo_adaptive_behavior

        assert callable(demo_adaptive_behavior.main)


class TestDemoAwake:
    """Tests pour demo_awake.py."""

    def test_demo_awake_import(self):
        """Test que demo_awake peut être importé."""
        import examples.demo_awake as demo_awake

        assert hasattr(demo_awake, "main")

    def test_demo_awake_has_main(self):
        """Test que demo_awake a une fonction main."""
        import examples.demo_awake as demo_awake

        assert callable(demo_awake.main)


class TestDemoSanity:
    """Tests pour demo_sanity.py."""

    @patch("examples.demo_sanity.httpx")
    def test_demo_sanity_import(self, mock_httpx):
        """Test que demo_sanity peut être importé."""
        import examples.demo_sanity as demo_sanity

        assert hasattr(demo_sanity, "main")

    @patch("examples.demo_sanity.httpx")
    def test_demo_sanity_has_main(self, mock_httpx):
        """Test que demo_sanity a une fonction main."""
        import examples.demo_sanity as demo_sanity

        assert callable(demo_sanity.main)
