#!/usr/bin/env python3
"""Tests pour les démos supplémentaires (follow_object, memory, adaptive_behavior, awake, sanity)."""

import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).parent.parent))


class TestDemoFollowObject:
    """Tests pour demo_follow_object.py."""

    def test_demo_follow_object_import(self):
        """Test que demo_follow_object peut être importé."""
        import examples.demo_follow_object  # noqa: F401

        assert True


class TestDemoMemory:
    """Tests pour demo_memory.py."""

    def test_demo_memory_import(self):
        """Test que demo_memory peut être importé."""
        import examples.demo_memory  # noqa: F401

        assert True


class TestDemoAdaptiveBehavior:
    """Tests pour demo_adaptive_behavior.py."""

    def test_demo_adaptive_behavior_import(self):
        """Test que demo_adaptive_behavior peut être importé."""
        import examples.demo_adaptive_behavior  # noqa: F401

        assert True


class TestDemoAwake:
    """Tests pour demo_awake.py."""

    def test_demo_awake_import(self):
        """Test que demo_awake peut être importé."""
        import examples.demo_awake  # noqa: F401

        assert True


class TestDemoSanity:
    """Tests pour demo_sanity.py."""

    def test_demo_sanity_import(self):
        """Test que demo_sanity peut être importé."""
        import examples.demo_sanity  # noqa: F401

        assert True
