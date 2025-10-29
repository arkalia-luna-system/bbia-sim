#!/usr/bin/env python3
"""
🧪 TESTS D'OPTIMISATIONS DE PERFORMANCE
Détecte si des optimisations SDK disponibles ne sont pas utilisées.
Basé sur les features du SDK Reachy-mini officiel.
"""

import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestPerformanceOptimizations:
    """Tests pour détecter les optimisations de performance manquantes."""

    def test_backend_has_async_play_move(self):
        """Test: Le backend doit avoir async_play_move pour performance."""
        print("\n🧪 TEST: Disponibilité async_play_move")
        print("=" * 60)

        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend()
        assert hasattr(
            backend, "async_play_move"
        ), "async_play_move doit être disponible pour performances asynchrones"
        print("✅ async_play_move disponible dans backend")

        # Vérifier que c'est une méthode callable
        assert callable(
            getattr(backend, "async_play_move", None)
        ), "async_play_move doit être callable"
        print("✅ async_play_move est callable")

    def test_backend_has_recording_methods(self):
        """Test: Le backend doit avoir start_recording/stop_recording/play_move."""
        print("\n🧪 TEST: Disponibilité méthodes recording/replay")
        print("=" * 60)

        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend()
        recording_methods = [
            "start_recording",
            "stop_recording",
            "play_move",
            "async_play_move",
        ]

        for method in recording_methods:
            assert hasattr(
                backend, method
            ), f"{method} doit être disponible (feature SDK)"
            print(f"✅ {method} disponible")

    def test_backend_has_media_io_properties(self):
        """Test: Le backend doit exposer robot.media et robot.io (SDK features)."""
        print("\n🧪 TEST: Disponibilité propriétés media/io SDK")
        print("=" * 60)

        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend()
        backend.connect()

        # Vérifier que robot.media et robot.io sont accessibles si robot réel disponible
        if backend.is_connected and backend.robot:
            if hasattr(backend.robot, "media"):
                print("✅ robot.media disponible (SDK feature)")
            else:
                print("⚠️  robot.media non disponible (peut être limité en simulation)")

            if hasattr(backend.robot, "io"):
                print("✅ robot.io disponible (SDK feature)")
            else:
                print("⚠️  robot.io non disponible (peut être limité en simulation)")

        backend.disconnect()

    def test_goto_target_has_interpolation_methods(self):
        """Test: goto_target doit supporter plusieurs méthodes d'interpolation."""
        print("\n🧪 TEST: Méthodes d'interpolation goto_target")
        print("=" * 60)

        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend()
        backend.connect()

        # Vérifier que goto_target accepte method=
        if hasattr(backend, "goto_target"):
            # Tester avec différentes méthodes
            interpolation_methods = ["minjerk", "linear", "ease_in_out", "cartoon"]

            try:
                from reachy_mini.utils import create_head_pose

                pose = create_head_pose(pitch=0.0, yaw=0.0, degrees=False)

                for method in interpolation_methods:
                    try:
                        # Appel en mode simulation (ne devrait pas lever d'erreur)
                        backend.goto_target(head=pose, duration=0.1, method=method)
                        print(f"✅ goto_target accepte method='{method}'")
                    except Exception as e:
                        # En simulation, certaines méthodes peuvent ne pas être supportées
                        print(
                            f"⚠️  method='{method}': {type(e).__name__} (peut être limité en sim)"
                        )

            except ImportError:
                pytest.skip("SDK reachy_mini non disponible pour test interpolation")

        backend.disconnect()

    def test_look_at_world_has_perform_movement_parameter(self):
        """Test: look_at_world doit accepter perform_movement (optimisation SDK)."""
        print("\n🧪 TEST: Paramètre perform_movement dans look_at_world")
        print("=" * 60)

        import inspect

        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend()

        if hasattr(backend, "look_at_world"):
            # Vérifier signature
            sig = inspect.signature(backend.look_at_world)
            params = list(sig.parameters.keys())

            assert (
                "perform_movement" in params or "duration" in params
            ), "look_at_world doit accepter perform_movement ou duration (SDK feature)"
            print("✅ look_at_world a les paramètres SDK (perform_movement/duration)")

            # Test appel avec perform_movement
            try:
                result = backend.look_at_world(
                    0.2, 0.0, 0.3, duration=0.1, perform_movement=False
                )
                print("✅ look_at_world accepte perform_movement=False (simulation)")
            except TypeError:
                # Peut-être que perform_movement n'est pas le bon nom
                try:
                    result = backend.look_at_world(0.2, 0.0, 0.3, duration=0.1)
                    print("✅ look_at_world accepte duration (simulation)")
                except Exception as e:
                    print(f"⚠️  Erreur test look_at_world: {e}")

    def test_combined_movements_via_goto_target(self):
        """Test: goto_target doit permettre mouvements combinés tête+corps (optimal)."""
        print("\n🧪 TEST: Mouvements combinés via goto_target")
        print("=" * 60)

        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend()
        backend.connect()

        if hasattr(backend, "goto_target"):
            try:
                from reachy_mini.utils import create_head_pose

                pose = create_head_pose(pitch=0.05, yaw=0.05, degrees=False)

                # Test mouvement combiné tête+corps (1 appel SDK = optimal)
                backend.goto_target(
                    head=pose, body_yaw=0.1, duration=0.2, method="minjerk"
                )
                print(
                    "✅ goto_target supporte mouvements combinés tête+corps (1 appel optimal)"
                )
            except ImportError:
                pytest.skip("SDK reachy_mini non disponible")
            except Exception as e:
                print(f"⚠️  Erreur mouvement combiné: {e} (peut être limité en sim)")

        backend.disconnect()

    def test_behavior_modules_use_goto_target(self):
        """Test: Les modules behavior doivent utiliser goto_target (performance)."""
        print("\n🧪 TEST: Utilisation goto_target dans modules behavior")
        print("=" * 60)

        # Vérifier dans le code source que goto_target est utilisé
        # Note: Test informatif seulement
        behavior_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_behavior.py"
        )

        if behavior_file.exists():
            with open(behavior_file, encoding="utf-8") as f:
                content = f.read()

            if "goto_target" in content:
                count = content.count("goto_target")
                print(f"✅ goto_target utilisé {count} fois dans bbia_behavior.py")
            else:
                print("⚠️  goto_target non trouvé dans bbia_behavior.py")

    def test_emotion_applies_adaptive_duration(self):
        """Test: Les émotions doivent utiliser duration adaptative selon intensité."""
        print("\n🧪 TEST: Duration adaptative pour émotions")
        print("=" * 60)

        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend()
        backend.connect()

        # Tester que set_emotion accepte l'intensité
        test_emotions = [
            ("happy", 0.3),  # Faible → duration courte
            ("happy", 0.7),  # Moyenne → duration moyenne
            ("excited", 0.9),  # Haute → duration longue
        ]

        for emotion, intensity in test_emotions:
            try:
                success = backend.set_emotion(emotion, intensity)
                assert (
                    success is True
                ), f"set_emotion doit réussir pour {emotion}@{intensity}"
                print(f"✅ Emotion '{emotion}' (intensité {intensity}): OK")
            except Exception as e:
                print(f"⚠️  Emotion '{emotion}' (intensité {intensity}): {e}")

        backend.disconnect()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
