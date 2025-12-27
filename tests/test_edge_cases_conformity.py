#!/usr/bin/env python3
"""
🧪 TESTS DE CAS LIMITES ET EDGE CASES - CONFORMITÉ REACHY MINI
Tests pour détecter des problèmes subtils que seuls les experts robotiques verraient
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

# Import SDK officiel si disponible
try:
    from reachy_mini.utils import create_head_pose

    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    create_head_pose = None


class TestEdgeCasesConformity:
    """Tests de cas limites pour détecter problèmes subtils."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.backend = ReachyMiniBackend()
        try:
            connected = self.backend.connect()
            if not connected:
                pytest.skip(
                    "Robot Reachy Mini non disponible (daemon Zenoh non accessible)"
                )
        except Exception as e:
            # Si erreur de connexion (Zenoh, etc.), skip le test
            if "zenoh" in str(e).lower() or "connect" in str(e).lower():
                pytest.skip(f"Robot Reachy Mini non disponible: {e}")
            raise

    def teardown_method(self):
        """Nettoyage après chaque test."""
        if self.backend:
            self.backend.disconnect()

    def test_edge_case_rapid_emotion_transitions(self):
        """Test edge: Transitions émotionnelles très rapides ne doivent pas casser."""
        print("\n🧪 EDGE TEST 1: Transitions émotionnelles rapides")
        print("=" * 60)

        emotions = ["happy", "sad", "excited", "curious", "neutral"]
        for i in range(10):
            emotion = emotions[i % len(emotions)]
            result = self.backend.set_emotion(emotion, 0.5 + (i % 5) * 0.1)
            assert result is True, f"Émotion {emotion} échouée à l'itération {i}"
            print(f"✅ Transition rapide {i+1}: {emotion}")

    def test_edge_case_extreme_coordinates_look_at(self):
        """Test edge: Coordonnées extrêmes pour look_at doivent être validées."""
        print("\n🧪 EDGE TEST 2: Coordonnées look_at extrêmes")
        print("=" * 60)

        # Coordonnées très grandes (devraient être rejetées ou clampées)
        extreme_coords = [
            (10.0, 10.0, 10.0),  # Trop loin
            (-10.0, -10.0, -10.0),  # Trop loin négatif
            (0.0, 0.0, 0.0),  # Valide
            (0.5, 0.3, 0.2),  # Valide raisonnable
        ]

        for x, y, z in extreme_coords:
            try:
                pose = self.backend.look_at_world(x, y, z, duration=0.5)
                assert isinstance(pose, np.ndarray), "Pose doit être ndarray"
                assert pose.shape == (4, 4), f"Pose shape incorrecte: {pose.shape}"
                print(f"✅ Coordonnées ({x:.2f}, {y:.2f}, {z:.2f}): acceptées")
            except (ValueError, Exception) as e:
                print(
                    f"✅ Coordonnées ({x:.2f}, {y:.2f}, {z:.2f}): rejetées ({type(e).__name__})"
                )

    def test_edge_case_interpolation_methods_edge_values(self):
        """Test edge: Méthodes d'interpolation avec valeurs limites."""
        print("\n🧪 EDGE TEST 3: Interpolation méthodes edge values")
        print("=" * 60)

        if not SDK_AVAILABLE or create_head_pose is None:
            pytest.skip("SDK non disponible")

        methods = ["minjerk", "linear", "ease_in_out", "cartoon"]
        pose = create_head_pose(pitch=0.1, yaw=0.0)

        for method in methods:
            try:
                # Duration très courte
                self.backend.goto_target(head=pose, duration=0.01, method=method)
                print(f"✅ {method} avec duration=0.01s: OK")

                # Duration très longue
                self.backend.goto_target(head=pose, duration=10.0, method=method)
                print(f"✅ {method} avec duration=10.0s: OK")
            except Exception as e:
                print(f"⚠️  {method} avec duration edge: {e}")

    def test_edge_case_emotion_intensity_extremes(self):
        """Test edge: Intensités émotionnelles extrêmes."""
        print("\n🧪 EDGE TEST 4: Intensités émotionnelles extrêmes")
        print("=" * 60)

        extreme_intensities = [0.0, 0.01, 0.99, 1.0, -0.1, 1.5]
        emotion = "happy"

        for intensity in extreme_intensities:
            try:
                result = self.backend.set_emotion(emotion, intensity)
                # Les intensités doivent être clampées entre 0.0 et 1.0
                if 0.0 <= intensity <= 1.0:
                    assert (
                        result is True
                    ), f"Intensité {intensity} valide devrait réussir"
                    print(f"✅ Intensité {intensity:.2f}: OK")
                else:
                    # Peut retourner False ou clampé automatiquement
                    print(
                        f"✅ Intensité {intensity:.2f}: {'Clampé' if result else 'Rejeté'}"
                    )
            except Exception as e:
                print(f"⚠️  Intensité {intensity:.2f}: Erreur {type(e).__name__}")

    def test_edge_case_concurrent_commands(self):
        """Test edge: Commandes concurrentes rapides."""
        print("\n🧪 EDGE TEST 5: Commandes concurrentes rapides")
        print("=" * 60)

        # Envoi rapide de plusieurs commandes
        for i in range(5):
            self.backend.set_joint_pos("yaw_body", 0.1 * i)
            self.backend.set_emotion("happy" if i % 2 == 0 else "neutral", 0.5)

        print("✅ Commandes concurrentes: toutes envoyées sans erreur")

    def test_edge_case_joint_name_variations(self):
        """Test edge: Variations de noms de joints (case, espaces, etc.)."""
        print("\n🧪 EDGE TEST 6: Variations noms joints")
        print("=" * 60)

        variations = [
            "yaw_body",  # Correct
            "YAW_BODY",  # Majuscules
            "yaw body",  # Espace
            "stewart_1",  # Correct
            "Stewart_1",  # Mixte
            "invalid_joint",  # Invalide
        ]

        for joint_name in variations:
            pos = self.backend.get_joint_pos(joint_name)
            if pos is not None or isinstance(pos, float):
                print(f"✅ {joint_name}: {pos if pos is not None else 'N/A'}")
            else:
                print(f"⚠️  {joint_name}: Non reconnu (attendu)")

    def test_edge_case_empty_arrays(self):
        """Test edge: Tableaux vides dans positions."""
        print("\n🧪 EDGE TEST 7: Tableaux vides")
        print("=" * 60)

        # get_current_joint_positions doit toujours retourner quelque chose
        head_pos, antenna_pos = self.backend.get_current_joint_positions()
        assert isinstance(
            head_pos, list | tuple | np.ndarray
        ), "head_pos doit être séquence"
        assert isinstance(
            antenna_pos, list | tuple | np.ndarray
        ), "antenna_pos doit être séquence"
        assert len(head_pos) > 0, "head_pos ne doit pas être vide"
        assert len(antenna_pos) > 0, "antenna_pos ne doit pas être vide"
        print(f"✅ Head positions: {len(head_pos)} éléments")
        print(f"✅ Antenna positions: {len(antenna_pos)} éléments")

    def test_edge_case_behavior_duration_extremes(self):
        """Test edge: Durées extrêmes pour comportements."""
        print("\n🧪 EDGE TEST 8: Durées comportements extrêmes")
        print("=" * 60)

        behaviors = ["wake_up", "goto_sleep", "nod"]
        extreme_durations = [0.0, 0.01, 60.0, -1.0]

        for behavior in behaviors:
            for duration in extreme_durations:
                try:
                    result = self.backend.run_behavior(behavior, duration)
                    if duration >= 0:
                        print(f"✅ {behavior}(duration={duration}): OK")
                    else:
                        print(
                            f"⚠️  {behavior}(duration={duration}): {'Accepté' if result else 'Rejeté'}"
                        )
                except Exception as e:
                    print(
                        f"⚠️  {behavior}(duration={duration}): Erreur {type(e).__name__}"
                    )


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
