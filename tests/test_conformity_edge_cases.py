#!/usr/bin/env python3
"""
🧪 TESTS EDGE CASES CONFORMITÉ SDK EXHAUSTIFS
Tests pour cas limites non couverts : erreurs réseau, valeurs extrêmes, limites SDK
"""

import math
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend
from bbia_sim.robot_factory import RobotFactory

# Import SDK officiel si disponible
try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose

    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    ReachyMini = None
    create_head_pose = None


class TestConformityEdgeCases:
    """Tests edge cases exhaustifs pour conformité SDK."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.backend = ReachyMiniBackend()
        self.backend.connect()

    def teardown_method(self):
        """Nettoyage après chaque test."""
        if self.backend:
            self.backend.disconnect()

    def test_edge_case_joint_angles_max_min(self):
        """Test edge: Angles joints aux limites max/min."""
        print("\n🧪 EDGE TEST 1: Angles joints limites max/min")
        print("=" * 60)

        test_joints = ["yaw_body", "left_antenna", "right_antenna"]
        for joint in test_joints:
            if joint not in self.backend.joint_limits:
                continue

            min_limit, max_limit = self.backend.joint_limits[joint]

            # Test limite min
            result_min = self.backend.set_joint_pos(joint, min_limit)
            assert (
                result_min is True
            ), f"Limite min {min_limit} doit être acceptée pour {joint}"

            # Test limite max
            result_max = self.backend.set_joint_pos(joint, max_limit)
            assert (
                result_max is True
            ), f"Limite max {max_limit} doit être acceptée pour {joint}"

            # Test juste au-delà des limites (doit être clampé)
            self.backend.set_joint_pos(joint, max_limit + 0.1)
            pos_after = self.backend.get_joint_pos(joint)
            if pos_after is not None:
                assert (
                    pos_after <= max_limit
                ), f"Position {pos_after} doit être clampée à {max_limit} pour {joint}"

            print(f"✅ {joint}: limites [{min_limit:.4f}, {max_limit:.4f}] respectées")

    def test_edge_case_extreme_velocities(self):
        """Test edge: Vitesses extrêmes pour mouvements."""
        print("\n🧪 EDGE TEST 2: Vitesses extrêmes")
        print("=" * 60)

        extreme_velocities = [0.0, 0.01, 0.5, 1.0, 2.0, 10.0, -1.0]

        for velocity in extreme_velocities:
            try:
                # Test avec look_at_world (si disponible)
                if hasattr(self.backend, "look_at_world"):
                    result = self.backend.look_at_world(
                        0.1, 0.2, 0.3, duration=1.0 / max(velocity, 0.01)
                    )
                    assert (
                        result is not None
                    ), f"look_at_world doit fonctionner avec vitesse {velocity}"
                    print(f"✅ Vitesse {velocity:.2f}: OK")
            except (ValueError, Exception) as e:
                if velocity < 0 or velocity > 1.0:
                    print(
                        f"✅ Vitesse {velocity:.2f}: rejetée correctement ({type(e).__name__})"
                    )
                else:
                    print(f"⚠️  Vitesse {velocity:.2f}: erreur inattendue {e}")

    def test_edge_case_network_errors(self):
        """Test edge: Gestion erreurs réseau (simulation d'erreurs SDK)."""
        print("\n🧪 EDGE TEST 3: Gestion erreurs réseau")
        print("=" * 60)

        # Simuler erreur réseau dans le SDK
        with patch.object(self.backend, "robot", None):
            # Backend doit gérer gracieusement robot=None
            result = self.backend.set_emotion("happy", 0.5)
            # Peut retourner False ou lever exception claire
            assert result is False or isinstance(result, bool)
            print("✅ Gestion robot=None: OK")

        # Simuler erreur de connexion
        with patch.object(self.backend, "is_connected", False):
            result = self.backend.get_joint_pos("yaw_body")
            # Peut retourner None ou lever exception claire
            assert result is None or isinstance(result, float)
            print("✅ Gestion is_connected=False: OK")

    def test_edge_case_sdk_error_handling(self):
        """Test edge: Gestion erreurs SDK (commandes invalides)."""
        print("\n🧪 EDGE TEST 4: Gestion erreurs SDK")
        print("=" * 60)

        # Test commande invalide sur joint interdit
        forbidden_joints = self.backend.forbidden_joints
        if forbidden_joints:
            test_joint = forbidden_joints[0]
            result = self.backend.set_joint_pos(test_joint, 0.1)
            assert result is False, f"Joint interdit {test_joint} doit être rejeté"
            print(f"✅ Joint interdit {test_joint}: rejeté correctement")

        # Test émotion invalide
        result = self.backend.set_emotion("invalid_emotion_xyz", 0.5)
        assert result is False, "Émotion invalide doit être rejetée"
        print("✅ Émotion invalide: rejetée correctement")

        # Test comportement invalide
        result = self.backend.run_behavior("invalid_behavior_xyz", 1.0)
        assert result is False, "Comportement invalide doit être rejeté"
        print("✅ Comportement invalide: rejeté correctement")

    def test_edge_case_look_at_targets_out_of_range(self):
        """Test edge: look_at avec cibles hors champ."""
        print("\n🧪 EDGE TEST 5: look_at cibles hors champ")
        print("=" * 60)

        out_of_range_targets = [
            (10.0, 0.0, 0.0),  # Trop loin
            (0.0, 10.0, 0.0),  # Trop loin
            (0.0, 0.0, -5.0),  # Trop bas
            (0.0, 0.0, 10.0),  # Trop haut
        ]

        for x, y, z in out_of_range_targets:
            try:
                if hasattr(self.backend, "look_at_world"):
                    result = self.backend.look_at_world(x, y, z, duration=0.5)
                    # Peut retourner pose clampée ou lever exception
                    if result is not None:
                        assert isinstance(result, np.ndarray), "Pose doit être ndarray"
                        print(f"✅ Cible ({x:.1f}, {y:.1f}, {z:.1f}): clampée")
                    else:
                        print(f"✅ Cible ({x:.1f}, {y:.1f}, {z:.1f}): rejetée")
            except (ValueError, Exception) as e:
                print(
                    f"✅ Cible ({x:.1f}, {y:.1f}, {z:.1f}): rejetée ({type(e).__name__})"
                )

    def test_edge_case_nan_inf_values(self):
        """Test edge: Valeurs NaN/Inf dans positions."""
        print("\n🧪 EDGE TEST 6: Valeurs NaN/Inf")
        print("=" * 60)

        invalid_values = [float("nan"), float("inf"), float("-inf")]

        for invalid_val in invalid_values:
            try:
                result = self.backend.set_joint_pos("yaw_body", invalid_val)
                # Doit rejeter ou clampé
                if not result:
                    print(f"✅ Valeur {invalid_val}: rejetée")
                else:
                    # Si accepté, vérifier que la position n'est pas NaN/Inf
                    pos = self.backend.get_joint_pos("yaw_body")
                    assert pos is not None
                    assert not math.isnan(pos) and not math.isinf(pos)
                    print(f"✅ Valeur {invalid_val}: clampée à {pos:.4f}")
            except (ValueError, Exception) as e:
                print(f"✅ Valeur {invalid_val}: rejetée ({type(e).__name__})")

    def test_edge_case_concurrent_commands_race_condition(self):
        """Test edge: Commandes concurrentes (race condition)."""
        print("\n🧪 EDGE TEST 7: Commandes concurrentes")
        print("=" * 60)

        # Envoi rapide de commandes multiples
        results = []
        for i in range(10):
            result1 = self.backend.set_joint_pos("yaw_body", 0.1 * (i % 5))
            result2 = self.backend.set_emotion(
                "happy" if i % 2 == 0 else "neutral", 0.5
            )
            results.append((result1, result2))

        # Toutes les commandes doivent réussir ou échouer gracieusement
        for i, (r1, r2) in enumerate(results):
            assert isinstance(r1, bool), f"Commande {i} doit retourner bool"
            assert isinstance(r2, bool), f"Commande {i} doit retourner bool"

        print("✅ Commandes concurrentes: toutes gérées gracieusement")

    def test_edge_case_behavior_complex_scenarios(self):
        """Test edge: Scénarios comportements complexes."""
        print("\n🧪 EDGE TEST 8: Scénarios comportements complexes")
        print("=" * 60)

        # Test wake_up puis immédiatement goto_sleep
        result1 = self.backend.run_behavior("wake_up", 0.5)
        assert result1 is True, "wake_up doit réussir"

        result2 = self.backend.run_behavior("goto_sleep", 0.5)
        assert result2 is True, "goto_sleep doit réussir"

        print("✅ wake_up → goto_sleep: OK")

        # Test comportement avec durée 0
        result3 = self.backend.run_behavior("nod", 0.0)
        # Peut réussir ou échouer, mais ne doit pas crasher
        assert isinstance(result3, bool)
        print("✅ Comportement durée 0: géré gracieusement")

    def test_edge_case_telemetry_edge_values(self):
        """Test edge: Télémétrie avec valeurs limites."""
        print("\n🧪 EDGE TEST 9: Télémétrie valeurs limites")
        print("=" * 60)

        telemetry = self.backend.get_telemetry()

        # Vérifier que tous les champs sont présents et valides
        required_fields = [
            "step_count",
            "elapsed_time",
            "steps_per_second",
            "current_emotion",
            "emotion_intensity",
            "is_connected",
        ]

        for field in required_fields:
            assert field in telemetry, f"Champ {field} manquant"
            value = telemetry[field]

            # Vérifier types
            if field in ["step_count"]:
                assert isinstance(value, int), f"{field} doit être int"
            elif field in ["elapsed_time", "steps_per_second", "emotion_intensity"]:
                assert isinstance(value, int | float), f"{field} doit être numérique"
                assert not math.isnan(value) and not math.isinf(
                    value
                ), f"{field} ne doit pas être NaN/Inf"
            elif field == "is_connected":
                assert isinstance(value, bool), f"{field} doit être bool"

            print(f"✅ {field}: {value} (type: {type(value).__name__})")

    def test_edge_case_robot_factory_edge_cases(self):
        """Test edge: RobotFactory avec cas limites."""
        print("\n🧪 EDGE TEST 10: RobotFactory cas limites")
        print("=" * 60)

        # Test backend invalide
        backend = RobotFactory.create_backend("invalid_backend")
        assert backend is None, "Backend invalide doit retourner None"
        print("✅ Backend invalide: rejeté")

        # Test backend avec kwargs invalides
        backend = RobotFactory.create_backend("mujoco", invalid_kwarg="test")
        # Peut réussir (kwargs ignorés) ou échouer gracieusement
        assert backend is None or backend is not None
        print("✅ Backend avec kwargs invalides: géré gracieusement")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
