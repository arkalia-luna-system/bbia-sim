#!/usr/bin/env python3
"""
🧪 TESTS DE CONFORMITÉ STRICTE REACHY-MINI
Tests ultra-stricts pour détecter tous les problèmes d'expert robotique
Basé sur les spécifications exactes du SDK officiel Reachy Mini
"""

import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

# JOINTS LIMITS EXACTES du modèle XML officiel (tirées de reachy_mini_REAL_OFFICIAL.xml)
EXACT_JOINT_LIMITS = {
    "stewart_1": (-0.8377580409572196, 1.3962634015955222),
    "stewart_2": (-1.396263401595614, 1.2217304763958803),
    "stewart_3": (-0.8377580409572173, 1.3962634015955244),
    "stewart_4": (-1.3962634015953894, 0.8377580409573525),
    "stewart_5": (-1.2217304763962082, 1.396263401595286),
    "stewart_6": (-1.3962634015954123, 0.8377580409573296),
    "yaw_body": (-2.792526803190975, 2.792526803190879),
    # Antennes: limites conservatrices pour sécurité
    "left_antenna": (-1.0, 1.0),
    "right_antenna": (-1.0, 1.0),
}

# Tolérance pour comparaison des limites (erreurs de précision float)
LIMIT_TOLERANCE = 1e-10


class TestReachyMiniStrictConformity:
    """Tests ultra-stricts de conformité pour détecter tous les problèmes."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.backend = ReachyMiniBackend()
        self.backend.connect()

    def teardown_method(self):
        """Nettoyage après chaque test."""
        if self.backend:
            self.backend.disconnect()

    def test_strict_joint_limits_exact_values(self):
        """Test STRICT: Vérifier que les limites de joints sont EXACTEMENT conformes au XML officiel."""
        print("\n🧪 TEST STRICT 1: Limites exactes des joints")
        print("=" * 70)

        errors = []
        for joint_name, (expected_min, expected_max) in EXACT_JOINT_LIMITS.items():
            if joint_name not in self.backend.joint_limits:
                errors.append(
                    f"❌ Joint {joint_name} MANQUANT dans joint_limits du backend"
                )
                continue

            actual_min, actual_max = self.backend.joint_limits[joint_name]

            # Vérification STRICTE: les valeurs doivent être EXACTEMENT identiques
            min_diff = abs(actual_min - expected_min)
            max_diff = abs(actual_max - expected_max)

            if min_diff > LIMIT_TOLERANCE:
                errors.append(
                    f"❌ {joint_name}: min limite incorrecte - "
                    f"attendu {expected_min:.15f}, obtenu {actual_min:.15f} "
                    f"(diff: {min_diff:.2e})"
                )
            elif max_diff > LIMIT_TOLERANCE:
                errors.append(
                    f"❌ {joint_name}: max limite incorrecte - "
                    f"attendu {expected_max:.15f}, obtenu {actual_max:.15f} "
                    f"(diff: {max_diff:.2e})"
                )
            else:
                print(
                    f"✅ {joint_name}: limites EXACTES [{actual_min:.15f}, {actual_max:.15f}]"
                )

        assert len(errors) == 0, "Erreurs limites joints:\n" + "\n".join(errors)

    def test_strict_forbidden_joints_enforcement(self):
        """Test STRICT: Vérifier que les joints interdits sont IMPOSSIBLES à contrôler."""
        print("\n🧪 TEST STRICT 2: Protection stricte joints interdits")
        print("=" * 70)

        # Note: Antennes maintenant optionnelles (commentées dans forbidden_joints)
        # Tester avec joints passifs qui sont toujours interdits
        forbidden_joints = ["passive_1", "passive_2"]  # Joints vraiment interdits
        errors = []

        for joint in forbidden_joints:
            # Test 1: set_joint_pos doit retourner False
            result = self.backend.set_joint_pos(joint, 0.1)
            if result:
                errors.append(
                    f"❌ {joint}: set_joint_pos a réussi alors que le joint est interdit!"
                )
            else:
                print(f"✅ {joint}: set_joint_pos correctement bloqué")

            # Test 2: Le joint doit être dans forbidden_joints
            if joint not in self.backend.forbidden_joints:
                errors.append(
                    f"❌ {joint}: devrait être dans forbidden_joints mais absent!"
                )
            else:
                print(f"✅ {joint}: présent dans forbidden_joints")

        assert len(errors) == 0, "Erreurs protection joints:\n" + "\n".join(errors)

    def test_strict_stewart_individual_control_impossible(self):
        """Test STRICT: Vérifier que les joints stewart NE PEUVENT PAS être contrôlés individuellement."""
        print("\n🧪 TEST STRICT 3: Interdiction contrôle individuel stewart")
        print("=" * 70)

        stewart_joints = [f"stewart_{i}" for i in range(1, 7)]
        errors = []

        for joint in stewart_joints:
            result = self.backend.set_joint_pos(joint, 0.1)
            # CRITIQUE: set_joint_pos doit retourner False car contrôle individuel impossible (IK)
            if result:
                errors.append(
                    f"❌ {joint}: set_joint_pos a réussi alors que le contrôle individuel est "
                    "impossible (plateforme Stewart utilise IK). "
                    "Utiliser goto_target() ou set_target_head_pose() à la place!"
                )
            else:
                print(
                    f"✅ {joint}: set_joint_pos correctement bloqué (controle individuel impossible)"
                )

        assert len(errors) == 0, "Erreurs contrôle stewart:\n" + "\n".join(errors)

    def test_strict_clamping_multi_level(self):
        """Test STRICT: Vérifier le clamping multi-niveaux (hardware puis sécurité)."""
        print("\n🧪 TEST STRICT 4: Clamping multi-niveaux")
        print("=" * 70)

        errors = []
        test_cases = [
            (
                "yaw_body",
                3.0,
                2.792526803190879,
            ),  # Hors limite hardware → clamp hardware
            (
                "yaw_body",
                0.5,
                0.3,
            ),  # Dans limite hardware mais > safe_amplitude → clamp sécurité
            ("stewart_1", 2.0, 1.3962634015955222),  # Hors limite hardware
            ("stewart_1", 0.5, 0.3),  # > safe_amplitude → clamp sécurité
        ]

        for joint_name, input_value, expected_max_value in test_cases:
            # Positionner le joint
            self.backend.set_joint_pos(joint_name, input_value)

            # Lire la position réelle (doit être clampée)
            actual_value = self.backend.get_joint_pos(joint_name)

            # Vérifier que la valeur est clampée correctement
            if actual_value is None:
                errors.append(f"❌ {joint_name}: get_joint_pos retourne None")
                continue

            # La valeur doit être <= expected_max_value (clampé)
            if abs(actual_value) > abs(expected_max_value) + 0.01:
                errors.append(
                    f"❌ {joint_name}: Clamping échoué - "
                    f"entrée {input_value:.3f}, attendu <= {expected_max_value:.3f}, "
                    f"obtenu {actual_value:.3f}"
                )
            else:
                print(
                    f"✅ {joint_name}: Clamping correct "
                    f"{input_value:.3f} → {actual_value:.3f} (limite: {expected_max_value:.3f})"
                )

        assert len(errors) == 0, "Erreurs clamping:\n" + "\n".join(errors)

    def test_strict_head_positions_structure_validation(self):
        """Test STRICT: Vérifier la structure robuste de head_positions (6 ou 12 éléments)."""
        print("\n🧪 TEST STRICT 5: Validation structure head_positions")
        print("=" * 70)

        head_pos, antenna_pos = self.backend.get_current_joint_positions()

        errors = []

        # Vérifier que head_pos est une liste
        if not isinstance(head_pos, list):
            errors.append(
                f"❌ head_positions doit être une liste, obtenu {type(head_pos)}"
            )

        # Vérifier la longueur (6 ou 12)
        if len(head_pos) not in [6, 12]:
            errors.append(
                f"❌ head_positions doit avoir 6 ou 12 éléments, obtenu {len(head_pos)}"
            )

        # Vérifier qu'il n'y a pas de NaN ou inf
        for i, val in enumerate(head_pos):
            if math.isnan(val) or math.isinf(val):
                errors.append(f"❌ head_positions[{i}] contient NaN ou inf: {val}")

        # Vérifier antenna_positions
        if not isinstance(antenna_pos, list):
            errors.append(
                f"❌ antenna_positions doit être une liste, obtenu {type(antenna_pos)}"
            )

        if len(antenna_pos) != 2:
            errors.append(
                f"❌ antenna_positions doit avoir 2 éléments, obtenu {len(antenna_pos)}"
            )

        if errors:
            print("\n".join(errors))
            raise AssertionError(
                "Erreurs structure head_positions:\n" + "\n".join(errors)
            )

        print(
            f"✅ head_positions: {len(head_pos)} éléments, "
            f"antenna_positions: {len(antenna_pos)} éléments, aucun NaN/inf"
        )

    def test_strict_goto_target_method_validation(self):
        """Test STRICT: Vérifier la validation robuste de la méthode d'interpolation."""
        print("\n🧪 TEST STRICT 6: Validation méthode goto_target")
        print("=" * 70)

        try:
            from reachy_mini.utils import create_head_pose

            pose = create_head_pose(pitch=0.1, yaw=0.0, degrees=False)
            errors = []

            # Test méthodes valides
            valid_methods = ["minjerk", "MIN_JERK", "linear", "LINEAR"]
            for method in valid_methods:
                try:
                    self.backend.goto_target(head=pose, duration=0.5, method=method)
                    print(f"✅ Méthode '{method}' acceptée")
                except Exception as e:
                    errors.append(f"❌ Méthode valide '{method}' rejetée: {e}")

            # Test méthode invalide (devrait être gérée gracieusement)
            try:
                self.backend.goto_target(
                    head=pose, duration=0.5, method="invalid_method"
                )
                print("✅ Méthode invalide gérée (fallback accepté)")
            except Exception as e:
                # C'est OK si une erreur est levée pour méthode invalide
                print(f"ℹ️  Méthode invalide rejetée (acceptable): {e}")

        except ImportError:
            print("⚠️  SDK reachy_mini non disponible, test ignoré")
            pytest.skip("SDK non disponible")

        assert len(errors) == 0, "Erreurs validation méthode:\n" + "\n".join(errors)

    def test_strict_yaw_body_reading_robustness(self):
        """Test STRICT: Vérifier la lecture robuste de yaw_body (multi-méthode fallback)."""
        print("\n🧪 TEST STRICT 7: Robustesse lecture yaw_body")
        print("=" * 70)

        # Test que get_joint_pos retourne une valeur valide pour yaw_body
        yaw_value = self.backend.get_joint_pos("yaw_body")

        errors = []

        if yaw_value is None:
            errors.append("❌ get_joint_pos('yaw_body') retourne None")
        elif math.isnan(yaw_value):
            errors.append("❌ get_joint_pos('yaw_body') retourne NaN")
        elif math.isinf(yaw_value):
            errors.append("❌ get_joint_pos('yaw_body') retourne inf")
        elif not isinstance(yaw_value, int | float):
            errors.append(
                f"❌ get_joint_pos('yaw_body') retourne {type(yaw_value)} au lieu de float"
            )
        else:
            print(f"✅ yaw_body lu correctement: {yaw_value:.6f} rad")

        # Vérifier que la valeur est dans des limites raisonnables
        if isinstance(yaw_value, int | float):
            if abs(yaw_value) > 3.2:  # Légèrement > limite max (2.79)
                errors.append(
                    f"⚠️  yaw_body hors limites raisonnables: {yaw_value:.6f} rad"
                )

        assert len(errors) == 0, "Erreurs lecture yaw_body:\n" + "\n".join(errors)

    def test_strict_stewart_joint_reading_robustness(self):
        """Test STRICT: Vérifier la lecture robuste de tous les joints stewart."""
        print("\n🧪 TEST STRICT 8: Robustesse lecture joints stewart")
        print("=" * 70)

        errors = []
        stewart_joints = [f"stewart_{i}" for i in range(1, 7)]

        for joint_name in stewart_joints:
            value = self.backend.get_joint_pos(joint_name)

            if value is None:
                errors.append(f"❌ {joint_name}: get_joint_pos retourne None")
            elif math.isnan(value):
                errors.append(f"❌ {joint_name}: get_joint_pos retourne NaN")
            elif math.isinf(value):
                errors.append(f"❌ {joint_name}: get_joint_pos retourne inf")
            elif not isinstance(value, int | float):
                errors.append(
                    f"❌ {joint_name}: type incorrect {type(value)} au lieu de float"
                )
            else:
                # Vérifier que la valeur est dans les limites hardware
                min_limit, max_limit = EXACT_JOINT_LIMITS[joint_name]
                if value < min_limit - 0.1 or value > max_limit + 0.1:
                    errors.append(
                        f"⚠️  {joint_name}: valeur {value:.6f} hors limites "
                        f"[{min_limit:.6f}, {max_limit:.6f}]"
                    )
                else:
                    print(f"✅ {joint_name}: {value:.6f} rad (dans limites)")

        assert len(errors) == 0, "Erreurs lecture stewart:\n" + "\n".join(errors)

    def test_strict_parameter_validation(self):
        """Test STRICT: Vérifier la validation stricte de tous les paramètres."""
        print("\n🧪 TEST STRICT 9: Validation stricte paramètres")
        print("=" * 70)

        errors = []

        # Test 1: duration doit être positive
        try:
            from reachy_mini.utils import create_head_pose

            pose = create_head_pose(pitch=0.1)
            self.backend.goto_target(head=pose, duration=-1.0)
            errors.append("❌ goto_target avec duration négative accepté!")
        except (ValueError, Exception):
            print("✅ Duration négative correctement rejetée")

        # Test 2: antennas doit être convertible en liste
        try:
            antennas_array = np.array([0.1, -0.1])
            self.backend.goto_target(antennas=antennas_array, duration=0.5)
            print("✅ antennas numpy array converti en liste")
        except Exception as e:
            errors.append(f"❌ Conversion antennas numpy array échouée: {e}")

        # Test 3: look_at_world coordonnées invalides doivent être gérées
        try:
            result = self.backend.look_at_world(1e10, 1e10, 1e10)
            if result is not None:
                print("✅ look_at_world avec coordonnées extrêmes géré")
            else:
                errors.append(
                    "❌ look_at_world devrait retourner une pose même pour coordonnées extrêmes"
                )
        except Exception as e:
            print(
                f"ℹ️  look_at_world avec coordonnées extrêmes rejeté (acceptable): {e}"
            )

        assert len(errors) == 0, "Erreurs validation paramètres:\n" + "\n".join(errors)

    def test_strict_performance_latency(self):
        """Test STRICT: Vérifier que la latence des opérations est acceptable."""
        print("\n🧪 TEST STRICT 10: Performance latence")
        print("=" * 70)

        iterations = 50
        operations = []

        # Mesurer latence set_joint_pos
        start = time.time()
        for _ in range(iterations):
            self.backend.set_joint_pos("yaw_body", 0.1)
        set_joint_latency = (time.time() - start) / iterations * 1000  # ms
        operations.append(("set_joint_pos", set_joint_latency, 10.0))

        # Mesurer latence get_joint_pos
        start = time.time()
        for _ in range(iterations):
            self.backend.get_joint_pos("yaw_body")
        get_joint_latency = (time.time() - start) / iterations * 1000  # ms
        operations.append(("get_joint_pos", get_joint_latency, 5.0))

        errors = []
        for op_name, latency_ms, max_latency_ms in operations:
            if latency_ms > max_latency_ms:
                errors.append(
                    f"❌ {op_name}: latence {latency_ms:.2f}ms > {max_latency_ms:.2f}ms"
                )
            else:
                print(
                    f"✅ {op_name}: latence {latency_ms:.2f}ms < {max_latency_ms:.2f}ms"
                )

        assert len(errors) == 0, "Erreurs performance:\n" + "\n".join(errors)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
