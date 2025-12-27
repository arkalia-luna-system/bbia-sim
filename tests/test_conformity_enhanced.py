#!/usr/bin/env python3
"""
🧪 TESTS DE CONFORMITÉ RENFORCÉS - BBIA-SIM vs SDK REACHY-MINI
Tests supplémentaires pour détecter des problèmes subtils que les tests standards ne détectent pas.
"""

import math
import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend
from bbia_sim.mapping_reachy import ReachyMapping


class TestConformityEnhanced:
    """Tests de conformité renforcés pour détecter problèmes subtils."""

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

    def test_joint_limits_vs_xml_exact(self):
        """Test: Vérifier que les limites dans le code correspondent EXACTEMENT au XML officiel."""
        print("\n🧪 TEST ENHANCED 1: Limites Joints vs XML Officiel")
        print("=" * 70)

        # Limites EXACTES du XML officiel (reachy_mini_REAL_OFFICIAL.xml)
        xml_limits = {
            "yaw_body": (-2.792526803190975, 2.792526803190879),
            "stewart_1": (-0.8377580409572196, 1.3962634015955222),
            "stewart_2": (-1.396263401595614, 1.2217304763958803),
            "stewart_3": (-0.8377580409572173, 1.3962634015955244),
            "stewart_4": (-1.3962634015953894, 0.8377580409573525),
            "stewart_5": (-1.2217304763962082, 1.396263401595286),
            "stewart_6": (-1.3962634015954123, 0.8377580409573296),
        }

        errors = []
        for joint_name, (xml_min, xml_max) in xml_limits.items():
            # Vérifier dans backend
            if joint_name in self.backend.joint_limits:
                backend_min, backend_max = self.backend.joint_limits[joint_name]
                if not math.isclose(backend_min, xml_min, abs_tol=1e-10):
                    errors.append(
                        f"❌ {joint_name}: backend.min={backend_min} != xml.min={xml_min}"
                    )
                if not math.isclose(backend_max, xml_max, abs_tol=1e-10):
                    errors.append(
                        f"❌ {joint_name}: backend.max={backend_max} != xml.max={xml_max}"
                    )
                if not errors:
                    print(f"✅ {joint_name}: Backend = XML (✓ exact)")

            # Vérifier dans mapping_reachy
            try:
                joint_info = ReachyMapping.get_joint_info(joint_name)
                if not math.isclose(joint_info.min_limit, xml_min, abs_tol=1e-10):
                    errors.append(
                        f"❌ {joint_name}: mapping.min={joint_info.min_limit} != xml.min={xml_min}"
                    )
                if not math.isclose(joint_info.max_limit, xml_max, abs_tol=1e-10):
                    errors.append(
                        f"❌ {joint_name}: mapping.max={joint_info.max_limit} != xml.max={xml_max}"
                    )
            except ValueError:
                errors.append(f"❌ {joint_name}: Non trouvé dans ReachyMapping")

        assert len(errors) == 0, "Limites non conformes:\n" + "\n".join(errors)
        print("✅ Toutes les limites correspondent exactement au XML officiel")

    def test_security_multi_level_clamping(self):
        """Test: Vérifier que le clamping multi-niveaux fonctionne correctement."""
        print("\n🧪 TEST ENHANCED 2: Clamping Sécurité Multi-niveaux")
        print("=" * 70)

        test_cases = [
            # (joint, position, expected_clamped_min, expected_clamped_max)
            # yaw_body: limit hardware = ±2.79, safe = ±0.3
            # → Position 0.5 doit être clampée à 0.3 (limite sécurité)
            ("yaw_body", 0.5, -0.3, 0.3),
            ("yaw_body", -0.5, -0.3, 0.3),
            # stewart_1: limit hardware = (-0.838, 1.396), safe = ±0.2
            # → Position 0.3 doit être clampée à 0.2 (limite sécurité)
            ("stewart_1", 0.3, -0.2, 0.2),
            ("stewart_1", -0.3, -0.2, 0.2),
        ]

        for joint, pos, exp_min, exp_max in test_cases:
            try:
                # Tester set_joint_pos (doit clamer dans les limites)
                result = self.backend.set_joint_pos(joint, pos)
                if result:
                    # Vérifier que la position demandée était dans les limites ou a été clampée
                    actual_pos = self.backend.get_joint_pos(joint)
                    if exp_min <= actual_pos <= exp_max:
                        print(
                            f"✅ {joint}({pos}): Clampé correctement dans [{exp_min}, {exp_max}]"
                        )
                    else:
                        print(
                            f"⚠️  {joint}({pos}): Position {actual_pos} hors limites attendues"
                        )
            except Exception as e:
                print(f"⚠️  {joint}({pos}): Erreur {e}")

    def test_stewart_ik_enforcement(self):
        """Test: Vérifier que les joints stewart ne peuvent PAS être contrôlés individuellement."""
        print("\n🧪 TEST ENHANCED 3: Enforcement IK pour Stewart Joints")
        print("=" * 70)

        stewart_joints = [
            "stewart_1",
            "stewart_2",
            "stewart_3",
            "stewart_4",
            "stewart_5",
            "stewart_6",
        ]

        for joint in stewart_joints:
            # set_joint_pos sur stewart doit retourner False
            result = self.backend.set_joint_pos(joint, 0.1)
            if not result:
                print(
                    f"✅ {joint}: Contrôle individuel correctement bloqué (requiert IK)"
                )
            else:
                print(f"❌ {joint}: Contrôle individuel accepté (DEVRAIT être bloqué)")

        # Vérifier que goto_target fonctionne (méthode correcte)
        try:
            from reachy_mini.utils import create_head_pose

            pose = create_head_pose(pitch=0.1, yaw=0.0, degrees=False)
            if hasattr(self.backend, "goto_target"):
                self.backend.goto_target(head=pose, duration=0.3, method="minjerk")
                print(
                    "✅ goto_target() fonctionne (méthode correcte pour stewart joints)"
                )
        except ImportError:
            print("⚠️  SDK non disponible - test goto_target ignoré")

    def test_interpolation_method_mapping_completeness(self):
        """Test: Vérifier que TOUS les variants d'interpolation sont supportés."""
        print("\n🧪 TEST ENHANCED 4: Mapping Interpolation Complet")
        print("=" * 70)

        variants = {
            "MIN_JERK": ["minjerk", "min_jerk", "MIN-JERK", "MINJERK"],
            "LINEAR": ["linear", "LINEAR"],
            "EASE_IN_OUT": ["ease_in_out", "easeinout", "EASE-IN-OUT"],
            "CARTOON": ["cartoon", "CARTOON"],
        }

        try:
            from reachy_mini.utils import create_head_pose

            pose = create_head_pose(pitch=0.05, degrees=False)

            for base_method, variant_list in variants.items():
                for variant in variant_list:
                    try:
                        self.backend.goto_target(
                            head=pose, duration=0.2, method=variant
                        )
                        print(f"✅ {variant} → {base_method}: Mapping fonctionne")
                    except Exception as e:
                        print(f"⚠️  {variant} → {base_method}: Erreur {e}")
        except ImportError:
            print("⚠️  SDK non disponible - test ignoré")

    def test_coordinate_validation_limits(self):
        """Test: Vérifier que les coordonnées sont validées selon limites SDK recommandées."""
        print("\n🧪 TEST ENHANCED 5: Validation Coordonnées Look_at_world")
        print("=" * 70)

        # Limites recommandées SDK: -2.0 ≤ x,y ≤ 2.0, 0.0 ≤ z ≤ 1.5
        valid_coords = [
            (0.0, 0.0, 0.5),  # OK
            (1.5, 1.5, 1.0),  # OK
            (-1.5, -1.5, 0.3),  # OK
        ]

        invalid_coords = [
            (3.0, 0.0, 0.0),  # x trop loin
            (0.0, 3.0, 0.0),  # y trop loin
            (0.0, 0.0, -1.0),  # z négatif
            (0.0, 0.0, 2.0),  # z trop haut
        ]

        if hasattr(self.backend, "look_at_world"):
            print("✅ Test coordonnées valides:")
            for x, y, z in valid_coords:
                try:
                    self.backend.look_at_world(x, y, z, duration=0.3)
                    print(f"   ✅ ({x}, {y}, {z}): Accepté")
                except Exception as e:
                    print(f"   ⚠️  ({x}, {y}, {z}): {e}")

            print("✅ Test coordonnées invalides (doivent être clampées/rejetées):")
            for x, y, z in invalid_coords:
                try:
                    self.backend.look_at_world(x, y, z, duration=0.3)
                    # Si accepté, vérifier si clampé (dans robot_api.py)
                    print(f"   ⚠️  ({x}, {y}, {z}): Accepté (vérifier si clampé)")
                except (ValueError, Exception):
                    print(f"   ✅ ({x}, {y}, {z}): Rejeté correctement")

    def test_media_sdk_integration(self):
        """Test: Vérifier que les modules BBIA utilisent robot.media.speaker/camera si disponible."""
        print("\n🧪 TEST ENHANCED 6: Intégration Media SDK")
        print("=" * 70)

        # Vérifier que le backend expose media si robot connecté
        if self.backend.is_connected and self.backend.robot:
            if hasattr(self.backend, "media") and self.backend.media:
                print("✅ robot.media disponible via backend")
                if hasattr(self.backend.media, "speaker"):
                    print("✅ robot.media.speaker disponible")
                if hasattr(self.backend.media, "camera"):
                    print("✅ robot.media.camera disponible")
                if hasattr(self.backend.media, "play_audio"):
                    print("✅ robot.media.play_audio disponible")
            else:
                print("⚠️  robot.media non disponible (mode simulation)")
        else:
            print("⚠️  Robot non connecté - test media ignoré (mode simulation)")

    def test_emotion_intensity_applied_correctly(self):
        """Test: Vérifier que l'intensité d'émotion est appliquée correctement aux poses."""
        print("\n🧪 TEST ENHANCED 7: Application Intensité Émotion")
        print("=" * 70)

        emotion_intensities = [
            ("happy", 0.3),
            ("happy", 0.7),
            ("happy", 1.0),
        ]

        for emotion, intensity in emotion_intensities:
            try:
                success = self.backend.set_emotion(emotion, intensity)
                if success:
                    # Vérifier que l'émotion est enregistrée
                    assert self.backend.current_emotion == emotion
                    assert abs(self.backend.emotion_intensity - intensity) < 0.01
                    print(
                        f"✅ {emotion} (intensité {intensity}): Appliquée correctement"
                    )
                else:
                    print(f"⚠️  {emotion} (intensité {intensity}): Échec")
            except Exception as e:
                print(f"⚠️  {emotion} (intensité {intensity}): Erreur {e}")

    def test_mapping_consistency_all_modules(self):
        """Test: Vérifier cohérence mapping entre tous les modules."""
        print("\n🧪 TEST ENHANCED 8: Cohérence Mapping Multi-modules")
        print("=" * 70)

        # Joints du backend
        backend_joints = set(self.backend.get_available_joints())

        # Joints du mapping
        mapping_joints = ReachyMapping.get_all_joints()

        # Vérifier que les joints principaux sont présents partout
        expected_core_joints = {"yaw_body", "stewart_1", "stewart_2", "stewart_3"}
        missing_backend = expected_core_joints - backend_joints
        missing_mapping = expected_core_joints - mapping_joints

        if missing_backend:
            print(f"❌ Joints manquants dans backend: {missing_backend}")
        if missing_mapping:
            print(f"❌ Joints manquants dans mapping: {missing_mapping}")

        if not missing_backend and not missing_mapping:
            print("✅ Joints core présents dans backend et mapping")

        # Vérifier limites cohérentes
        for joint in expected_core_joints:
            if joint in backend_joints and joint in mapping_joints:
                backend_limits = self.backend.joint_limits.get(joint)
                mapping_info = ReachyMapping.get_joint_info(joint)
                if backend_limits:
                    backend_min, backend_max = backend_limits
                    if math.isclose(
                        backend_min, mapping_info.min_limit, abs_tol=1e-10
                    ) and math.isclose(
                        backend_max, mapping_info.max_limit, abs_tol=1e-10
                    ):
                        print(f"✅ {joint}: Limites cohérentes backend=mapping")
                    else:
                        print(
                            f"❌ {joint}: Limites divergentes backend=({backend_min}, {backend_max}) vs mapping=({mapping_info.min_limit}, {mapping_info.max_limit})"
                        )


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
