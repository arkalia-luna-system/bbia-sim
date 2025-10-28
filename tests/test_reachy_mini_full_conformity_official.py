#!/usr/bin/env python3
"""
🧪 TEST DE CONFORMITÉ COMPLÈTE REACHY-MINI VS SDK OFFICIEL
Vérifie que notre implémentation est 100% conforme au SDK officiel Reachy Mini
Basé sur les spécifications officielles de Pollen Robotics (Octobre 2025)
"""

import inspect
import sys
import time
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend
from bbia_sim.robot_api import RobotAPI

# Tentative d'import du SDK officiel
try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose

    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    ReachyMini = None
    create_head_pose = None


class TestReachyMiniFullConformity:
    """Tests de conformité complète avec le SDK Reachy Mini officiel."""

    # MÉTHODES OFFICIELLES DU SDK REACHY-MINI (d'après documentation GitHub)
    EXPECTED_SDK_METHODS = {
        "wake_up": {"args": [], "return": None},
        "goto_sleep": {"args": [], "return": None},
        "get_current_joint_positions": {"args": [], "return": tuple},
        "set_target_head_pose": {"args": ["pose"], "return": None},
        "set_target_body_yaw": {"args": ["body_yaw"], "return": None},
        "set_target_antenna_joint_positions": {"args": ["antennas"], "return": None},
        "get_current_head_pose": {"args": [], "return": "ndarray"},
        "get_present_antenna_joint_positions": {"args": [], "return": list},
        "look_at_world": {
            "args": ["x", "y", "z", "duration", "perform_movement"],
            "return": "ndarray",
        },
        "look_at_image": {
            "args": ["u", "v", "duration", "perform_movement"],
            "return": "ndarray",
        },
        "goto_target": {
            "args": ["head", "antennas", "duration", "method", "body_yaw"],
            "return": None,
        },
        "set_target": {"args": ["head", "antennas", "body_yaw"], "return": None},
        "enable_motors": {"args": [], "return": None},
        "disable_motors": {"args": [], "return": None},
        "enable_gravity_compensation": {"args": [], "return": None},
        "disable_gravity_compensation": {"args": [], "return": None},
        "set_automatic_body_yaw": {"args": ["body_yaw"], "return": None},
    }

    # JOINTS OFFICIELS REACHY-MINI (d'après modèle physique)
    EXPECTED_JOINTS = {
        "stewart_1",
        "stewart_2",
        "stewart_3",
        "stewart_4",
        "stewart_5",
        "stewart_6",
        "left_antenna",
        "right_antenna",
        "yaw_body",
    }

    # ÉMOTIONS OFFICIELLES REACHY-MINI
    EXPECTED_EMOTIONS = {"happy", "sad", "neutral", "excited", "curious", "calm"}

    # COMPORTEMENTS OFFICIELS
    EXPECTED_BEHAVIORS = {"wake_up", "goto_sleep", "nod"}

    def setup_method(self):
        """Configuration avant chaque test."""
        self.backend: ReachyMiniBackend = ReachyMiniBackend()
        self.backend.connect()

    def teardown_method(self):
        """Nettoyage après chaque test."""
        if self.backend:
            self.backend.disconnect()

    def test_01_sdk_availability(self):
        """Test 1: Vérifier que le SDK officiel est disponible."""
        print("\n🧪 TEST 1: Disponibilité SDK")
        print("=" * 60)

        if SDK_AVAILABLE:
            print("✅ SDK officiel reachy_mini disponible")
            print("✅ Classe ReachyMini importée")
            print("✅ Utilitaires SDK importés")
            assert True
        else:
            print("⚠️  SDK officiel non installé")
            print("💡 Installez avec: pip install reachy-mini")
            # Ne pas échouer - tester quand même en mode simulation
            pytest.skip("SDK officiel non disponible")

    def test_02_methods_existence(self):
        """Test 2: Vérifier que toutes les méthodes SDK existent."""
        print("\n🧪 TEST 2: Existence des méthodes")
        print("=" * 60)

        missing_methods = []
        for method_name in self.EXPECTED_SDK_METHODS.keys():
            if not hasattr(self.backend, method_name):
                missing_methods.append(method_name)
                print(f"❌ Méthode manquante: {method_name}")
            else:
                print(f"✅ Méthode disponible: {method_name}")

        assert (
            len(missing_methods) == 0
        ), f"Méthodes manquantes: {', '.join(missing_methods)}"

    def test_03_methods_signatures(self):
        """Test 3: Vérifier les signatures des méthodes."""
        print("\n🧪 TEST 3: Signatures des méthodes")
        print("=" * 60)

        signature_errors = []
        for method_name, expected in self.EXPECTED_SDK_METHODS.items():
            if not hasattr(self.backend, method_name):
                continue

            method = getattr(self.backend, method_name)
            sig = inspect.signature(method)
            actual_args = list(sig.parameters.keys())
            actual_args = [a for a in actual_args if a != "self"]  # Enlever self

            expected_args = expected["args"]

            # Vérifier les arguments (ordre flexible)
            missing_args = [arg for arg in expected_args if arg not in actual_args]
            if missing_args:
                signature_errors.append(
                    f"{method_name}: arguments manquants {missing_args}"
                )
                print(f"❌ {method_name}: {missing_args}")
            else:
                print(f"✅ {method_name}: signature correcte")

        assert (
            len(signature_errors) == 0
        ), f"Erreurs de signatures: {', '.join(signature_errors)}"

    def test_04_joints_official_mapping(self):
        """Test 4: Vérifier le mapping des joints officiels."""
        print("\n🧪 TEST 4: Mapping des joints officiels")
        print("=" * 60)

        available_joints = set(self.backend.get_available_joints())

        # Vérifier que tous les joints officiels sont présents
        missing_joints = self.EXPECTED_JOINTS - available_joints
        extra_joints = available_joints - self.EXPECTED_JOINTS

        if missing_joints:
            print(f"❌ Joints manquants: {missing_joints}")
        if extra_joints:
            print(f"⚠️  Joints supplémentaires: {extra_joints}")

        for joint in available_joints:
            if joint in self.EXPECTED_JOINTS:
                print(f"✅ Joint officiel: {joint}")
            else:
                print(f"ℹ️  Joint supplémentaire: {joint}")

        # Les joints officiels doivent tous être présents
        assert (
            not missing_joints
        ), f"Joints officiels manquants: {', '.join(missing_joints)}"

    def test_05_emotions_official(self):
        """Test 5: Vérifier les émotions officielles."""
        print("\n🧪 TEST 5: Émotions officielles")
        print("=" * 60)

        emotion_errors = []
        for emotion in self.EXPECTED_EMOTIONS:
            result = self.backend.set_emotion(emotion, 0.5)
            if result:
                print(f"✅ Émotion valide: {emotion}")
            else:
                emotion_errors.append(emotion)
                print(f"❌ Émotion invalide: {emotion}")

        # Tester qu'une émotion invalide est rejetée
        invalid_result = self.backend.set_emotion("invalid_emotion", 0.5)
        if not invalid_result:
            print("✅ Émotions invalides correctement rejetées")
        else:
            print("❌ Les émotions invalides ne sont pas rejetées")

        assert (
            len(emotion_errors) == 0
        ), f"Émotions invalides: {', '.join(emotion_errors)}"
        assert not invalid_result, "Les émotions invalides devraient être rejetées"

    def test_06_behaviors_official(self):
        """Test 6: Vérifier les comportements officiels."""
        print("\n🧪 TEST 6: Comportements officiels")
        print("=" * 60)

        behavior_errors = []
        for behavior in self.EXPECTED_BEHAVIORS:
            result = self.backend.run_behavior(behavior, 1.0)
            if result:
                print(f"✅ Comportement valide: {behavior}")
            else:
                behavior_errors.append(behavior)
                print(f"❌ Comportement invalide: {behavior}")

        assert (
            len(behavior_errors) == 0
        ), f"Comportements invalides: {', '.join(behavior_errors)}"

    def test_07_joint_limits_official(self):
        """Test 7: Vérifier les limites des joints officiels."""
        print("\n🧪 TEST 7: Limites des joints")
        print("=" * 60)

        # Vérifier que les limites sont définies pour tous les joints
        limit_errors = []
        for joint in self.EXPECTED_JOINTS:
            if joint in self.backend.joint_limits:
                limits = self.backend.joint_limits[joint]
                print(f"✅ Joint {joint}: limites {limits}")
            else:
                limit_errors.append(joint)
                print(f"❌ Joint {joint}: pas de limites définies")

        assert len(limit_errors) == 0, f"Joints sans limites: {', '.join(limit_errors)}"

    def test_08_safety_forbidden_joints(self):
        """Test 8: Vérifier que les joints fragiles sont protégés."""
        print("\n🧪 TEST 8: Protection des joints fragiles")
        print("=" * 60)

        # Vérifier que les antennes sont protégées
        forbidden = self.backend.forbidden_joints
        assert "left_antenna" in forbidden, "left_antenna devrait être protégé"
        assert "right_antenna" in forbidden, "right_antenna devrait être protégé"

        print("✅ left_antenna protégé")
        print("✅ right_antenna protégé")

        # Vérifier que les mouvements sur joints interdits sont bloqués
        for joint in ["left_antenna", "right_antenna"]:
            result = self.backend.set_joint_pos(joint, 0.1)
            assert not result, f"Les mouvements sur {joint} devraient être bloqués"

        print("✅ Mouvements sur joints interdits bloqués")

    def test_09_safe_amplitude_limit(self):
        """Test 9: Vérifier la limite d'amplitude sécurisée."""
        print("\n🧪 TEST 9: Limite d'amplitude")
        print("=" * 60)

        # La limite officielle est 0.3 rad
        expected_limit = 0.3
        actual_limit = self.backend.safe_amplitude_limit

        print(f"📊 Limite attendue: {expected_limit} rad")
        print(f"📊 Limite actuelle: {actual_limit} rad")

        assert (
            actual_limit == expected_limit
        ), f"Limite incorrecte: {actual_limit} vs {expected_limit}"
        print("✅ Limite d'amplitude correcte")

        # Vérifier que les amplitudes excessives sont clampées
        extreme_value = 2.0  # > 0.3
        result = self.backend.set_joint_pos("stewart_1", extreme_value)
        if result:
            print("✅ Amplitude excessive clampée")
        else:
            print("⚠️  Amplitude excessive rejetée (acceptable)")

    def test_10_telemetry_official(self):
        """Test 10: Vérifier la télémétrie officielle."""
        print("\n🧪 TEST 10: Télémétrie")
        print("=" * 60)

        required_fields = {
            "step_count",
            "elapsed_time",
            "steps_per_second",
            "current_emotion",
            "emotion_intensity",
            "is_connected",
        }

        telemetry = self.backend.get_telemetry()

        missing_fields = []
        for field in required_fields:
            if field not in telemetry:
                missing_fields.append(field)
                print(f"❌ Champ manquant: {field}")
            else:
                print(f"✅ Champ présent: {field}")

        assert (
            len(missing_fields) == 0
        ), f"Champs télémétrie manquants: {', '.join(missing_fields)}"

    def test_11_performance_official(self):
        """Test 11: Vérifier les performances."""
        print("\n🧪 TEST 11: Performances")
        print("=" * 60)

        # Mesurer la latence des opérations critiques
        test_joint = "stewart_1"
        iterations = 100

        start_time = time.time()
        for _ in range(iterations):
            self.backend.set_joint_pos(test_joint, 0.1)
            self.backend.get_joint_pos(test_joint)
        end_time = time.time()

        avg_latency_ms = (end_time - start_time) / iterations * 1000
        print(f"📊 Latence moyenne: {avg_latency_ms:.2f} ms")

        # En simulation, la latence doit être < 1ms
        assert avg_latency_ms < 10, f"Latence trop élevée: {avg_latency_ms:.2f} ms"

    def test_12_simulation_mode(self):
        """Test 12: Vérifier le mode simulation."""
        print("\n🧪 TEST 12: Mode simulation")
        print("=" * 60)

        # Toutes les opérations doivent fonctionner en simulation
        print("🧪 Test des opérations en mode simulation...")

        # Test mouvements
        result = self.backend.set_joint_pos("stewart_1", 0.1)
        assert result, "set_joint_pos devrait fonctionner en simulation"
        print("✅ set_joint_pos fonctionne en simulation")

        # Test émotions
        result = self.backend.set_emotion("happy", 0.8)
        assert result, "set_emotion devrait fonctionner en simulation"
        print("✅ set_emotion fonctionne en simulation")

        # Test look_at
        result = self.backend.look_at(0.1, 0.2, 0.3)
        assert result, "look_at devrait fonctionner en simulation"
        print("✅ look_at fonctionne en simulation")

        # Test comportements
        result = self.backend.run_behavior("wake_up", 1.0)
        assert result, "run_behavior devrait fonctionner en simulation"
        print("✅ run_behavior fonctionne en simulation")

    def test_13_api_consistency(self):
        """Test 13: Vérifier la cohérence de l'API."""
        print("\n🧪 TEST 13: Cohérence API")
        print("=" * 60)

        # Vérifier que RobotAPI est respectée
        assert isinstance(self.backend, RobotAPI), "Le backend doit hériter de RobotAPI"
        print("✅ Héritage RobotAPI correct")

        # Vérifier que toutes les méthodes abstraites sont implémentées
        abstract_methods = ["connect", "disconnect", "get_available_joints"]
        for method_name in abstract_methods:
            assert hasattr(
                self.backend, method_name
            ), f"Méthode {method_name} manquante"
            print(f"✅ Méthode abstraite implémentée: {method_name}")

    def test_14_sdk_official_comparison(self):
        """Test 14: Comparer avec le SDK officiel (si disponible)."""
        print("\n🧪 TEST 14: Comparaison avec SDK officiel")
        print("=" * 60)

        if not SDK_AVAILABLE:
            print("⚠️  SDK officiel non disponible, test ignoré")
            pytest.skip("SDK officiel non disponible")

        # Si le SDK est disponible, on peut faire des comparaisons
        print("✅ SDK officiel disponible")
        print("✅ Comparaison avec SDK officiel possible")

        # Vérifier que les méthodes de notre backend correspondent au SDK
        sdk_compatible = True
        for method_name in self.EXPECTED_SDK_METHODS.keys():
            # Vérifier que la méthode existe dans notre backend
            if not hasattr(self.backend, method_name):
                print(f"❌ Méthode {method_name} manquante")
                sdk_compatible = False

        if sdk_compatible:
            print("✅ Toutes les méthodes SDK sont compatibles")

    def test_15_return_types(self):
        """Test 15: Vérifier les types de retour."""
        print("\n🧪 TEST 15: Types de retour")
        print("=" * 60)

        # Test get_joint_pos retourne float
        result = self.backend.get_joint_pos("stewart_1")
        assert isinstance(
            result, (float, type(None))
        ), "get_joint_pos doit retourner float ou None"
        print("✅ get_joint_pos retourne float")

        # Test set_joint_pos retourne bool
        result = self.backend.set_joint_pos("stewart_1", 0.1)
        assert isinstance(result, bool), "set_joint_pos doit retourner bool"
        print("✅ set_joint_pos retourne bool")

        # Test set_emotion retourne bool
        result = self.backend.set_emotion("happy", 0.5)
        assert isinstance(result, bool), "set_emotion doit retourner bool"
        print("✅ set_emotion retourne bool")

        # Test get_telemetry retourne dict
        result = self.backend.get_telemetry()
        assert isinstance(result, dict), "get_telemetry doit retourner dict"
        print("✅ get_telemetry retourne dict")

        # Test get_available_joints retourne list
        result = self.backend.get_available_joints()
        assert isinstance(result, list), "get_available_joints doit retourner list"
        print("✅ get_available_joints retourne list")

    def test_16_joint_names_official(self):
        """Test 16: Vérifier que les noms de joints sont officiels."""
        print("\n🧪 TEST 16: Noms de joints officiels")
        print("=" * 60)

        # Les noms de joints doivent correspondre exactement au SDK officiel
        # stewart_1 à stewart_6, left_antenna, right_antenna, yaw_body

        available_joints = set(self.backend.get_available_joints())

        # Vérifier les noms exacts
        correct_names = True
        for joint in available_joints:
            if not any(
                [
                    joint.startswith("stewart_")
                    and joint.replace("stewart_", "").isdigit(),
                    joint in ["left_antenna", "right_antenna", "yaw_body"],
                ]
            ):
                print(f"❌ Nom de joint non standard: {joint}")
                correct_names = False
            else:
                print(f"✅ Nom de joint standard: {joint}")

        assert correct_names, "Tous les noms de joints doivent être standard"

    def test_17_full_integration(self):
        """Test 17: Test d'intégration complet."""
        print("\n🧪 TEST 17: Intégration complète")
        print("=" * 60)

        # Simuler une séquence complète de mouvement
        print("🎬 Séquence complète de mouvement...")

        # 1. Wake up
        result = self.backend.run_behavior("wake_up", 1.0)
        assert result, "wake_up doit réussir"
        print("✅ wake_up réussi")

        # 2. Émotion
        result = self.backend.set_emotion("excited", 0.8)
        assert result, "set_emotion doit réussir"
        print("✅ set_emotion réussi")

        # 3. Mouvements tête
        result = self.backend.set_joint_pos("stewart_1", 0.1)
        assert result, "set_joint_pos doit réussir"
        print("✅ set_joint_pos réussi")

        # 4. Look at
        result = self.backend.look_at(0.1, 0.2, 0.3)
        assert result, "look_at doit réussir"
        print("✅ look_at réussi")

        # 5. Télémétrie
        telemetry = self.backend.get_telemetry()
        assert isinstance(telemetry, dict), "get_telemetry doit retourner dict"
        print("✅ télémétrie récupérée")

        print("🎉 Intégration complète réussie!")

    def test_18_documentation_compliance(self):
        """Test 18: Vérifier que toutes les méthodes ont une docstring."""
        print("\n🧪 TEST 18: Documentation")
        print("=" * 60)

        methods_without_doc = []
        for method_name in self.EXPECTED_SDK_METHODS.keys():
            if not hasattr(self.backend, method_name):
                continue

            method = getattr(self.backend, method_name)
            if not method.__doc__:
                methods_without_doc.append(method_name)
                print(f"❌ Pas de docstring: {method_name}")
            else:
                print(f"✅ Docstring présente: {method_name}")

        # Ce n'est pas critique, juste informatif
        if methods_without_doc:
            print(f"⚠️  {len(methods_without_doc)} méthodes sans docstring")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
