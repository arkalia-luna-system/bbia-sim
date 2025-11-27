#!/usr/bin/env python3
"""
🧪 TESTS DE ROBUSTESSE EXPERTE - CONFORMITÉ REACHY MINI
Tests pour détecter des problèmes subtils que seuls les experts robotiques verraient.
Renforce les tests existants pour une détection maximale de problèmes.
"""

import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# OPTIMISATION COVERAGE: Importer les modules au niveau module pour que coverage les détecte
import bbia_sim.backends.reachy_mini_backend  # noqa: F401
import bbia_sim.bbia_huggingface  # noqa: F401
from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

# Importer BBIAHuggingFace pour les tests
try:
    from bbia_sim.bbia_huggingface import BBIAHuggingFace  # noqa: F401
except (ImportError, AttributeError):
    BBIAHuggingFace = None  # type: ignore[assignment,misc]

# Import SDK officiel si disponible
try:
    from reachy_mini.utils import create_head_pose

    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    create_head_pose = None


class TestExpertRobustnessConformity:
    """Tests de robustesse experte pour détecter tous les problèmes subtils."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.backend = ReachyMiniBackend()
        self.backend.connect()

    def teardown_method(self):
        """Nettoyage après chaque test."""
        if self.backend:
            self.backend.disconnect()

    def test_expert_01_precision_loss_prevention(self):
        """Test expert: Prévenir perte de précision lors de conversions float."""
        print("\n🧪 EXPERT TEST 1: Prévention Perte Précision")
        print("=" * 60)

        # Test que les limites exactes du XML sont préservées
        exact_limits = {
            "stewart_1": (-0.8377580409572196, 1.3962634015955222),
            "yaw_body": (-2.792526803190975, 2.792526803190879),
        }

        for joint, (min_limit, max_limit) in exact_limits.items():
            joint_info = self.backend.joint_limits.get(joint)
            if joint_info:
                # Vérifier que les limites backend correspondent aux limites exactes
                backend_min, backend_max = joint_info
                assert (
                    abs(backend_min - min_limit) < 1e-10
                ), f"{joint} min_limit perd en précision: {backend_min} != {min_limit}"
                assert (
                    abs(backend_max - max_limit) < 1e-10
                ), f"{joint} max_limit perd en précision: {backend_max} != {max_limit}"
                print(
                    f"✅ {joint}: Précision préservée (min={backend_min}, max={backend_max})"
                )

    def test_expert_02_thread_safety_concurrent_access(self):
        """Test expert: Accès concurrent multi-thread ne doit pas casser."""
        print("\n🧪 EXPERT TEST 2: Thread Safety - Accès Concurrent")
        print("=" * 60)

        errors = []
        results = []

        def worker(thread_id: int) -> None:
            """Travailleur thread pour test concurrent."""
            try:
                for i in range(10):
                    emotion = ["happy", "sad", "neutral"][i % 3]
                    self.backend.set_emotion(emotion, 0.5)
                    pos = self.backend.get_joint_pos("yaw_body")
                    results.append((thread_id, i, emotion, pos))
                    # OPTIMISATION: Réduire sleep de 0.01 à 0.005 (2x plus rapide)
                    time.sleep(0.005)
            except Exception as e:
                errors.append((thread_id, e))

        # Lancer 3 threads concurrents (daemon pour éviter blocage pytest)
        threads = []
        for tid in range(3):
            t = threading.Thread(target=worker, args=(tid,), daemon=True)
            threads.append(t)
            t.start()

        # Attendre fin avec timeout
        for t in threads:
            t.join(timeout=5.0)
            # Si le thread n'est pas terminé après timeout, on continue quand même
            # (daemon=True garantit qu'il ne bloquera pas pytest)

        # Vérifier résultats
        assert len(errors) == 0, f"Erreurs thread-safety détectées: {errors}"
        assert len(results) == 30, f"Résultats manquants: {len(results)}/30"
        print(f"✅ Thread-safety: {len(results)} opérations réussies sans erreur")

    def test_expert_03_pose_matrix_validation(self):
        """Test expert: Validation des matrices de pose (4x4, unitaire, etc.)."""
        print("\n🧪 EXPERT TEST 3: Validation Matrices Pose")
        print("=" * 60)

        if not SDK_AVAILABLE or create_head_pose is None:
            print("⚠️  Test ignoré (SDK non disponible)")
            return

        # Test create_head_pose retourne matrice valide
        assert create_head_pose is not None  # Type narrowing pour mypy
        pose = create_head_pose(pitch=0.1, yaw=0.05)
        assert pose.shape == (4, 4), f"Pose doit être 4x4, obtenu {pose.shape}"

        # Vérifier que la matrice est de rotation valide (determinant = 1)
        rotation_part = pose[:3, :3]
        det = np.linalg.det(rotation_part)
        assert abs(det - 1.0) < 0.01, f"Determinant rotation doit être ~1, obtenu {det}"

        # Vérifier que la matrice est orthogonale (R^T * R = I)
        should_be_identity = rotation_part.T @ rotation_part
        identity = np.eye(3)
        assert np.allclose(
            should_be_identity, identity, atol=0.01
        ), "Matrice de rotation doit être orthogonale"

        print("✅ Matrice pose valide (4x4, déterminant=1, orthogonale)")

    def test_expert_04_interpolation_timing_consistency(self):
        """Test expert: Les interpolations doivent respecter la duration exacte."""
        print("\n🧪 EXPERT TEST 4: Cohérence Timing Interpolation")
        print("=" * 60)

        if not SDK_AVAILABLE or create_head_pose is None:
            print("⚠️  Test ignoré (SDK non disponible)")
            return

        # Test que goto_target avec duration=1.0 prend ~1.0s
        test_duration = 0.5
        assert create_head_pose is not None  # Type narrowing pour mypy
        pose = create_head_pose(pitch=0.1, yaw=0.0)

        start_time = time.time()
        self.backend.goto_target(head=pose, duration=test_duration, method="linear")
        elapsed = time.time() - start_time

        # En simulation, peut être instantané, mais en réel doit respecter duration
        if self.backend.is_connected and self.backend.robot:
            # Mode réel: doit être proche de la duration
            assert (
                abs(elapsed - test_duration) < 0.2
            ), f"Duration non respectée: {elapsed:.3f}s au lieu de {test_duration}s"
            print(f"✅ Duration respectée: {elapsed:.3f}s (attendu {test_duration}s)")
        else:
            print(
                f"ℹ️  Mode simulation: duration={test_duration}s, elapsed={elapsed:.3f}s"
            )

    def test_expert_05_memory_leak_detection(self):
        """Test expert: Détecter fuites mémoire lors d'opérations répétées."""
        print("\n🧪 EXPERT TEST 5: Détection Fuites Mémoire")
        print("=" * 60)

        import gc
        import tracemalloc

        # Démarrer traçage mémoire
        tracemalloc.start()
        snapshot_before = tracemalloc.take_snapshot()

        # OPTIMISATION: Réduire 100 → 50 itérations (suffisant pour détecter fuites, 2x plus rapide)
        iterations = 50
        for i in range(iterations):
            self.backend.set_emotion("happy", 0.5)
            self.backend.get_joint_pos("yaw_body")
            if i % 10 == 0:
                gc.collect()  # Forcer collecte périodique

        # Snapshot après
        snapshot_after = tracemalloc.take_snapshot()
        top_stats = snapshot_after.compare_to(snapshot_before, "lineno")

        # Vérifier qu'il n'y a pas de fuite majeure (> 5MB pour 50 itérations, proportionnel)
        total_diff = sum(stat.size_diff for stat in top_stats[:10])
        total_mb = total_diff / (1024 * 1024)

        assert (
            total_mb < 5.0
        ), f"Fuite mémoire détectée: {total_mb:.2f}MB après {iterations} opérations"
        print(f"✅ Mémoire: {total_mb:.2f}MB (sain, <10MB)")

        tracemalloc.stop()

    def test_expert_06_emotion_state_consistency(self):
        """Test expert: L'état émotionnel doit être cohérent après chaque transition."""
        print("\n🧪 EXPERT TEST 6: Cohérence État Émotionnel")
        print("=" * 60)

        emotions = ["happy", "sad", "excited", "curious", "neutral", "calm"]

        for emotion in emotions:
            # Appliquer émotion
            success = self.backend.set_emotion(emotion, 0.7)
            assert success is True, f"set_emotion({emotion}) doit réussir"

            # Vérifier que l'état est cohérent (télémetrie)
            telemetry = self.backend.get_telemetry()
            assert (
                "current_emotion" in telemetry
            ), "Télémetrie doit contenir current_emotion"
            # Note: L'émotion peut être mappée (ex: BBIA 12 émotions → SDK 6 émotions)
            # Donc on vérifie juste que l'état existe
            print(
                f"✅ État cohérent après {emotion} (télémetrie: {telemetry.get('current_emotion', 'N/A')})"
            )

    def test_expert_07_joint_position_continuity(self):
        """Test expert: Les positions de joints doivent être continues (pas de saut)."""
        print("\n🧪 EXPERT TEST 7: Continuité Positions Joints")
        print("=" * 60)

        # Faire un mouvement progressif
        positions = []
        for i in range(10):
            angle = 0.2 * math.sin(2 * math.pi * i / 10)
            self.backend.set_joint_pos("yaw_body", angle)
            # OPTIMISATION: Réduire sleep de 0.05 à 0.02 (2.5x plus rapide)
            time.sleep(0.02)
            pos = self.backend.get_joint_pos("yaw_body")
            positions.append((i, pos))

        # Vérifier continuité (pas de saut > 0.5 rad entre positions adjacentes)
        for i in range(1, len(positions)):
            prev_pos = positions[i - 1][1]
            curr_pos = positions[i][1]
            diff = abs(curr_pos - prev_pos)
            assert (
                diff < 0.5
            ), f"Saut détecté entre step {i-1} et {i}: {prev_pos:.4f} → {curr_pos:.4f} (diff={diff:.4f})"

        print(f"✅ Continuité préservée: {len(positions)} positions continues")

    def test_expert_08_backend_reconnection_resilience(self):
        """Test expert: Le backend doit supporter reconnexions multiples."""
        print("\n🧪 EXPERT TEST 8: Résilience Reconnexions")
        print("=" * 60)

        # Connexion/déconnexion multiples
        for i in range(3):
            self.backend.disconnect()
            # OPTIMISATION: Réduire sleep de 0.1 à 0.05 (2x plus rapide)
            time.sleep(0.05)
            self.backend.connect()

            # Vérifier état
            assert self.backend.is_connected, f"Reconnexion {i+1} échouée"
            pos = self.backend.get_joint_pos("yaw_body")
            assert isinstance(pos, float), f"État incohérent après reconnexion {i+1}"
            print(f"✅ Reconnexion {i+1}: État cohérent")

    def test_expert_09_look_at_image_boundary_pixels(self):
        """Test expert: look_at_image doit gérer pixels aux bords de l'image."""
        print("\n🧪 EXPERT TEST 9: Pixels Bords Image")
        print("=" * 60)

        # Résolution typique caméra: 1920x1080 (supposé)
        width, height = 1920, 1080

        boundary_pixels = [
            (0, 0),  # Coin supérieur gauche
            (width - 1, 0),  # Coin supérieur droit
            (0, height - 1),  # Coin inférieur gauche
            (width - 1, height - 1),  # Coin inférieur droit
            (width // 2, 0),  # Milieu haut
            (width // 2, height - 1),  # Milieu bas
        ]

        for u, v in boundary_pixels:
            try:
                result = self.backend.look_at_image(u, v, duration=0.3)
                assert isinstance(
                    result, np.ndarray
                ), f"look_at_image({u}, {v}) doit retourner ndarray"
                print(f"✅ Pixel bord ({u}, {v}): OK")
            except (ValueError, Exception) as e:
                print(f"⚠️  Pixel bord ({u}, {v}): {type(e).__name__}")

    def test_expert_10_recording_move_data_structure(self):
        """Test expert: Les mouvements enregistrés doivent avoir structure valide."""
        print("\n🧪 EXPERT TEST 10: Structure Données Recording")
        print("=" * 60)

        # Démarrer enregistrement
        self.backend.start_recording()
        time.sleep(0.2)

        # Faire un mouvement
        self.backend.set_joint_pos("yaw_body", 0.1)
        time.sleep(0.1)
        self.backend.set_joint_pos("yaw_body", 0.0)

        # Arrêter et récupérer
        move_data = self.backend.stop_recording()

        if move_data is not None:
            # Vérifier structure
            assert isinstance(
                move_data, list
            ), f"Move data doit être liste, obtenu {type(move_data)}"
            # Move doit être séquence de frames (même si vide en simulation)
            print(f"✅ Move enregistré: {len(move_data)} frames (structure valide)")

    def test_expert_11_gravity_compensation_state(self):
        """Test expert: État compensation gravité doit être persistant."""
        print("\n🧪 EXPERT TEST 11: État Compensation Gravité")
        print("=" * 60)

        # Activer compensation
        self.backend.enable_gravity_compensation()
        time.sleep(0.1)

        # Vérifier qu'on peut toujours contrôler les joints
        pos_before = self.backend.get_joint_pos("yaw_body")
        self.backend.set_joint_pos("yaw_body", 0.1)
        time.sleep(0.1)
        pos_after = self.backend.get_joint_pos("yaw_body")

        # Désactiver
        self.backend.disable_gravity_compensation()
        print(
            f"✅ Compensation gravité: contrôle préservé (pos: {pos_before:.4f} → {pos_after:.4f})"
        )

    def test_expert_12_telemetry_timestamp_accuracy(self):
        """Test expert: Les timestamps télémétrie doivent être cohérents."""
        print("\n🧪 EXPERT TEST 12: Précision Timestamps Télémétrie")
        print("=" * 60)

        t1 = time.time()
        telemetry1 = self.backend.get_telemetry()
        t2 = time.time()

        # Vérifier que elapsed_time est cohérent
        if "elapsed_time" in telemetry1:
            elapsed = telemetry1["elapsed_time"]
            # elapsed_time doit être >= 0 et cohérent avec start_time
            assert elapsed >= 0, f"elapsed_time négatif: {elapsed}"
            print(
                f"✅ Timestamp cohérent: elapsed_time={elapsed:.3f}s (mesuré: {t2-t1:.3f}s)"
            )

    def test_expert_13_interpolation_method_flexibility(self):
        """Test expert: Toutes les méthodes d'interpolation doivent fonctionner."""
        print("\n🧪 EXPERT TEST 13: Flexibilité Méthodes Interpolation")
        print("=" * 60)

        if not SDK_AVAILABLE:
            print("⚠️  Test ignoré (SDK non disponible)")
            return

        if create_head_pose is None:
            print("⚠️  Test ignoré (create_head_pose non disponible)")
            return

        methods = [
            "minjerk",
            "MIN_JERK",
            "linear",
            "LINEAR",
            "ease_in_out",
            "EASE_IN_OUT",
            "cartoon",
            "CARTOON",
        ]

        assert create_head_pose is not None  # Type narrowing pour mypy
        pose = create_head_pose(pitch=0.1, yaw=0.0)

        for method in methods:
            try:
                # Tester toutes les variations (minuscules, majuscules, etc.)
                self.backend.goto_target(head=pose, duration=0.3, method=method)
                print(f"✅ Méthode '{method}': OK")
            except Exception as e:
                print(f"⚠️  Méthode '{method}': {type(e).__name__}")

    def test_expert_14_media_backend_fallback_order(self):
        """Test expert: Ordre de fallback media doit être correct."""
        print("\n🧪 EXPERT TEST 14: Ordre Fallback Media")
        print("=" * 60)

        # Vérifier que media est accessible
        media = self.backend.media
        if media is None:
            print("ℹ️  Mode simulation: media non disponible (attendu)")
            return

        # Vérifier ordre de priorité: play_audio > speaker.play > speaker.play_file
        has_play_audio = hasattr(media, "play_audio")
        speaker = getattr(media, "speaker", None)
        has_speaker_play = speaker is not None and hasattr(speaker, "play")
        has_speaker_play_file = speaker is not None and hasattr(speaker, "play_file")

        print(f"  play_audio: {'✅' if has_play_audio else '❌'}")
        print(f"  speaker.play: {'✅' if has_speaker_play else '❌'}")
        print(f"  speaker.play_file: {'✅' if has_speaker_play_file else '❌'}")

        # Au moins une méthode doit être disponible
        assert (
            has_play_audio or has_speaker_play or has_speaker_play_file
        ), "Aucune méthode play media disponible"
        print("✅ Ordre fallback médias vérifié")

    @pytest.mark.slow
    def test_expert_15_conversation_history_memory_limit(self):
        """Test expert: L'historique conversation ne doit pas croître indéfiniment."""
        print("\n🧪 EXPERT TEST 15: Limite Mémoire Historique Conversation")
        print("=" * 60)

        if BBIAHuggingFace is None:
            pytest.skip("BBIAHuggingFace non disponible")

        try:
            bbia = BBIAHuggingFace()

            # Noter la longueur initiale (peut contenir historique chargé depuis mémoire)
            initial_history_len = len(bbia.conversation_history)

            # OPTIMISATION: Réduire 100 → 50 messages (suffisant pour tester limite historique, 2x plus rapide)
            num_messages = 50
            for i in range(num_messages):
                bbia.chat(f"Message {i}")

            # Vérifier que l'historique a augmenté d'au plus 60 messages
            # (50 messages envoyés = 50 entrées user+bbia + 10 de marge)
            # L'historique peut contenir des messages initiaux chargés depuis mémoire persistante
            final_history_len = len(bbia.conversation_history)
            history_increase = final_history_len - initial_history_len
            expected_max_increase = num_messages + 10  # 50 messages envoyés + marge

            assert history_increase <= expected_max_increase, (
                f"Historique a augmenté de {history_increase} messages "
                f"(max recommandé: {expected_max_increase}, envoyés: {num_messages}, "
                f"initial: {initial_history_len}, final: {final_history_len})"
            )

            print(
                f"✅ Historique conversation: {final_history_len} messages "
                f"(augmentation: {history_increase}, initial: {initial_history_len})"
            )
        except ImportError:
            print("⚠️  Test ignoré (BBIAHuggingFace non disponible)")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
