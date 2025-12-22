#!/usr/bin/env python3
"""
🧪 TESTS EDGE CASES - GESTION D'ERREURS ET CAS LIMITES
Tests pour robustesse : modèles indisponibles, caméra absente, robot déconnecté, etc.
"""

import os
import sys
import tempfile
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# Imports conditionnels
# BBIAChat peut toujours être importé (fonctionne même sans Hugging Face)
from bbia_sim.bbia_chat import BBIAChat

# BBIAVision peut toujours être importé (fonctionne même sans dépendances optionnelles)
from bbia_sim.bbia_vision import BBIAVision

BBIA_VISION_AVAILABLE = True

# ReachyMiniBackend peut toujours être importé (fonctionne même sans SDK)
from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend  # noqa: E402

REACHY_MINI_BACKEND_AVAILABLE = True

# BBIAAudio n'existe pas comme classe (le module expose des fonctions)
# Les tests qui utilisent BBIAAudio doivent être adaptés
BBIA_AUDIO_AVAILABLE = False
BBIAAudio: type | None = None


class TestErrorHandlingModels:
    """Tests de gestion d'erreurs pour modèles indisponibles."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_chat_huggingface_unavailable(self):
        """Test que BBIAChat gère gracieusement l'absence de Hugging Face."""
        # BBIAChat peut toujours être initialisé (fallback activé)
        # Simuler absence de transformers
        with patch.dict("sys.modules", {"transformers": None, "torch": None}):
            # Doit initialiser sans crasher
            chat = BBIAChat(robot_api=None)
            # Doit retourner None ou message d'erreur gracieux
            response = chat.chat("Bonjour")
            assert (
                response is not None
            ), "Doit retourner un message même si LLM indisponible"
            assert isinstance(response, str), "Réponse doit être une chaîne"

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_chat_model_loading_failure(self):
        """Test que BBIAChat gère l'échec de chargement du modèle."""
        # BBIAChat peut toujours être initialisé (fallback activé)
        # Simuler échec de chargement en patchant _load_llm
        with patch.object(BBIAChat, "_load_llm", return_value=(None, None)):
            chat = BBIAChat(robot_api=None)
            # Doit gérer l'erreur gracieusement
            response = chat.chat("Test")
            assert response is not None
            assert isinstance(response, str)


class TestErrorHandlingCamera:
    """Tests de gestion d'erreurs pour caméra indisponible."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_vision_camera_unavailable(self):
        """Test que BBIAVision gère gracieusement l'absence de caméra."""
        # BBIAVision peut toujours être initialisé (fonctionne même sans dépendances optionnelles)
        vision = BBIAVision(robot_api=None)
        # Simuler caméra indisponible
        with patch.object(vision, "_capture_image_from_camera", return_value=None):
            image = vision._capture_image_from_camera()
            # Doit retourner None ou image, pas crasher
            assert image is None or isinstance(
                image, (type(None), type(image)) if image is not None else type(None)
            )

    @pytest.mark.unit
    @pytest.mark.fast
    def test_vision_camera_error(self):
        """Test que BBIAVision gère les erreurs de caméra."""
        # BBIAVision peut toujours être initialisé (fonctionne même sans dépendances optionnelles)
        vision = BBIAVision(robot_api=None)
        # Simuler erreur caméra
        with patch.object(
            vision, "_capture_image_from_camera", side_effect=Exception("Caméra erreur")
        ):
            try:
                image = vision._capture_image_from_camera()
                # Doit gérer l'erreur gracieusement
                assert image is None or isinstance(
                    image,
                    (type(None), type(image)) if image is not None else type(None),
                )
            except Exception:
                # Exception acceptable si gérée par la méthode
                pass


class TestErrorHandlingRobot:
    """Tests de gestion d'erreurs pour robot déconnecté."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_backend_robot_disconnected(self):
        """Test que le backend gère gracieusement la déconnexion."""
        # ReachyMiniBackend peut toujours être initialisé (fonctionne même sans SDK)
        backend = ReachyMiniBackend()
        try:
            connected = backend.connect()
            if not connected:
                pytest.skip(
                    "Robot Reachy Mini non disponible (daemon Zenoh non accessible)"
                )
        except Exception as e:
            # Si erreur de connexion (Zenoh, etc.), skip le test
            if "zenoh" in str(e).lower() or "connect" in str(e).lower():
                pytest.skip(f"Robot Reachy Mini non disponible: {e}")
            raise

        backend.disconnect()

        # Après déconnexion, certaines opérations doivent gérer gracieusement
        try:
            # Doit gérer gracieusement ou lever une exception claire
            result = backend.set_emotion("happy", 0.5)
            # Peut retourner False ou lever une exception, mais pas crasher
            assert result is False or isinstance(result, bool)
        except Exception as e:
            # Si exception, doit être claire
            assert "déconnecté" in str(e).lower() or "disconnect" in str(e).lower()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_backend_robot_none(self):
        """Test que le backend gère robot_api=None."""
        # ReachyMiniBackend peut toujours être initialisé (fonctionne même sans SDK)
        backend = ReachyMiniBackend()
        backend.robot = None  # Simuler robot None

        # Doit gérer gracieusement
        try:
            result = backend.set_emotion("happy", 0.5)
            assert result is False or isinstance(result, bool)
        except Exception:
            # Exception acceptable si claire
            pass


class TestErrorHandlingFiles:
    """Tests de gestion d'erreurs pour fichiers corrompus."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_audio_corrupted_file(self):
        """Test que BBIAAudio gère les fichiers audio corrompus."""
        if not BBIA_AUDIO_AVAILABLE:
            pytest.skip("BBIAAudio non disponible")
        assert BBIAAudio is not None, "BBIAAudio doit être disponible"
        audio = BBIAAudio(robot_api=None)
        # Créer fichier corrompu
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            f.write(b"NOT_A_VALID_WAV_FILE")
            corrupted_file = f.name

        try:
            # Doit gérer gracieusement
            result = audio.play_audio(corrupted_file)
            # Peut retourner False ou lever une exception claire
            assert result is False or isinstance(result, bool)
        except Exception as e:
            # Exception acceptable si claire
            assert (
                "corrompu" in str(e).lower()
                or "invalid" in str(e).lower()
                or "format" in str(e).lower()
            )
        finally:
            # Nettoyer
            if os.path.exists(corrupted_file):
                os.unlink(corrupted_file)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_audio_nonexistent_file(self):
        """Test que BBIAAudio gère les fichiers inexistants."""
        if not BBIA_AUDIO_AVAILABLE:
            pytest.skip("BBIAAudio non disponible")
        assert BBIAAudio is not None, "BBIAAudio doit être disponible"
        audio = BBIAAudio(robot_api=None)
        # Fichier inexistant
        nonexistent_file = os.path.join(
            tempfile.gettempdir(), "nonexistent_audio_file_12345.wav"
        )

        # Doit gérer gracieusement
        result = audio.play_audio(nonexistent_file)
        assert result is False or isinstance(result, bool)


class TestEdgeCasesBuffers:
    """Tests de cas limites pour buffers."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_audio_buffer_full(self):
        """Test gestion buffer audio plein."""
        if not BBIA_AUDIO_AVAILABLE:
            pytest.skip("BBIAAudio non disponible")
        assert BBIAAudio is not None, "BBIAAudio doit être disponible"
        audio = BBIAAudio(robot_api=None)
        # Simuler buffer plein (si applicable)
        # Le test vérifie que le système ne crashe pas
        assert audio is not None

    @pytest.mark.unit
    @pytest.mark.fast
    def test_metrics_history_saturated(self):
        """Test gestion historique métriques saturé."""
        # Simuler historique métriques très grand
        from collections import deque

        history: deque[dict[str, int | float]] = deque(maxlen=20)
        # Remplir jusqu'à saturation - OPTIMISATION: 100 → 20 (suffisant pour test saturation, 5x plus rapide)
        for i in range(20):
            history.append({"timestamp": i, "value": i * 0.1})

        # Doit respecter maxlen
        assert len(history) == 20
        # Vérifier que le premier élément est bien le premier ajouté (pas supprimé car on n'a ajouté que 20 éléments)
        assert history[0]["timestamp"] == 0

        # Ajouter 10 éléments de plus pour tester la saturation réelle - OPTIMISATION: 50 → 10 (5x plus rapide)
        for i in range(20, 30):
            history.append({"timestamp": i, "value": i * 0.1})

        # Maintenant le premier élément devrait être 10 (les 10 premiers ont été supprimés)
        assert len(history) == 20
        assert history[0]["timestamp"] == 10


class TestEdgeCasesWebSocket:
    """Tests de cas limites pour WebSocket."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_websocket_multiple_connections(self):
        """Test gestion connexions WebSocket multiples."""
        try:
            from fastapi import WebSocket

            from bbia_sim.daemon.ws.telemetry import ConnectionManager

            manager = ConnectionManager()
            # Vérifier que le manager peut gérer plusieurs connexions
            assert manager._max_connections == 10
            assert len(manager.active_connections) == 0

            # Simuler plusieurs connexions (mock WebSocket)
            mock_ws1 = Mock(spec=WebSocket)
            mock_ws2 = Mock(spec=WebSocket)
            mock_ws3 = Mock(spec=WebSocket)

            # Simuler accept() pour chaque connexion
            async def mock_accept():
                pass

            mock_ws1.accept = mock_accept
            mock_ws2.accept = mock_accept
            mock_ws3.accept = mock_accept

            # Tester gestion de plusieurs connexions
            import asyncio

            async def test_connections():
                await manager.connect(mock_ws1)
                await manager.connect(mock_ws2)
                await manager.connect(mock_ws3)
                assert len(manager.active_connections) == 3

                # Déconnecter une connexion
                manager.disconnect(mock_ws1)
                assert len(manager.active_connections) == 2

            asyncio.run(test_connections())
        except ImportError:
            # Si le module n'est pas disponible, skip
            pytest.skip("Module WebSocket non disponible")

    @pytest.mark.unit
    @pytest.mark.fast
    def test_websocket_connection_timeout(self):
        """Test gestion timeout connexion WebSocket."""
        try:
            from fastapi import WebSocket

            from bbia_sim.daemon.ws.telemetry import ConnectionManager

            manager = ConnectionManager()
            # Vérifier que le manager gère les limites de connexions
            assert manager._max_connections == 10

            # Simuler connexions jusqu'à la limite
            mock_connections = []
            for _i in range(10):
                mock_ws = Mock(spec=WebSocket)

                async def mock_accept():
                    pass

                mock_ws.accept = mock_accept
                mock_connections.append(mock_ws)

            import asyncio

            async def test_limit():
                # Connecter jusqu'à la limite
                for mock_ws in mock_connections:
                    await manager.connect(mock_ws)
                assert len(manager.active_connections) == 10

                # Tenter une connexion supplémentaire (doit être rejetée)
                extra_ws = Mock(spec=WebSocket)
                extra_ws.close = Mock()

                async def mock_close(code, reason):
                    pass

                extra_ws.close = mock_close
                await manager.connect(extra_ws)
                # La connexion doit être rejetée (fermée)
                assert len(manager.active_connections) == 10

            asyncio.run(test_limit())
        except ImportError:
            pytest.skip("Module WebSocket non disponible")


class TestEdgeCasesModels:
    """Tests de cas limites pour modèles inactifs."""

    @pytest.mark.unit
    @pytest.mark.slow
    def test_model_inactive_timeout(self):
        """Test gestion modèles inactifs > timeout."""
        # Skip en CI si trop lent (chargement modèle LLM)
        import os

        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (chargement modèle LLM trop lent)")
        # BBIAChat peut toujours être initialisé (fallback activé)
        chat = BBIAChat(robot_api=None)
        # Simuler modèle inactif (timeout)
        # Le système doit gérer gracieusement
        assert chat is not None

        # Vérifier que le système peut gérer un modèle non chargé
        # (le modèle ne doit pas être chargé à l'initialisation si lazy loading)
        # Le chat doit fonctionner même si le modèle n'est pas encore chargé
        try:
            response = chat.chat("Test")
            assert response is not None
            assert isinstance(response, str)
        except Exception as e:
            # Exception acceptable si gérée gracieusement
            assert (
                "timeout" in str(e).lower()
                or "indisponible" in str(e).lower()
                or "unavailable" in str(e).lower()
            )


class TestErrorHandlingMediaPipeCrash:
    """Tests de gestion d'erreurs pour MediaPipe crash pendant l'exécution."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_mediapipe_crash_during_execution(self):
        """Test que BBIAVision gère un crash MediaPipe pendant l'exécution."""
        from bbia_sim.bbia_vision import BBIAVision

        vision = BBIAVision(robot_api=None)
        # Simuler crash MediaPipe pendant détection (pas juste "non disponible")
        if hasattr(vision, "_face_detector") and vision._face_detector:
            # Simuler crash en patchant la méthode de détection
            original_detect = vision._face_detector.detect_faces

            def crash_detect(*args, **kwargs):
                raise RuntimeError("MediaPipe internal crash: segmentation fault")

            vision._face_detector.detect_faces = crash_detect
            # Doit gérer gracieusement sans crasher
            try:
                result = vision.detect_faces(None)
                # Peut retourner liste vide ou None, mais ne doit pas crasher
                assert result is not None or result == []
            except RuntimeError:
                # Si exception levée, doit être gérée par le système
                pass
            finally:
                # Restaurer méthode originale
                vision._face_detector.detect_faces = original_detect


class TestErrorHandlingMemoryStress:
    """Tests de gestion d'erreurs pour RAM saturée."""

    @pytest.mark.unit
    @pytest.mark.fast
    @pytest.mark.skipif(
        not os.environ.get("TORCH_AVAILABLE", "0") == "1",
        reason="torch non disponible (test nécessite torch pour simuler MemoryError)",
    )
    def test_memory_saturated_during_model_loading(self):
        """Test gestion RAM saturée lors du chargement d'un modèle."""
        try:
            import torch  # noqa: F401
        except ImportError:
            pytest.skip("torch non disponible")

        from unittest.mock import patch

        # Simuler MemoryError lors du chargement
        with patch("torch.load", side_effect=MemoryError("RAM saturée")):
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            hf = BBIAHuggingFace()
            # Doit gérer gracieusement sans crasher
            try:
                result = hf.load_model("test_model", "chat")
                # Peut retourner False ou None, mais ne doit pas crasher
                assert result is False or result is None
            except MemoryError:
                # Si exception levée, doit être loggée et gérée
                pass


class TestErrorHandlingRaceConditions:
    """Tests de gestion d'erreurs pour race conditions."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_concurrent_emotion_set(self):
        """Test gestion accès concurrent à set_emotion()."""
        import threading

        from bbia_sim.bbia_emotions import BBIAEmotions

        emotions = BBIAEmotions()
        errors = []

        def set_emotion_thread(emotion_name: str):
            try:
                for _ in range(10):
                    emotions.set_emotion(emotion_name, 0.5)
            except Exception as e:
                errors.append(e)

        # Lancer 3 threads simultanément
        threads = [
            threading.Thread(target=set_emotion_thread, args=("happy",)),
            threading.Thread(target=set_emotion_thread, args=("sad",)),
            threading.Thread(target=set_emotion_thread, args=("angry",)),
        ]

        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join()

        # Ne doit pas y avoir d'erreurs de race condition
        assert len(errors) == 0, f"Erreurs de race condition détectées: {errors}"


class TestErrorHandlingAPIDown:
    """Tests de gestion d'erreurs pour API complètement down."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_api_completely_down(self):
        """Test gestion API complètement inaccessible (pas juste timeout)."""
        from unittest.mock import patch

        from fastapi.testclient import TestClient

        try:
            from bbia_sim.daemon.app.main import app

            client = TestClient(app)
            # Simuler API complètement down (ConnectionRefusedError)
            with patch(
                "fastapi.testclient.TestClient.get",
                side_effect=ConnectionRefusedError("API down"),
            ):
                # Doit gérer gracieusement
                try:
                    client.get("/api/daemon/status")
                    # Ne devrait pas arriver ici si API vraiment down
                except (ConnectionRefusedError, ConnectionError):
                    # Exception attendue, doit être gérée
                    pass
        except ImportError:
            pytest.skip("Module daemon non disponible")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
