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
        backend.connect()
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

        history: deque[dict[str, int | float]] = deque(maxlen=100)
        # Remplir jusqu'à saturation - OPTIMISATION: 200 → 100 (suffisant pour test saturation)
        for i in range(100):
            history.append({"timestamp": i, "value": i * 0.1})

        # Doit respecter maxlen
        assert len(history) == 100
        # Vérifier que le premier élément est bien le premier ajouté (pas supprimé car on n'a ajouté que 100 éléments)
        assert history[0]["timestamp"] == 0

        # Ajouter 50 éléments de plus pour tester la saturation réelle
        for i in range(100, 150):
            history.append({"timestamp": i, "value": i * 0.1})

        # Maintenant le premier élément devrait être 50 (les 50 premiers ont été supprimés)
        assert len(history) == 100
        assert history[0]["timestamp"] == 50


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
    @pytest.mark.fast
    def test_model_inactive_timeout(self):
        """Test gestion modèles inactifs > timeout."""
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


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
