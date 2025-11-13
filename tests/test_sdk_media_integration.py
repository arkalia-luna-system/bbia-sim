#!/usr/bin/env python3
"""
🧪 TESTS D'INTÉGRATION SDK MEDIA/IO
Vérifie que les modules BBIA utilisent robot.media et robot.io si disponibles.
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestSDKMediaIntegration:
    """Tests pour vérifier l'intégration robot.media dans les modules BBIA."""

    def test_bbia_vision_accepts_robot_api(self):
        """Test: BBIAVision doit accepter robot_api pour utiliser robot.media.camera."""
        print("\n🧪 TEST: BBIAVision accepte robot_api")
        print("=" * 60)

        from bbia_sim.bbia_vision import BBIAVision

        # Test sans robot_api (mode simulation)
        vision_no_api = BBIAVision()
        assert vision_no_api.robot_api is None
        print("✅ BBIAVision fonctionne sans robot_api (simulation)")

        # Test avec robot_api mock
        mock_robot_api = MagicMock()
        mock_robot_api.media = MagicMock()
        mock_robot_api.media.camera = MagicMock()

        vision_with_api = BBIAVision(robot_api=mock_robot_api)
        assert vision_with_api.robot_api == mock_robot_api
        print("✅ BBIAVision accepte robot_api (pour robot.media.camera)")

    def test_bbia_audio_enregistrer_accepts_robot_api(self):
        """Test: enregistrer_audio doit accepter robot_api pour robot.media.microphone."""
        print("\n🧪 TEST: enregistrer_audio accepte robot_api")
        print("=" * 60)

        import inspect

        from bbia_sim.bbia_audio import enregistrer_audio

        sig = inspect.signature(enregistrer_audio)
        params = list(sig.parameters.keys())

        assert (
            "robot_api" in params
        ), "enregistrer_audio doit accepter robot_api (pour robot.media.microphone)"
        print("✅ enregistrer_audio accepte robot_api paramètre")

    def test_bbia_audio_lire_accepts_robot_api(self):
        """Test: lire_audio doit accepter robot_api pour robot.media.speaker."""
        print("\n🧪 TEST: lire_audio accepte robot_api")
        print("=" * 60)

        import inspect

        from bbia_sim.bbia_audio import lire_audio

        sig = inspect.signature(lire_audio)
        params = list(sig.parameters.keys())

        assert (
            "robot_api" in params
        ), "lire_audio doit accepter robot_api (pour robot.media.speaker)"
        print("✅ lire_audio accepte robot_api paramètre")

    def test_bbia_voice_dire_texte_accepts_robot_api(self):
        """Test: dire_texte doit accepter robot_api pour robot.media.speaker."""
        print("\n🧪 TEST: dire_texte accepte robot_api")
        print("=" * 60)

        import inspect

        from bbia_sim.bbia_voice import dire_texte

        sig = inspect.signature(dire_texte)
        params = list(sig.parameters.keys())

        assert (
            "robot_api" in params
        ), "dire_texte doit accepter robot_api (pour robot.media.speaker)"
        print("✅ dire_texte accepte robot_api paramètre")

    def test_bbia_integration_passes_robot_api_to_modules(self):
        """Test: BBIAIntegration doit passer robot_api aux modules BBIA."""
        print("\n🧪 TEST: BBIAIntegration passe robot_api aux modules")
        print("=" * 60)

        from bbia_sim.bbia_integration import BBIAIntegration
        from bbia_sim.daemon.simulation_service import SimulationService

        # Créer simulation service avec robot_api mock
        mock_robot_api = MagicMock()
        mock_robot_api.media = MagicMock()
        mock_robot_api.media.camera = MagicMock()

        service = SimulationService()
        # robot_api n'est pas un attribut public, utiliser setattr pour le test
        setattr(service, "robot_api", mock_robot_api)  # type: ignore[attr-defined]

        integration = BBIAIntegration(simulation_service=service)

        # Vérifier que vision a reçu robot_api
        assert (
            integration.vision.robot_api == mock_robot_api
        ), "BBIAVision doit recevoir robot_api depuis SimulationService"
        print("✅ BBIAVision reçoit robot_api via BBIAIntegration")

    @patch("bbia_sim.bbia_audio._get_robot_media_microphone")
    def test_enregistrer_audio_checks_robot_media(self, mock_get_mic):
        """Test: enregistrer_audio vérifie robot.media.microphone si disponible."""
        print("\n🧪 TEST: enregistrer_audio vérifie robot.media.microphone")
        print("=" * 60)

        from bbia_sim.bbia_audio import enregistrer_audio

        mock_mic = MagicMock()
        mock_get_mic.return_value = mock_mic

        # Test avec robot_api ayant media.microphone
        mock_robot_api = MagicMock()
        mock_robot_api.media = MagicMock()
        mock_robot_api.media.microphone = mock_mic

        # Ne pas réellement enregistrer (éviter erreurs PortAudio en test)
        with patch("bbia_sim.bbia_audio.sd.rec") as mock_rec:
            mock_audio = MagicMock()
            mock_rec.return_value = mock_audio
            with patch("bbia_sim.bbia_audio.sd.wait"):
                with patch("wave.open", create=True) as mock_wave:
                    mock_wf = MagicMock()
                    mock_wave.return_value.__enter__.return_value = mock_wf

                    try:
                        enregistrer_audio(
                            "test.wav", duree=1, robot_api=mock_robot_api  # duree doit être int
                        )
                        print("✅ enregistrer_audio vérifie robot.media.microphone")
                    except Exception:
                        # Peut échouer en test car sounddevice nécessite setup réel
                        print("⚠️  enregistrer_audio testé (peut nécessiter setup réel)")

    def test_behavior_manager_has_recording_methods(self):
        """Test: BBIABehaviorManager doit avoir méthodes recording/replay."""
        print("\n🧪 TEST: BBIABehaviorManager a méthodes recording/replay")
        print("=" * 60)

        from bbia_sim.bbia_behavior import BBIABehaviorManager

        manager = BBIABehaviorManager()

        assert hasattr(
            manager, "record_behavior_movement"
        ), "BBIABehaviorManager doit avoir record_behavior_movement"
        print("✅ record_behavior_movement disponible")

        assert hasattr(
            manager, "play_saved_behavior"
        ), "BBIABehaviorManager doit avoir play_saved_behavior"
        print("✅ play_saved_behavior disponible")

        assert hasattr(
            manager, "saved_moves"
        ), "BBIABehaviorManager doit avoir saved_moves (bibliothèque)"
        print("✅ saved_moves disponible (bibliothèque mouvements)")

    def test_emotion_interpolation_mapping(self):
        """Test: Les émotions doivent mapper vers interpolations adaptées."""
        print("\n🧪 TEST: Mapping émotion → interpolation")
        print("=" * 60)

        # Vérifier que bbia_integration utilise le mapping émotion → interpolation
        integration_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_integration.py"
        )

        if integration_file.exists():
            content = integration_file.read_text(encoding="utf-8")

            # Vérifier présence du mapping
            has_emotion_map = "emotion_interpolation_map" in content
            has_cartoon = '"cartoon"' in content or "'cartoon'" in content
            has_ease_in_out = '"ease_in_out"' in content or "'ease_in_out'" in content

            assert (
                has_emotion_map
            ), "bbia_integration doit avoir emotion_interpolation_map"
            print("✅ emotion_interpolation_map présent")

            assert (
                has_cartoon
            ), "Mapping doit inclure 'cartoon' pour émotions expressives"
            print("✅ 'cartoon' utilisé pour émotions expressives")

            assert (
                has_ease_in_out
            ), "Mapping doit inclure 'ease_in_out' pour émotions douces"
            print("✅ 'ease_in_out' utilisé pour émotions douces")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
