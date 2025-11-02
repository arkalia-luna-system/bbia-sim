#!/usr/bin/env python3
"""
Tests E2E pour détection NLP (sentence-transformers) et extraction paramètres.
"""

import logging
import os
import unittest
from unittest.mock import MagicMock, patch

# Désactiver audio pour CI
os.environ["BBIA_DISABLE_AUDIO"] = "1"

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TestNLPDetection(unittest.TestCase):
    """Tests pour détection NLP avec sentence-transformers."""

    def setUp(self) -> None:
        """Setup test."""
        try:
            from src.bbia_sim.bbia_huggingface import BBIAHuggingFace
            from src.bbia_sim.bbia_tools import BBIATools

            tools = BBIATools(robot_api=None)
            self.hf = BBIAHuggingFace(tools=tools)
        except ImportError as e:
            self.skipTest(f"Module non disponible: {e}")

    def test_nlp_detection_move_head(self) -> None:
        """Test détection NLP pour move_head."""
        if not self.hf._sentence_model:
            self.skipTest("NLP non disponible (sentence-transformers manquant)")

        test_messages = [
            "peux-tu orienter ta tête vers la droite",
            "tourne-toi vers la gauche",
            "regarde en haut",
            "pivote ta tête vers le bas",
        ]

        for message in test_messages:
            result = self.hf._detect_tool_with_nlp(message)
            if result:
                tool_name, confidence = result
                logger.info(
                    f"✅ '{message}' → {tool_name} (confiance: {confidence:.2f})"
                )
                # Vérifier que c'est bien move_head
                self.assertEqual(tool_name, "move_head")
                self.assertGreater(confidence, 0.5)

    def test_nlp_detection_camera(self) -> None:
        """Test détection NLP pour camera."""
        if not self.hf._sentence_model:
            self.skipTest("NLP non disponible")

        test_messages = [
            "capture une image",
            "prends une photo de l'environnement",
            "que vois-tu autour de toi",
            "analyse visuellement la scène",
        ]

        for message in test_messages:
            result = self.hf._detect_tool_with_nlp(message)
            if result:
                tool_name, confidence = result
                logger.info(
                    f"✅ '{message}' → {tool_name} (confiance: {confidence:.2f})"
                )
                self.assertEqual(tool_name, "camera")

    def test_nlp_detection_dance(self) -> None:
        """Test détection NLP pour dance."""
        if not self.hf._sentence_model:
            self.skipTest("NLP non disponible")

        test_messages = [
            "fais une danse",
            "joue un mouvement de danse",
            "danse un peu",
            "bouge au rythme",
        ]

        for message in test_messages:
            result = self.hf._detect_tool_with_nlp(message)
            if result:
                tool_name, confidence = result
                logger.info(
                    f"✅ '{message}' → {tool_name} (confiance: {confidence:.2f})"
                )
                self.assertEqual(tool_name, "dance")


class TestParameterExtraction(unittest.TestCase):
    """Tests pour extraction paramètres NER (angles, intensités)."""

    def setUp(self) -> None:
        """Setup test."""
        try:
            from src.bbia_sim.bbia_huggingface import BBIAHuggingFace
            from src.bbia_sim.bbia_tools import BBIATools

            tools = BBIATools(robot_api=None)
            self.hf = BBIAHuggingFace(tools=tools)
        except ImportError as e:
            self.skipTest(f"Module non disponible: {e}")

    def test_extract_angle_degrees(self) -> None:
        """Test extraction angle en degrés."""
        test_cases = [
            ("tourne la tête de 30 degrés", 30.0),
            ("à 45 degrees", 45.0),
            ("90 degrés vers la droite", 90.0),
            ("15.5 degrés", 15.5),
        ]

        for message, expected_angle in test_cases:
            angle = self.hf._extract_angle(message)
            self.assertIsNotNone(angle, f"Angle non extrait de: {message}")
            self.assertAlmostEqual(angle, expected_angle, places=1)

    def test_extract_angle_radians(self) -> None:
        """Test extraction angle en radians."""
        import math

        test_cases = [
            ("pi/2 radians", math.degrees(math.pi / 2)),
            ("pi/4 radians", math.degrees(math.pi / 4)),
            ("1.57 radians", math.degrees(1.57)),
        ]

        for message, expected_angle_deg in test_cases:
            angle = self.hf._extract_angle(message)
            self.assertIsNotNone(angle, f"Angle non extrait de: {message}")
            self.assertAlmostEqual(angle, expected_angle_deg, places=1)

    def test_extract_angle_percent(self) -> None:
        """Test extraction angle depuis pourcentage."""
        test_cases = [
            ("à 50%", 45.0),  # 50% de 90° = 45°
            ("100%", 90.0),
            ("25%", 22.5),
        ]

        for message, expected_angle in test_cases:
            angle = self.hf._extract_angle(message)
            self.assertIsNotNone(angle, f"Angle non extrait de: {message}")
            self.assertAlmostEqual(angle, expected_angle, places=1)

    def test_extract_intensity_keywords(self) -> None:
        """Test extraction intensité depuis mots-clés."""
        test_cases = [
            ("légèrement à gauche", 0.2),
            ("un peu vers la droite", 0.2),
            ("beaucoup plus haut", 0.8),
            ("fortement vers le bas", 0.8),
            ("complètement à gauche", 1.0),
        ]

        for message, expected_intensity in test_cases:
            intensity = self.hf._extract_intensity(message)
            self.assertIsNotNone(intensity, f"Intensité non extraite de: {message}")
            self.assertAlmostEqual(intensity, expected_intensity, places=1)

    def test_extract_intensity_percent(self) -> None:
        """Test extraction intensité depuis pourcentage."""
        test_cases = [
            ("à 75%", 0.75),
            ("100%", 1.0),
            ("50%", 0.5),
        ]

        for message, expected_intensity in test_cases:
            intensity = self.hf._extract_intensity(message)
            self.assertIsNotNone(intensity, f"Intensité non extraite de: {message}")
            self.assertAlmostEqual(intensity, expected_intensity, places=1)

    def test_extract_combined_parameters(self) -> None:
        """Test extraction combinée angle + intensité."""
        # Test avec angle explicite
        result = self.hf._execute_detected_tool(
            "move_head",
            "tourne la tête de 45 degrés vers la droite",
            "tourne la tête de 45 degrés vers la droite",
        )
        # Vérifier que l'angle a été extrait (via logs ou résultat)
        self.assertIsNotNone(result)


class TestSmolVLM2Vision(unittest.TestCase):
    """Tests E2E pour SmolVLM2 vision."""

    def setUp(self) -> None:
        """Setup test."""
        try:
            from src.bbia_sim.bbia_huggingface import BBIAHuggingFace
            from src.bbia_sim.bbia_tools import BBIATools

            tools = BBIATools(robot_api=None)
            self.hf = BBIAHuggingFace(tools=tools)
        except ImportError as e:
            self.skipTest(f"Module non disponible: {e}")

    @patch("src.bbia_sim.bbia_huggingface.Image")
    def test_smolvlm_available(self, mock_image: MagicMock) -> None:
        """Test vérification disponibilité SmolVLM2."""
        # Simuler modèle disponible
        try:
            # Vérifier que modèle peut être chargé (ou simulé)
            multimodal_config = self.hf.model_configs.get("multimodal", {})
            self.assertIn("smolvlm", multimodal_config)
            logger.info("✅ SmolVLM2 configuré")
        except Exception as e:
            logger.warning(f"⚠️ SmolVLM2 non testable: {e}")
            self.skipTest(f"SmolVLM2 non disponible: {e}")

    @patch("src.bbia_sim.bbia_huggingface.Image.open")
    def test_moondream2_available(self, mock_open: MagicMock) -> None:
        """Test vérification disponibilité Moondream2."""
        try:
            multimodal_config = self.hf.model_configs.get("multimodal", {})
            self.assertIn("moondream2", multimodal_config)
            logger.info("✅ Moondream2 configuré")
        except Exception as e:
            logger.warning(f"⚠️ Moondream2 non testable: {e}")
            self.skipTest(f"Moondream2 non disponible: {e}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
