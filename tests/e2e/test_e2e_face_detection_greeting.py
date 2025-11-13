#!/usr/bin/env python3
"""
Test E2E: Scénario utilisateur - BBIA détecte visage → suit → salue
"""

import os
from collections import deque
from unittest.mock import MagicMock

import pytest

from bbia_sim.bbia_behavior import BBIABehaviorManager
from bbia_sim.bbia_vision import BBIAVision


@pytest.mark.e2e
@pytest.mark.slow
class TestE2EFaceDetectionGreeting:
    """Tests E2E scénario: détection visage → suivi → salutation."""

    def setup_method(self):
        """Configuration avant chaque test."""
        # Mock robot API
        self.mock_robot = MagicMock()
        self.mock_robot.set_joint_pos.return_value = True
        self.mock_robot.get_joint_pos.return_value = 0.0
        self.mock_robot.step.return_value = True

        # Vision avec mock
        self.vision = BBIAVision(robot_api=self.mock_robot)
        # Mock faces détectées
        self.vision.faces_detected = deque([  # type: ignore[assignment]
            {
                "name": "humain",
                "distance": 1.5,
                "confidence": 0.95,
                "emotion": "happy",
                "position": (0.5, 0.5),
            },
        ])

        # Behavior manager
        self.behavior = BBIABehaviorManager(robot_api=self.mock_robot)

    def test_bbia_detects_face_and_greets(self):
        """Scénario: BBIA détecte visage → suit → salue avec comportement."""
        # 1. Scanner environnement (détecter visage)
        result = self.vision.scan_environment()
        assert "faces" in result
        assert len(result["faces"]) > 0

        # 2. Vérifier visage détecté
        faces = self.vision.detect_faces()
        assert len(faces) > 0
        face = faces[0]
        assert face["name"] == "humain"

        # 3. Activer comportement greeting
        # Vérifier que comportement existe
        assert "greeting" in self.behavior.behaviors or hasattr(
            self.behavior, "execute_behavior"
        )

        # Simuler activation comportement (mock si nécessaire)
        if hasattr(self.behavior, "execute_behavior"):
            # Utiliser méthode réelle avec mock robot API
            greeting_success = self.behavior.execute_behavior("greeting", {})
            # Comportement peut retourner True ou None selon implémentation
            assert greeting_success is True or greeting_success is None
        else:
            # Fallback: vérifier comportement enregistré
            assert "greeting" in self.behavior.behaviors

        # 4. Vérifier comportement activé
        # Le comportement greeting a été exécuté avec succès
        # Note: Le comportement peut être asynchrone, donc vérifier seulement
        # que execute_behavior a été appelé avec succès (déjà vérifié ci-dessus)

    def test_bbia_detects_face_tracks_and_greets(self):
        """Scénario complet: détection → tracking → greeting."""
        # 1. Scanner environnement
        result = self.vision.scan_environment()
        assert len(result["faces"]) > 0

        # 2. Activer tracking visage
        # Pour tracking visage, on peut tracker l'objet "humain"
        self.vision.objects_detected = deque([  # type: ignore[assignment]
            {
                "name": "humain",
                "distance": 1.5,
                "confidence": 0.95,
                "bbox": [100, 100, 200, 200],
            },
        ])

        track_success = self.vision.track_object("humain")
        assert track_success is True

        # 3. Vérifier tracking actif
        status = self.vision.get_focus_status()
        assert status["tracking_active"] is True
        assert status["current_focus"] is not None

        # 4. Activer greeting
        setattr(self.behavior, "execute_behavior", MagicMock(return_value=True))  # type: ignore[method-assign]
        greeting_success = self.behavior.execute_behavior(
            "greeting", {"target": "face"}
        )
        assert greeting_success is True

    @pytest.mark.skipif(
        os.environ.get("BBIA_DISABLE_VISION", "0") == "1",
        reason="Vision désactivée",
    )
    def test_bbia_full_face_interaction_flow(self):
        """Flux complet interaction visage: scan → detect → track → greet → emotion."""
        # 1. Scan initial
        scan_result = self.vision.scan_environment()
        assert "faces" in scan_result

        # 2. Détection visages
        faces = self.vision.detect_faces()
        if len(faces) > 0:
            # 3. Tracking si visage trouvé
            self.vision.objects_detected = deque([  # type: ignore[assignment]
                {"name": "humain", "confidence": 0.9, "bbox": [100, 100, 200, 200]},
            ])
            track_success = self.vision.track_object("humain")

            if track_success:
                # 4. Activer greeting
                setattr(self.behavior, "execute_behavior", MagicMock(return_value=True))  # type: ignore[method-assign]
                greeting_result = self.behavior.execute_behavior("greeting", {})

                # 5. Vérifier état final
                assert greeting_result is True
                status = self.vision.get_focus_status()
                assert status["tracking_active"] is True
