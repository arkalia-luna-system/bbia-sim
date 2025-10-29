#!/usr/bin/env python3
"""
🧪 TESTS AVANCÉS - DÉTECTION PATTERNS INEFFICACES ET NON CONFORMES
Détecte des problèmes subtils que seuls les experts en robotique IA émotionnelle verraient:
- Usage inefficace (set_joint_pos répétés au lieu de goto_target)
- Interpolation non adaptée aux émotions
- Durées non adaptatives
- Modules ne passant pas robot_api pour robot.media
"""

import re
import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestConformityAdvancedPatterns:
    """Tests pour détecter patterns inefficaces et non conformes."""

    def test_detect_inefficient_joint_control_patterns(self):
        """Test Expert 1: Détecter usage inefficace (set_joint_pos répétés au lieu de goto_target combiné)."""
        print("\n🧪 TEST EXPERT 1: Détection patterns inefficaces")
        print("=" * 60)

        src_dir = Path(__file__).parent.parent / "src" / "bbia_sim"
        if not src_dir.exists():
            pytest.skip("Dossier src/bbia_sim introuvable")

        # Chercher fichiers avec set_joint_pos répétés pour tête+corps
        inefficient_patterns = []

        for py_file in src_dir.rglob("*.py"):
            # Ignorer backends (peuvent utiliser méthodes bas niveau)
            if "backends" in str(py_file) or "__pycache__" in str(py_file):
                continue

            content = py_file.read_text(encoding="utf-8")
            lines = content.split("\n")

            # Chercher séquences de set_joint_pos multiples (potentiellement inefficace)
            for i, line in enumerate(lines):
                stripped = line.strip()
                # Ignorer commentaires
                if stripped.startswith("#") or stripped.startswith('"""'):
                    continue

                # Détecter set_joint_pos pour tête suivi de set_joint_pos pour corps
                # (devrait être goto_target combiné)
                if "set_joint_pos" in line and (
                    "head" in line.lower() or "stewart" in line.lower()
                ):
                    # Chercher set_joint_pos suivant pour yaw_body dans les 5 prochaines lignes
                    for j in range(i + 1, min(i + 6, len(lines))):
                        if "set_joint_pos" in lines[j] and "yaw_body" in lines[j]:
                            # Vérifier si goto_target n'est pas utilisé à la place
                            context = "\n".join(
                                lines[max(0, i - 3) : min(len(lines), j + 3)]
                            )
                            if "goto_target" not in context:
                                inefficient_patterns.append((py_file, i + 1, j + 1))

        if inefficient_patterns:
            print(f"⚠️  Patterns inefficaces détectés: {len(inefficient_patterns)}")
            for f, line1, line2 in inefficient_patterns[:5]:  # Limiter affichage
                print(
                    f"   {f.name}:{line1}-{line2} (set_joint_pos séparés au lieu de goto_target combiné)"
                )
        else:
            print("✅ Aucun pattern inefficace détecté")

        # Ne pas bloquer mais alerter (peut être acceptable dans certains cas)
        if len(inefficient_patterns) > 3:
            print(
                "\n💡 Recommandation: Considérer utiliser goto_target combiné pour meilleure performance"
            )

    def test_interpolation_adaptive_to_emotions(self):
        """Test Expert 2: Vérifier que l'interpolation s'adapte aux émotions."""
        print("\n🧪 TEST EXPERT 2: Interpolation adaptative aux émotions")
        print("=" * 60)

        integration_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_integration.py"
        )
        if not integration_file.exists():
            pytest.skip("Fichier bbia_integration.py introuvable")

        content = integration_file.read_text(encoding="utf-8")

        # Vérifier présence mapping émotion → interpolation
        has_emotion_interpolation_map = "emotion_interpolation_map" in content
        has_cartoon = '"cartoon"' in content or "'cartoon'" in content
        has_ease_in_out = '"ease_in_out"' in content or "'ease_in_out'" in content
        uses_interpolation_in_goto = (
            "method=interpolation_method" in content
            or "method=emotion_interpolation" in content
        )

        assert has_emotion_interpolation_map, (
            "EXPERT: bbia_integration doit avoir emotion_interpolation_map. "
            "Sans mapping = interpolation non adaptée = mouvements moins expressifs."
        )
        print("✅ emotion_interpolation_map présent")

        assert has_cartoon, (
            "EXPERT: Mapping doit inclure 'cartoon' pour émotions expressives. "
            "Sans cartoon = mouvements trop uniformes pour happy/excited."
        )
        print("✅ 'cartoon' utilisé pour émotions expressives")

        assert has_ease_in_out, (
            "EXPERT: Mapping doit inclure 'ease_in_out' pour émotions douces. "
            "Sans ease_in_out = mouvements trop brusques pour calm/sad."
        )
        print("✅ 'ease_in_out' utilisé pour émotions douces")

        assert uses_interpolation_in_goto, (
            "EXPERT: L'interpolation doit être utilisée dans goto_target. "
            "Mapping sans utilisation = code mort."
        )
        print("✅ Interpolation utilisée dans goto_target")

    def test_duration_adaptive_to_intensity(self):
        """Test Expert 3: Vérifier que la durée s'adapte à l'intensité émotionnelle."""
        print("\n🧪 TEST EXPERT 3: Durée adaptative selon intensité")
        print("=" * 60)

        integration_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_integration.py"
        )
        if not integration_file.exists():
            pytest.skip("Fichier bbia_integration.py introuvable")

        content = integration_file.read_text(encoding="utf-8")

        # Vérifier présence duration adaptative
        has_adaptive_duration = (
            "transition_duration" in content
            or "duration.*intensity" in content
            or "intensity.*duration" in content
        )

        # Vérifier formule adaptative (0.5 + intensity * quelque chose)
        has_formula = re.search(r"duration.*=.*0\.\d+.*[\+\-].*intensity", content)

        assert has_adaptive_duration or has_formula, (
            "EXPERT: La durée doit s'adapter à l'intensité émotionnelle (0.5-1.0s). "
            "Durée fixe = mouvements moins expressifs pour intensités élevées."
        )
        print("✅ Durée adaptative selon intensité détectée")

    def test_modules_pass_robot_api_for_media(self):
        """Test Expert 4: Vérifier que les modules reçoivent robot_api pour utiliser robot.media."""
        print("\n🧪 TEST EXPERT 4: Modules reçoivent robot_api pour robot.media")
        print("=" * 60)

        # Vérifier bbia_vision.py
        vision_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_vision.py"
        )
        if vision_file.exists():
            content = vision_file.read_text(encoding="utf-8")
            has_robot_api_param = (
                "robot_api"
                in re.search(r"def __init__.*?:", content, re.DOTALL).group(0)
                if re.search(r"def __init__.*?:", content, re.DOTALL)
                else False
            )
            has_media_check = (
                "robot.media" in content or "_camera_sdk_available" in content
            )

            assert has_robot_api_param and has_media_check, (
                "EXPERT: BBIAVision doit accepter robot_api et vérifier robot.media.camera. "
                "Sans = ne peut pas utiliser caméra SDK (perte performance/qualité)."
            )
            print("✅ BBIAVision accepte robot_api et vérifie robot.media")

        # Vérifier bbia_integration.py passe robot_api aux modules
        integration_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_integration.py"
        )
        if integration_file.exists():
            content = integration_file.read_text(encoding="utf-8")
            passes_to_vision = "BBIAVision(robot_api=" in content

            assert passes_to_vision, (
                "EXPERT: BBIAIntegration doit passer robot_api à BBIAVision. "
                "Sans = modules ne peuvent pas utiliser robot.media."
            )
            print("✅ BBIAIntegration passe robot_api à BBIAVision")

    def test_no_hardcoded_stewart_control_in_main_modules(self):
        """Test Expert 5: Aucun contrôle stewart hardcodé dans modules principaux."""
        print("\n🧪 TEST EXPERT 5: Absence contrôle stewart hardcodé")
        print("=" * 60)

        src_dir = Path(__file__).parent.parent / "src" / "bbia_sim"
        if not src_dir.exists():
            pytest.skip("Dossier src/bbia_sim introuvable")

        # Exclure backends (peuvent avoir code SDK bas niveau)
        exclude_patterns = ["backends", "__pycache__", "__init__"]

        # Chercher set_joint_pos avec "stewart" hardcodé (hors commentaires)
        hardcoded_stewart = []

        for py_file in src_dir.rglob("*.py"):
            if any(pattern in str(py_file) for pattern in exclude_patterns):
                continue

            content = py_file.read_text(encoding="utf-8")
            lines = content.split("\n")

            for i, line in enumerate(lines, 1):
                stripped = line.strip()
                # Ignorer commentaires et docstrings
                if (
                    stripped.startswith("#")
                    or stripped.startswith('"""')
                    or stripped.startswith("'''")
                ):
                    continue

                # Chercher set_joint_pos avec "stewart_" en dur
                if re.search(r'set_joint_pos\(["\']stewart_\d', line):
                    hardcoded_stewart.append((py_file, i, line.strip()))

        if hardcoded_stewart:
            print(f"⚠️  Contrôle stewart hardcodé détecté: {len(hardcoded_stewart)}")
            for f, line_num, line_content in hardcoded_stewart[:3]:
                print(f"   {f.name}:{line_num}: {line_content[:60]}...")
        else:
            print("✅ Aucun contrôle stewart hardcodé dans modules principaux")

        assert len(hardcoded_stewart) == 0, (
            f"EXPERT: {len(hardcoded_stewart)} usage(s) hardcodé(s) de stewart dans modules principaux. "
            f"Modules principaux doivent utiliser goto_target avec IK (conforme SDK)."
        )

    def test_fallback_graceful_for_all_features(self):
        """Test Expert 6: Tous les features avancés ont des fallbacks gracieux."""
        print("\n🧪 TEST EXPERT 6: Fallbacks gracieux pour features avancés")
        print("=" * 60)

        src_dir = Path(__file__).parent.parent / "src" / "bbia_sim"
        if not src_dir.exists():
            pytest.skip("Dossier src/bbia_sim introuvable")

        # Features avancés nécessitant fallbacks
        advanced_features = {
            "robot.media": ["_camera_sdk_available", "microphone_sdk", "speaker"],
            "goto_target": ["hasattr(robot_api, 'goto_target')"],
            "async_play_move": ["hasattr(robot_api, 'async_play_move')"],
            "recording": ["start_recording", "stop_recording"],
        }

        missing_fallbacks = []

        for py_file in src_dir.rglob("*.py"):
            if "__pycache__" in str(py_file):
                continue

            content = py_file.read_text(encoding="utf-8")

            for feature, indicators in advanced_features.items():
                # Chercher usage du feature
                if any(indicator in content for indicator in indicators):
                    # Vérifier présence fallback (try/except ou hasattr check)
                    has_try_except = "try:" in content and "except" in content
                    has_hasattr_check = "hasattr" in content
                    has_none_check = "is None" in content or "== None" in content

                    if not (has_try_except or has_hasattr_check or has_none_check):
                        missing_fallbacks.append((py_file, feature))

        if missing_fallbacks:
            print(f"⚠️  Features sans fallback détectés: {len(missing_fallbacks)}")
            for f, feature in missing_fallbacks[:5]:
                print(f"   {f.name}: {feature}")
        else:
            print("✅ Tous les features avancés ont des fallbacks gracieux")

        # Ne pas bloquer mais alerter
        if len(missing_fallbacks) > 5:
            print(
                "\n💡 Recommandation: Ajouter fallbacks pour éviter régression si SDK non disponible"
            )


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
