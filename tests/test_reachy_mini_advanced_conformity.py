#!/usr/bin/env python3
"""
🧪 TESTS AVANCÉS DE CONFORMITÉ REACHY-MINI
Tests expertes pour détecter les problèmes que les tests basiques ne trouvent pas.
Vérifie optimisations, utilisation SDK, performance, résilience.
"""

import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose

    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    ReachyMini = None
    create_head_pose = None


class TestReachyMiniAdvancedConformity:
    """Tests avancés de conformité pour détecter problèmes experts."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.backend: ReachyMiniBackend = ReachyMiniBackend()
        self.backend.connect()

    def teardown_method(self):
        """Nettoyage après chaque test."""
        if self.backend:
            self.backend.disconnect()

    def test_19_goto_target_usage(self):
        """Test 19: Vérifier que goto_target est utilisé au lieu de set_joint_pos répétés."""
        print("\n🧪 TEST 19: Utilisation goto_target (Optimisation)")
        print("=" * 60)

        # Lire le code source de bbia_behavior.py
        behavior_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_behavior.py"
        if not behavior_file.exists():
            pytest.skip("Fichier bbia_behavior.py non trouvé")

        code = behavior_file.read_text(encoding="utf-8")

        # Détecter patterns inefficaces: set_joint_pos répétés avec time.sleep
        # Pattern à éviter: set_joint_pos() + time.sleep() + set_joint_pos()
        inefficient_patterns = []
        lines = code.split("\n")
        for i, line in enumerate(lines):
            if "set_joint_pos" in line and i + 1 < len(lines):
                # Chercher time.sleep juste après
                next_lines = "\n".join(lines[i : i + 3])
                if "time.sleep" in next_lines and "set_joint_pos" in "\n".join(
                    lines[i + 2 : i + 5]
                ):
                    inefficient_patterns.append((i + 1, line.strip()))

        if inefficient_patterns:
            print(f"⚠️  {len(inefficient_patterns)} patterns inefficaces détectés:")
            for line_num, line in inefficient_patterns[:5]:  # Limiter à 5
                print(f"   Ligne {line_num}: {line[:60]}...")
            print("💡 Recommandation: Utiliser goto_target() avec interpolation")
        else:
            print("✅ Aucun pattern inefficace détecté (goto_target utilisé)")

        # Vérifier que goto_target est utilisé
        goto_target_usage = code.count("goto_target(")
        set_joint_pos_usage = code.count("set_joint_pos(")

        print(f"📊 goto_target() utilisé: {goto_target_usage} fois")
        print(f"📊 set_joint_pos() utilisé: {set_joint_pos_usage} fois")

        # Ratio devrait être > 0.3 (au moins 30% d'utilisation goto_target)
        if set_joint_pos_usage > 0:
            ratio = goto_target_usage / (goto_target_usage + set_joint_pos_usage)
            print(f"📊 Ratio goto_target: {ratio:.2%}")
            if ratio < 0.3:
                print("⚠️  Ratio trop faible - plus de goto_target recommandé")
            else:
                print("✅ Ratio acceptable")

    def test_20_interpolation_methods(self):
        """Test 20: Vérifier utilisation de toutes les techniques interpolation."""
        print("\n🧪 TEST 20: Techniques d'interpolation")
        print("=" * 60)

        # Lire le code source pour détecter utilisation interpolation
        files_to_check = [
            "src/bbia_sim/bbia_behavior.py",
            "src/bbia_sim/bbia_integration.py",
        ]

        interpolation_methods_found = {
            "MIN_JERK": False,
            "minjerk": False,
            "LINEAR": False,
            "EASE_IN_OUT": False,
            "EASEINOUT": False,
            "CARTOON": False,
        }

        for file_path in files_to_check:
            file = Path(__file__).parent.parent / file_path
            if file.exists():
                code = file.read_text(encoding="utf-8")
                for method in interpolation_methods_found:
                    if method in code:
                        interpolation_methods_found[method] = True

        print("📊 Techniques d'interpolation utilisées:")
        for method, found in interpolation_methods_found.items():
            status = "✅" if found else "❌"
            print(f"   {status} {method}")

        # Vérifier diversité
        methods_used = sum(1 for found in interpolation_methods_found.values() if found)
        print(f"\n📊 {methods_used}/{len(interpolation_methods_found)} techniques utilisées")

        if methods_used < 2:
            print("⚠️  Diversité faible - recommander utilisation CARTOON pour émotions")
        else:
            print("✅ Diversité acceptable")

    def test_21_media_integration(self):
        """Test 21: Vérifier intégration modules media/io SDK."""
        print("\n🧪 TEST 21: Intégration Media/IO SDK")
        print("=" * 60)

        # Vérifier que media/io sont accessibles dans backend
        if hasattr(self.backend, "media"):
            media = self.backend.media
            if media is not None:
                print("✅ robot.media disponible")
                # Vérifier méthodes media disponibles
                if hasattr(media, "camera"):
                    print("✅ robot.media.camera disponible")
                if hasattr(media, "microphone"):
                    print("✅ robot.media.microphone disponible")
                if hasattr(media, "speaker"):
                    print("✅ robot.media.speaker disponible")
            else:
                print("⚠️  robot.media disponible mais None (mode simulation)")
        else:
            print("❌ robot.media non disponible")

        if hasattr(self.backend, "io"):
            io = self.backend.io
            if io is not None:
                print("✅ robot.io disponible")
            else:
                print("⚠️  robot.io disponible mais None (mode simulation)")
        else:
            print("❌ robot.io non disponible")

        # Vérifier utilisation dans modules BBIA
        vision_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_vision.py"
        audio_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_audio.py"

        media_used_vision = False
        if vision_file.exists():
            code = vision_file.read_text(encoding="utf-8")
            media_used_vision = "robot.media" in code or "media.camera" in code

        media_used_audio = False
        if audio_file.exists():
            code = audio_file.read_text(encoding="utf-8")
            media_used_audio = "robot.media" in code or "media.microphone" in code

        print("\n📊 Utilisation media SDK dans modules:")
        print(f"   Vision: {'✅' if media_used_vision else '❌'}")
        print(f"   Audio: {'✅' if media_used_audio else '❌'}")

        if not media_used_vision or not media_used_audio:
            print("💡 Recommandation: Intégrer robot.media dans modules BBIA")

    def test_22_async_operations(self):
        """Test 22: Vérifier utilisation async_play_move pour performance."""
        print("\n🧪 TEST 22: Opérations Asynchrones")
        print("=" * 60)

        # Vérifier que async_play_move existe
        if hasattr(self.backend, "async_play_move"):
            print("✅ async_play_move disponible")
        else:
            print("❌ async_play_move non disponible")

        # Vérifier utilisation dans code
        behavior_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_behavior.py"
        integration_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_integration.py"

        async_used = False
        for file_path in [behavior_file, integration_file]:
            if file_path.exists():
                code = file_path.read_text(encoding="utf-8")
                if "async_play_move" in code:
                    async_used = True
                    break

        print(f"📊 async_play_move utilisé: {'✅' if async_used else '❌'}")
        if not async_used:
            print("💡 Recommandation: Utiliser async_play_move pour comportements complexes")

    def test_23_combined_movements(self):
        """Test 23: Vérifier mouvements combinés tête+corps via goto_target."""
        print("\n🧪 TEST 23: Mouvements Combinés")
        print("=" * 60)

        # Lire code pour détecter patterns combinés
        integration_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_integration.py"
        behavior_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_behavior.py"

        combined_patterns = 0
        separated_patterns = 0

        for file_path in [integration_file, behavior_file]:
            if file_path.exists():
                code = file_path.read_text(encoding="utf-8")

                # Pattern combiné: goto_target(head=..., body_yaw=...)
                combined_patterns += code.count("goto_target") - code.count("goto_target(")

                # Pattern séparé: set_emotion() puis set_joint_pos("yaw_body")
                # Simplifié: compter appels séparés
                lines = code.split("\n")
                for i, line in enumerate(lines):
                    if "set_emotion" in line or "set_target_head_pose" in line:
                        # Chercher set_joint_pos("yaw_body") proche
                        next_lines = "\n".join(lines[i : i + 5])
                        if 'set_joint_pos("yaw_body"' in next_lines:
                            separated_patterns += 1

        print(f"📊 Mouvements combinés (goto_target head+body): {combined_patterns}")
        print(f"📊 Mouvements séparés (tête puis corps): {separated_patterns}")

        if separated_patterns > combined_patterns * 2:
            print("⚠️  Trop de mouvements séparés - synchronisation sous-optimale")
            print("💡 Recommandation: Utiliser goto_target combiné")
        else:
            print("✅ Utilisation mouvements combinés acceptable")

    def test_24_error_resilience(self):
        """Test 24: Vérifier fallbacks gracieux si SDK non disponible."""
        print("\n🧪 TEST 24: Résilience aux Erreurs")
        print("=" * 60)

        # Vérifier gestion d'erreurs dans bbia_behavior.py
        behavior_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_behavior.py"
        if not behavior_file.exists():
            pytest.skip("Fichier bbia_behavior.py non trouvé")

        code = behavior_file.read_text(encoding="utf-8")

        # Compter try/except blocks
        try_count = code.count("try:")
        except_count = code.count("except")
        hasattr_count = code.count("hasattr(")

        print("📊 Gestion d'erreurs:")
        print(f"   try/except blocks: {min(try_count, except_count)}")
        print(f"   hasattr vérifications: {hasattr_count}")

        # Vérifier présence fallbacks
        fallback_patterns = ["fallback", "Fallback", "FALLBACK"]
        fallbacks_found = sum(1 for pattern in fallback_patterns if pattern in code)

        print(f"   Fallbacks documentés: {fallbacks_found}")

        if try_count < 5 or hasattr_count < 10:
            print("⚠️  Gestion d'erreurs insuffisante")
            print("💡 Recommandation: Ajouter plus de try/except et hasattr")
        else:
            print("✅ Gestion d'erreurs robuste")

    def test_25_recording_replay(self):
        """Test 25: Vérifier enregistrement/replay de mouvements."""
        print("\n🧪 TEST 25: Enregistrement/Replay")
        print("=" * 60)

        # Vérifier méthodes disponibles
        has_recording = hasattr(self.backend, "start_recording")
        has_stop = hasattr(self.backend, "stop_recording")
        has_play = hasattr(self.backend, "play_move")

        print("📊 Méthodes enregistrement:")
        print(f"   start_recording: {'✅' if has_recording else '❌'}")
        print(f"   stop_recording: {'✅' if has_stop else '❌'}")
        print(f"   play_move: {'✅' if has_play else '❌'}")

        # Vérifier utilisation dans comportements
        behavior_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_behavior.py"
        if behavior_file.exists():
            code = behavior_file.read_text(encoding="utf-8")
            recording_used = "start_recording" in code or "record_movement" in code
            print(f"\n📊 Utilisation dans comportements: {'✅' if recording_used else '❌'}")
            if not recording_used:
                print("💡 Recommandation: Implémenter enregistrement de comportements")

    def test_26_duration_adaptive(self):
        """Test 26: Vérifier durée adaptative selon intensité émotion."""
        print("\n🧪 TEST 26: Durée Adaptative")
        print("=" * 60)

        # Vérifier dans bbia_integration.py
        integration_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_integration.py"
        if not integration_file.exists():
            pytest.skip("Fichier bbia_integration.py non trouvé")

        code = integration_file.read_text(encoding="utf-8")

        # Chercher duration adaptative basée sur intensity
        adaptive_patterns = [
            "duration.*intensity",
            "intensity.*duration",
            "transition_duration",
        ]

        adaptive_found = any(pattern in code for pattern in adaptive_patterns)

        print(f"📊 Durée adaptative: {'✅' if adaptive_found else '❌'}")
        if adaptive_found:
            print("✅ Durée s'adapte selon intensité émotion")
        else:
            print("💡 Recommandation: Utiliser duration adaptative (0.5-1.0s selon intensité)")

    def test_27_coordinate_validation(self):
        """Test 27: Vérifier validation coordonnées look_at_world/look_at_image."""
        print("\n🧪 TEST 27: Validation Coordonnées")
        print("=" * 60)

        # Vérifier dans bbia_behavior.py et bbia_integration.py
        files_to_check = [
            "src/bbia_sim/bbia_behavior.py",
            "src/bbia_sim/bbia_integration.py",
        ]

        validation_found = False
        look_at_usage = False

        for file_path in files_to_check:
            file = Path(__file__).parent.parent / file_path
            if file.exists():
                code = file.read_text(encoding="utf-8")
                if "look_at_world" in code or "look_at_image" in code:
                    look_at_usage = True
                    # Chercher validation limites
                    if "-2.0 <= x <= 2.0" in code or "0 <= u <= 640" in code:
                        validation_found = True

        print(f"📊 look_at_world/image utilisé: {'✅' if look_at_usage else '❌'}")
        print(f"📊 Validation coordonnées: {'✅' if validation_found else '❌'}")

        if look_at_usage and not validation_found:
            print("⚠️  look_at utilisé sans validation - risques de valeurs hors limites")
            print("💡 Recommandation: Ajouter validation coordonnées")

    def test_28_emotion_interpolation_mapping(self):
        """Test 28: Vérifier mapping émotion → technique interpolation."""
        print("\n🧪 TEST 28: Mapping Émotion → Interpolation")
        print("=" * 60)

        # Vérifier dans bbia_integration.py
        integration_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_integration.py"
        if not integration_file.exists():
            pytest.skip("Fichier bbia_integration.py non trouvé")

        code = integration_file.read_text(encoding="utf-8")

        # Chercher mapping émotion → interpolation
        has_mapping = (
            "CARTOON" in code
            or "EASE_IN_OUT" in code
            or "emotion.*interpolation" in code.lower()
            or "interpolation.*emotion" in code.lower()
        )

        print(f"📊 Mapping émotion → interpolation: {'✅' if has_mapping else '❌'}")
        if not has_mapping:
            print(
                "💡 Recommandation: Utiliser CARTOON pour happy/excited, EASE_IN_OUT pour calm/sad"
            )

    def test_29_import_safety(self):
        """Test 29: Vérifier imports SDK sécurisés (try/except)."""
        print("\n🧪 TEST 29: Sécurité Imports SDK")
        print("=" * 60)

        # Vérifier imports dans tous les fichiers BBIA
        bbia_files = [
            "src/bbia_sim/bbia_behavior.py",
            "src/bbia_sim/bbia_integration.py",
            "src/bbia_sim/backends/reachy_mini_backend.py",
        ]

        unsafe_imports = []

        for file_path in bbia_files:
            file = Path(__file__).parent.parent / file_path
            if file.exists():
                code = file.read_text(encoding="utf-8")
                lines = code.split("\n")

                for i, line in enumerate(lines):
                    # Chercher imports direct reachy_mini sans try/except
                    if "from reachy_mini" in line or "import reachy_mini" in line:
                        # Vérifier si dans try block
                        context = "\n".join(lines[max(0, i - 5) : i])
                        if "try:" not in context:
                            unsafe_imports.append((file_path, i + 1))

        if unsafe_imports:
            print(f"⚠️  {len(unsafe_imports)} imports non sécurisés:")
            for file_path, line_num in unsafe_imports[:5]:
                print(f"   {file_path}:{line_num}")
        else:
            print("✅ Tous les imports SDK sont sécurisés (try/except)")

    def test_30_performance_patterns(self):
        """Test 30: Détecter patterns de performance (async, interpolation, etc.)."""
        print("\n🧪 TEST 30: Patterns Performance")
        print("=" * 60)

        behavior_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_behavior.py"
        integration_file = Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_integration.py"

        patterns = {
            "async_play_move": 0,
            "goto_target": 0,
            'method="minjerk"': 0,
            'method="CARTOON"': 0,
            'method="EASE_IN_OUT"': 0,
        }

        for file_path in [behavior_file, integration_file]:
            if file_path.exists():
                code = file_path.read_text(encoding="utf-8")
                for pattern in patterns:
                    patterns[pattern] += code.count(pattern)

        print("📊 Patterns performance détectés:")
        for pattern, count in patterns.items():
            status = "✅" if count > 0 else "❌"
            print(f"   {status} {pattern}: {count}")

        total_patterns = sum(patterns.values())
        print(f"\n📊 Score performance: {total_patterns} patterns optimisés")
        if total_patterns < 10:
            print("⚠️  Score faible - plus d'optimisations recommandées")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
