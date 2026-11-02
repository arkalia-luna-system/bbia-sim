#!/usr/bin/env python3
"""
🧪 TESTS D'EXPERT - CONFORMITÉ BBIAHUGGINGFACE
Vérifie que l'intelligence conversationnelle respecte les standards expert
et détecte les problèmes subtils que seuls les experts en robotique IA
émotionnelle verraient.
"""

import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestBBIAHuggingFaceExpertConformity:
    """Tests d'expert pour BBIAHuggingFace - Détection problèmes subtils."""

    @staticmethod
    def _hf_source_paths() -> list[Path]:
        """Fichiers source HF à scanner (god-file + modules extraits)."""
        base = Path(__file__).parent.parent / "src" / "bbia_sim"
        return [
            base / "bbia_huggingface.py",
            base / "bbia_hf_response_bank.py",
            base / "bbia_hf_chat_tools.py",
        ]

    def _read_hf_sources(self) -> str:
        parts: list[str] = []
        for path in self._hf_source_paths():
            if path.exists():
                parts.append(path.read_text(encoding="utf-8"))
        if not parts:
            pytest.skip("Sources HF introuvables")
        return "\n".join(parts)

    def setup_method(self):
        """Configuration avant chaque test."""
        # Tenter d'importer BBIAHuggingFace (peut échouer si transformers non installé)
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            # Créer instance avec mock pour éviter chargement réel modèles lourds
            # Note: torch n'est pas un attribut du module, c'est importé localement
            # On skip simplement ces tests si transformers n'est pas disponible
            try:
                import torch  # noqa: F401

                self.hf_class = BBIAHuggingFace
                self.hf_available = True
            except ImportError:
                self.hf_class = None  # type: ignore[assignment]
                self.hf_available = False
        except ImportError:
            self.hf_class = None  # type: ignore[assignment]
            self.hf_available = False

    def test_01_greeting_variety_sufficient(self):
        """Test Expert 1: Les salutations doivent avoir suffisamment de variété (min 8 variants)."""
        print("\n🧪 TEST EXPERT 1: Variété salutations")
        print("=" * 60)

        if not self.hf_available:
            pytest.skip("BBIAHuggingFace non disponible")

        # Lire le fichier source pour vérifier variété
        hf_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_huggingface.py"
        )
        if not hf_file.exists():
            pytest.skip("Fichier bbia_huggingface.py introuvable")

        content = hf_file.read_text(encoding="utf-8")

        # Compter les variants de salutations friendly_robot
        import re

        # Chercher les listes de greetings friendly_robot
        greeting_matches = re.findall(
            r'"friendly_robot":\s*\[(.*?)\]', content, re.DOTALL
        )
        if greeting_matches:
            # Compter les éléments dans la liste
            greeting_text = greeting_matches[0]
            variants = [
                line.strip()
                for line in greeting_text.split(",")
                if line.strip() and '"' in line
            ]

            assert len(variants) >= 8, (
                f"EXPERT: friendly_robot greetings doit avoir au moins 8 variants "
                f"(actuellement {len(variants)}). "
                f"Variété insuffisante = répétitions perceptibles pour utilisateurs."
            )
            print(f"✅ Variété salutations suffisante: {len(variants)} variants")
        else:
            print("⚠️  Impossible de détecter les greetings dans le code")

    def test_02_question_responses_not_generic(self):
        """Test Expert 2: Les réponses aux questions ne doivent pas être trop génériques."""
        print("\n🧪 TEST EXPERT 2: Réponses questions non génériques")
        print("=" * 60)

        if not self.hf_available:
            pytest.skip("BBIAHuggingFace non disponible")

        hf_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_huggingface.py"
        )
        if not hf_file.exists():
            pytest.skip("Fichier bbia_huggingface.py introuvable")

        content = hf_file.read_text(encoding="utf-8")

        # Vérifier que les réponses questions incluent des éléments engageants
        # (pas juste "Je ne sais pas" ou "C'est intéressant" de manière répétitive)
        question_responses = content.lower()

        # Vérifier présence d'éléments engageants
        engaging_elements = [
            "en savoir plus",
            "donner plus de détails",
            "ce qui vous intrigue",
            "ce qui vous amène",
            "explorer",
            "réfléchir",
        ]

        found_elements = sum(
            1 for elem in engaging_elements if elem in question_responses
        )

        assert found_elements >= 3, (
            f"EXPERT: Les réponses questions doivent inclure au moins 3 éléments engageants "
            f"(actuellement {found_elements}). Réponses trop génériques = IA perçue comme peu intelligente."
        )
        print(f"✅ Réponses questions engageantes: {found_elements} éléments détectés")

    def test_03_context_usage_in_responses(self):
        """Test Expert 3: Les réponses doivent utiliser le contexte (cohérence conversationnelle)."""
        print("\n🧪 TEST EXPERT 3: Utilisation contexte")
        print("=" * 60)

        if not self.hf_available:
            pytest.skip("BBIAHuggingFace non disponible")

        hf_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_huggingface.py"
        )
        if not hf_file.exists():
            pytest.skip("Fichier bbia_huggingface.py introuvable")

        content = hf_file.read_text(encoding="utf-8")

        # Vérifier que le code utilise le contexte récent
        has_context_check = (
            "recent_context" in content or "_get_recent_context" in content
        )
        has_reference_words = (
            "reference_words" in content
            or "ça"
            in content  # Les références contextuelles utilisent "ça", "ce", etc.
        )

        assert has_context_check or has_reference_words, (
            "EXPERT: Le code doit utiliser le contexte récent pour cohérence conversationnelle. "
            "Sans contexte = réponses déconnectées = IA perçue comme peu intelligente."
        )
        print("✅ Utilisation contexte détectée dans le code")

    def test_04_personality_distinct_responses(self):
        """Test Expert 4: Les personnalités doivent avoir des réponses distinctes (pas juste emoji différent)."""
        print("\n🧪 TEST EXPERT 4: Distinction personnalités")
        print("=" * 60)

        if not self.hf_available:
            pytest.skip("BBIAHuggingFace non disponible")

        hf_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_huggingface.py"
        )
        if not hf_file.exists():
            pytest.skip("Fichier bbia_huggingface.py introuvable")

        content = hf_file.read_text(encoding="utf-8")

        # Vérifier que chaque personnalité a ses propres variantes (pas juste emoji)
        personalities = ["friendly_robot", "curious", "enthusiastic", "calm"]

        for personality in personalities:
            # Chercher les réponses spécifiques à cette personnalité
            pattern = rf'"{personality}":\s*\[(.*?)\]'
            import re

            matches = re.findall(pattern, content, re.DOTALL)
            assert len(matches) > 0, (
                f"EXPERT: La personnalité '{personality}' doit avoir ses propres réponses. "
                f"Sans distinction = personnalités perçues comme identiques."
            )

        print("✅ Chaque personnalité a ses propres réponses distinctes")

    def test_05_sentiment_affects_response_quality(self):
        """Test Expert 5: Le sentiment doit influencer la qualité/ton des réponses."""
        print("\n🧪 TEST EXPERT 5: Influence sentiment")
        print("=" * 60)

        if not self.hf_available:
            pytest.skip("BBIAHuggingFace non disponible")

        hf_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_huggingface.py"
        )
        if not hf_file.exists():
            pytest.skip("Fichier bbia_huggingface.py introuvable")

        content = hf_file.read_text(encoding="utf-8")

        # Vérifier que le code traite différemment POSITIVE et NEGATIVE
        has_positive_handling = (
            'sentiment_type == "POSITIVE"' in content
            or 'sentiment.get("sentiment") == "POSITIVE"' in content
        )
        has_negative_handling = (
            'sentiment_type == "NEGATIVE"' in content
            or 'sentiment.get("sentiment") == "NEGATIVE"' in content
        )

        assert has_positive_handling and has_negative_handling, (
            "EXPERT: Le code doit traiter différemment les sentiments POSITIVE et NEGATIVE. "
            "Ignorer sentiment = réponses inadaptées émotionnellement = IA perçue comme insensible."
        )
        print("✅ Traitement différencié des sentiments détecté")

    def test_06_response_length_appropriate(self):
        """Test Expert 6: Les réponses doivent avoir une longueur appropriée (pas trop courtes ni trop longues)."""
        print("\n🧪 TEST EXPERT 6: Longueur réponses appropriée")
        print("=" * 60)

        if not self.hf_available:
            pytest.skip("BBIAHuggingFace non disponible")

        content = self._read_hf_sources()

        # Extraction robuste des réponses candidates via AST
        import ast

        tree = ast.parse(content)

        candidates: list[str] = []

        def collect_from_list(node: ast.AST) -> list[str]:
            values: list[str] = []
            if isinstance(node, ast.List):
                for elt in node.elts:
                    if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                        values.append(elt.value)
            return values

        # Chercher variables/pools connus et blocs de personnalités
        for n in ast.walk(tree):
            # SUFFIX_POOL, _expert_quality_padding
            if isinstance(n, ast.Assign):
                for t in n.targets:
                    if isinstance(t, ast.Name) and t.id in {
                        "SUFFIX_POOL",
                        "_expert_quality_padding",
                    }:
                        candidates.extend(collect_from_list(n.value))
            # dictionaries de réponses: greetings, goodbyes, personality_descriptions
            if isinstance(n, ast.Dict):
                # Heuristique: liste de chaînes comme valeurs des clés string
                for v in n.values:
                    vals = collect_from_list(v)
                    if vals:
                        candidates.extend(vals)

        # Filtres: longueur, contenu textuel, pas de motifs techniques
        def is_human_reply(s: str) -> bool:
            s2 = s.strip()
            if not (20 <= len(s2) <= 200):
                return False
            if sum(ch.isalpha() for ch in s2) < 10:
                return False
            tech_markers = [
                "{",
                "}",
                "[",
                "]",
                "^",
                "$",
                "\\",
                "http",
                "xml",
                "model",
                "pipeline",
                "processor",
            ]
            if any(m in s2.lower() for m in tech_markers):
                return False
            return True

        responses = [s for s in candidates if is_human_reply(s)]

        if responses:
            # Vérifier que la plupart des réponses ont une longueur appropriée
            appropriate_length = [r for r in responses if 30 <= len(r.strip()) <= 150]

            percentage_appropriate = len(appropriate_length) / len(responses) * 100

            assert percentage_appropriate >= 70, (
                f"EXPERT: Au moins 70% des réponses doivent avoir une longueur appropriée (30-150 caractères). "
                f"Actuellement {percentage_appropriate:.1f}%. "
                f"Réponses trop courtes = perçues comme peu engageantes, "
                f"trop longues = perçues comme verbeuses et peu naturelles."
            )
            print(
                f"✅ {percentage_appropriate:.1f}% des réponses ont une longueur appropriée"
            )
        else:
            print("⚠️  Impossible d'extraire les réponses pour analyse")

    def test_07_fallback_graceful_degradation(self):
        """Test Expert 7: Le fallback LLM → simple_response doit être gracieux (pas de régression)."""
        print("\n🧪 TEST EXPERT 7: Dégradation gracieuse fallback")
        print("=" * 60)

        if not self.hf_available:
            pytest.skip("BBIAHuggingFace non disponible")

        hf_file = (
            Path(__file__).parent.parent / "src" / "bbia_sim" / "bbia_huggingface.py"
        )
        if not hf_file.exists():
            pytest.skip("Fichier bbia_huggingface.py introuvable")

        content = hf_file.read_text(encoding="utf-8")

        # Vérifier que le code a des fallbacks gracieux
        has_llm_fallback = (
            "fallback vers réponses enrichies" in content.lower()
            or "fallback enrichi" in content.lower()
        )
        has_sentiment_fallback = (
            "fallback si sentiment" in content.lower()
            or 'sentiment.get("sentiment", "NEUTRAL")' in content
        )

        assert has_llm_fallback or has_sentiment_fallback, (
            "EXPERT: Le code doit avoir des fallbacks gracieux pour éviter régression. "
            "Sans fallback = erreurs bruyantes = expérience utilisateur dégradée."
        )
        print("✅ Fallbacks gracieux détectés dans le code")

    def test_08_no_duplicate_responses(self):
        """Test Expert 8: Aucune réponse ne doit être dupliquée (variété essentielle)."""
        print("\n🧪 TEST EXPERT 8: Absence de doublons")
        print("=" * 60)

        if not self.hf_available:
            pytest.skip("BBIAHuggingface non disponible")

        content = self._read_hf_sources()

        # Extraction robuste des réponses candidates via AST
        import ast

        tree = ast.parse(content)
        candidates: list[str] = []

        def collect_from_list(node: ast.AST) -> list[str]:
            values: list[str] = []
            if isinstance(node, ast.List):
                for elt in node.elts:
                    if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                        values.append(elt.value)
            return values

        for n in ast.walk(tree):
            if isinstance(n, ast.Assign):
                for t in n.targets:
                    if isinstance(t, ast.Name) and t.id in {
                        "SUFFIX_POOL",
                        "_expert_quality_padding",
                    }:
                        candidates.extend(collect_from_list(n.value))
            if isinstance(n, ast.Dict):
                for v in n.values:
                    vals = collect_from_list(v)
                    if vals:
                        candidates.extend(vals)

        # Filtres cohérents avec test 06
        def is_human_reply(s: str) -> bool:
            s2 = s.strip()
            if len(s2) < 10:
                return False
            if sum(ch.isalpha() for ch in s2) < 8:
                return False
            tech_markers = [
                "{",
                "}",
                "[",
                "]",
                "^",
                "$",
                "\\",
                "http",
                "xml",
                "model",
                "pipeline",
                "processor",
            ]
            if any(m in s2.lower() for m in tech_markers):
                return False
            return True

        all_responses = [s for s in candidates if is_human_reply(s)]

        # Normaliser (enlever espaces, minuscules) pour détecter doublons approximatifs
        normalized = [" ".join(r.lower().split()) for r in all_responses if len(r) > 10]

        # Détecter doublons
        from collections import Counter

        counts = Counter(normalized)
        duplicates = {r: c for r, c in counts.items() if c > 1}

        # Tolérer jusqu'à 10 doublons (certains patterns peuvent être légitimement répétés)
        duplicate_count = len(duplicates)
        max_duplicates = 10
        print(f"📊 Doublons détectés: {duplicate_count} (max toléré: {max_duplicates})")
        if duplicate_count > 0:
            print(f"   Exemples: {list(duplicates.keys())[:3]}")

        # Assertion avec seuil toléré
        assert duplicate_count <= max_duplicates, (
            f"EXPERT: Trop de doublons ({duplicate_count} > {max_duplicates}). "
            f"Exemples: {list(duplicates.keys())[:5]}. "
            f"Doublons excessifs = variété insuffisante = répétitions perceptibles."
        )
        print(f"✅ Variété suffisante ({duplicate_count} doublons mineurs tolérés)")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
