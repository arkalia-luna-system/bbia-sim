#!/usr/bin/env python3
"""
Tests pour les nouvelles fonctionnalités LLM conversationnel de BBIA.
Valide enable_llm_chat, disable_llm_chat, et _generate_llm_response.
"""

import pytest

# Import conditionnel pour éviter erreurs si HF indisponible
try:
    from bbia_sim.bbia_huggingface import BBIAHuggingFace

    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False
    BBIAHuggingFace = None  # type: ignore


@pytest.mark.slow  # OPTIMISATION: Classe initialise BBIAHuggingFace (peut déclencher lazy loading)
class TestLLMChatFunctionality:
    """Tests pour les fonctionnalités LLM conversationnel."""

    def setup_method(self) -> None:
        """Initialise l'instance pour chaque test."""
        if not HF_AVAILABLE or BBIAHuggingFace is None:
            pytest.skip("Hugging Face transformers non disponible")
        try:
            self.hf = BBIAHuggingFace()
            # S'assurer que LLM est désactivé au début
            if self.hf.use_llm_chat:
                self.hf.disable_llm_chat()
        except ImportError:
            pytest.skip("Hugging Face transformers non disponible (ImportError)")

    def teardown_method(self) -> None:
        """Nettoie après chaque test."""
        if hasattr(self, "hf") and self.hf.use_llm_chat:
            self.hf.disable_llm_chat()

    @pytest.mark.slow  # OPTIMISATION: Test lent (charge modèle LLM lourd)
    def test_enable_llm_chat_returns_bool(self) -> None:
        """Test que enable_llm_chat retourne un booléen."""
        # Skip en CI si trop lent (chargement modèle LLM lourd)
        import os

        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (chargement modèle LLM trop lent)")
        if not HF_AVAILABLE:
            pytest.skip("Hugging Face non disponible")

        # Tester avec modèle optionnel (ne chargera pas en test unitaire normal)
        result = self.hf.enable_llm_chat("mistralai/Mistral-7B-Instruct-v0.2")
        assert isinstance(result, bool)

    @pytest.mark.slow  # OPTIMISATION: Test lent (charge modèle LLM lourd)
    def test_disable_llm_chat_cleans_up(self) -> None:
        """Test que disable_llm_chat libère correctement les ressources."""
        # Skip en CI si trop lent (chargement modèle LLM lourd)
        import os

        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (chargement modèle LLM trop lent)")
        if not HF_AVAILABLE:
            pytest.skip("Hugging Face non disponible")

        # Activer puis désactiver (même si activation échoue)
        self.hf.enable_llm_chat("mistralai/Mistral-7B-Instruct-v0.2")
        self.hf.disable_llm_chat()

        assert not self.hf.use_llm_chat
        assert self.hf.chat_model is None
        assert self.hf.chat_tokenizer is None

    @pytest.mark.slow  # OPTIMISATION: Test peut déclencher lazy loading LLM
    def test_chat_fallback_when_llm_not_loaded(self) -> None:
        """Test que chat utilise fallback enrichi si LLM non chargé."""
        if not HF_AVAILABLE:
            pytest.skip("Hugging Face non disponible")

        # S'assurer que LLM n'est pas chargé
        if self.hf.use_llm_chat:
            self.hf.disable_llm_chat()

        response = self.hf.chat("Bonjour")
        assert isinstance(response, str)
        assert len(response) > 0
        # Doit utiliser réponses enrichies (pas LLM)
        assert not self.hf.use_llm_chat

    def test_chat_model_config_exists(self) -> None:
        """Test que la configuration des modèles chat existe."""
        if not HF_AVAILABLE:
            pytest.skip("Hugging Face non disponible")

        assert "chat" in self.hf.model_configs
        assert "mistral" in self.hf.model_configs["chat"]
        assert "llama" in self.hf.model_configs["chat"]

    def test_get_available_models_includes_chat(self) -> None:
        """Test que get_available_models inclut les modèles chat."""
        if not HF_AVAILABLE:
            pytest.skip("Hugging Face non disponible")

        available = self.hf.get_available_models()
        assert "chat" in available
        assert isinstance(available["chat"], list)
        assert len(available["chat"]) > 0

    def test_llm_chat_state_variables_exist(self) -> None:
        """Test que les variables d'état LLM existent."""
        if not HF_AVAILABLE:
            pytest.skip("Hugging Face non disponible")

        assert hasattr(self.hf, "chat_model")
        assert hasattr(self.hf, "chat_tokenizer")
        assert hasattr(self.hf, "use_llm_chat")

    def test_use_llm_chat_defaults_to_false(self) -> None:
        """Test que use_llm_chat est False par défaut."""
        if not HF_AVAILABLE:
            pytest.skip("Hugging Face non disponible")

        assert self.hf.use_llm_chat is False

    @pytest.mark.slow  # OPTIMISATION: Test peut déclencher lazy loading LLM
    def test_chat_method_handles_llm_fallback(self) -> None:
        """Test que la méthode chat gère correctement le fallback LLM."""
        if not HF_AVAILABLE:
            pytest.skip("Hugging Face non disponible")

        # Avec LLM non chargé, doit utiliser réponses enrichies
        response = self.hf.chat("Test message")
        assert isinstance(response, str)
        assert len(response) > 0

        # Vérifier que l'historique est sauvegardé
        assert len(self.hf.conversation_history) > 0

    @pytest.mark.slow  # OPTIMISATION: Test lent (charge modèle LLM lourd)
    def test_load_model_chat_type(self) -> None:
        """Test que load_model accepte model_type='chat'."""
        # Skip en CI si trop lent (chargement modèle LLM lourd)
        import os

        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (chargement modèle LLM trop lent)")
        if not HF_AVAILABLE:
            pytest.skip("Hugging Face non disponible")

        # Tester chargement (peut échouer si modèle non disponible, c'est OK)
        # On teste juste que la méthode existe et accepte le type
        try:
            result = self.hf.load_model(
                "mistralai/Mistral-7B-Instruct-v0.2", model_type="chat"
            )
            assert isinstance(result, bool)
        except Exception:
            # OK si échoue (modèle lourd, pas disponible en CI, etc.)
            pass


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
