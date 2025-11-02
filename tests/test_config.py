"""Tests unitaires pour la configuration."""

from unittest.mock import patch

from src.bbia_sim.daemon.config import Settings


class TestSettings:
    """Tests pour la classe Settings."""

    def test_default_values(self):
        """Test valeurs par défaut."""
        settings = Settings()

        assert settings.environment == "dev"
        assert settings.api_title == "BBIA-SIM API - Écosystème Reachy Mini"
        assert settings.api_host == "0.0.0.0"  # nosec B104
        assert settings.api_port == 8000
        assert settings.api_token == "bbia-secret-key-dev"
        assert settings.simulation_headless is True
        assert settings.telemetry_enabled is True

    def test_init_with_kwargs(self):
        """Test initialisation avec paramètres."""
        settings = Settings(environment="prod", api_port=9000)

        assert settings.environment == "prod"
        assert settings.api_port == 9000

    @patch("dotenv.load_dotenv")
    def test_init_dev_environment_loads_dotenv(self, mock_load_dotenv):
        """Test chargement dotenv en environnement dev."""
        Settings(environment="dev")

        mock_load_dotenv.assert_called_once()

    @patch("dotenv.load_dotenv", side_effect=ImportError)
    def test_init_dev_environment_dotenv_import_error(self, mock_load_dotenv):
        """Test gestion ImportError lors du chargement dotenv."""
        # Ne devrait pas lever d'exception
        settings = Settings(environment="dev")

        assert settings.environment == "dev"

    def test_get_cors_origins_dev(self):
        """Test origines CORS en environnement dev."""
        settings = Settings(environment="dev")

        origins = settings.get_cors_origins()
        assert origins == ["*"]

    def test_get_cors_origins_prod_with_specific_origins(self):
        """Test origines CORS en environnement prod avec origines spécifiques."""
        settings = Settings(environment="prod", cors_origins=["https://example.com"])

        origins = settings.get_cors_origins()
        assert origins == ["https://example.com"]

    def test_get_cors_origins_prod_with_wildcard(self):
        """Test origines CORS en environnement prod avec wildcard."""
        settings = Settings(environment="prod", cors_origins=["*"])

        origins = settings.get_cors_origins()
        assert origins == []  # Aucune origine autorisée par défaut en prod

    def test_get_simulation_config(self):
        """Test configuration simulation."""
        settings = Settings()

        config = settings.get_simulation_config()

        assert "model_path" in config
        assert "headless" in config
        assert "step_frequency" in config
        assert "duration" in config
        assert config["headless"] is True

    def test_is_production_true(self):
        """Test détection environnement production."""
        settings = Settings(environment="prod")

        assert settings.is_production() is True

    def test_is_production_false(self):
        """Test détection environnement non-production."""
        settings = Settings(environment="dev")

        assert settings.is_production() is False

    def test_get_security_headers_prod(self):
        """Test headers sécurité en production."""
        settings = Settings(environment="prod")

        headers = settings.get_security_headers()

        assert "X-Content-Type-Options" in headers
        assert "X-Frame-Options" in headers
        assert "X-XSS-Protection" in headers
        assert "Referrer-Policy" in headers
        assert "Strict-Transport-Security" in headers

    def test_get_security_headers_dev(self):
        """Test headers sécurité en développement."""
        settings = Settings(environment="dev")

        headers = settings.get_security_headers()

        assert headers == {}  # Pas de headers stricts en dev

    def test_mask_token_long(self):
        """Test masquage token long."""
        settings = Settings()

        masked = settings.mask_token("very-long-token-12345")

        assert masked == "very...2345"

    def test_mask_token_short(self):
        """Test masquage token court."""
        settings = Settings()

        masked = settings.mask_token("short")

        assert masked == "***"

    def test_mask_token_exactly_8_chars(self):
        """Test masquage token exactement 8 caractères."""
        settings = Settings()

        masked = settings.mask_token("12345678")

        assert masked == "***"  # <= 8 caractères

    def test_mask_token_9_chars(self):
        """Test masquage token 9 caractères."""
        settings = Settings()

        masked = settings.mask_token("123456789")

        assert masked == "1234...6789"  # > 8 caractères
