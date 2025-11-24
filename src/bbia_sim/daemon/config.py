"""Configuration pour le daemon BBIA-SIM."""

from typing import Any

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Configuration de l'application."""

    model_config = SettingsConfigDict(env_file=".env", env_prefix="BBIA_")

    # Environnement
    environment: str = "dev"  # dev/prod

    # API Configuration
    api_title: str = "BBIA-SIM API - Écosystème Reachy Mini"
    api_description: str = "API REST et WebSocket pour le contrôle du robot Reachy Mini"
    api_version: str = "1.3.2"
    api_host: str = "0.0.0.0"  # nosec B104
    api_port: int = 8000
    api_reload: bool = True

    # Security
    api_token: str = "bbia-secret-key-dev"
    cors_origins: list[str] = ["*"]  # En production, spécifier les domaines autorisés

    # Simulation
    simulation_model_path: str = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    simulation_headless: bool = (
        True  # Headless par défaut pour éviter les problèmes macOS
    )
    simulation_step_frequency: float = 0.01  # 100 Hz
    simulation_duration: int | None = None

    # WebSocket
    telemetry_frequency: float = 10.0  # Hz
    telemetry_enabled: bool = True

    # Logging
    log_level: str = "INFO"
    log_format: str = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"

    # Security limits (prod)
    max_request_size: int = 1024 * 1024  # 1MB
    request_timeout: int = 30  # seconds
    rate_limit_requests: int = 100  # per minute
    rate_limit_window: int = 60  # seconds

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Chargement depuis .env en dev uniquement
        if self.environment == "dev":
            try:
                from dotenv import load_dotenv

                load_dotenv()
            except ImportError:
                pass  # python-dotenv pas installé, pas grave

    def get_cors_origins(self) -> list[str]:
        """Retourne les origines CORS autorisées selon l'environnement.

        En production, CORS strict : uniquement les origines spécifiées.
        En développement, permissif pour faciliter le développement.

        Returns:
            Liste des origines CORS autorisées
        """
        if self.is_production():
            # En prod, utiliser les origines spécifiées ou rejeter tout
            # CORS strict en production : aucune origine par défaut si "*"
            if self.cors_origins == ["*"]:
                return []  # Aucune origine autorisée par défaut en prod
            return self.cors_origins
        # En dev, permissif mais documenté
        return ["*"]

    def get_simulation_config(self) -> dict[str, Any]:
        """Retourne la configuration de la simulation."""
        return {
            "model_path": self.simulation_model_path,
            "headless": self.simulation_headless,
            "step_frequency": self.simulation_step_frequency,
            "duration": self.simulation_duration,
        }

    def is_production(self) -> bool:
        """Indique si l'environnement est en production."""
        return self.environment.lower() == "prod"

    def get_security_headers(self) -> dict[str, str]:
        """Retourne les headers de sécurité selon l'environnement."""
        if self.is_production():
            return {
                "X-Content-Type-Options": "nosniff",
                "X-Frame-Options": "DENY",
                "X-XSS-Protection": "1; mode=block",
                "Referrer-Policy": "strict-origin-when-cross-origin",
                "Strict-Transport-Security": "max-age=31536000; includeSubDomains",
            }
        return {}  # Pas de headers stricts en dev

    def mask_token(self, token: str) -> str:
        """Masque un token pour les logs."""
        if len(token) <= 8:
            return "***"
        return f"{token[:4]}...{token[-4:]}"


settings = Settings()
