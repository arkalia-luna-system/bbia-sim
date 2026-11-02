"""Configuration pour le daemon BBIA-SIM."""

from __future__ import annotations

import os
from typing import Any

from pydantic import model_validator
from pydantic_settings import BaseSettings, SettingsConfigDict

# Token publié dans .env.example / docker-compose — interdit en production.
DEFAULT_DEV_API_TOKEN = (
    "bbia-secret-key-dev"  # nosec B105 — défaut dev connu, refusé en prod
)

# Alias documentés (compose, .env.example, README) → champs Settings (env_prefix BBIA_).
# pydantic-settings n'applique pas AliasChoices de façon fiable avec env_prefix ; on
# résout explicitement : forme canonique d'abord, puis alias historique.
_ENV_ALIASES: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("api_token", ("BBIA_API_TOKEN", "BBIA_TOKEN")),
    ("environment", ("BBIA_ENVIRONMENT", "BBIA_ENV")),
    ("api_host", ("BBIA_API_HOST", "BBIA_HOST")),
    ("api_port", ("BBIA_API_PORT", "BBIA_PORT")),
    ("api_reload", ("BBIA_API_RELOAD", "BBIA_RELOAD")),
)


def _load_dotenv_early() -> None:
    """Charge ``.env`` avant construction Settings (aliases + champs)."""
    try:
        from dotenv import load_dotenv

        load_dotenv()
    except ImportError:
        pass


_load_dotenv_early()


class Settings(BaseSettings):
    """Configuration de l'application."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_prefix="BBIA_",
        extra="ignore",
    )

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
    api_token: str = DEFAULT_DEV_API_TOKEN
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
    # Anticipation Pollen #1494 : keepalive WS configurable (secondes)
    ws_keepalive_timeout: float = 30.0
    # Client SDK → daemon Pollen (#1494 suite / #1466) — aussi lisibles hors Settings
    # via bbia_sim.sdk_connection (BBIA_SDK_CONNECT_TIMEOUT, _RECONNECT_*, _HEARTBEAT_*)
    sdk_connect_timeout: float = 5.0
    sdk_reconnect_attempts: int = 3
    sdk_reconnect_delay: float = 0.5

    # Logging
    log_level: str = "INFO"
    log_format: str = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"

    # Security limits (prod)
    max_request_size: int = 1024 * 1024  # 1MB
    request_timeout: int = 30  # seconds
    rate_limit_requests: int = 100  # per minute
    rate_limit_window: int = 60  # seconds

    @model_validator(mode="before")
    @classmethod
    def _apply_documented_env_aliases(cls, data: Any) -> Any:
        """Mappe ``BBIA_TOKEN`` / ``BBIA_ENV`` (et HOST/PORT/RELOAD) vers les champs."""
        if not isinstance(data, dict):
            data = {}
        else:
            data = dict(data)
        for field, keys in _ENV_ALIASES:
            for key in keys:
                if key in os.environ:
                    data[field] = os.environ[key]
                    break
        return data

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

    def uses_insecure_api_token(self) -> bool:
        """True si le token est vide ou égal au défaut de développement."""
        token = self.api_token if isinstance(self.api_token, str) else ""
        return (not token.strip()) or token == DEFAULT_DEV_API_TOKEN

    def require_secure_production_token(self) -> None:
        """Fail-fast en prod si token vide / défaut connu.

        Raises:
            RuntimeError: environnement prod avec secret faible.
        """
        if self.is_production() and self.uses_insecure_api_token():
            raise RuntimeError(
                "Production refusée: définissez un BBIA_API_TOKEN (ou BBIA_TOKEN) "
                "fort — le défaut 'bbia-secret-key-dev' et les tokens vides sont "
                "interdits. Voir .env.example."
            )

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
