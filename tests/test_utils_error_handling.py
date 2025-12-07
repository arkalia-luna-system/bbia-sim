#!/usr/bin/env python3
"""
Tests unitaires pour le module utils.error_handling.

Vérifie que safe_execute, safe_import et safe_execute_with_exceptions
fonctionnent correctement dans tous les cas.
"""

import logging
from unittest.mock import MagicMock, patch

import pytest

from bbia_sim.utils.error_handling import (
    safe_execute,
    safe_execute_with_exceptions,
    safe_import,
)

logger = logging.getLogger(__name__)


class TestSafeExecute:
    """Tests pour safe_execute()."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_success(self):
        """Test safe_execute avec fonction qui réussit."""
        result = safe_execute(lambda: 42, fallback=None)
        assert result == 42

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_failure_with_fallback(self):
        """Test safe_execute avec fonction qui échoue, retourne fallback."""
        result = safe_execute(
            lambda: 1 / 0,  # ZeroDivisionError
            fallback=0,
        )
        assert result == 0

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_failure_with_logger_debug(self):
        """Test safe_execute log en DEBUG si critical=False."""
        mock_logger = MagicMock(spec=logging.Logger)
        safe_execute(
            lambda: 1 / 0,
            fallback=None,
            logger_instance=mock_logger,
            error_msg="Test erreur",
            critical=False,
        )
        mock_logger.debug.assert_called_once()
        assert "Test erreur" in str(mock_logger.debug.call_args)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_failure_with_logger_error(self):
        """Test safe_execute log en ERROR si critical=True."""
        mock_logger = MagicMock(spec=logging.Logger)
        safe_execute(
            lambda: 1 / 0,
            fallback=None,
            logger_instance=mock_logger,
            error_msg="Test erreur critique",
            critical=True,
        )
        mock_logger.error.assert_called_once()
        assert "Test erreur critique" in str(mock_logger.error.call_args)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_reraise(self):
        """Test safe_execute avec reraise=True."""
        with pytest.raises(ZeroDivisionError):
            safe_execute(
                lambda: 1 / 0,
                fallback=None,
                reraise=True,
            )

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_no_logger(self):
        """Test safe_execute sans logger (ne doit pas crasher)."""
        result = safe_execute(
            lambda: 1 / 0,
            fallback=42,
            logger_instance=None,
        )
        assert result == 42

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_with_string_result(self):
        """Test safe_execute avec résultat string."""
        result = safe_execute(lambda: "success", fallback="failure")
        assert result == "success"

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_with_list_result(self):
        """Test safe_execute avec résultat list."""
        result = safe_execute(lambda: [1, 2, 3], fallback=[])
        assert result == [1, 2, 3]


class TestSafeImport:
    """Tests pour safe_import()."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_import_success(self):
        """Test safe_import avec module existant."""
        result = safe_import("os")
        assert result is not None
        assert hasattr(result, "path")

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_import_failure_import_error(self):
        """Test safe_import avec ImportError."""
        result = safe_import("module_qui_n_existe_pas_12345")
        assert result is None

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_import_failure_with_logger(self):
        """Test safe_import log en DEBUG si ImportError."""
        mock_logger = MagicMock(spec=logging.Logger)
        result = safe_import(
            "module_qui_n_existe_pas_12345",
            logger_instance=mock_logger,
        )
        assert result is None
        mock_logger.debug.assert_called_once()
        assert "module_qui_n_existe_pas_12345" in str(mock_logger.debug.call_args)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_import_unexpected_error(self):
        """Test safe_import avec erreur inattendue (pas ImportError)."""
        mock_logger = MagicMock(spec=logging.Logger)
        # Simuler une erreur inattendue lors de l'import
        with patch(
            "builtins.__import__", side_effect=RuntimeError("Erreur inattendue")
        ):
            result = safe_import("os", logger_instance=mock_logger)
            assert result is None
            mock_logger.warning.assert_called_once()
            assert "Erreur inattendue" in str(mock_logger.warning.call_args)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_import_no_logger(self):
        """Test safe_import sans logger (ne doit pas crasher)."""
        result = safe_import("module_qui_n_existe_pas_12345", logger_instance=None)
        assert result is None


class TestSafeExecuteWithExceptions:
    """Tests pour safe_execute_with_exceptions()."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_with_exceptions_success(self):
        """Test safe_execute_with_exceptions avec fonction qui réussit."""
        result = safe_execute_with_exceptions(
            lambda: 42,
            expected_exceptions=(ValueError,),
            fallback=0,
        )
        assert result == 42

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_with_exceptions_expected_error(self):
        """Test safe_execute_with_exceptions avec exception attendue."""
        mock_logger = MagicMock(spec=logging.Logger)
        result = safe_execute_with_exceptions(
            lambda: int("not a number"),
            expected_exceptions=(ValueError,),
            fallback=0,
            logger_instance=mock_logger,
            error_msg="Erreur attendue",
            critical=False,
        )
        assert result == 0
        mock_logger.debug.assert_called_once()
        assert "ValueError" in str(mock_logger.debug.call_args)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_with_exceptions_expected_error_critical(self):
        """Test safe_execute_with_exceptions avec exception attendue, critical=True."""
        mock_logger = MagicMock(spec=logging.Logger)
        result = safe_execute_with_exceptions(
            lambda: int("not a number"),
            expected_exceptions=(ValueError,),
            fallback=0,
            logger_instance=mock_logger,
            error_msg="Erreur critique",
            critical=True,
        )
        assert result == 0
        mock_logger.error.assert_called_once()
        assert "ValueError" in str(mock_logger.error.call_args)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_with_exceptions_unexpected_error(self):
        """Test safe_execute_with_exceptions avec exception non attendue."""
        mock_logger = MagicMock(spec=logging.Logger)
        result = safe_execute_with_exceptions(
            lambda: 1 / 0,  # ZeroDivisionError, pas dans expected_exceptions
            expected_exceptions=(ValueError, TypeError),
            fallback=0,
            logger_instance=mock_logger,
            error_msg="Erreur",
        )
        assert result == 0
        mock_logger.warning.assert_called_once()
        assert "erreur inattendue" in str(mock_logger.warning.call_args).lower()
        assert "ZeroDivisionError" in str(mock_logger.warning.call_args)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_with_exceptions_multiple_expected(self):
        """Test safe_execute_with_exceptions avec plusieurs exceptions attendues."""
        mock_logger = MagicMock(spec=logging.Logger)
        result = safe_execute_with_exceptions(
            lambda: int("not a number"),
            expected_exceptions=(ValueError, TypeError, AttributeError),
            fallback=0,
            logger_instance=mock_logger,
        )
        assert result == 0
        mock_logger.debug.assert_called_once()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_with_exceptions_no_logger(self):
        """Test safe_execute_with_exceptions sans logger (ne doit pas crasher)."""
        result = safe_execute_with_exceptions(
            lambda: 1 / 0,
            expected_exceptions=(ValueError,),
            fallback=42,
            logger_instance=None,
        )
        assert result == 42


class TestErrorHandlingIntegration:
    """Tests d'intégration pour vérifier que le module fonctionne bien."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_real_world_scenario(self):
        """Test scénario réel : configuration variable d'environnement."""
        import os

        result = safe_execute(
            lambda: os.environ.setdefault("TEST_KEY", "test_value"),
            fallback=None,
            logger_instance=logger,
            error_msg="Impossible de configurer variable d'environnement",
            critical=False,
        )
        assert result is not None
        assert os.environ.get("TEST_KEY") == "test_value"
        # Nettoyer
        os.environ.pop("TEST_KEY", None)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_import_real_world_scenario(self):
        """Test scénario réel : import conditionnel de module optionnel."""
        # Tester avec un module qui existe
        numpy = safe_import("numpy", logger_instance=logger)
        if numpy is not None:
            assert hasattr(numpy, "array")

        # Tester avec un module qui n'existe pas
        fake_module = safe_import("fake_module_xyz_123", logger_instance=logger)
        assert fake_module is None

    @pytest.mark.unit
    @pytest.mark.fast
    def test_safe_execute_with_exceptions_real_world_scenario(self):
        """Test scénario réel : gestion erreurs connexion robot."""
        result = safe_execute_with_exceptions(
            lambda: int("invalid"),  # Simule erreur de connexion
            expected_exceptions=(ValueError, ConnectionError, RuntimeError),
            fallback=False,
            logger_instance=logger,
            error_msg="Erreur lors de la connexion au robot",
            critical=True,
        )
        assert result is False


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
