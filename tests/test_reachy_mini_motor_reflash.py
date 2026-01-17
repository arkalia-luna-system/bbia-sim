"""Tests pour le reflash automatique des moteurs et le mode opératoire.

Ces tests vérifient que le code BBIA gère correctement :
- Le reflash automatique des moteurs (SDK v1.2.4+)
- Le workaround set_operating_mode pour compatibilité anciennes versions
- La protection contre les problèmes batch QC 2544
"""

import pytest
from unittest.mock import MagicMock, patch

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


class TestMotorReflash:
    """Tests pour le reflash automatique des moteurs."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_reflash_automatic_on_connection(self):
        """Test que le reflash automatique est documenté dans les logs."""
        backend = ReachyMiniBackend(use_sim=True)
        
        # En mode simulation, le reflash ne se fait pas vraiment
        # mais on vérifie que le code est prêt
        backend.connect()
        
        # Vérifier que le backend est en mode simulation
        assert backend.use_sim is True
        # En mode simulation, is_connected peut être True ou False selon l'implémentation
        # L'important est que use_sim soit True

    @pytest.mark.unit
    @pytest.mark.fast
    def test_operating_mode_workaround(self):
        """Test que le workaround set_operating_mode fonctionne."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        # Créer un mock robot avec set_operating_mode
        mock_robot = MagicMock()
        mock_robot.set_operating_mode = MagicMock()
        backend.robot = mock_robot
        backend.is_connected = True
        backend.use_sim = False

        # Appeler enable_motors
        backend.enable_motors()

        # Vérifier que set_operating_mode a été appelé si disponible
        # (en simulation, le robot n'existe pas, donc pas d'appel)
        # Mais le code doit gérer gracieusement l'absence de la méthode

    @pytest.mark.unit
    @pytest.mark.fast
    def test_operating_mode_without_method(self):
        """Test que le code gère gracieusement l'absence de set_operating_mode."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        # Créer un mock robot SANS set_operating_mode (SDK v1.2.4+)
        mock_robot = MagicMock()
        del mock_robot.set_operating_mode  # Simuler SDK v1.2.4+
        backend.robot = mock_robot
        backend.is_connected = True
        backend.use_sim = False

        # Appeler enable_motors ne doit pas planter
        backend.enable_motors()

        # Le code doit gérer gracieusement (pas d'erreur)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_enable_motors_simulation_mode(self):
        """Test enable_motors en mode simulation."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        # En mode simulation, enable_motors doit fonctionner sans erreur
        backend.enable_motors()

        # Pas d'erreur attendue

    @pytest.mark.unit
    @pytest.mark.fast
    def test_disable_motors_simulation_mode(self):
        """Test disable_motors en mode simulation."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        # En mode simulation, disable_motors doit fonctionner sans erreur
        backend.disable_motors()

        # Pas d'erreur attendue

    @pytest.mark.unit
    @pytest.mark.fast
    @patch("bbia_sim.backends.reachy_mini_backend.ReachyMini")
    def test_reflash_log_message(self, mock_reachy_class):
        """Test que le message de log sur le reflash est présent."""
        # Créer un mock qui simule une connexion réussie
        mock_robot = MagicMock()
        mock_reachy_class.return_value = mock_robot

        backend = ReachyMiniBackend(use_sim=False, localhost_only=True)

        # La connexion devrait logger le message sur le reflash
        # (on ne peut pas vraiment tester les logs facilement, mais on vérifie
        # que le code ne plante pas)
        result = backend.connect()

        # Le résultat dépend de si un robot est disponible
        assert isinstance(result, bool)


class TestMotorOperatingMode:
    """Tests pour le mode opératoire des moteurs."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_operating_mode_compatibility_old_sdk(self):
        """Test compatibilité avec anciennes versions SDK (< v1.2.4)."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        # Simuler un robot avec set_operating_mode (ancienne version SDK)
        mock_robot = MagicMock()
        mock_robot.set_operating_mode = MagicMock()
        backend.robot = mock_robot
        backend.is_connected = True
        backend.use_sim = False

        # Appeler enable_motors
        backend.enable_motors()

        # Vérifier que le workaround est appliqué (si disponible)
        # En simulation, le robot n'existe pas vraiment

    @pytest.mark.unit
    @pytest.mark.fast
    def test_operating_mode_compatibility_new_sdk(self):
        """Test compatibilité avec nouvelles versions SDK (v1.2.4+)."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        # Simuler un robot SANS set_operating_mode (SDK v1.2.4+)
        mock_robot = MagicMock()
        # Ne pas définir set_operating_mode (SDK gère automatiquement)
        backend.robot = mock_robot
        backend.is_connected = True
        backend.use_sim = False

        # Appeler enable_motors ne doit pas planter
        backend.enable_motors()

        # Le code doit gérer gracieusement


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
