#!/usr/bin/env python3
"""Tests pour BBIA Awake - Import direct pour coverage."""

import sys
from pathlib import Path
from unittest.mock import patch

import pytest

# S'assurer que src est dans le path pour coverage
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# Importer le module directement - coverage doit le détecter
# IMPORTANT: Import direct (pas dans try/except) pour que coverage le détecte
import bbia_sim.bbia_awake  # noqa: F401
from bbia_sim.bbia_awake import start_bbia_sim


def test_start_bbia_sim_function_exists():
    """Test que la fonction start_bbia_sim existe."""
    assert callable(start_bbia_sim)


@patch("bbia_sim.bbia_awake.print")
@patch("bbia_sim.bbia_awake.time.sleep")
def test_start_bbia_sim_execution(mock_sleep, mock_print):
    """Test exécution de start_bbia_sim avec mocks."""
    start_bbia_sim()

    # Vérifier que print a été appelé plusieurs fois (séquence de réveil)
    assert mock_print.call_count >= 3  # Au moins 3 messages

    # Vérifier que sleep a été appelé
    assert mock_sleep.call_count >= 1

    # Vérifier que le dernier message contient "réveillé"
    last_call = mock_print.call_args_list[-1]
    assert "réveillé" in str(last_call).lower() or "BBIA" in str(last_call)


@patch("bbia_sim.bbia_awake.print")
@patch("bbia_sim.bbia_awake.time.sleep")
def test_start_bbia_sim_contains_patterns(mock_sleep, mock_print):
    """Test que la séquence contient les patterns essentiels."""
    start_bbia_sim()

    # Récupérer tous les messages imprimés
    all_messages = " ".join(str(call) for call in mock_print.call_args_list)

    # Patterns essentiels (au moins un doit être présent)
    essential_patterns = ["Lumière", "Mouvements", "Première pensée", "réveillé"]
    found_patterns = [
        p for p in essential_patterns if p.lower() in all_messages.lower()
    ]

    # Au moins 2 patterns doivent être trouvés
    assert len(found_patterns) >= 2, f"Patterns trouvés: {found_patterns}"


def test_start_bbia_sim_no_errors():
    """Test que start_bbia_sim s'exécute sans erreur."""
    with patch("bbia_sim.bbia_awake.print"), patch("bbia_sim.bbia_awake.time.sleep"):
        try:
            start_bbia_sim()
        except Exception as e:
            pytest.fail(f"start_bbia_sim a levé une exception: {e}")
