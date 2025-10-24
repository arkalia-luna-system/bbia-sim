"""Configuration pytest pour les tests e2e."""

import pytest

# Marqueurs pour les tests e2e
pytestmark = [
    pytest.mark.e2e,
    pytest.mark.asyncio,
]
