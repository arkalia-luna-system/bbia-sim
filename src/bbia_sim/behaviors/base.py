#!/usr/bin/env python3
"""Classe de base pour tous les comportements BBIA.

Ce module définit l'interface commune que tous les comportements
doivent implémenter.
"""

from __future__ import annotations

# Import de BBIABehavior depuis bbia_behavior pour éviter la duplication
# et assurer la compatibilité avec register_behavior
from ..bbia_behavior import BBIABehavior

__all__ = ["BBIABehavior"]
