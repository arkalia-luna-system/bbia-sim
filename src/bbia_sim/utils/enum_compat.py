"""Compatibilité enum pour Python < 3.11 (StrEnum ajouté en 3.11)."""

import sys

if sys.version_info >= (3, 11):  # noqa: UP036
    from enum import StrEnum
else:
    from enum import Enum

    class StrEnum(str, Enum):  # noqa: UP042
        """StrEnum pour Python 3.10 (sinon import depuis enum en 3.11+)."""

        pass


__all__ = ["StrEnum"]
