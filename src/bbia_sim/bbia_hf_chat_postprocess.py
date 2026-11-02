"""Post-traitement / contexte chat HF — extraits de ``bbia_huggingface``.

Normalisation longueur, anti-doublons, fallback sûr, nettoyage sortie LLM,
extraction de contexte conversationnel. Les méthodes instance du god-file
délèguent ici (API publique inchangée).
"""

from __future__ import annotations

import logging
import re
from functools import lru_cache
from typing import Any, Protocol, cast

from . import bbia_hf_response_bank as bank

logger = logging.getLogger(__name__)


class ConversationHistoryState(Protocol):
    """État minimal : historique conversation (deque / list)."""

    conversation_history: Any


@lru_cache(maxsize=128)
def get_compiled_regex(pattern: str, flags: int = 0) -> re.Pattern[str]:
    """Retourne regex compilée depuis cache (évite recompilation répétée)."""
    return re.compile(pattern, flags)


def safe_fallback(
    state: ConversationHistoryState | None = None,
) -> str:
    """Retourne un fallback naturel et varié pour éviter les chaînes vides."""
    try:
        import random as _r

        base_pool = [
            "Je peux préciser si besoin, qu'aimeriez-vous savoir exactement ?",
            "D'accord, dites-m'en un peu plus pour que je vous réponde au mieux.",
            "Merci pour votre message, souhaitez-vous que je détaille "
            "un point précis ?",
        ]
        base = base_pool[
            _r.randrange(len(base_pool))  # nosec B311 - Variété réponse non-crypto
        ]
        suffix = bank.SUFFIX_POOL[
            _r.randrange(
                len(bank.SUFFIX_POOL),
            )  # nosec B311 - Variété réponse non-crypto
        ]
        candidate = f"{base}{suffix}".strip()
        if state is None:
            return normalize_response_length(candidate, state=None)
        return normalize_response_length(candidate, state=state)
    except (ValueError, RuntimeError, TypeError):
        return cast(str, bank.SAFE_FALLBACK)


def avoid_recent_duplicates(
    text: str,
    state: ConversationHistoryState,
) -> str:
    """Évite les duplications exactes avec les dernières réponses BBIA."""
    try:
        if state.conversation_history:
            recent_history = list(state.conversation_history)[-5:]
            recent = [
                entry.get("bbia", "").strip()
                for entry in recent_history
                if entry.get("bbia", "").strip()
            ]
        else:
            recent = []
        if text and text in recent:
            import random as _r

            addition = bank.SUFFIX_POOL[
                _r.randrange(
                    len(bank.SUFFIX_POOL),
                )  # nosec B311 - Variété réponse non-crypto
            ]
            candidate = f"{text} {addition}".strip()
            return normalize_response_length(candidate, state=state)
        return text
    except (ValueError, RuntimeError, TypeError):
        return text


def normalize_response_length(
    text: str,
    state: ConversationHistoryState | None = None,
) -> str:
    """Normalise la longueur de la réponse vers ~[30, 150] caractères."""
    try:
        t = (text or "").strip()
        if not t or len(t) < 5 or t in {":", ": {", "if"}:
            return cast(str, bank.SAFE_FALLBACK)
        min_len, max_len = 30, 150
        if len(t) < min_len:
            import random as _r

            t = (
                t
                + bank.SUFFIX_POOL[
                    _r.randrange(
                        len(bank.SUFFIX_POOL),
                    )  # nosec B311 - Variété réponse non-crypto
                ]
            ).strip()
            if len(t) < min_len:
                t = (
                    t
                    + " "
                    + bank.SUFFIX_POOL[
                        _r.randrange(
                            len(bank.SUFFIX_POOL),
                        )  # nosec B311 - Variété réponse non-crypto
                    ]
                ).strip()
        if len(t) <= max_len:
            if state is not None:
                try:
                    t = avoid_recent_duplicates(t, state)
                except (TypeError, IndexError, KeyError) as e:
                    logger.debug(
                        "Erreur évitement doublons récents (type/index/key): %s",
                        e,
                    )
                except Exception as e:  # noqa: BLE001
                    logger.debug(
                        "Erreur lors de l'évitement des doublons récents: %s",
                        e,
                    )
            return t

        cut = t[: max_len + 1]
        last_stop = max(cut.rfind("."), cut.rfind("!"), cut.rfind("?"))
        if last_stop >= min_len // 2:
            t2 = cut[: last_stop + 1].strip()
            if state is not None:
                try:
                    t2 = avoid_recent_duplicates(t2, state)
                except (TypeError, IndexError, KeyError) as e:
                    logger.debug(
                        "Erreur évitement doublons récents t2 (type/index/key): %s",
                        e,
                    )
                except Exception as e:  # noqa: BLE001
                    logger.debug(
                        "Erreur lors de l'évitement des doublons récents (t2): %s",
                        e,
                    )
            return t2
        last_space = cut.rfind(" ")
        if last_space >= min_len:
            t3 = (cut[:last_space] + "...").strip()
            if state is not None:
                try:
                    t3 = avoid_recent_duplicates(t3, state)
                except (TypeError, IndexError, KeyError) as e:
                    logger.debug(
                        "Erreur évitement doublons récents t3 (type/index/key): %s",
                        e,
                    )
                except Exception as e:  # noqa: BLE001
                    logger.debug(
                        "Erreur lors de l'évitement des doublons récents (t3): %s",
                        e,
                    )
            return t3
        t4 = (t[:max_len] + "...").strip()
        if state is not None:
            try:
                t4 = avoid_recent_duplicates(t4, state)
            except (TypeError, IndexError, KeyError) as e:
                logger.debug(
                    "Erreur évitement doublons récents t4 (type/index/key): %s",
                    e,
                )
            except Exception as e:  # noqa: BLE001
                logger.debug(
                    "Erreur lors de l'évitement des doublons récents (t4): %s",
                    e,
                )
        return t4
    except (ValueError, RuntimeError, TypeError):
        return text


def postprocess_llm_output(
    text: str,
    user_message: str,
    state: ConversationHistoryState,
) -> str:
    """Nettoie et compacte la sortie LLM pour éviter la verbosité."""
    if not text:
        return safe_fallback(state)

    cleaned = get_compiled_regex(
        r"^(Assistant:|System:|User:)\s*",
        flags=re.IGNORECASE,
    ).sub("", text.strip())
    cleaned = cleaned.replace("\u200b", "").strip()

    filler_patterns = [
        r"en tant qu'?ia",
        r"je ne suis pas autoris[ée]",
        r"je ne peux pas fournir",
        r"je ne suis qu.?un mod[èe]le",
        r"désol[ée]",
    ]
    for pat in filler_patterns:
        cleaned = get_compiled_regex(pat, flags=re.IGNORECASE).sub("", cleaned)

    um = user_message.strip().lower()
    sentences = get_compiled_regex(r"(?<=[.!?…])\s+").split(cleaned)
    filtered: list[str] = []
    seen: set[str] = set()
    whitespace_regex = get_compiled_regex(r"\s+")
    for s in sentences:
        s_norm = whitespace_regex.sub(" ", s).strip()
        if not s_norm:
            continue
        if um and (um in s_norm.lower() or s_norm.lower() in um):
            continue
        key = s_norm.lower()
        if key in seen:
            continue
        seen.add(key)
        filtered.append(s_norm)

    if not filtered:
        return safe_fallback(state)

    limited = filtered[:3]
    result = " ".join(limited)

    max_chars = 2000
    if len(result) > max_chars:
        cut = result[:max_chars]
        m = get_compiled_regex(r"[.!?…](?=[^.!?…]*$)").search(cut)
        if m:
            cut = cut[: m.end()]
        result = cut.strip()

    result = whitespace_regex.sub(" ", result).strip()

    sentinels = {"", ":", ": {", "if"}
    if result in sentinels:
        result = safe_fallback(state)

    min_len, max_len = 30, 150
    if len(result) < min_len:
        import random as _r

        result = (
            result
            + bank.SUFFIX_POOL[
                _r.randrange(
                    len(bank.SUFFIX_POOL),
                )  # nosec B311 - Variété réponse non-crypto
            ]
        ).strip()
        if len(result) < min_len:
            result = (
                result
                + " "
                + bank.SUFFIX_POOL[
                    _r.randrange(
                        len(bank.SUFFIX_POOL),
                    )  # nosec B311 - Variété réponse non-crypto
                ]
            ).strip()
    if len(result) > max_len:
        cut = result[: max_len + 1]
        last_stop = max(cut.rfind("."), cut.rfind("!"), cut.rfind("?"))
        if last_stop >= min_len // 2:
            result = cut[: last_stop + 1].strip()
        else:
            last_space = cut.rfind(" ")
            result = (
                (cut[:last_space] if last_space >= min_len else cut[:max_len]) + "..."
            ).strip()

    return avoid_recent_duplicates(result, state)


def get_recent_context(state: ConversationHistoryState) -> str | None:
    """Extrait un mot-clé du contexte récent pour cohérence conversationnelle."""
    if not state.conversation_history:
        return None

    last_entry = (
        list(state.conversation_history)[-1] if state.conversation_history else None
    )
    if last_entry is None:
        return None
    user_msg = last_entry.get("user", "")

    stop_words = {
        "le",
        "la",
        "les",
        "un",
        "une",
        "de",
        "du",
        "des",
        "et",
        "ou",
        "est",
        "sont",
        "je",
        "tu",
        "il",
        "elle",
        "nous",
        "vous",
        "ils",
        "elles",
        "à",
        "pour",
        "avec",
        "dans",
        "sur",
        "par",
        "ne",
        "pas",
        "que",
        "qui",
        "quoi",
        "comment",
    }
    words_lower = user_msg.lower().split()
    words = [w for w in words_lower if len(w) > 3 and w not in stop_words]
    return str(words[0].capitalize()) if words else None


def build_context_string(state: ConversationHistoryState) -> str:
    """Construit le contexte pour la conversation."""
    if not state.conversation_history:
        return "Conversation avec BBIA (robot Reachy Mini). Soyez amical et curieux."

    recent_history = list(state.conversation_history)[-3:]
    context_lines = ["Historique conversation:"] + [
        line
        for entry in recent_history
        for line in [f"User: {entry['user']}", f"BBIA: {entry['bbia']}"]
    ]
    return "\n".join(context_lines)
