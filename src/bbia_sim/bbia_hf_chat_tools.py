"""Helpers chat / outils HF — extraits de ``bbia_huggingface``.

Découpe non big-bang : patterns mots-clés, descriptions NLP, mappings
paramètres, extracteurs d'angle/intensité. L'orchestration
(``_detect_and_execute_tools`` / NLP / exécution) reste sur
``BBIAHuggingFace`` et consomme ces constantes / fonctions.
"""

from __future__ import annotations

import math
from typing import Any

# Descriptions sémantiques pour détection NLP (sentence-transformers).
TOOL_NLP_DESCRIPTIONS: dict[str, str] = {
    "move_head": (
        "Déplacer ou tourner la tête du robot vers une direction "
        "(gauche, droite, haut, bas, orienter la tête)"
    ),
    "camera": (
        "Capturer une image, prendre une photo, analyser "
        "l'environnement visuel, "
        "regarder autour, que vois-tu"
    ),
    "dance": (
        "Faire danser le robot, jouer une danse, mouvement de danse, "
        "bouger au rythme"
    ),
    "play_emotion": (
        "Jouer une émotion sur le robot (joie, tristesse, curiosité, "
        "excitation, calme, colère, surprise)"
    ),
    "stop_dance": "Arrêter la danse en cours, stopper la danse",
    "stop_emotion": "Arrêter l'émotion en cours, stopper l'émotion",
    "head_tracking": (
        "Activer ou désactiver le suivi automatique du visage, tracking visage"
    ),
    "do_nothing": "Rester inactif, ne rien faire, reste tranquille",
}

# Mappings FR → EN pour paramètres d'outils (NLP + mots-clés).
MOVE_HEAD_MAPPINGS: dict[str, str] = {
    "gauche": "left",
    "droite": "right",
    "haut": "up",
    "bas": "down",
}

PLAY_EMOTION_MAPPINGS: dict[str, str] = {
    "heureux": "happy",
    "joyeux": "happy",
    "content": "happy",
    "joie": "happy",
    "triste": "sad",
    "tristesse": "sad",
    "curieux": "curious",
    "curiosité": "curious",
    "excité": "excited",
    "excitation": "excited",
    "calme": "calm",
    "neutre": "neutral",
    "colère": "angry",
    "angry": "angry",
    "surprise": "surprised",
    "surpris": "surprised",
}

SENTIMENT_TO_EMOTION: dict[str, str] = {
    "POSITIVE": "happy",
    "NEGATIVE": "sad",
    "NEUTRAL": "neutral",
}

INTENSITY_KEYWORDS: dict[str, float] = {
    "légèrement": 0.2,
    "un peu": 0.2,
    "subtilement": 0.2,
    "doucement": 0.3,
    "modérément": 0.5,
    "moyennement": 0.5,
    "normalement": 0.5,
    "beaucoup": 0.8,
    "fortement": 0.8,
    "énormément": 0.9,
    "maximalement": 1.0,
    "complètement": 1.0,
    "totalement": 1.0,
}

# Patterns mots-clés (détection sans NLP).
TOOL_KEYWORD_PATTERNS: dict[str, dict[str, Any]] = {
    "move_head": {
        "keywords": [
            "tourne la tête",
            "bouge la tête",
            "regarde à gauche",
            "regarde à droite",
            "regarde en haut",
            "regarde en bas",
            "tourne ta tête",
            "bouge ta tête",
            "orienter la tête",
            "orienter ta tête",
            "déplacer la tête",
            "diriger la tête",
            "pivoter la tête",
            "pivote la tête",
            "bouger la tête",
            "tourner la tête",
            "regarde vers la gauche",
            "regarde vers la droite",
            "regarde vers le haut",
            "regarde vers le bas",
            "gauche",
            "droite",
            "haut",
            "bas",
            "à gauche",
            "à droite",
            "en haut",
            "en bas",
            "tourne tête",
            "bouge tête",
            "orienter tête",
            "regarde gauche",
            "regarde droite",
        ],
        "mappings": MOVE_HEAD_MAPPINGS,
    },
    "camera": {
        "keywords": [
            "capture une image",
            "prends une photo",
            "regarde autour",
            "analyse l'environnement",
            "que vois-tu",
            "prends une image",
            "capture une photo",
            "prends une photo",
            "fais une photo",
            "fais une image",
            "photographie",
            "photo",
            "image",
            "regarde",
            "qu'est-ce que tu vois",
            "qu'est-ce que tu observes",
            "décris ce que tu vois",
            "analyse l'image",
            "analyse l'environnement",
            "que vois-tu autour",
            "qu'y a-t-il autour",
            "regarde ce qui t'entoure",
        ],
    },
    "dance": {
        "keywords": [
            "danse",
            "fais danser",
            "joue une danse",
            "danse un peu",
            "danse pour moi",
            "fais une danse",
            "lance une danse",
            "joue une danse",
            "montre une danse",
            "exécute une danse",
            "danser",
            "dance",
            "bouge au rythme",
            "bouge en rythme",
        ],
    },
    "play_emotion": {
        "keywords": [
            "sois heureux",
            "sois triste",
            "montre de la joie",
            "montre de la curiosité",
            "sois joyeux",
            "sois content",
            "sois excité",
            "sois calme",
            "sois neutre",
            "sois curieux",
            "montre de la joie",
            "montre de la tristesse",
            "montre de l'excitation",
            "montre de la curiosité",
            "montre du calme",
            "être heureux",
            "être triste",
            "être joyeux",
            "être calme",
            "être excité",
            "joie",
            "tristesse",
            "curiosité",
            "excitation",
            "calme",
            "neutre",
            "colère",
            "surprise",
        ],
        "mappings": PLAY_EMOTION_MAPPINGS,
    },
}

# Paramètres par défaut / datasets outils.
DEFAULT_DANCE_PARAMS: dict[str, str] = {
    "move_name": "happy_dance",
    "dataset": "pollen-robotics/reachy-mini-dances-library",
}

NLP_CONFIDENCE_THRESHOLD: float = 0.6


def extract_angle(message: str) -> float | None:
    """Extrait un angle depuis un message (degrés, radians, %)."""
    message_lower = message.lower()[:1000]

    def _extract_number_before(token: str) -> float | None:
        idx = message_lower.find(token)
        if idx <= 0:
            return None
        prefix = message_lower[:idx].rstrip()
        chars: list[str] = []
        for ch in reversed(prefix):
            if ch.isdigit() or ch in {".", ","}:
                chars.append(ch)
                continue
            if chars:
                break
        if not chars:
            return None
        try:
            return float("".join(reversed(chars)).replace(",", "."))
        except ValueError:
            return None

    for token in ("degrés", "degré", "degrees", "degree"):
        value = _extract_number_before(token)
        if value is not None:
            return value

    for token in ("radians", "radian"):
        idx = message_lower.find(token)
        if idx <= 0:
            continue
        expr = (
            message_lower[:idx].strip().split()[-1]
            if message_lower[:idx].strip()
            else ""
        )
        if "/" in expr and "pi" in expr:
            parts = expr.replace(" ", "").split("/", maxsplit=1)
            if len(parts) == 2:
                try:
                    denom = float(parts[1])
                    if denom != 0:
                        return math.degrees(math.pi / denom)
                except ValueError:
                    pass
        else:
            try:
                return math.degrees(float(expr.replace(",", ".")))
            except ValueError:
                pass

    if "%" in message_lower:
        pct = _extract_number_before("%")
        if pct is not None:
            return (pct / 100.0) * 90.0

    return None


def extract_intensity(message: str) -> float | None:
    """Extrait une intensité (0–1) depuis mots-clés ou pourcentage."""
    message_lower = message.lower()

    for keyword, intensity in INTENSITY_KEYWORDS.items():
        if keyword in message_lower:
            return intensity

    msg = message_lower[:1000]
    percent_idx = msg.find("%")
    if percent_idx > 0:
        prefix = msg[:percent_idx].rstrip()
        chars: list[str] = []
        for ch in reversed(prefix):
            if ch.isdigit() or ch in {".", ","}:
                chars.append(ch)
                continue
            if chars:
                break
        if chars:
            try:
                pct = float("".join(reversed(chars)).replace(",", "."))
                return min(pct / 100.0, 1.0)
            except ValueError:
                pass

    return None


def format_tool_result_text(result: dict[str, Any] | None, tool_name: str) -> str:
    """Formate le dict retourné par ``execute_tool`` en message utilisateur."""
    if result is None:
        return f"❌ Erreur lors de l'exécution de l'outil '{tool_name}'"
    if result.get("status") == "success":
        detail = result.get("detail", "Action exécutée")
        return f"✅ {detail}"
    error_detail = result.get("detail", "Erreur inconnue")
    return f"⚠️ {error_detail}"
