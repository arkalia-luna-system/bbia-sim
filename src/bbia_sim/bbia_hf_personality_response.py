"""Réponses personnalité HF — extraits de ``bbia_huggingface``.

Génération de réponses règles (salutations, sentiment, questions, contexte)
et adaptation emoji selon ``bbia_personality``. Les méthodes instance du
god-file délèguent ici (API publique inchangée).
"""

from __future__ import annotations

from typing import Any, Protocol, cast

from .bbia_hf_chat_postprocess import get_recent_context, normalize_response_length
from .utils.types import SentimentDict


class PersonalityResponseState(Protocol):
    """État minimal : personnalité + historique conversation."""

    bbia_personality: str
    conversation_history: Any


def generate_simple_response(
    message: str,
    sentiment: SentimentDict,
    state: PersonalityResponseState,
) -> str:
    """Génère réponse intelligente basée sur sentiment, contexte et personnalité.

    Args:
        message: Message utilisateur
        sentiment: Analyse sentiment du message

    Returns:
        Réponse intelligente et adaptative

    """
    import random

    message_lower = message.lower()
    sentiment_type = sentiment.get("sentiment", "NEUTRAL")
    sentiment_score = sentiment.get("score", 0.5)

    # Contexte : utiliser l'historique pour répondre de manière plus cohérente
    recent_context = get_recent_context(state)

    # Salutations - Réponses variées selon personnalité
    if any(
        word in message_lower for word in ["bonjour", "salut", "hello", "hi", "hey"]
    ):
        greetings = {
            "friendly_robot": [
                "Bonjour ! Ravi de vous revoir ! Comment allez-vous aujourd'hui ?",
                "Salut ! Qu'est-ce qui vous amène aujourd'hui ?",
                "Bonjour ! Que puis-je pour vous ?",
                "Coucou ! Ça fait plaisir de vous voir !",
                "Hey ! Content de vous retrouver !",
                "Salut ! Quoi de neuf depuis la dernière fois ?",
                "Bonjour ! Je suis là si vous avez besoin de moi.",
                "Hello ! Comment se passe votre journée ?",
                "Salut ! Prêt pour une nouvelle interaction ?",
                "Bonjour ! En quoi puis-je vous aider aujourd'hui ?",
            ],
            "curious": [
                "Bonjour ! Qu'est-ce qui vous intéresse aujourd'hui ?",
                "Salut ! J'ai hâte de savoir ce qui vous préoccupe !",
                "Hello ! Dites-moi, qu'est-ce qui vous passionne ?",
            ],
            "enthusiastic": [
                "Bonjour ! Super de vous voir ! Je suis prêt à interagir !",
                "Salut ! Quelle belle journée pour discuter ensemble !",
                "Hello ! Je suis tout excité de passer du temps avec vous !",
            ],
            "calm": [
                "Bonjour. Je suis là, prêt à vous écouter tranquillement.",
                "Salut. Comment vous sentez-vous aujourd'hui ?",
                "Bonjour. Que puis-je faire pour vous ?",
            ],
        }
        variants = greetings.get(state.bbia_personality, greetings["friendly_robot"])
        return cast(
            str,
            normalize_response_length(
                random.choice(variants),  # nosec B311 - Variété réponse non-crypto
                state=state,
            ),
        )

    # Au revoir - Réponses émotionnelles selon contexte
    if any(
        word in message_lower
        for word in ["au revoir", "bye", "goodbye", "à bientôt", "adieu"]
    ):
        goodbyes = {
            "friendly_robot": [
                "Au revoir ! Ce fut un plaisir de discuter avec vous. "
                "Revenez quand vous voulez !",
                "À bientôt ! N'hésitez pas à revenir pour continuer "
                "notre conversation.",
                "Au revoir ! J'espère vous revoir bientôt. Portez-vous bien !",
            ],
            "curious": [
                "Au revoir ! J'espère qu'on pourra continuer nos "
                "échanges intéressants !",
                "À bientôt ! J'ai encore plein de questions à vous poser !",
                "Au revoir ! Revenez pour partager de nouvelles découvertes !",
            ],
            "enthusiastic": [
                "Au revoir ! C'était génial de discuter ! Revenez vite !",
                "À bientôt ! J'ai hâte de vous revoir pour de nouvelles " "aventures !",
                "Au revoir ! C'était super ! Revenez quand vous voulez !",
            ],
            "calm": [
                "Au revoir. Je suis là quand vous aurez besoin de moi.",
                "À bientôt. Prenez soin de vous.",
                "Au revoir. Revenez quand vous vous sentirez prêt.",
            ],
        }
        variants = goodbyes.get(state.bbia_personality, goodbyes["friendly_robot"])
        return cast(
            str,
            normalize_response_length(
                random.choice(variants),  # nosec B311 - Variété réponse non-crypto
                state=state,
            ),
        )

    # Positif - Réponses adaptées selon intensité
    if (
        sentiment_type == "POSITIVE"
        or sentiment_score > 0.7
        or any(
            word in message_lower
            for word in [
                "content",
                "heureux",
                "joie",
                "cool",
                "génial",
                "super",
                "excellent",
                "fantastique",
            ]
        )
    ):
        positive_responses = {
            "friendly_robot": [
                "C'est vraiment formidable ! Je suis content que vous "
                "vous sentiez bien. Pourquoi cela vous rend-il heureux "
                "aujourd'hui ?",
                "Super nouvelle ! Continuez comme ça, vous allez très "
                "bien ! Racontez-moi ce qui vous motive, j'aimerais "
                "comprendre.",
                "C'est excellent ! Votre bonne humeur est contagieuse ! "
                "Comment aimeriez-vous explorer cette dynamique positive ?",
            ],
            "curious": [
                "Super ! Qu'est-ce qui vous rend si heureux ?",
                "Content de l'entendre ! Racontez-moi plus sur ce qui vous plaît !",
                "C'est bien ! J'aimerais en savoir plus sur ce qui vous réjouit !",
            ],
            "enthusiastic": [
                "YES ! C'est génial ! Je partage votre enthousiasme !",
                "Formidable ! Votre joie m'energise aussi !",
                "Super ! C'est trop cool ! Continuons sur cette lancée !",
            ],
            "calm": [
                "C'est bien. Je ressens votre sérénité.",
                "Ravi de savoir que vous allez bien.",
                "C'est une belle nouvelle. Profitez de ce moment.",
            ],
        }
        variants = positive_responses.get(
            state.bbia_personality,
            positive_responses["friendly_robot"],
        )
        return cast(
            str,
            normalize_response_length(
                random.choice(variants),  # nosec B311 - Variété réponse non-crypto
                state=state,
            ),
        )

    # Négatif - Réponses empathiques
    if (
        sentiment_type == "NEGATIVE"
        or sentiment_score < 0.3
        or any(
            word in message_lower
            for word in [
                "triste",
                "mal",
                "déçu",
                "problème",
                "difficile",
                "stressé",
            ]
        )
    ):
        negative_responses = {
            "friendly_robot": [
                "Je comprends que vous ne vous sentiez pas bien. "
                "Je suis là pour vous écouter.",
                "C'est difficile parfois. Voulez-vous en parler ? Je vous écoute.",
                "Je ressens votre malaise. Comment puis-je vous aider "
                "à vous sentir mieux ?",
            ],
            "curious": [
                "Qu'est-ce qui vous préoccupe ? J'aimerais comprendre "
                "pour mieux vous aider.",
                "Votre message reflète de la tristesse. Partagez-moi "
                "ce qui vous tracasse.",
                "Qu'est-ce qui cause cette difficulté ? Je veux vous aider.",
            ],
            "enthusiastic": [
                "Courage ! Même dans les moments difficiles, "
                "on peut trouver des raisons d'espérer !",  # noqa: E501
                "Je comprends que c'est dur, "
                "mais vous êtes capable de surmonter ça !",  # noqa: E501
                "On va s'en sortir ! Parlez-moi de ce qui ne va pas, "
                "on va trouver une solution !",  # noqa: E501
            ],
            "calm": [
                "Prenez votre temps. Je suis là, sans jugement.",
                "Respirez. Tout va s'arranger. Je vous écoute.",
                "Je comprends. Parlez-moi de ce qui vous trouble, à votre rythme.",
            ],
        }
        variants = negative_responses.get(
            state.bbia_personality,
            negative_responses["friendly_robot"],
        )
        return cast(
            str,
            normalize_response_length(
                random.choice(variants),  # nosec B311 - Variété réponse non-crypto
                state=state,
            ),
        )

    # Questions - Réponses adaptées selon type de question
    # AMÉLIORATION INTELLIGENCE: Détection type question pour réponses pertinentes
    if message_lower.count("?") > 0 or any(
        word in message_lower
        for word in ["qui", "quoi", "comment", "pourquoi", "où", "quand", "combien"]
    ):
        # Détection type de question pour réponses plus intelligentes
        question_responses: dict[str, list[str]] = {
            "friendly_robot": [
                "Bonne question ! Laissez-moi réfléchir... "
                "Comment puis-je vous aider ?",
                "Je comprends votre interrogation. Pouvez-vous me donner plus de "
                "détails pour que je puisse mieux vous répondre ?",
                "Intéressant ! Cette question mérite réflexion. "
                "Qu'est-ce que vous en pensez vous-même ?",
                "Ah, excellente question ! C'est quoi qui vous intrigue " "là-dedans ?",
                "Hmm, intéressant. Dites-moi plus sur ce qui vous pousse à vous "
                "poser cette question.",
                "Ça m'intrigue aussi ! Qu'est-ce qui vous amène à vous "
                "demander ça ?",
                "Très bonne question ! Qu'est-ce qui a provoqué cette curiosité "
                "chez vous ?",
                "Excellente question ! J'aimerais bien comprendre ce qui motive "
                "votre questionnement.",
                "Hmm, c'est une question qui mérite qu'on s'y attarde. "
                "Qu'est-ce qui vous a poussé à la formuler ?",
                "Intéressant angle d'approche ! Racontez-moi le contexte "
                "autour de cette question.",
            ],
            "curious": [
                "Ah, j'aime cette question ! Qu'est-ce qui vous amène à vous "
                "demander ça ?",
                "Fascinant ! Pourquoi cette question vous préoccupe-t-elle ?",
                "Excellente question ! J'aimerais explorer ça ensemble avec vous.",
            ],
            "enthusiastic": [
                "Super question ! Je suis tout excité de réfléchir à ça " "avec vous !",
                "Génial ! Cette question pique ma curiosité ! Partons explorer !",
                "Wow ! Quelle question intéressante ! Analysons ça ensemble !",
            ],
            "calm": [
                "Bonne question. Prenons le temps d'y réfléchir calmement.",
                "Intéressant. Que ressentez-vous face à cette question ?",
                "Je comprends. Explorons ça ensemble, sans précipitation.",
            ],
        }
        variants = question_responses.get(
            state.bbia_personality,
            question_responses["friendly_robot"],
        )
        return cast(
            str,
            normalize_response_length(
                random.choice(variants),  # nosec B311 - Variété réponse non-crypto
                state=state,
            ),
        )

    # Référence au contexte précédent si disponible
    # AMÉLIORATION INTELLIGENCE: Utilisation du contexte pour cohérence
    # conversationnelle
    if recent_context:
        # Vérifier si le message actuel fait référence au contexte précédent
        # (références: ça, ce, ce truc, etc.)
        reference_words = [
            "ça",
            "ce",
            "cette",
            "ce truc",
            "cette chose",
            "là",
            "cela",
        ]
        has_reference = any(ref in message_lower for ref in reference_words)

        if (
            has_reference
            or random.random() < 0.4  # nosec B311 - Variété réponse non-crypto
        ):  # 40% de chance si référence, sinon 30%
            context_responses = {
                "friendly_robot": [
                    f"Ah, vous parlez de {recent_context.lower()} ? "
                    "C'est intéressant ! Continuons sur ce sujet.",
                    f"En lien avec {recent_context.lower()}, j'aimerais en "
                    "savoir plus. "
                    "Qu'est-ce qui vous préoccupe là-dessus ?",
                    f"Vous mentionnez {recent_context.lower()}. Ça m'intrigue. "
                    "Dites-moi en plus si vous voulez.",
                    f"Je vois le lien avec {recent_context.lower()}. "
                    "C'est fascinant ! Racontez-moi davantage.",
                    f"En continuant sur {recent_context.lower()}, "
                    "qu'est-ce qui vous passionne le plus ?",
                ],
                "curious": [
                    f"Ah oui, {recent_context.lower()} ! "
                    "C'est exactement ce qui m'intéresse !",
                    f"En rapport avec {recent_context.lower()}, j'ai plein de "
                    "questions !",
                    f"{recent_context.lower()} me passionne ! "
                    "Continuons à explorer ça ensemble.",
                ],
                "enthusiastic": [
                    f"C'est génial, {recent_context.lower()} ! "
                    "Continuons à creuser ça !",
                    f"Super, {recent_context.lower()} ! C'est trop intéressant !",
                    f"{recent_context.lower()} ? Wow, allons plus loin là-dessus !",
                ],
                "calm": [
                    f"Je comprends le lien avec {recent_context.lower()}. "
                    "Explorons ça sereinement.",
                    f"En lien avec {recent_context.lower()}, prenons le temps "
                    "d'y réfléchir.",
                    f"Je vois votre réflexion sur {recent_context.lower()}. "
                    "Continuons calmement.",
                ],
            }
            variants = context_responses.get(
                state.bbia_personality,
                context_responses["friendly_robot"],
            )
            return cast(
                str,
                normalize_response_length(
                    random.choice(variants),  # nosec B311 - Variété réponse non-crypto
                    state=state,
                ),
            )

    # Réponses génériques variées selon personnalité et sentiment
    # AMÉLIORATION INTELLIGENCE: Réponses naturelles, engageantes, moins robotiques
    # Enrichi avec 15 variantes pour friendly_robot pour éviter répétition
    generic_responses = {
        "friendly_robot": [
            "Intéressant ! J'aimerais en savoir plus sur votre point de vue. "
            "Qu'est-ce qui vous a amené à penser ça ?",
            "Je vois ce que vous voulez dire. Racontez-moi pourquoi vous "
            "pensez ainsi, "
            "je vous écoute attentivement.",
            "Merci de partager ça avec moi. Qu'est-ce qui vous intéresse le plus "
            "dans tout ça ?",
            "Hmm, c'est captivant. Vous pouvez m'en dire plus si vous voulez, "
            "je suis curieux.",
            "Ah d'accord, je comprends. Explorons ça ensemble si ça vous dit, "
            "j'adorerais en discuter.",
            "J'ai noté. Dites-moi tout ce qui vous vient à l'esprit, sans filtre.",
            "Ça m'intrigue ! Racontez-moi davantage, j'aime apprendre de vous.",
            "C'est fascinant. Qu'est-ce qui vous a amené à penser ça ? "
            "Je suis vraiment curieux.",
            "Wow, ça sonne intéressant. Comment voulez-vous développer ? "
            "J'aimerais mieux comprendre.",
            "C'est noté. Qu'est-ce qui vous pousse à réfléchir ainsi ?",
            "Ah, c'est un point de vue intéressant. "
            "Qu'est-ce qui vous fait penser ainsi ?",
            "Je comprends votre perspective. Pourquoi avez-vous cette vision ? "
            "J'aimerais approfondir.",
            "C'est une réflexion qui pique ma curiosité. D'où vient cette idée ?",
            "Hmm, vous m'intriguez ! Comment avez-vous développé cette pensée ?",
            "Ça me fait réfléchir. Qu'est-ce qui vous a conduit là-dessus ?",
        ],
        "curious": [
            "Hmm, intrigant ! Qu'est-ce qui vous a amené à dire ça ?",
            "Fascinant ! J'aimerais creuser cette idée avec vous.",
            "Intéressant ! Que cache cette réflexion ?",
        ],
        "enthusiastic": [
            "Cool ! C'est super intéressant ! Continuons à explorer !",
            "Génial ! J'adore découvrir de nouvelles choses avec vous !",
            "Wow ! C'est passionnant ! Allons plus loin !",
        ],
        "calm": [
            "Je comprends. Pourquoi avez-vous cette réflexion ? "
            "Explorons cela ensemble.",
            "Intéressant. Comment avez-vous développé cette idée ? "
            "Continuons cette conversation sereinement.",
            "Je vois. Qu'est-ce qui vous amène à penser ainsi ? "
            "Partagez-moi vos pensées, sans précipitation.",
        ],
    }
    variants = generic_responses.get(
        state.bbia_personality,
        generic_responses["friendly_robot"],
    )
    return cast(
        str,
        normalize_response_length(
            random.choice(variants),  # nosec B311 - Variété réponse non-crypto
            state=state,
        ),
    )


def adapt_response_to_personality(
    response: str,
    sentiment: SentimentDict,  # noqa: ARG002
    state: PersonalityResponseState,
) -> str:
    """Adapte la réponse selon la personnalité BBIA avec nuances expressives.

    Args:
        response: Réponse de base
        sentiment: Analyse sentiment

    Returns:
        Réponse adaptée avec emoji et nuances selon personnalité

    """
    # Les réponses sont déjà adaptées dans generate_simple_response,
    # on ajoute juste l'emoji ici pour cohérence
    personality_emojis = {
        "friendly_robot": "🤖",
        "curious": "🤔",
        "enthusiastic": "🎉",
        "calm": "😌",
    }
    emoji = personality_emojis.get(state.bbia_personality, "💬")

    # Si la réponse contient déjà un emoji, ne pas en ajouter (éviter doublon)
    if response.startswith(emoji) or any(char in emoji for char in response[:3]):
        return response

    return f"{emoji} {response}"
