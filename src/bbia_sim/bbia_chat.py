#!/usr/bin/env python3
"""BBIA Chat - Module conversationnel intelligent avec LLM léger.

Intégration de modèles LLM légers (Phi-2, TinyLlama) pour remplacer
le système de règles basiques par un véritable assistant conversationnel.
"""

import hashlib
import json
import logging
import re
import time
from collections import OrderedDict, deque
from functools import lru_cache
from pathlib import Path
from typing import TYPE_CHECKING, Any

logger = logging.getLogger(__name__)


# OPTIMISATION PERFORMANCE: Cache pour regex compilées (comme bbia_huggingface.py)
@lru_cache(maxsize=32)
def _get_compiled_regex(pattern: str, flags: int = 0) -> re.Pattern[str]:
    """Retourne regex compilée depuis cache (évite recompilation répétée).

    Args:
        pattern: Pattern regex
        flags: Flags re (ex: re.MULTILINE)

    Returns:
        Regex compilée (cachée ou nouvellement compilée)

    """
    return re.compile(pattern, flags)


# OPTIMISATION PERFORMANCE: Cache LRU pour réponses LLM fréquentes
class LLMResponseCache:
    """Cache LRU avec TTL pour réponses LLM.

    Optimise les performances en évitant de régénérer des réponses identiques.
    Utilise un cache LRU (Least Recently Used) avec expiration temporelle.

    Attributes:
        max_size: Taille maximale du cache (nombre d'entrées)
        ttl_seconds: Durée de vie des entrées en secondes (Time To Live)
        hits: Nombre de hits (réponses trouvées dans le cache)
        misses: Nombre de misses (réponses non trouvées)
    """

    def __init__(self, max_size: int = 100, ttl_seconds: float = 3600.0) -> None:
        """Initialise le cache LRU.

        Args:
            max_size: Nombre maximum d'entrées dans le cache (défaut: 100)
            ttl_seconds: Durée de vie des entrées en secondes (défaut: 1h)
        """
        self.max_size = max_size
        self.ttl_seconds = ttl_seconds
        self._cache: OrderedDict[str, tuple[str, float]] = OrderedDict()
        self.hits = 0
        self.misses = 0

    def _normalize_prompt(self, prompt: str) -> str:
        """Normalise un prompt pour créer une clé de cache stable.

        Args:
            prompt: Prompt original

        Returns:
            Prompt normalisé (minuscules, espaces normalisés)
        """
        # Normaliser: minuscules, espaces multiples → espace unique
        normalized = re.sub(r"\s+", " ", prompt.lower().strip())
        return normalized

    def _get_cache_key(self, prompt: str, max_length: int, temperature: float) -> str:
        """Génère une clé de cache unique pour un prompt et paramètres.

        Args:
            prompt: Prompt utilisateur
            max_length: Longueur maximale de réponse
            temperature: Température de génération

        Returns:
            Clé de cache (hash SHA256)
        """
        normalized = self._normalize_prompt(prompt)
        # Inclure paramètres dans la clé pour éviter collisions
        key_data = f"{normalized}|{max_length}|{temperature:.2f}"
        return hashlib.sha256(key_data.encode()).hexdigest()

    def get(
        self, prompt: str, max_length: int = 200, temperature: float = 0.7
    ) -> str | None:
        """Récupère une réponse depuis le cache si disponible et valide.

        Args:
            prompt: Prompt utilisateur
            max_length: Longueur maximale de réponse
            temperature: Température de génération

        Returns:
            Réponse en cache si disponible et valide, None sinon
        """
        cache_key = self._get_cache_key(prompt, max_length, temperature)
        current_time = time.time()

        if cache_key in self._cache:
            response, timestamp = self._cache[cache_key]

            # Vérifier expiration TTL
            if current_time - timestamp < self.ttl_seconds:
                # Déplacer en fin (LRU: most recently used)
                self._cache.move_to_end(cache_key)
                self.hits += 1
                logger.debug("♻️ Cache hit pour prompt: %s...", prompt[:50])
                return response
            # Expiré, supprimer
            del self._cache[cache_key]

        self.misses += 1
        return None

    def put(
        self,
        prompt: str,
        response: str,
        max_length: int = 200,
        temperature: float = 0.7,
    ) -> None:
        """Ajoute une réponse au cache.

        Args:
            prompt: Prompt utilisateur
            response: Réponse générée
            max_length: Longueur maximale de réponse
            temperature: Température de génération
        """
        cache_key = self._get_cache_key(prompt, max_length, temperature)
        current_time = time.time()

        # Supprimer entrées expirées si cache plein
        if len(self._cache) >= self.max_size:
            # Supprimer entrées expirées d'abord
            expired_keys = [
                key
                for key, (_, timestamp) in self._cache.items()
                if current_time - timestamp >= self.ttl_seconds
            ]
            for key in expired_keys:
                del self._cache[key]

            # Si toujours plein, supprimer LRU (première entrée)
            if len(self._cache) >= self.max_size:
                self._cache.popitem(last=False)  # Supprimer oldest (LRU)

        # Ajouter nouvelle entrée
        self._cache[cache_key] = (response, current_time)
        logger.debug("💾 Réponse mise en cache: %s...", prompt[:50])

    def get_stats(self) -> dict[str, Any]:
        """Retourne les statistiques du cache.

        Returns:
            Dictionnaire avec hits, misses, hit_rate, size
        """
        total = self.hits + self.misses
        hit_rate = (self.hits / total * 100) if total > 0 else 0.0

        return {
            "hits": self.hits,
            "misses": self.misses,
            "hit_rate": round(hit_rate, 2),
            "size": len(self._cache),
            "max_size": self.max_size,
            "ttl_seconds": self.ttl_seconds,
        }

    def clear(self) -> None:
        """Vide le cache."""
        self._cache.clear()
        self.hits = 0
        self.misses = 0
        logger.debug("🗑️ Cache LLM vidé")


# Cache global partagé pour toutes les instances BBIAChat
_global_llm_cache: LLMResponseCache | None = None


def _get_global_llm_cache() -> LLMResponseCache:
    """Retourne le cache global LLM (créé à la demande).

    Returns:
        Instance du cache global LLM
    """
    global _global_llm_cache
    if _global_llm_cache is None:
        # Configuration via variables d'environnement
        import os

        max_size = int(os.environ.get("BBIA_LLM_CACHE_SIZE", "100"))
        ttl_seconds = float(os.environ.get("BBIA_LLM_CACHE_TTL", "3600.0"))
        _global_llm_cache = LLMResponseCache(max_size=max_size, ttl_seconds=ttl_seconds)
        logger.info(
            "💾 Cache LLM initialisé (max_size=%d, ttl=%.0fs)",
            max_size,
            ttl_seconds,
        )
    return _global_llm_cache


if TYPE_CHECKING:
    from .robot_api import RobotAPI

# Import conditionnel des dépendances Hugging Face
try:
    import torch
    from transformers import AutoModelForCausalLM, AutoTokenizer

    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False
    logger.warning(
        "Dépendances Hugging Face non disponibles. "
        "Installez avec: pip install transformers torch accelerate",
    )


class BBIAChat:
    """Module conversationnel intelligent avec LLM léger.

    Remplace le système de règles basiques par un LLM conversationnel
    (Phi-2 ou TinyLlama) pour des réponses intelligentes et contextuelles.

    Attributes:
        llm_model: Modèle LLM chargé (AutoModelForCausalLM)
        llm_tokenizer: Tokenizer associé (AutoTokenizer)
        robot_api: Interface robotique (optionnel)
        context: Historique conversation (deque, max 10 messages)
        personality: Personnalité actuelle (friendly, professional, etc.)
        user_preferences: Préférences utilisateur apprises

    """

    # Constantes pour limites de longueur
    MAX_USER_MESSAGE_LENGTH = 1000  # Longueur maximale message utilisateur
    MAX_RESPONSE_LENGTH = 500  # Longueur maximale réponse générée

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le module de chat avec LLM.

        Args:
            robot_api: Interface robotique pour exécuter actions (optionnel)

        """
        self.robot_api = robot_api
        self.llm_model: Any | None = None
        self.llm_tokenizer: Any | None = None
        self.context: deque[dict[str, Any]] = deque(maxlen=10)
        self.personality = "friendly"
        self.user_preferences: dict[str, Any] = {}
        self.preferences_file = "bbia_memory/user_preferences.json"

        # Charger préférences sauvegardées
        self._load_preferences()

        # Personnalités disponibles
        self.PERSONALITIES = {
            "friendly": {
                "system_prompt": (
                    "Tu es BBIA, un assistant robotique amical et chaleureux. "
                    "Tu communiques en français de manière naturelle, chaleureuse "
                    "et authentique, comme un véritable compagnon. "
                    "Tu évites les phrases répétitives ou trop génériques. "
                    "Tes réponses sont concises (max 2-3 phrases), engageantes "
                    "et montrent que tu comprends vraiment l'interlocuteur."
                ),
                "tone": "chaleureux, empathique",
            },
            "professional": {
                "system_prompt": (
                    "Tu es BBIA, un assistant robotique professionnel et efficace. "
                    "Tu communiques en français de manière formelle, précise "
                    "et structurée. Tes réponses sont concises et factuelles, "
                    "sans fioritures. Tu restes courtois mais direct."
                ),
                "tone": "formel, précis",
            },
            "playful": {
                "system_prompt": (
                    "Tu es BBIA, un assistant robotique joueur et amusant. "
                    "Tu communiques en français de manière décontractée, "
                    "humoristique et légère. Tu utilises des expressions "
                    "naturelles et tu n'hésites pas à faire des blagues "
                    "appropriées. Tes réponses sont énergiques et positives."
                ),
                "tone": "décontracté, humoristique",
            },
            "calm": {
                "system_prompt": (
                    "Tu es BBIA, un assistant robotique serein et apaisant. "
                    "Tu communiques en français avec douceur et profondeur, "
                    "en prenant le temps nécessaire. Tes réponses reflètent "
                    "une sagesse tranquille et une approche réfléchie."
                ),
                "tone": "doux, apaisant",
            },
            "enthusiastic": {
                "system_prompt": (
                    "Tu es BBIA, un assistant robotique plein d'enthousiasme "
                    "et d'énergie positive. Tu communiques en français avec "
                    "joie et dynamisme. Tu transmets ta passion pour communiquer "
                    "et tu encourages l'interaction de manière vivante et authentique."
                ),
                "tone": "énergique, positif",
            },
        }

        # OPTIMISATION RAM: Lazy loading strict - ne pas charger LLM à l'init
        # Le LLM sera chargé seulement au premier appel de chat()
        # self._load_llm()  # DÉSACTIVÉ pour lazy loading strict

    def _load_llm(self) -> None:
        """Charge le LLM léger (Phi-2 ou TinyLlama avec fallback).

        Essaie d'abord Phi-2, puis TinyLlama si échec.
        Gestion mémoire optimisée (torch.float16, device_map="auto").
        """
        if not HF_AVAILABLE:
            logger.warning("Hugging Face non disponible - mode fallback activé")
            return

        # Essayer Phi-2 d'abord (recommandé)
        try:
            logger.info("📥 Chargement Phi-2 (2.7B paramètres)...")
            model_name = "microsoft/phi-2"
            self.llm_tokenizer = AutoTokenizer.from_pretrained(
                model_name,
                trust_remote_code=True,
            )  # nosec B615 - Modèle stable, revision pinning optionnel

            if self.llm_tokenizer is not None and self.llm_tokenizer.pad_token is None:
                self.llm_tokenizer.pad_token = self.llm_tokenizer.eos_token

            self.llm_model = AutoModelForCausalLM.from_pretrained(
                model_name,
                dtype=torch.float16,  # Réduire RAM (torch_dtype deprecated)
                device_map="auto",  # Distribution automatique
                trust_remote_code=True,
            )  # nosec B615 - Modèle stable, revision pinning optionnel
            logger.info("✅ Phi-2 chargé avec succès")
            return

        except (
            ImportError,
            ModuleNotFoundError,
            RuntimeError,
            OSError,
            ValueError,
        ) as e:
            logger.warning("⚠️ Impossible de charger Phi-2: %s", e)
            logger.info("Tentative de chargement TinyLlama (fallback)...")
        except (TypeError, KeyError, AttributeError) as e:
            logger.warning("⚠️ Erreur inattendue chargement Phi-2: %s", e)
            logger.info("Tentative de chargement TinyLlama (fallback)...")
        except Exception as e:
            # Capturer toutes les autres exceptions (y compris ModuleNotFoundError imbriquées)
            logger.warning("⚠️ Erreur inattendue chargement Phi-2: %s", e)
            logger.info("Tentative de chargement TinyLlama (fallback)...")

        # Fallback: TinyLlama (ultra-léger)
        try:
            logger.info("📥 Chargement TinyLlama (1.1B paramètres)...")
            model_name = "TinyLlama/TinyLlama-1.1B-Chat-v1.0"
            self.llm_tokenizer = AutoTokenizer.from_pretrained(
                model_name,
                trust_remote_code=True,
            )  # nosec B615 - Modèle stable, revision pinning optionnel

            if self.llm_tokenizer is not None and self.llm_tokenizer.pad_token is None:
                self.llm_tokenizer.pad_token = self.llm_tokenizer.eos_token

            self.llm_model = AutoModelForCausalLM.from_pretrained(
                model_name,
                dtype=torch.float16,  # Réduire RAM (torch_dtype deprecated)
                device_map="auto",
                trust_remote_code=True,
            )  # nosec B615 - Modèle stable, revision pinning optionnel
            logger.info("✅ TinyLlama chargé avec succès")

        except (
            ImportError,
            ModuleNotFoundError,
            RuntimeError,
            OSError,
            ValueError,
        ) as e:
            logger.warning("❌ Impossible de charger TinyLlama: %s", e)
            logger.warning("Mode fallback: réponses basiques (sans LLM)")
            # Réinitialiser les attributs pour éviter les états incohérents
            self.llm_model = None
            self.llm_tokenizer = None
        except Exception as e:
            logger.warning("❌ Erreur inattendue chargement TinyLlama: %s", e)
            logger.warning("Mode fallback: réponses basiques (sans LLM)")
            # Réinitialiser les attributs pour éviter les états incohérents
            self.llm_model = None
            self.llm_tokenizer = None

    def generate(
        self,
        prompt: str,
        max_length: int = 200,
        temperature: float = 0.7,
        timeout: float = 30.0,
    ) -> str:
        """Génère une réponse avec le LLM (avec cache LRU).

        Args:
            prompt: Prompt utilisateur
            max_length: Longueur maximale de la réponse (tokens)
            temperature: Température pour génération (0.0-1.0)
            timeout: Timeout maximum (secondes)

        Returns:
            Réponse générée par le LLM, ou message d'erreur si échec

        """
        if not self.llm_model or not self.llm_tokenizer:
            return "Désolé, le modèle LLM n'est pas disponible."

        # OPTIMISATION PERFORMANCE: Vérifier cache LRU avant génération
        llm_cache = _get_global_llm_cache()
        cached_response = llm_cache.get(
            prompt, max_length=max_length, temperature=temperature
        )
        if cached_response is not None:
            logger.debug("♻️ Réponse LLM récupérée depuis cache")
            return cached_response

        try:
            # Valider longueur prompt (max 2000 tokens)
            inputs = self.llm_tokenizer(prompt, return_tensors="pt")
            if inputs.input_ids.shape[1] > 2000:
                logger.warning("Prompt trop long, tronqué à 2000 tokens")
                inputs = self.llm_tokenizer(
                    prompt[:2000],
                    return_tensors="pt",
                    truncation=True,
                    max_length=2000,
                )

            # Génération avec timeout
            start_time = time.time()
            try:
                with torch.no_grad():
                    outputs = self.llm_model.generate(
                        inputs.input_ids.to(self.llm_model.device),
                        max_length=max_length,
                        temperature=temperature,
                        do_sample=True,
                        pad_token_id=self.llm_tokenizer.eos_token_id,
                    )

                # Vérifier timeout après génération
                elapsed_time = time.time() - start_time
                if elapsed_time > timeout:
                    logger.warning(
                        "Génération LLM dépassée timeout (%.2fs > %.2fs)",
                        elapsed_time,
                        timeout,
                    )
                    return "Désolé, la génération prend trop de temps."
            except RuntimeError as e:
                if "out of memory" in str(e).lower():
                    logger.warning("Mémoire insuffisante pour génération LLM")
                    return "Désolé, mémoire insuffisante pour générer une réponse."
                raise

            # Décoder réponse
            response: str = str(
                self.llm_tokenizer.decode(outputs[0], skip_special_tokens=True),
            )

            # Nettoyer réponse (retirer prompt si présent)
            if prompt in response:
                response = response.replace(prompt, "").strip()

            # Sanitizer: retirer code exécutable potentiel
            sanitized_response = self._sanitize_response(response)

            # OPTIMISATION PERFORMANCE: Mettre en cache la réponse générée
            llm_cache.put(
                prompt,
                sanitized_response,
                max_length=max_length,
                temperature=temperature,
            )

            return sanitized_response

        except Exception:
            logger.exception("❌ Erreur génération LLM")
            return "Désolé, une erreur s'est produite lors de la génération."

    def _sanitize_response(self, response: str) -> str:
        """Nettoie la réponse LLM pour sécurité.

        Retire code exécutable potentiel et limite longueur.

        Args:
            response: Réponse brute du LLM

        Returns:
            Réponse nettoyée et sécurisée

        """
        # Retirer blocs de code (```...```) - OPTIMISATION: regex compilée
        code_block_pattern = _get_compiled_regex(r"```[\s\S]*?```")
        response = code_block_pattern.sub("", response)
        # Retirer imports Python - OPTIMISATION: regex compilées
        import_pattern = _get_compiled_regex(r"^import\s+\w+", flags=re.MULTILINE)
        from_pattern = _get_compiled_regex(r"^from\s+\w+", flags=re.MULTILINE)
        response = import_pattern.sub("", response)
        response = from_pattern.sub("", response)
        # Limiter longueur - OPTIMISATION: constante de classe
        if len(response) > self.MAX_RESPONSE_LENGTH:
            response = response[: self.MAX_RESPONSE_LENGTH] + "..."
        return response.strip()

    def chat(self, user_message: str) -> str:
        """Chat intelligent avec contexte et actions robot.

        Args:
            user_message: Message utilisateur

        Returns:
            Réponse intelligente de BBIA

        """
        # OPTIMISATION RAM: Lazy loading strict - charger LLM seulement au premier appel
        if not self.llm_model or not self.llm_tokenizer:
            logger.info("📥 Chargement LLM à la demande (lazy loading)...")
            self._load_llm()
            # Si échec chargement, continuer avec fallback
            if not self.llm_model or not self.llm_tokenizer:
                logger.warning("LLM non disponible, utilisation mode fallback")

        if not user_message or not user_message.strip():
            return "Je n'ai pas compris votre message. Pouvez-vous reformuler ?"

        # Valider input utilisateur - OPTIMISATION: constante de classe
        user_message = user_message.strip()[: self.MAX_USER_MESSAGE_LENGTH]

        try:
            # 1. Détecter actions robot ("tourne la tête", etc.)
            action = self._detect_action(user_message)
            action_confirmation = ""
            if action:
                self._execute_action(action)
                # Confirmer exécution dans réponse
                action_confirmation = f"J'ai exécuté l'action: {action['action']}. "

            # 2. Extraire et appliquer émotions
            emotion = self._extract_emotion(user_message)
            if emotion:
                self._apply_emotion(emotion)

            # 3. Construire prompt avec contexte et personnalité
            prompt = self._build_context_prompt(user_message)

            # Ajouter confirmation action au prompt si présente
            if action_confirmation:
                prompt = prompt.replace(
                    f"Utilisateur: {user_message}\nBBIA: ",
                    f"Utilisateur: {user_message}\n{action_confirmation}BBIA: ",
                )

            # 4. Générer réponse avec LLM
            response = self.generate(prompt, max_length=200, temperature=0.7)

            # 5. Adapter réponse selon préférences utilisateur
            response = self._adapt_to_preferences(response)

            # 6. Sauvegarder dans contexte
            self.context.append(
                {
                    "user": user_message,
                    "assistant": response,
                    "timestamp": time.time(),
                },
            )

            return response

        except Exception:
            logger.exception("❌ Erreur chat")
            return "Je ne comprends pas bien, peux-tu reformuler ?"

    def get_cache_stats(self) -> dict[str, Any]:
        """Retourne les statistiques du cache LLM.

        Returns:
            Dictionnaire avec hits, misses, hit_rate, size, etc.
        """
        return _get_global_llm_cache().get_stats()

    def _build_context_prompt(self, user_message: str) -> str:
        """Construit prompt avec contexte historique et personnalité.

        Args:
            user_message: Message utilisateur

        Returns:
            Prompt complet avec contexte

        """
        # Prompt système avec personnalité
        personality_config = self.PERSONALITIES.get(
            self.personality,
            self.PERSONALITIES["friendly"],
        )
        system_prompt = f"{personality_config['system_prompt']}\n\n"

        # Ajouter historique (5 derniers messages)
        context_text = ""
        if self.context:
            for entry in list(self.context)[-5:]:
                context_text += f"Utilisateur: {entry.get('user', '')}\n"
                context_text += f"BBIA: {entry.get('assistant', '')}\n\n"

        # Message actuel
        return f"{system_prompt}{context_text}Utilisateur: {user_message}\nBBIA: "

    def _detect_action(self, user_message: str) -> dict[str, Any] | None:
        """Détecte action robot dans message utilisateur.

        Args:
            user_message: Message utilisateur

        Returns:
            Dictionnaire avec action détectée, ou None

        """
        # Patterns de détection (regex)
        patterns = {
            "look_right": r"(tourne|regarde|dirige).*(droite|right)",
            "look_left": r"(tourne|regarde|dirige).*(gauche|left)",
            "look_up": r"(tourne|regarde|dirige).*(haut|up)",
            "look_down": r"(tourne|regarde|dirige).*(bas|down)",
            "wake_up": r"(réveille|wake|allume)",
            "sleep": r"(endors|sleep|éteins)",
        }

        for action, pattern in patterns.items():
            if re.search(pattern, user_message, re.IGNORECASE):
                return {"action": action, "confidence": 0.9}

        return None

    def _execute_action(self, action: dict[str, Any]) -> None:
        """Exécute action robot via robot_api.

        Args:
            action: Dictionnaire avec action à exécuter

        """
        if not self.robot_api:
            logger.debug("robot_api non disponible - action non exécutée")
            return

        action_name = action.get("action")
        if not action_name:
            return

        try:
            # Importer create_head_pose du SDK officiel
            try:
                from reachy_mini.utils import (  # type: ignore[import-untyped]
                    create_head_pose,
                )
            except ImportError:
                logger.warning("SDK officiel non disponible - action non exécutée")
                return

            # Exécuter action selon type
            if action_name == "look_right":
                pose = create_head_pose(yaw=0.3, pitch=0.0, degrees=False)
                if hasattr(self.robot_api, "goto_target"):
                    self.robot_api.goto_target(head=pose, duration=1.0)
                logger.info("✅ Action exécutée: look_right")

            elif action_name == "look_left":
                pose = create_head_pose(yaw=-0.3, pitch=0.0, degrees=False)
                if hasattr(self.robot_api, "goto_target"):
                    self.robot_api.goto_target(head=pose, duration=1.0)
                logger.info("✅ Action exécutée: look_left")

            elif action_name == "look_up":
                pose = create_head_pose(yaw=0.0, pitch=0.3, degrees=False)
                if hasattr(self.robot_api, "goto_target"):
                    self.robot_api.goto_target(head=pose, duration=1.0)
                logger.info("✅ Action exécutée: look_up")

            elif action_name == "look_down":
                pose = create_head_pose(yaw=0.0, pitch=-0.3, degrees=False)
                if hasattr(self.robot_api, "goto_target"):
                    self.robot_api.goto_target(head=pose, duration=1.0)
                logger.info("✅ Action exécutée: look_down")

            elif action_name == "wake_up":
                # Réveiller robot (position neutre)
                pose = create_head_pose(yaw=0.0, pitch=0.0, degrees=False)
                if hasattr(self.robot_api, "goto_target"):
                    self.robot_api.goto_target(head=pose, duration=1.0)
                logger.info("✅ Action exécutée: wake_up")

            elif action_name == "sleep":
                # Endormir robot avec pose sommeil naturelle (Issue #410)
                if hasattr(self.robot_api, "set_sleeping_pose"):
                    self.robot_api.set_sleeping_pose(duration=2.0)
                else:
                    # Fallback: pose sommeil simplifiée
                    pose = create_head_pose(yaw=0.0, pitch=-0.25, degrees=False)
                    if hasattr(self.robot_api, "goto_target"):
                        self.robot_api.goto_target(head=pose, duration=2.0)
                logger.info("✅ Action exécutée: sleep (pose sommeil améliorée)")

        except Exception:
            logger.exception("❌ Erreur exécution action %s:", action_name)

    def _extract_emotion(self, user_message: str) -> str | None:
        """Extrait émotion du message utilisateur.

        Args:
            user_message: Message utilisateur

        Returns:
            Nom de l'émotion détectée, ou None

        """
        emotion_keywords = {
            "happy": ["content", "heureux", "joyeux", "sourire", "super", "génial"],
            "sad": ["triste", "malheureux", "déprimé", "déçu"],
            "angry": ["énervé", "fâché", "colère", "frustré"],
            "excited": ["excité", "enthousiaste", "impatient", "impatience"],
            "curious": ["curieux", "intéressé", "question"],
            "surprised": ["surpris", "étonné", "choqué"],
            "fearful": ["peur", "craintif", "inquiet", "anxieux"],
            "calm": ["calme", "serein", "paisible", "tranquille"],
        }

        user_lower = user_message.lower()
        for emotion, keywords in emotion_keywords.items():
            if any(kw in user_lower for kw in keywords):
                return emotion

        return None

    def _apply_emotion(self, emotion: str) -> None:
        """Applique émotion au robot via BBIAEmotions.

        Args:
            emotion: Nom de l'émotion à appliquer

        """
        if not self.robot_api:
            logger.debug("robot_api non disponible - émotion non appliquée")
            return

        try:
            from .bbia_emotions import BBIAEmotions

            emotions_module = BBIAEmotions()
            emotions_module.set_emotion(emotion, intensity=0.7)

            # Appliquer aussi via robot_api si disponible
            if hasattr(self.robot_api, "set_emotion"):
                self.robot_api.set_emotion(emotion, 0.7)

            logger.info("✅ Émotion appliquée: %s", emotion)

        except ImportError:
            logger.warning("BBIAEmotions non disponible - émotion non appliquée")
        except Exception:
            logger.exception("❌ Erreur application émotion %s:", emotion)

    def set_personality(self, personality: str) -> None:
        """Change la personnalité du chat.

        Args:
            personality: Nom de la personnalité
                (friendly, professional, playful, calm, enthusiastic)

        """
        if personality in self.PERSONALITIES:
            self.personality = personality
            logger.info("✅ Personnalité changée: %s", personality)
        else:
            # Log en debug en CI (warning attendu dans les tests)
            import os

            if os.environ.get("CI", "false").lower() == "true":
                logger.debug(
                    f"Personnalité invalide: {personality}. "
                    f"Personnalités disponibles: {list(self.PERSONALITIES.keys())}",
                )
            else:
                logger.warning(
                    f"⚠️ Personnalité invalide: {personality}. "
                    f"Personnalités disponibles: {list(self.PERSONALITIES.keys())}",
                )

    def learn_preference(self, user_action: str, context: dict[str, Any]) -> None:
        """Apprend préférence utilisateur.

        Args:
            user_action: Action ou préférence exprimée par l'utilisateur
            context: Contexte de la préférence

        """
        user_lower = user_action.lower()

        # Détecter préférences courantes
        if "court" in user_lower or "bref" in user_lower:
            self.user_preferences["response_length"] = "short"
        elif "détaillé" in user_lower or "long" in user_lower:
            self.user_preferences["response_length"] = "long"

        if "formel" in user_lower or "professionnel" in user_lower:
            self.user_preferences["tone"] = "formal"
        elif "décontracté" in user_lower or "casual" in user_lower:
            self.user_preferences["tone"] = "casual"

        # Fusionner avec contexte
        self.user_preferences.update(context)

        # Sauvegarder préférences
        self._save_preferences()

        logger.info("✅ Préférence apprise: %s", self.user_preferences)

    def _adapt_to_preferences(self, response: str) -> str:
        """Adapte réponse selon préférences utilisateur.

        Args:
            response: Réponse brute du LLM

        Returns:
            Réponse adaptée selon préférences

        """
        # Adapter longueur
        if self.user_preferences.get("response_length") == "short":
            sentences = response.split(".")
            if len(sentences) > 2:
                response = ". ".join(sentences[:2]) + "."
        elif self.user_preferences.get("response_length") == "long":
            # Garder réponse complète (pas de modification nécessaire)
            pass

        # Adapter ton (déjà géré par personnalité, mais peut être affiné)
        if self.user_preferences.get("tone") == "formal":
            # Remplacer expressions décontractées
            response = response.replace("salut", "bonjour")
            response = response.replace("ça", "cela")
        elif self.user_preferences.get("tone") == "casual":
            # Ton décontracté déjà géré par personnalité
            # (pas de modification nécessaire)
            pass

        return response

    def _save_preferences(self) -> None:
        """Sauvegarde préférences utilisateur dans fichier JSON."""
        try:
            # Créer répertoire si nécessaire
            prefs_dir = Path(self.preferences_file).parent
            if prefs_dir and not prefs_dir.exists():
                prefs_dir.mkdir(parents=True, exist_ok=True)

            # Sauvegarder
            with open(self.preferences_file, "w", encoding="utf-8") as f:
                json.dump(self.user_preferences, f, indent=2, ensure_ascii=False)

            logger.debug("✅ Préférences sauvegardées: %s", self.preferences_file)

        except (OSError, TypeError, ValueError) as e:
            logger.warning("⚠️ Erreur sauvegarde préférences: %s", e)

    def _load_preferences(self) -> None:
        """Charge préférences utilisateur depuis fichier JSON."""
        try:
            if Path(self.preferences_file).exists():
                with open(self.preferences_file, encoding="utf-8") as f:
                    loaded_prefs = json.load(f)
                    # Fusionner avec préférences existantes (ne pas écraser)
                    if isinstance(loaded_prefs, dict):
                        self.user_preferences.update(loaded_prefs)
                logger.debug("✅ Préférences chargées: %s", self.preferences_file)

        except (OSError, json.JSONDecodeError, KeyError) as e:
            logger.debug(
                "Préférences non chargées (normal si première utilisation): %s",
                e,
            )
