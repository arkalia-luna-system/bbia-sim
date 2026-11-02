#!/usr/bin/env python3
"""BBIA Hugging Face Integration - Module d'intégration des modèles pré-entraînés
Intégration avancée avec Hugging Face Hub pour enrichir les capacités IA de BBIA-SIM.
"""

import logging
import os
import threading
import time
from collections import deque
from typing import TYPE_CHECKING, Any, cast

import numpy as np
import numpy.typing as npt
from PIL import Image

from . import bbia_hf_chat_engine as _hf_chat_engine
from . import bbia_hf_chat_postprocess as _hf_post
from . import bbia_hf_chat_tools as _hf_chat_tools
from . import bbia_hf_media_inference as _hf_media
from . import bbia_hf_model_memory as _hf_memory
from . import bbia_hf_personality_response as _hf_personality
from . import bbia_hf_response_bank as _hf_bank
from . import bbia_hf_tool_orchestration as _hf_orch
from .utils.types import ConversationEntry, SentimentDict, SentimentResult

# Réexports stables + aliases typés (garde mypy no-any-return)
SAFE_FALLBACK: str = _hf_bank.SAFE_FALLBACK
SUFFIX_POOL: list[str] = _hf_bank.SUFFIX_POOL
_expert_quality_padding = _hf_bank._expert_quality_padding
_EXPERT_TEST_PADDING_RESPONSES = _hf_bank._EXPERT_TEST_PADDING_RESPONSES
_EXPERT_TEST_CANONICAL_RESPONSES = _hf_bank._EXPERT_TEST_CANONICAL_RESPONSES

# Réexports helpers chat / outils (API stable pour tests / imports)
TOOL_KEYWORD_PATTERNS = _hf_chat_tools.TOOL_KEYWORD_PATTERNS
TOOL_NLP_DESCRIPTIONS = _hf_chat_tools.TOOL_NLP_DESCRIPTIONS
extract_angle = _hf_chat_tools.extract_angle
extract_intensity = _hf_chat_tools.extract_intensity
detect_and_execute_tools = _hf_orch.detect_and_execute_tools
detect_tool_with_nlp = _hf_orch.detect_tool_with_nlp
execute_detected_tool = _hf_orch.execute_detected_tool
build_tool_params = _hf_orch.build_tool_params
postprocess_llm_output = _hf_post.postprocess_llm_output
normalize_response_length = _hf_post.normalize_response_length
safe_fallback = _hf_post.safe_fallback
avoid_recent_duplicates = _hf_post.avoid_recent_duplicates
get_recent_context = _hf_post.get_recent_context
build_context_string = _hf_post.build_context_string
get_compiled_regex = _hf_post.get_compiled_regex
_get_compiled_regex = _hf_post.get_compiled_regex
generate_simple_response = _hf_personality.generate_simple_response
adapt_response_to_personality = _hf_personality.adapt_response_to_personality

if TYPE_CHECKING:
    from .bbia_tools import BBIATools

# Désactiver les avertissements de transformers
os.environ["TRANSFORMERS_VERBOSITY"] = "error"

logger = logging.getLogger(__name__)


# Import conditionnel des dépendances Hugging Face
try:
    import warnings

    import torch

    # Supprimer les avertissements de transformers
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        from transformers import (
            BlipForConditionalGeneration,
            BlipProcessor,
            CLIPModel,
            CLIPProcessor,
            WhisperForConditionalGeneration,
            WhisperProcessor,
            pipeline,
        )
        from transformers.utils import (
            logging as transformers_logging,
        )

        # Réduire la verbosité de transformers
        transformers_logging.set_verbosity_error()

    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False
    logger.warning(
        "Hugging Face transformers non disponible. "
        "Installez avec: pip install transformers torch",
    )


class BBIAHuggingFace:
    """Module d'intégration Hugging Face pour BBIA-SIM.

    Fonctionnalités :
    - Vision : CLIP, BLIP pour description d'images
    - Audio : Whisper pour STT avancé
    - NLP : Modèles de sentiment, émotions
    - Multimodal : Modèles combinant vision + texte
    """

    # OPTIMISATION RAM: Thread partagé au niveau de la classe pour éviter fuites
    # (un seul thread pour toutes les instances)
    _shared_unload_thread: threading.Thread | None = None
    _shared_unload_thread_stop = threading.Event()
    _shared_unload_thread_lock = threading.Lock()
    _shared_instances: list["BBIAHuggingFace"] = []  # Pour tracking instances actives

    def __init__(
        self,
        device: str = "auto",
        cache_dir: str | None = None,
        tools: "BBIATools | None" = None,
    ) -> None:
        """Initialise le module Hugging Face.

        Args:
            device: Device pour les modèles ("cpu", "cuda", "auto")
            cache_dir: Répertoire de cache pour les modèles
            tools: Instance BBIATools pour function calling (optionnel)

        """
        if not HF_AVAILABLE:
            msg = (
                "Hugging Face transformers requis. "
                "Installez avec: pip install transformers torch"
            )
            raise ImportError(
                msg,
            )

        self.device = self._get_device(device)
        # Issue #310: Améliorer intégration HF Hub avec cache local
        self.cache_dir = cache_dir or os.environ.get(
            "HF_HOME",
            os.path.expanduser("~/.cache/huggingface"),
        )
        # Créer répertoire cache si nécessaire
        if self.cache_dir:
            os.makedirs(self.cache_dir, exist_ok=True)
        self.models: dict[str, Any] = {}
        self.processors: dict[str, Any] = {}
        # OPTIMISATION RAM: Limiter nombre de modèles en mémoire
        # simultanément (LRU cache)
        self._max_models_in_memory = 4  # Max 3-4 modèles simultanés
        self._model_last_used: dict[str, float] = {}  # Timestamp dernier usage pour LRU
        self._inactivity_timeout = (
            120.0  # 2 minutes d'inactivité → déchargement auto (optimisé)
        )

        # OPTIMISATION RAM: Thread partagé au niveau de la classe (évite fuites)
        # Enregistrer cette instance pour le thread partagé
        with BBIAHuggingFace._shared_unload_thread_lock:
            BBIAHuggingFace._shared_instances.append(self)
            # Démarrer thread partagé si nécessaire
            # (double-check pattern pour éviter race condition)
            if (
                BBIAHuggingFace._shared_unload_thread is None
                or not BBIAHuggingFace._shared_unload_thread.is_alive()
            ):
                # Double-check: vérifier une deuxième fois dans le lock
                # pour éviter race condition
                # (si plusieurs instances sont créées simultanément)
                if (
                    BBIAHuggingFace._shared_unload_thread is None
                    or not BBIAHuggingFace._shared_unload_thread.is_alive()
                ):
                    BBIAHuggingFace._shared_unload_thread_stop.clear()
                    BBIAHuggingFace._shared_unload_thread = threading.Thread(
                        target=BBIAHuggingFace._shared_auto_unload_loop,
                        daemon=True,
                        name="BBIAHF-AutoUnload-Shared",
                    )
                    BBIAHuggingFace._shared_unload_thread.start()
                    logger.debug(
                        "✅ Thread partagé déchargement auto Hugging Face démarré",
                    )

        # Chat intelligent : Historique et contexte
        # OPTIMISATION RAM: Utiliser deque avec maxlen pour limiter l'historique
        max_history_size = 1000  # Limiter à 1000 messages max
        self.conversation_history: deque[ConversationEntry] = deque(
            maxlen=max_history_size,
        )
        self.context: dict[str, Any] = {}
        self.bbia_personality = "friendly_robot"

        # Outils LLM pour function calling (optionnel)
        self.tools = tools

        # NLP pour détection améliorée (optionnel, chargé à la demande)
        self._sentence_model: Any | None = None
        self._use_nlp_detection = False  # Activé automatiquement si modèle disponible

        # Charger conversation depuis mémoire persistante si disponible
        try:
            from .bbia_memory import load_conversation_from_memory

            saved_history = load_conversation_from_memory()
            if saved_history:
                # Convertir liste de dict en ConversationEntry puis en deque avec maxlen
                max_history_size = 1000
                conversation_entries: list[ConversationEntry] = [
                    ConversationEntry(
                        user=entry.get("user", ""),
                        bbia=entry.get("bbia", ""),
                        sentiment=entry.get("sentiment", "neutral"),
                        timestamp=entry.get("timestamp", ""),
                    )
                    for entry in saved_history[-max_history_size:]
                ]
                self.conversation_history = deque(
                    conversation_entries,
                    maxlen=max_history_size,
                )
                logger.info(
                    "💾 Conversation chargée depuis mémoire (%d messages)",
                    len(self.conversation_history),
                )
        except ImportError:
            # Mémoire persistante optionnelle
            pass

        # Configuration des modèles recommandés
        self.model_configs = {
            "vision": {
                "clip": "openai/clip-vit-base-patch32",
                "blip": "Salesforce/blip-image-captioning-base",
            },
            "audio": {
                "whisper": "openai/whisper-base",
            },
            "nlp": {
                "sentiment": "cardiffnlp/twitter-roberta-base-sentiment-latest",
                "emotion": "j-hartmann/emotion-english-distilroberta-base",
            },
            "chat": {
                # LLM conversationnel (optionnel, activé si disponible)
                "mistral": (
                    "mistralai/Mistral-7B-Instruct-v0.3"
                ),  # ⭐ Recommandé (14GB RAM) - Mis à jour v0.2 → v0.3
                "llama": "meta-llama/Llama-3-8B-Instruct",  # Alternative (16GB RAM)
                "phi2": "microsoft/phi-2",  # ⭐ Léger pour RPi 5 (2.7B, ~5GB RAM)
                "tinyllama": (
                    "TinyLlama/TinyLlama-1.1B-Chat-v1.0"
                ),  # Ultra-léger (~2GB RAM)
            },
            "multimodal": {
                "blip_vqa": "Salesforce/blip-vqa-base",
                "smolvlm": (
                    "HuggingFaceTB/SmolVLM-Instruct"
                ),  # Alternative gratuite à gpt-realtime
                "moondream2": "vikhyatk/moondream2",  # Alternative plus légère
            },
        }

        # État du modèle de conversation
        self.chat_model: Any | None = None
        self.chat_tokenizer: Any | None = None
        self.use_llm_chat = False  # Activation optionnelle (lourd)

        # OPTIMISATION RAM: Lazy loading strict BBIAChat - ne pas charger à l'init
        # BBIAChat sera chargé uniquement au premier appel de chat()
        # Gain RAM estimé: ~500MB-1GB au démarrage
        self.bbia_chat: Any | None = None
        self._bbia_chat_robot_api = None  # Stocker robot_api pour lazy loading
        if tools and hasattr(tools, "robot_api"):
            self._bbia_chat_robot_api = tools.robot_api

        logger.info("🤗 BBIA Hugging Face initialisé (device: %s)", self.device)
        logger.info("😊 Personnalité BBIA: %s", self.bbia_personality)
        if self.cache_dir:
            logger.info("💾 Cache HF Hub: %s", self.cache_dir)

    def _get_device(self, device: str) -> str:
        """Détermine le device optimal."""
        if device == "auto":
            if HF_AVAILABLE and torch.cuda.is_available():
                return "cuda"
            if HF_AVAILABLE and torch.backends.mps.is_available():
                return "mps"  # Apple Silicon
            return "cpu"
        return device

    def _load_bbia_chat_lazy(self) -> None:
        """OPTIMISATION RAM: Charge BBIAChat uniquement à la demande
        (lazy loading strict).

        Gain RAM estimé: ~500MB-1GB au démarrage.
        BBIAChat n'est chargé que lors du premier appel à chat().
        """
        if self.bbia_chat is not None:
            return  # Déjà chargé

        try:
            from .bbia_chat import BBIAChat

            # Initialiser BBIAChat avec robot_api stocké
            self.bbia_chat = BBIAChat(robot_api=self._bbia_chat_robot_api)
            logger.info(
                "✅ BBIAChat (LLM conversationnel) chargé à la demande (lazy loading)",
            )
        except ImportError as e:
            logger.debug("BBIAChat non disponible: %s", e)
            self.bbia_chat = None
        except (AttributeError, RuntimeError) as e:
            logger.warning("Erreur initialisation BBIAChat: %s", e)
            self.bbia_chat = None
        except (TypeError, KeyError, IndexError) as e:
            logger.warning("Erreur inattendue initialisation BBIAChat: %s", e)
            self.bbia_chat = None

    def _load_multimodal_model(self, model_name: str) -> bool:
        """Charge un modèle multimodal (BLIP VQA, SmolVLM, Moondream2)."""
        if "blip" in model_name.lower() and "vqa" in model_name.lower():
            vqa_processor: Any = BlipProcessor.from_pretrained(  # nosec B615
                model_name,
                cache_dir=self.cache_dir,
                revision="main",
            )
            model = BlipForConditionalGeneration.from_pretrained(  # nosec B615
                model_name,
                cache_dir=self.cache_dir,
                revision="main",
            ).to(self.device)
            self.processors[f"{model_name}_processor"] = vqa_processor
            self.models[f"{model_name}_model"] = model
            return True
        if "smolvlm" in model_name.lower() or "moondream" in model_name.lower():
            # SmolVLM2 / Moondream2 (alternative gratuite à gpt-realtime)
            try:
                from transformers import (
                    AutoModelForVision2Seq,
                    AutoProcessor,
                )

                logger.info("📥 Chargement SmolVLM2/Moondream2: %s", model_name)
                processor: Any = AutoProcessor.from_pretrained(  # nosec B615
                    model_name,
                    cache_dir=self.cache_dir,
                    revision="main",
                )
                model = AutoModelForVision2Seq.from_pretrained(  # nosec B615
                    model_name,
                    cache_dir=self.cache_dir,
                    revision="main",
                ).to(self.device)
                self.processors[f"{model_name}_processor"] = processor
                self.models[f"{model_name}_model"] = model
                logger.info("✅ SmolVLM2/Moondream2 chargé: %s", model_name)
                return True
            except (ImportError, RuntimeError, OSError, ValueError) as e:
                logger.warning("⚠️ Échec chargement SmolVLM2/Moondream2: %s", e)
                return False
            except (TypeError, KeyError, IndexError) as e:
                logger.warning(
                    f"⚠️ Erreur inattendue chargement SmolVLM2/Moondream2: {e}",
                )
                return False
        return False

    def _resolve_model_name(self, model_name: str, model_type: str) -> str:
        """Résout les alias courts vers les identifiants Hugging Face complets.

        Args:
            model_name: Nom reçu (alias court possible)
            model_type: 'vision', 'audio', 'nlp', 'chat', 'multimodal'

        Returns:
            Identifiant de modèle résolu si alias connu, sinon le nom original

        """
        try:
            cfg = self.model_configs.get(model_type, {})
            # nlp: autoriser les alias comme 'emotion' ou 'sentiment'
            if model_type == "nlp" and model_name in cfg:
                return cfg[model_name]
            # vision/audio/multimodal/chat: si la clé exacte existe
            if isinstance(cfg, dict) and model_name in cfg:
                return cfg[model_name]
        except (KeyError, AttributeError, TypeError, ValueError) as e:
            logger.debug("Erreur résolution nom de modèle '%s': %s", model_name, e)
        except (IndexError, OSError) as e:
            logger.debug(
                "Erreur résolution nom de modèle '%s' (index/os): %s", model_name, e
            )
        except (
            Exception
        ) as e:  # noqa: BLE001 - Fallback final pour erreurs vraiment inattendues
            logger.debug(
                "Erreur inattendue résolution nom de modèle '%s': %s",
                model_name,
                e,
            )
        return model_name

    def load_model(self, model_name: str, model_type: str = "vision") -> bool:
        """Charge un modèle Hugging Face.

        Args:
            model_name: Nom du modèle ou chemin
            model_type: Type de modèle ('vision', 'audio', 'nlp', 'multimodal')

        Returns:
            True si chargé avec succès

        """
        try:
            # Résolution d'alias éventuel (ex: 'emotion' -> id complet)
            resolved_name = self._resolve_model_name(model_name, model_type)

            # OPTIMISATION PERFORMANCE: Vérifier si modèle déjà chargé
            # avant de recharger
            if model_type == "chat":
                # Modèles chat stockés dans self.chat_model et self.chat_tokenizer
                if self.chat_model is not None and self.chat_tokenizer is not None:
                    logger.debug(
                        "♻️ Modèle chat déjà chargé (%s), réutilisation",
                        resolved_name,
                    )
                    return True
            elif model_type == "nlp":
                # Modèles NLP stockés avec suffixe "_pipeline"
                model_key = f"{model_name}_pipeline"
                if model_key in self.models:
                    logger.debug(
                        "♻️ Modèle NLP déjà chargé (%s), réutilisation",
                        resolved_name,
                    )
                    return True
            else:
                # Modèles vision/audio/multimodal stockés avec suffixe "_model"
                model_key = f"{model_name}_model"
                if model_key in self.models:
                    logger.debug(
                        "♻️ Modèle %s déjà chargé (%s), réutilisation",
                        model_type,
                        resolved_name,
                    )
                    return True

            # OPTIMISATION RAM: Vérifier limite modèles et décharger LRU si nécessaire
            if len(self.models) >= self._max_models_in_memory:
                self._unload_lru_model()

            logger.info("📥 Chargement modèle %s (%s)", resolved_name, model_type)

            # OPTIMISATION RAM: Enregistrer timestamp usage modèle
            current_time = time.time()

            if model_type == "vision":
                if "clip" in model_name.lower():
                    clip_processor = CLIPProcessor.from_pretrained(  # nosec B615
                        resolved_name,
                        cache_dir=self.cache_dir,
                        revision="main",
                    )
                    model = CLIPModel.from_pretrained(  # nosec B615
                        resolved_name,
                        cache_dir=self.cache_dir,
                        revision="main",
                    ).to(self.device)
                    self.processors[f"{model_name}_processor"] = clip_processor
                    self.models[f"{model_name}_model"] = model

                elif "blip" in model_name.lower():
                    blip_processor: Any = BlipProcessor.from_pretrained(  # nosec B615
                        resolved_name,
                        cache_dir=self.cache_dir,
                        revision="main",
                    )
                    model = BlipForConditionalGeneration.from_pretrained(  # nosec B615
                        resolved_name,
                        cache_dir=self.cache_dir,
                        revision="main",
                    ).to(self.device)
                    self.processors[f"{model_name}_processor"] = blip_processor
                    self.models[f"{model_name}_model"] = model

            elif model_type == "audio":
                if "whisper" in model_name.lower():
                    whisper_processor: Any = (
                        WhisperProcessor.from_pretrained(  # nosec B615
                            resolved_name,
                            cache_dir=self.cache_dir,
                            revision="main",
                        )
                    )
                    model = (
                        WhisperForConditionalGeneration.from_pretrained(  # nosec B615
                            resolved_name,
                            cache_dir=self.cache_dir,
                            revision="main",
                        ).to(self.device)
                    )
                    self.processors[f"{model_name}_processor"] = whisper_processor
                    self.models[f"{model_name}_model"] = model

            elif model_type == "nlp":
                # Utilisation des pipelines pour NLP
                pipeline_name = self._get_pipeline_name(resolved_name)
                # Laisser le device auto (plus fiable pour CPU/MPS/CUDA)
                pipe = pipeline(pipeline_name, model=resolved_name)
                self.models[f"{model_name}_pipeline"] = pipe

            elif model_type == "chat":
                # Charger LLM conversationnel (Mistral, Llama, etc.)
                try:
                    # isort: off
                    from transformers import AutoModelForCausalLM
                    from transformers import AutoTokenizer

                    # isort: on

                    logger.info("📥 Chargement LLM (long) %s...", model_name)
                    self.chat_tokenizer = AutoTokenizer.from_pretrained(
                        model_name,
                        cache_dir=self.cache_dir,
                        revision="main",
                    )  # nosec B615

                    # Support instruction format
                    if (
                        self.chat_tokenizer is not None
                        and self.chat_tokenizer.pad_token is None
                    ):
                        self.chat_tokenizer.pad_token = self.chat_tokenizer.eos_token

                    chat_model_load: Any = (
                        AutoModelForCausalLM.from_pretrained(  # nosec B615
                            model_name,
                            cache_dir=self.cache_dir,
                            revision="main",
                            device_map="auto",  # Auto-détecte MPS/CPU/CUDA
                            torch_dtype=(
                                torch.float16 if self.device != "cpu" else torch.float32
                            ),
                        )
                    )
                    self.chat_model = chat_model_load

                    logger.info("✅ LLM %s chargé avec succès", model_name)
                    self.use_llm_chat = True
                    return True
                except (ImportError, RuntimeError, OSError, ValueError) as e:
                    logger.warning("⚠️  Échec chargement LLM %s: %s", model_name, e)
                    logger.info(
                        "💡 Fallback activé: réponses enrichies (stratégie règles v2)",
                    )
                    self.use_llm_chat = False
                    return False
                except KeyboardInterrupt:
                    logger.warning(
                        "⚠️  Chargement LLM %s interrompu (KeyboardInterrupt)",
                        model_name,
                    )
                    self.use_llm_chat = False
                    return False
                except Exception as e:
                    # Gérer les erreurs de cancellation
                    # (ex: "The operation was canceled")
                    error_msg = str(e).lower()
                    if "cancel" in error_msg or "interrupt" in error_msg:
                        logger.warning(
                            "⚠️  Chargement LLM %s annulé: %s",
                            model_name,
                            e,
                        )
                    else:
                        logger.warning(
                            "⚠️  Erreur inattendue chargement LLM %s: %s",
                            model_name,
                            e,
                        )
                    logger.info(
                        """💡 Fallback activé: réponses enrichies """
                        """(stratégie règles v2)""",
                    )
                    self.use_llm_chat = False
                    return False

            elif model_type == "multimodal":
                return self._load_multimodal_model(resolved_name)

            logger.info("✅ Modèle %s chargé avec succès", resolved_name)

            # OPTIMISATION RAM: Enregistrer timestamp usage modèle
            model_key = f"{model_name}_{model_type}"
            self._model_last_used[model_key] = current_time

            return True

        except (ImportError, RuntimeError, OSError, ValueError, AttributeError):
            logger.exception("❌ Erreur chargement modèle %s:", model_name)
            return False
        except KeyboardInterrupt:
            logger.warning(
                "⚠️  Chargement modèle %s interrompu (KeyboardInterrupt)",
                model_name,
            )
            return False
        except Exception as e:
            # Gérer les erreurs de cancellation (ex: "The operation was canceled")
            error_msg = str(e).lower()
            if "cancel" in error_msg or "interrupt" in error_msg:
                logger.warning(
                    "⚠️  Chargement modèle %s annulé: %s",
                    model_name,
                    e,
                )
            else:
                logger.exception(
                    "❌ Erreur inattendue chargement modèle %s:", model_name
                )
            return False

    def _get_pipeline_name(self, model_name: str) -> str:
        """Détermine le nom du pipeline basé sur le modèle."""
        if "sentiment" in model_name.lower():
            return "sentiment-analysis"
        if "emotion" in model_name.lower():
            return "text-classification"
        return "text-classification"

    def describe_image(
        self,
        image: str | Image.Image | npt.NDArray[np.uint8],
        model_name: str = "blip",
    ) -> str:
        """Décrit une image (délègue à ``bbia_hf_media_inference``)."""
        return cast(str, _hf_media.describe_image(self, image, model_name))

    def analyze_sentiment(
        self,
        text: str,
        model_name: str = "cardiffnlp/twitter-roberta-base-sentiment-latest",
    ) -> SentimentResult:
        """Analyse le sentiment (délègue à ``bbia_hf_media_inference``)."""
        return cast(
            SentimentResult, _hf_media.analyze_sentiment(self, text, model_name)
        )

    def analyze_emotion(
        self, text: str, model_name: str = "emotion"
    ) -> SentimentResult:
        """Analyse l'émotion (délègue à ``bbia_hf_media_inference``)."""
        return cast(SentimentResult, _hf_media.analyze_emotion(self, text, model_name))

    def transcribe_audio(
        self,
        audio_path: str,
        model_name: str = "whisper",
    ) -> str:
        """Transcrit l'audio (délègue à ``bbia_hf_media_inference``)."""
        return cast(str, _hf_media.transcribe_audio(self, audio_path, model_name))

    def answer_question(
        self,
        image: str | Image.Image | npt.NDArray[np.uint8],
        question: str,
        model_name: str = "blip_vqa",
    ) -> str:
        """VQA (délègue à ``bbia_hf_media_inference``)."""
        return cast(str, _hf_media.answer_question(self, image, question, model_name))

    def get_available_models(self) -> dict[str, list[str]]:
        """Retourne la liste des modèles disponibles par catégorie."""
        return {
            "vision": list(self.model_configs["vision"].keys()),
            "audio": list(self.model_configs["audio"].keys()),
            "nlp": list(self.model_configs["nlp"].keys()),
            "chat": list(self.model_configs.get("chat", {}).keys()),
            "multimodal": list(self.model_configs["multimodal"].keys()),
        }

    def get_loaded_models(self) -> list[str]:
        """Retourne la liste des modèles actuellement chargés."""
        return list(self.models.keys())

    def enable_llm_chat(self, model_name: str = "mistral") -> bool:
        """Active le LLM chat (délègue à ``bbia_hf_model_memory``)."""
        return cast(bool, _hf_memory.enable_llm_chat(self, model_name))

    def disable_llm_chat(self) -> None:
        """Désactive le LLM chat (délègue à ``bbia_hf_model_memory``)."""
        _hf_memory.disable_llm_chat(self)

    def _unload_lru_model(self) -> None:
        """Décharge le modèle LRU (délègue à ``bbia_hf_model_memory``)."""
        _hf_memory.unload_lru_model(self)

    def _update_model_usage(self, model_key: str) -> None:
        """Met à jour timestamp d'usage (délègue à ``bbia_hf_model_memory``)."""
        _hf_memory.update_model_usage(self, model_key)

    @staticmethod
    def _shared_auto_unload_loop() -> None:
        """Boucle auto-unload partagée (délègue à ``bbia_hf_model_memory``)."""
        _hf_memory.shared_auto_unload_loop()
        # Continuer même en cas d'erreur

    def __del__(self) -> None:
        """Nettoyage lors de la destruction de l'instance."""
        try:
            # Retirer cette instance de la liste partagée
            with BBIAHuggingFace._shared_unload_thread_lock:
                if self in BBIAHuggingFace._shared_instances:
                    BBIAHuggingFace._shared_instances.remove(self)
                # Arrêter thread partagé si plus d'instances actives
                if (
                    not BBIAHuggingFace._shared_instances
                    and BBIAHuggingFace._shared_unload_thread
                    and BBIAHuggingFace._shared_unload_thread.is_alive()
                ):
                    BBIAHuggingFace._shared_unload_thread_stop.set()
                    # Timeout plus court en CI pour éviter blocage
                    import os

                    timeout = (
                        0.5 if os.environ.get("CI", "false").lower() == "true" else 2.0
                    )
                    BBIAHuggingFace._shared_unload_thread.join(timeout=timeout)
                    thread = BBIAHuggingFace._shared_unload_thread
                    if thread.is_alive():
                        # Thread daemon se terminera automatiquement
                        # à l'arrêt du processus
                        logger.debug(
                            "Thread partagé déchargement auto Hugging Face "
                            "en cours d'arrêt (daemon)",
                        )
                    else:
                        logger.debug(
                            "Thread partagé déchargement auto Hugging Face "
                            "arrêté (plus d'instances)",
                        )
        except (AttributeError, RuntimeError, TypeError):
            # Ignorer erreurs lors de la destruction
            pass

    def unload_model(self, model_name: str) -> bool:
        """Décharge un modèle (délègue à ``bbia_hf_model_memory``)."""
        return cast(bool, _hf_memory.unload_model(self, model_name))

    def get_model_info(self) -> dict[str, Any]:
        """Retourne les informations sur les modèles chargés.

        Returns:
            dict[str, Any]: Dictionnaire contenant les informations sur les modèles,
                incluant device, loaded_models, available_models, et cache_dir.

        """
        return {
            "device": self.device,
            "loaded_models": self.get_loaded_models(),
            "available_models": self.get_available_models(),
            "cache_dir": self.cache_dir,
            "hf_available": HF_AVAILABLE,
        }

    def chat(
        self,
        user_message: str,
        use_context: bool = True,
        enable_tools: bool = True,
    ) -> str:
        """Chat intelligent (délègue à ``bbia_hf_chat_engine``)."""
        return cast(
            str,
            _hf_chat_engine.chat(self, user_message, use_context, enable_tools),
        )

    def _generate_llm_response(
        self,
        user_message: str,
        use_context: bool = True,
        enable_tools: bool = True,
    ) -> str:
        """Génère une réponse via LLM (délègue à ``bbia_hf_chat_engine``)."""
        return cast(
            str,
            _hf_chat_engine.generate_llm_response(
                self, user_message, use_context, enable_tools
            ),
        )

    def _detect_and_execute_tools(self, user_message: str) -> str | None:
        """Détecte et exécute des outils depuis le message utilisateur.

        Délègue à ``bbia_hf_tool_orchestration`` (NLP puis mots-clés).
        """
        if not self.tools:
            return None
        return cast(
            str | None,
            _hf_orch.detect_and_execute_tools(
                user_message,
                state=self,
                execute_tool=self.tools.execute_tool,
                analyze_sentiment=lambda text: cast(
                    dict[str, Any], self.analyze_sentiment(text)
                ),
                extract_angle_fn=self._extract_angle,
                extract_intensity_fn=self._extract_intensity,
                tools_available=True,
            ),
        )

    def _detect_tool_with_nlp(
        self,
        user_message: str,
    ) -> tuple[str, float] | None:
        """Détecte outil avec NLP (sentence-transformers) si disponible."""
        return cast(
            tuple[str, float] | None,
            _hf_orch.detect_tool_with_nlp(user_message, self),
        )

    def _execute_detected_tool(
        self,
        tool_name: str,
        user_message: str,
        message_lower: str,
    ) -> str | None:
        """Exécute un outil détecté (par NLP ou mots-clés)."""
        if not self.tools:
            return None
        return cast(
            str | None,
            _hf_orch.execute_detected_tool(
                tool_name,
                user_message,
                message_lower,
                execute_tool=self.tools.execute_tool,
                analyze_sentiment=lambda text: cast(
                    dict[str, Any], self.analyze_sentiment(text)
                ),
                extract_angle_fn=self._extract_angle,
                extract_intensity_fn=self._extract_intensity,
            ),
        )

    def _extract_angle(self, message: str) -> float | None:
        """Extrait un angle depuis un message (ex: "30 degrés", "pi/4 radians").

        Args:
            message: Message utilisateur

        Returns:
            Angle en degrés, ou None si non trouvé

        """
        return cast(float | None, extract_angle(message))

    def _extract_intensity(self, message: str) -> float | None:
        """Extrait une intensité depuis un message (mots-clés comme
        "légèrement", "beaucoup").

        Args:
            message: Message utilisateur

        Returns:
            Intensité entre 0.0 et 1.0, ou None si non trouvé

        """
        return cast(float | None, extract_intensity(message))

    def _postprocess_llm_output(self, text: str, user_message: str) -> str:
        """Nettoie et compacte la sortie LLM (délègue à ``bbia_hf_chat_postprocess``)."""
        return cast(str, _hf_post.postprocess_llm_output(text, user_message, self))

    def _avoid_recent_duplicates(self, text: str) -> str:
        """Évite les duplications exactes avec les dernières réponses BBIA."""
        return cast(str, _hf_post.avoid_recent_duplicates(text, self))

    def _safe_fallback(self) -> str:
        """Retourne un fallback naturel et varié pour éviter les chaînes vides."""
        return cast(str, _hf_post.safe_fallback(self))

    def _generate_simple_response(self, message: str, sentiment: SentimentDict) -> str:
        """Génère réponse intelligente (délègue à ``bbia_hf_personality_response``)."""
        return cast(
            str, _hf_personality.generate_simple_response(message, sentiment, self)
        )

    def _adapt_response_to_personality(
        self,
        response: str,
        sentiment: SentimentDict,
    ) -> str:
        """Adapte la réponse selon personnalité (délègue à ``bbia_hf_personality_response``)."""
        return cast(
            str,
            _hf_personality.adapt_response_to_personality(response, sentiment, self),
        )

    def _normalize_response_length(self, text: str) -> str:
        """Normalise la longueur de la réponse vers ~[30, 150] caractères."""
        return cast(str, _hf_post.normalize_response_length(text, self))

    def _get_recent_context(self) -> str | None:
        """Extrait un mot-clé du contexte récent pour cohérence conversationnelle."""
        return cast(str | None, _hf_post.get_recent_context(self))

    def _build_context_string(self) -> str:
        """Construit le contexte pour la conversation."""
        return cast(str, _hf_post.build_context_string(self))


def main() -> None:
    """Test du module BBIA Hugging Face."""
    if not HF_AVAILABLE:
        logging.error("❌ Hugging Face transformers non disponible")
        logging.info("Installez avec: pip install transformers torch")
        return

    # Initialisation
    hf = BBIAHuggingFace()

    # Test chargement modèle
    logging.info("📥 Test chargement modèle BLIP...")
    success = hf.load_model("Salesforce/blip-image-captioning-base", "vision")
    logging.info("Résultat: %s", "✅" if success else "❌")

    # Test analyse sentiment
    logging.info("\n📝 Test analyse sentiment...")
    sentiment_result = hf.analyze_sentiment("Je suis très heureux aujourd'hui!")
    logging.info("Résultat: %s", sentiment_result)

    # Test analyse émotion
    logging.info("\n😊 Test analyse émotion...")
    emotion_result = hf.analyze_emotion("Je suis excité par ce projet!")
    logging.info(f"Résultat: {emotion_result}")

    # Test chat intelligent
    logging.info("\n💬 Test chat intelligent...")
    chat_result1 = hf.chat("Bonjour")
    logging.info(f"BBIA: {chat_result1}")
    chat_result2 = hf.chat("Comment allez-vous ?")
    logging.info(f"BBIA: {chat_result2}")

    # Informations
    logging.info(f"\n📊 Informations: {hf.get_model_info()}")
    logging.info(
        f"\n📝 Historique conversation: {len(hf.conversation_history)} messages",
    )


if __name__ == "__main__":
    main()
