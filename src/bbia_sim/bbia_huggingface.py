#!/usr/bin/env python3
"""
BBIA Hugging Face Integration - Module d'intégration des modèles pré-entraînés
Intégration avancée avec Hugging Face Hub pour enrichir les capacités IA de BBIA-SIM
"""

import logging
import os
from typing import Any, Optional, Union

import numpy as np
from PIL import Image

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
        from transformers.utils import logging as transformers_logging

        # Réduire la verbosité de transformers
        transformers_logging.set_verbosity_error()

    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False
    logger.warning(
        "Hugging Face transformers non disponible. Installez avec: pip install transformers torch"
    )


class BBIAHuggingFace:
    """Module d'intégration Hugging Face pour BBIA-SIM.

    Fonctionnalités :
    - Vision : CLIP, BLIP pour description d'images
    - Audio : Whisper pour STT avancé
    - NLP : Modèles de sentiment, émotions
    - Multimodal : Modèles combinant vision + texte
    """

    def __init__(self, device: str = "auto", cache_dir: Optional[str] = None) -> None:
        """Initialise le module Hugging Face.

        Args:
            device: Device pour les modèles ("cpu", "cuda", "auto")
            cache_dir: Répertoire de cache pour les modèles
        """
        if not HF_AVAILABLE:
            raise ImportError(
                "Hugging Face transformers requis. Installez avec: pip install transformers torch"
            )

        self.device = self._get_device(device)
        self.cache_dir = cache_dir
        self.models: dict[str, Any] = {}
        self.processors: dict[str, Any] = {}

        # Chat intelligent : Historique et contexte
        self.conversation_history: list[dict[str, Any]] = []
        self.context: dict[str, Any] = {}
        self.bbia_personality = "friendly_robot"

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
                "mistral": "mistralai/Mistral-7B-Instruct-v0.2",  # ⭐ Recommandé
                "llama": "meta-llama/Llama-3-8B-Instruct",  # Alternative
            },
            "multimodal": {
                "blip_vqa": "Salesforce/blip-vqa-base",
            },
        }

        # État du modèle de conversation
        self.chat_model: Optional[Any] = None
        self.chat_tokenizer: Optional[Any] = None
        self.use_llm_chat = False  # Activation optionnelle (lourd)

        logger.info(f"🤗 BBIA Hugging Face initialisé (device: {self.device})")
        logger.info(f"😊 Personnalité BBIA: {self.bbia_personality}")

    def _get_device(self, device: str) -> str:
        """Détermine le device optimal."""
        if device == "auto":
            if HF_AVAILABLE and torch.cuda.is_available():
                return "cuda"
            elif HF_AVAILABLE and torch.backends.mps.is_available():
                return "mps"  # Apple Silicon
            else:
                return "cpu"
        return device

    def _load_vision_model(self, model_name: str) -> bool:
        """Charge un modèle de vision (CLIP ou BLIP)."""
        if "clip" in model_name.lower():
            processor: Any = CLIPProcessor.from_pretrained(  # nosec B615
                model_name, cache_dir=self.cache_dir
            )
            model = CLIPModel.from_pretrained(  # nosec B615
                model_name, cache_dir=self.cache_dir
            ).to(self.device)
            self.processors[f"{model_name}_processor"] = processor  # type: ignore[assignment]
            self.models[f"{model_name}_model"] = model  # type: ignore[assignment]
            return True
        elif "blip" in model_name.lower():
            blip_processor: Any = BlipProcessor.from_pretrained(  # nosec B615
                model_name, cache_dir=self.cache_dir
            )
            model = BlipForConditionalGeneration.from_pretrained(  # nosec B615
                model_name, cache_dir=self.cache_dir
            ).to(self.device)
            self.processors[f"{model_name}_processor"] = blip_processor  # type: ignore[assignment]
            self.models[f"{model_name}_model"] = model  # type: ignore[assignment]
            return True
        return False

    def _load_audio_model(self, model_name: str) -> bool:
        """Charge un modèle audio (Whisper)."""
        if "whisper" in model_name.lower():
            whisper_processor: Any = WhisperProcessor.from_pretrained(  # nosec B615
                model_name, cache_dir=self.cache_dir
            )
            model = WhisperForConditionalGeneration.from_pretrained(  # nosec B615
                model_name, cache_dir=self.cache_dir
            ).to(self.device)
            self.processors[f"{model_name}_processor"] = whisper_processor  # type: ignore[assignment]
            self.models[f"{model_name}_model"] = model  # type: ignore[assignment]
            return True
        return False

    def _load_chat_model(self, model_name: str) -> bool:
        """Charge un modèle LLM conversationnel."""
        try:
            from transformers import AutoModelForCausalLM, AutoTokenizer

            logger.info(
                f"📥 Chargement LLM {model_name} (peut prendre 1-2 minutes)..."
            )
            self.chat_tokenizer = AutoTokenizer.from_pretrained(  # type: ignore[assignment]
                model_name, cache_dir=self.cache_dir
            )

            if (
                self.chat_tokenizer is not None
                and self.chat_tokenizer.pad_token is None
            ):
                self.chat_tokenizer.pad_token = self.chat_tokenizer.eos_token

            self.chat_model = AutoModelForCausalLM.from_pretrained(  # type: ignore # nosec B615
                model_name,
                cache_dir=self.cache_dir,
                device_map="auto",
                torch_dtype=(
                    torch.float16 if self.device != "cpu" else torch.float32
                ),
            )
            logger.info(f"✅ LLM {model_name} chargé avec succès")
            self.use_llm_chat = True
            return True
        except Exception as e:
            logger.warning(f"⚠️  Impossible de charger LLM {model_name}: {e}")
            logger.info("💡 Le système utilisera les réponses enrichies (règles)")
            self.use_llm_chat = False
            return False

    def _load_multimodal_model(self, model_name: str) -> bool:
        """Charge un modèle multimodal (BLIP VQA)."""
        if "blip" in model_name.lower() and "vqa" in model_name.lower():
            vqa_processor: Any = BlipProcessor.from_pretrained(  # type: ignore # nosec B615
                model_name, cache_dir=self.cache_dir
            )
            model = BlipForConditionalGeneration.from_pretrained(  # nosec B615
                model_name, cache_dir=self.cache_dir
            ).to(self.device)
            self.processors[f"{model_name}_processor"] = vqa_processor  # type: ignore[assignment]
            self.models[f"{model_name}_model"] = model  # type: ignore[assignment]
            return True
        return False

    def load_model(self, model_name: str, model_type: str = "vision") -> bool:
        """Charge un modèle Hugging Face.

        Args:
            model_name: Nom du modèle ou chemin
            model_type: Type de modèle ("vision", "audio", "nlp", "multimodal")

        Returns:
            True si chargé avec succès
        """
        try:
            logger.info(f"📥 Chargement modèle {model_name} ({model_type})")

            if model_type == "vision":
                if "clip" in model_name.lower():
                    clip_processor: Any = CLIPProcessor.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    )
                    model = CLIPModel.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    ).to(self.device)
                    self.processors[f"{model_name}_processor"] = clip_processor  # type: ignore[assignment]
                    self.models[f"{model_name}_model"] = model  # type: ignore[assignment]

                elif "blip" in model_name.lower():
                    blip_processor: Any = BlipProcessor.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    )
                    model = BlipForConditionalGeneration.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    ).to(self.device)
                    self.processors[f"{model_name}_processor"] = blip_processor  # type: ignore[assignment]
                    self.models[f"{model_name}_model"] = model  # type: ignore[assignment]

            elif model_type == "audio":
                if "whisper" in model_name.lower():
                    whisper_processor: Any = WhisperProcessor.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    )
                    model = WhisperForConditionalGeneration.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    ).to(self.device)
                    self.processors[f"{model_name}_processor"] = whisper_processor  # type: ignore[assignment]
                    self.models[f"{model_name}_model"] = model  # type: ignore[assignment]

            elif model_type == "nlp":
                # Utilisation des pipelines pour NLP
                pipeline_name = self._get_pipeline_name(model_name)
                pipe = pipeline(  # type: ignore[call-overload]
                    pipeline_name, model=model_name, device=self.device
                )
                self.models[f"{model_name}_pipeline"] = pipe

            elif model_type == "chat":
                # Charger LLM conversationnel (Mistral, Llama, etc.)
                try:
                    from transformers import AutoModelForCausalLM, AutoTokenizer

                    logger.info(
                        f"📥 Chargement LLM {model_name} (peut prendre 1-2 minutes)..."
                    )
                    self.chat_tokenizer = AutoTokenizer.from_pretrained(  # type: ignore[assignment]
                        model_name, cache_dir=self.cache_dir
                    )

                    # Support instruction format
                    if (
                        self.chat_tokenizer is not None
                        and self.chat_tokenizer.pad_token is None
                    ):
                        self.chat_tokenizer.pad_token = self.chat_tokenizer.eos_token

                    chat_model_load: Any = AutoModelForCausalLM.from_pretrained(  # nosec B615
                        model_name,
                        cache_dir=self.cache_dir,
                        device_map="auto",  # Auto-détecte MPS/CPU/CUDA
                        torch_dtype=(
                            torch.float16 if self.device != "cpu" else torch.float32
                        ),
                    )
                    self.chat_model = chat_model_load

                    logger.info(f"✅ LLM {model_name} chargé avec succès")
                    self.use_llm_chat = True
                    return True
                except Exception as e:
                    logger.warning(f"⚠️  Impossible de charger LLM {model_name}: {e}")
                    logger.info(
                        "💡 Le système utilisera les réponses enrichies (règles)"
                    )
                    self.use_llm_chat = False
                    return False

            elif model_type == "multimodal":
                if "blip" in model_name.lower() and "vqa" in model_name.lower():
                    vqa_processor: Any = BlipProcessor.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    )
                    model = BlipForConditionalGeneration.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    ).to(self.device)
                    self.processors[f"{model_name}_processor"] = vqa_processor  # type: ignore[assignment]
                    self.models[f"{model_name}_model"] = model  # type: ignore[assignment]

            logger.info(f"✅ Modèle {model_name} chargé avec succès")
            return True

        except Exception as e:
            logger.error(f"❌ Erreur chargement modèle {model_name}: {e}")
            return False

    def _get_pipeline_name(self, model_name: str) -> str:
        """Détermine le nom du pipeline basé sur le modèle."""
        if "sentiment" in model_name.lower():
            return "sentiment-analysis"
        elif "emotion" in model_name.lower():
            return "text-classification"
        else:
            return "text-classification"

    def describe_image(
        self, image: Union[str, Image.Image, np.ndarray], model_name: str = "blip"
    ) -> str:
        """Décrit une image avec BLIP ou CLIP.

        Args:
            image: Image à décrire (chemin, PIL Image, ou numpy array)
            model_name: Nom du modèle à utiliser

        Returns:
            Description textuelle de l'image
        """
        try:
            # Conversion de l'image
            if isinstance(image, str):
                image = Image.open(image)
            elif isinstance(image, np.ndarray):
                image = Image.fromarray(image)

            if "blip" in model_name.lower():
                processor_key = f"{model_name}_processor"
                model_key = f"{model_name}_model"

                if processor_key not in self.processors or model_key not in self.models:
                    self.load_model(model_name, "vision")

                processor = self.processors[processor_key]
                model = self.models[model_key]

                inputs = processor(image, return_tensors="pt").to(self.device)
                out = model.generate(**inputs, max_length=50)
                description = processor.decode(out[0], skip_special_tokens=True)

                return str(description)

            elif "clip" in model_name.lower():
                processor_key = f"{model_name}_processor"
                model_key = f"{model_name}_model"

                if processor_key not in self.processors or model_key not in self.models:
                    self.load_model(model_name, "vision")

                processor = self.processors[processor_key]
                model = self.models[model_key]

                inputs = processor(
                    text=["a photo of"], images=image, return_tensors="pt", padding=True
                ).to(self.device)
                outputs = model(**inputs)
                logits_per_image = outputs.logits_per_image
                probs = logits_per_image.softmax(dim=1)

                return f"CLIP analysis: {probs.cpu().numpy()}"

            return "Erreur: modèle non supporté"

        except Exception as e:
            logger.error(f"❌ Erreur description image: {e}")
            return "Erreur lors de la description de l'image"

    def analyze_sentiment(
        self,
        text: str,
        model_name: str = "cardiffnlp/twitter-roberta-base-sentiment-latest",
    ) -> dict[str, Any]:
        """Analyse le sentiment d'un texte.

        Args:
            text: Texte à analyser
            model_name: Nom du modèle à utiliser (modèle Hugging Face complet)

        Returns:
            Dictionnaire avec sentiment et score
        """
        try:
            model_key = f"{model_name}_pipeline"

            if model_key not in self.models:
                self.load_model(model_name, "nlp")

            pipeline = self.models[model_key]
            result = pipeline(text)

            return {
                "text": text,
                "sentiment": result[0]["label"],
                "score": result[0]["score"],
                "model": model_name,
            }

        except Exception as e:
            logger.error(f"❌ Erreur analyse sentiment: {e}")
            return {"error": str(e)}

    def analyze_emotion(self, text: str, model_name: str = "emotion") -> dict[str, Any]:
        """Analyse les émotions dans un texte.

        Args:
            text: Texte à analyser
            model_name: Nom du modèle à utiliser

        Returns:
            Dictionnaire avec émotion détectée et score
        """
        try:
            model_key = f"{model_name}_pipeline"

            if model_key not in self.models:
                self.load_model(model_name, "nlp")

            pipeline = self.models[model_key]
            result = pipeline(text)

            return {
                "text": text,
                "emotion": result[0]["label"],
                "score": result[0]["score"],
                "model": model_name,
            }

        except Exception as e:
            logger.error(f"❌ Erreur analyse émotion: {e}")
            return {"error": str(e)}

    def transcribe_audio(self, audio_path: str, model_name: str = "whisper") -> str:
        """Transcrit un fichier audio avec Whisper.

        Args:
            audio_path: Chemin vers le fichier audio
            model_name: Nom du modèle Whisper à utiliser

        Returns:
            Texte transcrit
        """
        try:
            processor_key = f"{model_name}_processor"
            model_key = f"{model_name}_model"

            if processor_key not in self.processors or model_key not in self.models:
                self.load_model(model_name, "audio")

            processor = self.processors[processor_key]
            model = self.models[model_key]

            # Chargement de l'audio
            audio, sample_rate = processor.load_audio(audio_path)
            inputs = processor(
                audio, sampling_rate=sample_rate, return_tensors="pt"
            ).to(self.device)

            # Transcription
            with torch.no_grad():
                predicted_ids = model.generate(inputs["input_features"])

            transcription = processor.batch_decode(
                predicted_ids, skip_special_tokens=True
            )[0]

            return str(transcription)

        except Exception as e:
            logger.error(f"❌ Erreur transcription audio: {e}")
            return "Erreur lors de la transcription"

    def answer_question(
        self,
        image: Union[str, Image.Image],
        question: str,
        model_name: str = "blip_vqa",
    ) -> str:
        """Répond à une question sur une image (VQA).

        Args:
            image: Image à analyser
            question: Question à poser
            model_name: Nom du modèle VQA à utiliser

        Returns:
            Réponse à la question
        """
        try:
            # Conversion de l'image
            if isinstance(image, str):
                image = Image.open(image)
            elif isinstance(image, np.ndarray):
                image = Image.fromarray(image)

            processor_key = f"{model_name}_processor"
            model_key = f"{model_name}_model"

            if processor_key not in self.processors or model_key not in self.models:
                self.load_model(model_name, "multimodal")

            processor = self.processors[processor_key]
            model = self.models[model_key]

            inputs = processor(image, question, return_tensors="pt").to(self.device)
            out = model.generate(**inputs, max_length=50)
            answer = processor.decode(out[0], skip_special_tokens=True)

            return str(answer)

        except Exception as e:
            logger.error(f"❌ Erreur VQA: {e}")
            return "Erreur lors de l'analyse de l'image"

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

    def enable_llm_chat(
        self, model_name: str = "mistralai/Mistral-7B-Instruct-v0.2"
    ) -> bool:
        """Active le LLM conversationnel (optionnel, lourd).

        Args:
            model_name: Modèle LLM à charger (Mistral ou Llama)

        Returns:
            True si chargé avec succès

        Note:
            - Requiert ~14GB RAM pour Mistral 7B
            - Premier chargement : 1-2 minutes
            - Support Apple Silicon (MPS) automatique
        """
        logger.info(f"📥 Activation LLM conversationnel: {model_name}")
        success = self.load_model(model_name, model_type="chat")
        if success:
            logger.info(
                "✅ LLM conversationnel activé - Conversations intelligentes disponibles"
            )
        else:
            logger.warning("⚠️  LLM non chargé - Utilisation réponses enrichies")
        return success

    def disable_llm_chat(self) -> None:
        """Désactive le LLM conversationnel pour libérer mémoire."""
        if self.chat_model:
            del self.chat_model
            del self.chat_tokenizer
            self.chat_model = None
            self.chat_tokenizer = None
            self.use_llm_chat = False
            import gc

            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            logger.info("🗑️ LLM conversationnel désactivé - Mémoire libérée")

    def unload_model(self, model_name: str) -> bool:
        """Décharge un modèle de la mémoire.

        Args:
            model_name: Nom du modèle à décharger

        Returns:
            True si déchargé avec succès
        """
        try:
            keys_to_remove = [key for key in self.models.keys() if model_name in key]
            for key in keys_to_remove:
                del self.models[key]

            keys_to_remove = [
                key for key in self.processors.keys() if model_name in key
            ]
            for key in keys_to_remove:
                del self.processors[key]

            logger.info(f"🗑️ Modèle {model_name} déchargé")
            return True

        except Exception as e:
            logger.error(f"❌ Erreur déchargement modèle {model_name}: {e}")
            return False

    def get_model_info(self) -> dict[str, Any]:
        """Retourne les informations sur les modèles chargés."""
        return {
            "device": self.device,
            "loaded_models": self.get_loaded_models(),
            "available_models": self.get_available_models(),
            "cache_dir": self.cache_dir,
            "hf_available": HF_AVAILABLE,
        }

    def chat(self, user_message: str, use_context: bool = True) -> str:
        """Chat intelligent avec BBIA avec contexte et analyse sentiment.

        Utilise LLM pré-entraîné (Mistral 7B) si disponible, sinon réponses enrichies.

        Args:
            user_message: Message de l'utilisateur
            use_context: Utiliser le contexte des messages précédents

        Returns:
            Réponse intelligente de BBIA
        """
        try:
            # 1. Analyser sentiment du message (avec gestion erreur)
            try:
                sentiment = self.analyze_sentiment(user_message)
            except Exception:
                # Fallback si sentiment indisponible
                sentiment = {"sentiment": "NEUTRAL", "score": 0.5}

            # 2. Générer réponse avec LLM si disponible, sinon réponses enrichies
            if self.use_llm_chat and self.chat_model and self.chat_tokenizer:
                # Utiliser LLM pré-entraîné (Mistral/Llama)
                bbia_response = self._generate_llm_response(user_message, use_context)
            else:
                # Fallback vers réponses enrichies (règles + variété)
                bbia_response = self._generate_simple_response(user_message, sentiment)

            # 3. Sauvegarder dans l'historique
            from datetime import datetime

            self.conversation_history.append(
                {
                    "user": user_message,
                    "bbia": bbia_response,
                    "sentiment": sentiment,
                    "timestamp": datetime.now().isoformat(),
                }
            )

            # 4. Adapter réponse selon personnalité BBIA (si pas LLM)
            if not self.use_llm_chat:
                adapted_response = self._adapt_response_to_personality(
                    bbia_response, sentiment
                )
            else:
                adapted_response = bbia_response  # LLM gère déjà la personnalité

            return adapted_response

        except Exception as e:
            logger.error(f"❌ Erreur chat: {e}")
            return "Je ne comprends pas bien, peux-tu reformuler ?"

    def _generate_llm_response(
        self, user_message: str, use_context: bool = True
    ) -> str:
        """Génère une réponse avec LLM pré-entraîné (Mistral/Llama).

        Args:
            user_message: Message utilisateur
            use_context: Utiliser contexte historique

        Returns:
            Réponse générée par LLM
        """
        try:
            if not self.chat_model or not self.chat_tokenizer:
                raise ValueError("LLM non chargé")

            # Construire prompt avec personnalité BBIA enrichie
            # AMÉLIORATION INTELLIGENCE: Prompt plus détaillé pour générer des réponses plus naturelles
            personality_descriptions = {
                "friendly_robot": (
                    "Tu es BBIA, un robot Reachy Mini amical, curieux et intelligent. "
                    "Tu communiques en français de manière naturelle, chaleureuse et authentique, "
                    "comme un véritable compagnon. Tu évites les phrases répétitives ou trop génériques. "
                    "Tes réponses sont concises (max 2-3 phrases), engageantes et montrent que tu "
                    "comprends vraiment l'interlocuteur. Tu utilises des expressions naturelles et "
                    "varies tes formulations pour ne jamais sonner robotique."
                ),
                "curious": (
                    "Tu es BBIA, un robot Reachy Mini extrêmement curieux et passionné par l'apprentissage. "
                    "Tu poses des questions pertinentes et montres un véritable intérêt pour comprendre. "
                    "Tes réponses sont exploratoires et invitent à approfondir."
                ),
                "enthusiastic": (
                    "Tu es BBIA, un robot Reachy Mini plein d'enthousiasme et d'énergie positive. "
                    "Tu transmets ta joie de communiquer et tu encourages l'interaction de manière "
                    "vivante et authentique."
                ),
                "calm": (
                    "Tu es BBIA, un robot Reachy Mini serein et apaisant. "
                    "Tu communiques avec douceur et profondeur, en prenant le temps nécessaire. "
                    "Tes réponses reflètent une sagesse tranquille."
                ),
            }
            system_prompt = personality_descriptions.get(
                self.bbia_personality,
                personality_descriptions["friendly_robot"],
            )

            # Construire messages pour format instruct
            messages = [{"role": "system", "content": system_prompt}]

            # Ajouter contexte si demandé
            if use_context and self.conversation_history:
                # Derniers 2 échanges pour contexte
                for entry in self.conversation_history[-2:]:
                    messages.append({"role": "user", "content": entry["user"]})
                    messages.append({"role": "assistant", "content": entry["bbia"]})

            # Ajouter message actuel
            messages.append({"role": "user", "content": user_message})

            # Appliquer template de chat (Mistral/Llama format)
            try:
                # Format instruct standard
                prompt = self.chat_tokenizer.apply_chat_template(
                    messages, tokenize=False, add_generation_prompt=True
                )
            except Exception:
                # Fallback si pas de chat template
                prompt = f"{system_prompt}\n\nUser: {user_message}\nAssistant:"

            # Tokeniser
            inputs = self.chat_tokenizer(
                prompt, return_tensors="pt", truncation=True, max_length=1024
            ).to(self.device)

            # Générer réponse
            with torch.no_grad():
                outputs = self.chat_model.generate(
                    **inputs,
                    max_new_tokens=150,  # Limiter longueur réponse
                    temperature=0.7,  # Créativité modérée
                    top_p=0.9,  # Nucleus sampling
                    do_sample=True,
                    pad_token_id=self.chat_tokenizer.eos_token_id,
                )

            # Décoder réponse
            generated_text = self.chat_tokenizer.decode(
                outputs[0][inputs["input_ids"].shape[1] :], skip_special_tokens=True
            ).strip()

            # Nettoyer réponse (enlever préfixes possibles)
            if "Assistant:" in generated_text:
                generated_text = generated_text.split("Assistant:")[-1].strip()

            logger.info(f"🤖 LLM réponse générée: {generated_text[:100]}...")
            return generated_text if generated_text else "Je comprends, continuez."

        except Exception as e:
            logger.warning(f"⚠️  Erreur génération LLM, fallback enrichi: {e}")
            # Fallback vers réponses enrichies
            try:
                sentiment = self.analyze_sentiment(user_message)
            except Exception:
                sentiment = {"sentiment": "NEUTRAL", "score": 0.5}
            return self._generate_simple_response(user_message, sentiment)

    def _generate_simple_response(self, message: str, sentiment: dict) -> str:
        """Génère une réponse intelligente et variée basée sur le sentiment, contexte et personnalité.

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
        recent_context = self._get_recent_context()

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
                    "Bonne journée ! Je suis là si vous avez besoin de moi.",
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
            variants = greetings.get(self.bbia_personality, greetings["friendly_robot"])
            return random.choice(variants)

        # Au revoir - Réponses émotionnelles selon contexte
        if any(
            word in message_lower
            for word in ["au revoir", "bye", "goodbye", "à bientôt", "adieu"]
        ):
            goodbyes = {
                "friendly_robot": [
                    "Au revoir ! Ce fut un plaisir de discuter avec vous. Revenez quand vous voulez !",
                    "À bientôt ! N'hésitez pas à revenir pour continuer notre conversation.",
                    "Au revoir ! J'espère vous revoir bientôt. Portez-vous bien !",
                ],
                "curious": [
                    "Au revoir ! J'espère qu'on pourra continuer nos échanges intéressants !",
                    "À bientôt ! J'ai encore plein de questions à vous poser !",
                    "Au revoir ! Revenez pour partager de nouvelles découvertes !",
                ],
                "enthusiastic": [
                    "Au revoir ! C'était génial de discuter ! Revenez vite !",
                    "À bientôt ! J'ai hâte de vous revoir pour de nouvelles aventures !",
                    "Au revoir ! C'était super ! Revenez quand vous voulez !",
                ],
                "calm": [
                    "Au revoir. Je suis là quand vous aurez besoin de moi.",
                    "À bientôt. Prenez soin de vous.",
                    "Au revoir. Revenez quand vous vous sentirez prêt.",
                ],
            }
            variants = goodbyes.get(self.bbia_personality, goodbyes["friendly_robot"])
            return random.choice(variants)

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
                    "C'est vraiment formidable ! Je suis content que vous vous sentiez bien.",
                    "Super nouvelle ! Continuez comme ça, vous allez très bien !",
                    "C'est excellent ! Votre bonne humeur est contagieuse !",
                ],
                "curious": [
                    "Intéressant ! Qu'est-ce qui vous rend si heureux ?",
                    "Chouette ! Racontez-moi plus sur ce qui vous plaît !",
                    "Wow ! J'aimerais en savoir plus sur ce qui vous réjouit !",
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
                self.bbia_personality, positive_responses["friendly_robot"]
            )
            return random.choice(variants)

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
                    "Je comprends que vous ne vous sentiez pas bien. Je suis là pour vous écouter.",
                    "C'est difficile parfois. Voulez-vous en parler ? Je vous écoute.",
                    "Je ressens votre malaise. Comment puis-je vous aider à vous sentir mieux ?",
                ],
                "curious": [
                    "Qu'est-ce qui vous préoccupe ? J'aimerais comprendre pour mieux vous aider.",
                    "Votre message reflète de la tristesse. Partagez-moi ce qui vous tracasse.",
                    "Qu'est-ce qui cause cette difficulté ? Je veux vous aider.",
                ],
                "enthusiastic": [
                    "Courage ! Même dans les moments difficiles, on peut trouver des raisons d'espérer !",
                    "Je comprends que c'est dur, mais vous êtes capable de surmonter ça !",
                    "On va s'en sortir ! Parlez-moi de ce qui ne va pas, on va trouver une solution !",
                ],
                "calm": [
                    "Prenez votre temps. Je suis là, sans jugement.",
                    "Respirez. Tout va s'arranger. Je vous écoute.",
                    "Je comprends. Parlez-moi de ce qui vous trouble, à votre rythme.",
                ],
            }
            variants = negative_responses.get(
                self.bbia_personality, negative_responses["friendly_robot"]
            )
            return random.choice(variants)

        # Questions - Réponses adaptées selon type de question
        # AMÉLIORATION INTELLIGENCE: Détection type de question pour réponses plus pertinentes
        if message_lower.count("?") > 0 or any(
            word in message_lower
            for word in ["qui", "quoi", "comment", "pourquoi", "où", "quand", "combien"]
        ):
            # Détection type de question pour réponses plus intelligentes
            question_responses = {
                "friendly_robot": [
                    "Bonne question ! Laissez-moi réfléchir... Comment puis-je vous aider ?",
                    "Je comprends votre interrogation. Pouvez-vous me donner plus de détails pour que je puisse mieux vous répondre ?",
                    "Intéressant ! Cette question mérite réflexion. Qu'est-ce que vous en pensez vous-même ?",
                    "Ah, excellente question ! C'est quoi qui vous intrigue là-dedans ?",
                    "Hmm, intéressant. Dites-moi plus sur ce qui vous pousse à vous poser cette question.",
                    "Ça m'intrigue aussi ! Qu'est-ce qui vous amène à vous demander ça ?",
                    "Très bonne question ! Qu'est-ce qui a provoqué cette curiosité chez vous ?",
                    "Excellente question ! J'aimerais bien comprendre ce qui motive votre questionnement.",
                    "Hmm, c'est une question qui mérite qu'on s'y attarde. Qu'est-ce qui vous a poussé à la formuler ?",
                    "Intéressant angle d'approche ! Racontez-moi le contexte autour de cette question.",
                ],
                "curious": [
                    "Ah, j'aime cette question ! Qu'est-ce qui vous amène à vous demander ça ?",
                    "Fascinant ! Pourquoi cette question vous préoccupe-t-elle ?",
                    "Excellente question ! J'aimerais explorer ça ensemble avec vous.",
                ],
                "enthusiastic": [
                    "Super question ! Je suis tout excité de réfléchir à ça avec vous !",
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
                self.bbia_personality, question_responses["friendly_robot"]
            )
            return random.choice(variants)

        # Référence au contexte précédent si disponible
        # AMÉLIORATION INTELLIGENCE: Meilleure utilisation du contexte pour cohérence conversationnelle
        if recent_context:
            # Vérifier si le message actuel fait référence au contexte précédent (références: ça, ce, ce truc, etc.)
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
                has_reference or random.random() < 0.4
            ):  # 40% de chance si référence, sinon 30%
                context_responses = {
                    "friendly_robot": [
                        f"Ah, vous parlez de {recent_context.lower()} ? C'est intéressant ! Continuons sur ce sujet.",
                        f"En lien avec {recent_context.lower()}, j'aimerais en savoir plus. Qu'est-ce qui vous préoccupe là-dessus ?",
                        f"Vous mentionnez {recent_context.lower()}. Ça m'intrigue. Dites-moi en plus si vous voulez.",
                        f"Je vois le lien avec {recent_context.lower()}. C'est fascinant ! Racontez-moi davantage.",
                        f"En continuant sur {recent_context.lower()}, qu'est-ce qui vous passionne le plus ?",
                    ],
                    "curious": [
                        f"Ah oui, {recent_context.lower()} ! C'est exactement ce qui m'intéresse !",
                        f"En rapport avec {recent_context.lower()}, j'ai plein de questions !",
                        f"{recent_context.lower()} me passionne ! Continuons à explorer ça ensemble.",
                    ],
                    "enthusiastic": [
                        f"C'est génial, {recent_context.lower()} ! Continuons à creuser ça !",
                        f"Super, {recent_context.lower()} ! C'est trop intéressant !",
                        f"{recent_context.lower()} ? Wow, allons plus loin là-dessus !",
                    ],
                    "calm": [
                        f"Je comprends le lien avec {recent_context.lower()}. Explorons ça sereinement.",
                        f"En lien avec {recent_context.lower()}, prenons le temps d'y réfléchir.",
                        f"Je vois votre réflexion sur {recent_context.lower()}. Continuons calmement.",
                    ],
                }
                variants = context_responses.get(
                    self.bbia_personality, context_responses["friendly_robot"]
                )
                return random.choice(variants)

        # Réponses génériques variées selon personnalité et sentiment
        # AMÉLIORATION INTELLIGENCE: Réponses plus naturelles, engageantes et moins robotiques
        # Enrichi avec 15 variantes pour friendly_robot pour éviter répétition
        generic_responses = {
            "friendly_robot": [
                "Intéressant ! J'aimerais en savoir plus sur votre point de vue. Qu'est-ce qui vous a amené à penser ça ?",
                "Je vois ce que vous voulez dire. Continuez, je vous écoute attentivement.",
                "Merci de partager ça avec moi. Qu'est-ce qui vous intéresse le plus dans tout ça ?",
                "Hmm, c'est captivant. Vous pouvez m'en dire plus si vous voulez, je suis curieux.",
                "Ah d'accord, je comprends. Explorons ça ensemble si ça vous dit, j'adorerais en discuter.",
                "J'ai noté. Dites-moi tout ce qui vous vient à l'esprit, sans filtre.",
                "Ça m'intrigue ! Racontez-moi davantage, j'aime apprendre de vous.",
                "C'est fascinant. Qu'est-ce qui vous a amené à penser ça ? Je suis vraiment curieux.",
                "Wow, ça sonne intéressant. Voulez-vous développer ? J'aimerais mieux comprendre.",
                "C'est noté. Partagez-moi vos réflexions, je trouve ça enrichissant.",
                "Ah, c'est un point de vue intéressant. Qu'est-ce qui vous fait penser ainsi ?",
                "Je comprends votre perspective. N'hésitez pas à creuser davantage, j'aime quand on approfondit.",
                "C'est une réflexion qui pique ma curiosité. D'où vient cette idée ?",
                "Hmm, vous m'intriguez ! J'aimerais creuser cette pensée avec vous.",
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
                "Je comprends. Prenons le temps d'explorer cela ensemble.",
                "Intéressant. Continuons cette conversation sereinement.",
                "Je vois. Partagez-moi vos pensées, sans précipitation.",
            ],
        }
        variants = generic_responses.get(
            self.bbia_personality, generic_responses["friendly_robot"]
        )
        return random.choice(variants)

    def _adapt_response_to_personality(
        self, response: str, sentiment: dict  # noqa: ARG002
    ) -> str:
        """Adapte la réponse selon la personnalité BBIA avec nuances expressives.

        Args:
            response: Réponse de base
            sentiment: Analyse sentiment

        Returns:
            Réponse adaptée avec emoji et nuances selon personnalité
        """
        # Les réponses sont déjà adaptées dans _generate_simple_response,
        # on ajoute juste l'emoji ici pour cohérence
        personality_emojis = {
            "friendly_robot": "🤖",
            "curious": "🤔",
            "enthusiastic": "🎉",
            "calm": "😌",
        }
        emoji = personality_emojis.get(self.bbia_personality, "💬")

        # Si la réponse contient déjà un emoji, ne pas en ajouter (éviter doublon)
        if response.startswith(emoji) or any(char in emoji for char in response[:3]):
            return response

        return f"{emoji} {response}"

    def _get_recent_context(self) -> Optional[str]:
        """Extrait un mot-clé du contexte récent pour cohérence conversationnelle.

        Returns:
            Mot-clé du dernier message utilisateur (si disponible)
        """
        if not self.conversation_history:
            return None

        # Prendre le dernier message utilisateur
        last_entry = self.conversation_history[-1]
        user_msg = last_entry.get("user", "")

        # Extraire les mots significatifs (exclure articles, prépositions)
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
        words = [
            w for w in user_msg.lower().split() if len(w) > 3 and w not in stop_words
        ]

        # Retourner le premier mot significatif si disponible
        return str(words[0].capitalize()) if words else None

    def _build_context_string(self) -> str:
        """Construit le contexte pour la conversation.

        Returns:
            Chaîne de contexte basée sur l'historique
        """
        if not self.conversation_history:
            return (
                "Conversation avec BBIA (robot Reachy Mini). Soyez amical et curieux."
            )

        context = "Historique conversation:\n"
        for entry in self.conversation_history[-3:]:  # Derniers 3 échanges
            context += f"User: {entry['user']}\n"
            context += f"BBIA: {entry['bbia']}\n"
        return context


def main() -> None:
    """Test du module BBIA Hugging Face."""
    if not HF_AVAILABLE:
        print("❌ Hugging Face transformers non disponible")
        print("Installez avec: pip install transformers torch")
        return

    # Initialisation
    hf = BBIAHuggingFace()

    # Test chargement modèle
    print("📥 Test chargement modèle BLIP...")
    success = hf.load_model("Salesforce/blip-image-captioning-base", "vision")
    print(f"Résultat: {'✅' if success else '❌'}")

    # Test analyse sentiment
    print("\n📝 Test analyse sentiment...")
    sentiment_result = hf.analyze_sentiment("Je suis très heureux aujourd'hui!")
    print(f"Résultat: {sentiment_result}")

    # Test analyse émotion
    print("\n😊 Test analyse émotion...")
    emotion_result = hf.analyze_emotion("Je suis excité par ce projet!")
    print(f"Résultat: {emotion_result}")

    # Test chat intelligent
    print("\n💬 Test chat intelligent...")
    chat_result1 = hf.chat("Bonjour")
    print(f"BBIA: {chat_result1}")
    chat_result2 = hf.chat("Comment allez-vous ?")
    print(f"BBIA: {chat_result2}")

    # Informations
    print(f"\n📊 Informations: {hf.get_model_info()}")
    print(f"\n📝 Historique conversation: {len(hf.conversation_history)} messages")


if __name__ == "__main__":
    main()
