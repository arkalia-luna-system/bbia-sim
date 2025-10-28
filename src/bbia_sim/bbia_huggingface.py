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

    def __init__(self, device: str = "auto", cache_dir: Optional[str] = None):
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
            "multimodal": {
                "blip_vqa": "Salesforce/blip-vqa-base",
            },
        }

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
                    processor = CLIPProcessor.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    )
                    model = CLIPModel.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    ).to(self.device)
                    self.processors[f"{model_name}_processor"] = processor
                    self.models[f"{model_name}_model"] = model

                elif "blip" in model_name.lower():
                    processor = (
                        BlipProcessor.from_pretrained(  # nosec B615 # type: ignore
                            model_name, cache_dir=self.cache_dir
                        )
                    )
                    model = BlipForConditionalGeneration.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    ).to(self.device)
                    self.processors[f"{model_name}_processor"] = processor  # type: ignore
                    self.models[f"{model_name}_model"] = model

            elif model_type == "audio":
                if "whisper" in model_name.lower():
                    processor = (
                        WhisperProcessor.from_pretrained(  # nosec B615 # type: ignore
                            model_name, cache_dir=self.cache_dir
                        )
                    )
                    model = (
                        WhisperForConditionalGeneration.from_pretrained(  # nosec B615
                            model_name, cache_dir=self.cache_dir
                        ).to(self.device)
                    )
                    self.processors[f"{model_name}_processor"] = processor  # type: ignore
                    self.models[f"{model_name}_model"] = model

            elif model_type == "nlp":
                # Utilisation des pipelines pour NLP
                pipeline_name = self._get_pipeline_name(model_name)
                pipe = pipeline(  # type: ignore[call-overload]
                    pipeline_name, model=model_name, device=self.device
                )
                self.models[f"{model_name}_pipeline"] = pipe

            elif model_type == "multimodal":
                if "blip" in model_name.lower() and "vqa" in model_name.lower():
                    processor = (
                        BlipProcessor.from_pretrained(  # nosec B615 # type: ignore
                            model_name, cache_dir=self.cache_dir
                        )
                    )
                    model = BlipForConditionalGeneration.from_pretrained(  # nosec B615
                        model_name, cache_dir=self.cache_dir
                    ).to(self.device)
                    self.processors[f"{model_name}_processor"] = processor  # type: ignore
                    self.models[f"{model_name}_model"] = model

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
            "multimodal": list(self.model_configs["multimodal"].keys()),
        }

    def get_loaded_models(self) -> list[str]:
        """Retourne la liste des modèles actuellement chargés."""
        return list(self.models.keys())

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

            # 2. Générer réponse basée sur le message et le sentiment
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

            # 4. Adapter réponse selon personnalité BBIA
            adapted_response = self._adapt_response_to_personality(
                bbia_response, sentiment
            )

            return adapted_response

        except Exception as e:
            logger.error(f"❌ Erreur chat: {e}")
            return "Je ne comprends pas bien, peux-tu reformuler ?"

    def _generate_simple_response(self, message: str, sentiment: dict) -> str:
        """Génère une réponse simple basée sur le sentiment et mots-clés.

        Args:
            message: Message utilisateur
            sentiment: Analyse sentiment du message

        Returns:
            Réponse basique et intelligente
        """
        message_lower = message.lower()

        # Salutations
        if any(word in message_lower for word in ["bonjour", "salut", "hello", "hi"]):
            return "Bonjour ! Comment allez-vous ? Je suis BBIA, votre robot compagnon."

        # Au revoir
        if any(
            word in message_lower
            for word in ["au revoir", "bye", "goodbye", "à bientôt"]
        ):
            return "Au revoir ! Ce fut un plaisir de discuter avec vous. À bientôt !"

        # Positif
        if sentiment.get("sentiment") == "POSITIVE" or any(
            word in message_lower for word in ["content", "heureux", "joie", "cool"]
        ):
            return "C'est super ! Je suis content pour vous. Continuez comme ça !"

        # Question
        if message_lower.count("?") > 0 or any(
            word in message_lower for word in ["qui", "quoi", "comment", "pourquoi"]
        ):
            return "C'est une bonne question ! Je réfléchis... Comment puis-je vous aider ?"

        # Réponse par défaut intelligente
        return "C'est intéressant. Dites-moi en plus s'il vous plaît."

    def _adapt_response_to_personality(self, response: str, sentiment: dict) -> str:
        """Adapte la réponse selon la personnalité BBIA.

        Args:
            response: Réponse de base
            sentiment: Analyse sentiment

        Returns:
            Réponse adaptée avec emoji selon personnalité
        """
        personality_responses = {
            "friendly_robot": f"🤖 {response}",
            "curious": f"🤔 {response}",
            "enthusiastic": f"🎉 {response}",
            "calm": f"😌 {response}",
        }
        return personality_responses.get(self.bbia_personality, f"💬 {response}")

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


def main():
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
