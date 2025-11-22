#!/usr/bin/env python3
"""BBIA Tools - Outils LLM pour contrôle robot
Expose les fonctions robot au LLM pour conversations intelligentes.

Conforme à l'app conversationnelle officielle Reachy Mini.
"""

import logging
from typing import TYPE_CHECKING, Any

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from .bbia_huggingface import BBIAHuggingFace
    from .bbia_vision import BBIAVision
    from .robot_api import RobotAPI


class BBIATools:
    """Outils LLM pour contrôle robot - conforme app officielle."""

    def __init__(
        self,
        robot_api: "RobotAPI | None" = None,
        vision: "BBIAVision | None" = None,
        hf_chat: "BBIAHuggingFace | None" = None,
    ) -> None:
        """Initialise les outils BBIA.

        Args:
            robot_api: Instance RobotAPI pour contrôle robot
            vision: Instance BBIAVision pour capture caméra
            hf_chat: Instance BBIAHuggingFace (optionnel)

        """
        self.robot_api = robot_api
        self.vision = vision
        self.hf_chat = hf_chat
        self.current_dance: str | None = None
        self.current_emotion: str | None = None
        self.head_tracking_enabled = True

        logger.info("🔧 BBIATools initialisé")

    def get_tools(self) -> list[dict[str, Any]]:
        """Retourne la liste des outils disponibles pour LLM (conforme app officielle).

        Returns:
            Liste d'outils au format JSON Schema pour LLM function calling

        """
        return [
            {
                "type": "function",
                "function": {
                    "name": "move_head",
                    "description": (
                        "Changer la position de la tête du robot (gauche/droite/haut/bas/avant)"
                    ),
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "direction": {
                                "type": "string",
                                "enum": ["left", "right", "up", "down", "forward"],
                                "description": "Direction du mouvement de tête",
                            },
                            "intensity": {
                                "type": "number",
                                "minimum": 0.0,
                                "maximum": 1.0,
                                "default": 0.5,
                                "description": (
                                    "Intensité du mouvement (0.0 = subtil, 1.0 = maximal)"
                                ),
                            },
                        },
                        "required": ["direction"],
                    },
                },
            },
            {
                "type": "function",
                "function": {
                    "name": "camera",
                    "description": (
                        "Capture une image de la caméra et analyse l'environnement avec vision AI"
                    ),
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "analyze": {
                                "type": "boolean",
                                "default": True,
                                "description": "Analyser l'image avec vision AI",
                            },
                        },
                    },
                },
            },
            {
                "type": "function",
                "function": {
                    "name": "head_tracking",
                    "description": (
                        "Activer ou désactiver le suivi automatique du visage"
                    ),
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "enabled": {
                                "type": "boolean",
                                "description": (
                                    "True pour activer, False pour désactiver"
                                ),
                            },
                        },
                        "required": ["enabled"],
                    },
                },
            },
            {
                "type": "function",
                "function": {
                    "name": "dance",
                    "description": (
                        "Faire danser le robot avec un mouvement de la bibliothèque de danses"
                    ),
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "move_name": {
                                "type": "string",
                                "description": (
                                    "Nom du mouvement de danse (ex: 'groovy_sway_and_roll', 'happy_dance')"
                                ),
                            },
                            "dataset": {
                                "type": "string",
                                "default": "pollen-robotics/reachy-mini-dances-library",
                                "description": (
                                    "Dataset HuggingFace contenant les danses"
                                ),
                            },
                        },
                        "required": ["move_name"],
                    },
                },
            },
            {
                "type": "function",
                "function": {
                    "name": "stop_dance",
                    "description": "Arrêter la danse en cours d'exécution",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                    },
                },
            },
            {
                "type": "function",
                "function": {
                    "name": "play_emotion",
                    "description": "Jouer une émotion sur le robot",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "emotion": {
                                "type": "string",
                                "enum": [
                                    "happy",
                                    "sad",
                                    "excited",
                                    "curious",
                                    "calm",
                                    "neutral",
                                    "angry",
                                    "surprised",
                                ],
                                "description": "Nom de l'émotion à jouer",
                            },
                            "intensity": {
                                "type": "number",
                                "minimum": 0.0,
                                "maximum": 1.0,
                                "default": 0.7,
                                "description": "Intensité de l'émotion",
                            },
                            "duration": {
                                "type": "number",
                                "minimum": 0.5,
                                "default": 5.0,
                                "description": "Durée en secondes",
                            },
                        },
                        "required": ["emotion"],
                    },
                },
            },
            {
                "type": "function",
                "function": {
                    "name": "stop_emotion",
                    "description": "Arrêter l'émotion en cours",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                    },
                },
            },
            {
                "type": "function",
                "function": {
                    "name": "do_nothing",
                    "description": "Rester inactif et ne rien faire",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "duration": {
                                "type": "number",
                                "minimum": 0.0,
                                "default": 2.0,
                                "description": "Durée d'inactivité en secondes",
                            },
                        },
                    },
                },
            },
        ]

    def execute_tool(
        self,
        tool_name: str,
        parameters: dict[str, Any],
    ) -> dict[str, Any]:
        """Exécute un outil par son nom avec paramètres (conforme app officielle).

        Args:
            tool_name: Nom de l'outil à exécuter
            parameters: Paramètres de l'outil

        Returns:
            Résultat de l'exécution avec statut et détails

        Raises:
            ValueError: Si l'outil n'existe pas ou paramètres invalides

        """
        logger.info("🔧 Exécution outil: %s avec params: %s", tool_name, parameters)

        if tool_name == "move_head":
            return self._execute_move_head(parameters)
        if tool_name == "camera":
            return self._execute_camera(parameters)
        if tool_name == "head_tracking":
            return self._execute_head_tracking(parameters)
        if tool_name == "dance":
            return self._execute_dance(parameters)
        if tool_name == "stop_dance":
            return self._execute_stop_dance(parameters)
        if tool_name == "play_emotion":
            return self._execute_play_emotion(parameters)
        if tool_name == "stop_emotion":
            return self._execute_stop_emotion(parameters)
        if tool_name == "do_nothing":
            return self._execute_do_nothing(parameters)
        raise ValueError(f"Outil inconnu: {tool_name}")

    def _execute_move_head(self, parameters: dict[str, Any]) -> dict[str, Any]:
        """Exécute le mouvement de tête."""
        if not self.robot_api:
            return {"status": "error", "detail": "robot_api non disponible"}

        direction = parameters.get("direction")
        intensity = parameters.get("intensity", 0.5)

        # Mapping direction → mouvement
        # Conforme SDK: utiliser goto_target avec create_head_pose
        try:
            from reachy_mini.utils import create_head_pose
        except ImportError:
            logger.warning("reachy_mini.utils non disponible - mouvement basique")
            return {"status": "error", "detail": "SDK officiel requis"}

        # Calculer pose selon direction et intensité
        # Intensité max: 0.3 rad (conforme SDK safe limits)
        max_angle = 0.3 * intensity

        if direction == "left":
            pose = create_head_pose(yaw=-max_angle, pitch=0.0)
        elif direction == "right":
            pose = create_head_pose(yaw=max_angle, pitch=0.0)
        elif direction == "up":
            pose = create_head_pose(yaw=0.0, pitch=max_angle)
        elif direction == "down":
            pose = create_head_pose(yaw=0.0, pitch=-max_angle)
        elif direction == "forward":
            pose = create_head_pose(yaw=0.0, pitch=0.0)  # Position neutre
        else:
            return {"status": "error", "detail": f"Direction invalide: {direction}"}

        # Exécuter mouvement
        try:
            if hasattr(self.robot_api, "goto_target"):
                self.robot_api.goto_target(head=pose, duration=0.8, method="minjerk")
            elif hasattr(self.robot_api, "set_target_head_pose"):
                self.robot_api.set_target_head_pose(pose)
            else:
                return {
                    "status": "error",
                    "detail": "Méthode goto_target non disponible",
                }

            return {
                "status": "success",
                "detail": f"Tête déplacée {direction} (intensité: {intensity:.2f})",
            }
        except (AttributeError, RuntimeError, ValueError) as e:
            logger.exception("Erreur move_head: %s", e)
            return {"status": "error", "detail": str(e)}
        except Exception as e:
            logger.exception("Erreur inattendue move_head: %s", e)
            return {"status": "error", "detail": str(e)}

    def _execute_camera(self, parameters: dict[str, Any]) -> dict[str, Any]:
        """Exécute la capture caméra et analyse."""
        if not self.vision:
            return {"status": "error", "detail": "vision non disponible"}

        analyze = parameters.get("analyze", True)

        try:
            # Capture image (utiliser _capture_image_from_camera ou méthode publique)
            image = None
            # Utiliser méthode publique si disponible, sinon méthode privée avec getattr
            if hasattr(self.vision, "capture_image"):
                image = self.vision.capture_image()
            elif hasattr(self.vision, "_capture_image_from_camera"):
                # OPTIMISATION: Accès direct plus efficace que getattr avec constante
                image = self.vision._capture_image_from_camera()  # noqa: SLF001
            else:
                return {"status": "error", "detail": "Méthode capture non disponible"}

            if image is None:
                return {"status": "error", "detail": "Échec capture image"}

            result: dict[str, Any] = {"status": "success", "image_captured": True}

            # Analyse si demandée
            if analyze:
                # Scan environnement (détection objets, visages)
                scan_result = self.vision.scan_environment()
                result["analysis"] = scan_result

                # Résumé textuel pour LLM
                objects = scan_result.get("objects", [])
                faces = scan_result.get("faces", [])

                summary_parts = []
                if objects:
                    summary_parts.append(
                        f"{len(objects)} objets détectés: {', '.join([o.get('name', 'unknown') for o in objects[:5]])}",
                    )
                if faces:
                    summary_parts.append(f"{len(faces)} visage(s) détecté(s)")

                result["summary"] = (
                    "; ".join(summary_parts)
                    if summary_parts
                    else "Aucun objet ou visage détecté"
                )

            return result
        except (AttributeError, RuntimeError, OSError, ValueError) as e:
            logger.exception("Erreur camera: %s", e)
            return {"status": "error", "detail": str(e)}
        except Exception as e:
            logger.exception("Erreur inattendue camera: %s", e)
            return {"status": "error", "detail": str(e)}

    def _execute_head_tracking(self, parameters: dict[str, Any]) -> dict[str, Any]:
        """Active/désactive le suivi automatique du visage."""
        enabled = parameters.get("enabled", True)
        self.head_tracking_enabled = enabled

        logger.info("Suivi visage: %s", "activé" if enabled else "désactivé")

        # Intégration VisionTrackingBehavior pour activation réelle
        if enabled and self.vision and self.robot_api:
            try:
                from .bbia_behavior import VisionTrackingBehavior

                # Créer comportement avec vision et robot_api
                tracking_behavior = VisionTrackingBehavior(
                    self.vision,
                    robot_api=self.robot_api,
                )
                # Exécuter le comportement de suivi visuel
                context: dict[str, Any] = {}
                result = tracking_behavior.execute(context)
                if result:
                    return {
                        "status": "success",
                        "detail": "Suivi visage activé - Objets détectés et suivis",
                    }
                return {
                    "status": "success",
                    "detail": (
                        "Suivi visage activé - Aucun objet détecté pour l'instant"
                    ),
                }
            except ImportError:
                logger.warning(
                    "VisionTrackingBehavior non disponible - suivi basique activé",
                )
            except (AttributeError, RuntimeError) as e:
                logger.exception("Erreur activation VisionTrackingBehavior: %s", e)
            except Exception as e:
                logger.exception(
                    "Erreur inattendue activation VisionTrackingBehavior: %s", e
                )

        return {
            "status": "success",
            "detail": f"Suivi visage {'activé' if enabled else 'désactivé'}",
        }

    def _execute_dance(self, parameters: dict[str, Any]) -> dict[str, Any]:
        """Exécute une danse depuis la bibliothèque."""
        if not self.robot_api:
            return {"status": "error", "detail": "robot_api non disponible"}

        move_name = parameters.get("move_name")
        dataset = parameters.get(
            "dataset",
            "pollen-robotics/reachy-mini-dances-library",
        )

        if not move_name:
            return {"status": "error", "detail": "move_name requis"}

        try:
            # Utiliser RecordedMoves pour jouer la danse
            from reachy_mini.motion.recorded_move import RecordedMoves

            recorded_moves = RecordedMoves(dataset)
            move = recorded_moves.get(move_name)

            # Issue #344: Améliorer enchaînement fluide des danses
            # Utiliser initial_goto_duration > 0 pour transition fluide depuis position actuelle
            initial_goto_duration = 0.5  # Transition de 0.5s pour enchaînement fluide

            # Jouer le mouvement
            if hasattr(self.robot_api, "play_move"):
                self.robot_api.play_move(
                    move,
                    play_frequency=100.0,
                    initial_goto_duration=initial_goto_duration,
                )
            elif hasattr(self.robot_api, "async_play_move"):
                import asyncio

                asyncio.create_task(
                    self.robot_api.async_play_move(move, 100.0, initial_goto_duration)
                )
            else:
                return {"status": "error", "detail": "play_move non disponible"}

            self.current_dance = move_name

            return {
                "status": "success",
                "detail": f"Danse '{move_name}' démarrée",
                "move_name": move_name,
            }
        except ImportError:
            return {"status": "error", "detail": "SDK officiel reachy_mini requis"}
        except ValueError as e:
            return {"status": "error", "detail": f"Mouvement non trouvé: {e}"}
        except (AttributeError, RuntimeError) as e:
            logger.exception("Erreur dance: %s", e)
            return {"status": "error", "detail": str(e)}
        except Exception as e:
            logger.exception("Erreur inattendue dance: %s", e)
            return {"status": "error", "detail": str(e)}

    def _execute_stop_dance(self, parameters: dict[str, Any]) -> dict[str, Any]:
        """Arrête la danse en cours."""
        if not self.current_dance:
            return {"status": "info", "detail": "Aucune danse en cours"}

        dance_name = self.current_dance
        self.current_dance = None

        # Implémenter arrêt réel du mouvement avec emergency_stop
        if self.robot_api:
            try:
                # Utiliser emergency_stop() pour arrêt immédiat et sécurisé
                if hasattr(self.robot_api, "emergency_stop"):
                    stop_success = self.robot_api.emergency_stop()
                    if stop_success:
                        logger.info(
                            f"Danse '{dance_name}' arrêtée via emergency_stop()",
                        )
                        return {
                            "status": "success",
                            "detail": (
                                f"Danse '{dance_name}' arrêtée immédiatement "
                                "(arrêt d'urgence)"
                            ),
                        }
                    logger.warning("emergency_stop() a échoué pour '%s'", dance_name)
                else:
                    logger.warning(
                        "robot_api.emergency_stop() non disponible - arrêt basique",
                    )
            except (AttributeError, RuntimeError) as e:
                logger.exception("Erreur arrêt danse: %s", e)
            except Exception as e:
                logger.exception("Erreur inattendue arrêt danse: %s", e)

        logger.info("Danse '%s' arrêtée", dance_name)

        return {
            "status": "success",
            "detail": f"Danse '{dance_name}' arrêtée",
        }

    def _execute_play_emotion(self, parameters: dict[str, Any]) -> dict[str, Any]:
        """Joue une émotion."""
        if not self.robot_api:
            return {"status": "error", "detail": "robot_api non disponible"}

        emotion = parameters.get("emotion")
        intensity = parameters.get("intensity", 0.7)
        duration = parameters.get("duration", 5.0)

        if not emotion:
            return {"status": "error", "detail": "emotion requis"}

        try:
            self.robot_api.set_emotion(emotion, intensity)
            self.current_emotion = emotion

            return {
                "status": "success",
                "detail": (
                    f"Émotion '{emotion}' jouée (intensité: {intensity:.2f}, durée: {duration:.1f}s)"
                ),
                "emotion": emotion,
            }
        except (AttributeError, RuntimeError, ValueError) as e:
            logger.exception("Erreur play_emotion: %s", e)
            return {"status": "error", "detail": str(e)}
        except Exception as e:
            logger.exception("Erreur inattendue play_emotion: %s", e)
            return {"status": "error", "detail": str(e)}

    def _execute_stop_emotion(self, parameters: dict[str, Any]) -> dict[str, Any]:
        """Arrête l'émotion en cours."""
        if not self.current_emotion:
            return {"status": "info", "detail": "Aucune émotion en cours"}

        emotion_name = self.current_emotion
        self.current_emotion = None

        try:
            if self.robot_api:
                self.robot_api.set_emotion("neutral", 0.5)
        except (AttributeError, RuntimeError, ValueError) as e:
            logger.warning("Erreur stop_emotion: %s", e)
        except Exception as e:
            logger.warning("Erreur inattendue stop_emotion: %s", e)

        return {
            "status": "success",
            "detail": f"Émotion '{emotion_name}' arrêtée",
        }

    def _execute_do_nothing(self, parameters: dict[str, Any]) -> dict[str, Any]:
        """Ne fait rien (inactivité)."""
        duration = parameters.get("duration", 2.0)

        logger.info("Mode inactif pendant %.1fs", duration)

        # Ne rien faire - le robot reste dans sa position actuelle
        return {
            "status": "success",
            "detail": f"Inactif pendant {duration:.1f}s",
        }
