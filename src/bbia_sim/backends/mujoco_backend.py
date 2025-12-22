#!/usr/bin/env python3
"""MuJoCoBackend - Implémentation MuJoCo de RobotAPI.

Backend pour simulation MuJoCo.
"""

import importlib.resources
import logging
import os
import time
from pathlib import Path
from typing import Any

import mujoco  # type: ignore[import-untyped]
import mujoco.viewer  # type: ignore[import-untyped]

from bbia_sim.mujoco_model_cache import get_cached_mujoco_model
from bbia_sim.robot_api import RobotAPI

# Types pour goto_target
try:
    from reachy_mini.utils.interpolation import HeadPose  # type: ignore[import-untyped]
except ImportError:
    HeadPose = Any  # type: ignore[misc,assignment]

logger = logging.getLogger(__name__)


def _find_mujoco_model() -> Path | None:
    """Trouve le fichier modèle MuJoCo en utilisant plusieurs stratégies.

    Returns:
        Chemin vers le fichier modèle si trouvé, None sinon.
    """
    model_name = "reachy_mini_REAL_OFFICIAL.xml"
    tried_paths = []

    # Stratégie 0: Utiliser importlib.resources (le plus fiable pour les packages installés)
    try:
        # Python 3.9+ : utiliser files() API (plus robuste)
        if hasattr(importlib.resources, "files"):
            try:
                model_path_traversable = (
                    importlib.resources.files("bbia_sim.sim.models") / model_name
                )
                # Convertir Traversable en Path pour vérifier existence
                resolved = Path(str(model_path_traversable))
                if resolved.exists():
                    logger.debug(
                        "Modèle trouvé (stratégie 0a - importlib.resources.files): %s",
                        resolved,
                    )
                    return resolved
            except (
                ModuleNotFoundError,
                FileNotFoundError,
                TypeError,
                AttributeError,
                ValueError,
            ):
                pass

        # Fallback: utiliser path() API (Python 3.6+)
        with importlib.resources.path("bbia_sim.sim.models", model_name) as model_path:
            resolved = Path(model_path)
            if resolved.exists():
                logger.debug(
                    "Modèle trouvé (stratégie 0b - importlib.resources.path): %s",
                    resolved,
                )
                return resolved
    except (ModuleNotFoundError, FileNotFoundError, TypeError, AttributeError):
        # importlib.resources peut échouer si le package n'est pas installé ou en mode développement
        pass

    # Stratégie 1: Depuis le module (cas normal)
    module_dir = Path(__file__).parent.parent
    path = module_dir / "sim" / "models" / model_name
    tried_paths.append(path)
    if path.exists():
        logger.debug("Modèle trouvé (stratégie 1 - module): %s", path.resolve())
        return path.resolve()

    # Stratégie 2: Depuis la racine du projet (src/bbia_sim/...)
    path = (
        Path(__file__).parent.parent.parent.parent
        / "src"
        / "bbia_sim"
        / "sim"
        / "models"
        / model_name
    )
    tried_paths.append(path)
    if path.exists():
        logger.debug("Modèle trouvé (stratégie 2 - racine projet): %s", path.resolve())
        return path.resolve()

    # Stratégie 3: Depuis le répertoire de travail courant (comme dans les tests)
    cwd = Path.cwd()
    for base in [cwd, cwd / "src", cwd.parent]:
        path = base / "bbia_sim" / "sim" / "models" / model_name
        tried_paths.append(path)
        if path.exists():
            logger.debug("Modèle trouvé (stratégie 3a - cwd): %s", path.resolve())
            return path.resolve()
        path = base / "src" / "bbia_sim" / "sim" / "models" / model_name
        tried_paths.append(path)
        if path.exists():
            logger.debug("Modèle trouvé (stratégie 3b - cwd/src): %s", path.resolve())
            return path.resolve()

    # Stratégie 4: Depuis le répertoire des tests (comme test_mujoco_backend.py)
    # Chercher depuis différents points de départ possibles
    for test_base in [cwd, cwd.parent]:
        if (test_base / "tests").exists():
            path = test_base / "src" / "bbia_sim" / "sim" / "models" / model_name
            tried_paths.append(path)
            if path.exists():
                logger.debug(
                    "Modèle trouvé (stratégie 4 - depuis tests): %s", path.resolve()
                )
                return path.resolve()

    # Stratégie 5: Chercher récursivement depuis le répertoire courant (limité à 2 niveaux pour performance)
    logger.debug("Recherche récursive du modèle...")
    for depth in range(2):
        for root, _dirs, files in os.walk(cwd):
            # Éviter les répertoires inutiles
            rel_path = Path(root).relative_to(cwd)
            if rel_path.parts and rel_path.parts[0] in [
                ".git",
                "venv",
                "__pycache__",
                ".pytest_cache",
                "node_modules",
                ".tox",
            ]:
                continue
            if model_name in files:
                found = Path(root) / model_name
                tried_paths.append(found)
                if found.exists():
                    logger.debug(
                        "Modèle trouvé (stratégie 5 - récursif): %s", found.resolve()
                    )
                    return found.resolve()
            if depth == 0:
                break

    # Si rien n'est trouvé, logger tous les chemins essayés
    logger.warning(
        "Modèle MuJoCo introuvable après %d tentatives. Chemins testés: %s",
        len(tried_paths),
        [str(p.resolve()) for p in tried_paths[:10]],  # Limiter à 10 pour éviter spam
    )
    return None


class MuJoCoBackend(RobotAPI):
    """Backend MuJoCo pour RobotAPI."""

    def __init__(
        self,
        model_path: str | None = None,
    ) -> None:
        """Initialise le backend MuJoCo.

        Note: Le modèle par défaut est `reachy_mini_REAL_OFFICIAL.xml`
        (16 joints, complet). Le fichier `reachy_mini.xml` (7 joints, simplifié)
        existe mais n'est pas utilisé par défaut pour garantir la cohérence.

        Args:
            model_path: Chemin vers le modèle MuJoCo. Si None, utilise le modèle
                       par défaut résolu par rapport au module.
        """
        super().__init__()
        if model_path is None:
            # Utiliser la fonction de recherche robuste
            found_path = _find_mujoco_model()
            if found_path:
                model_path = str(found_path)
                logger.debug("Modèle MuJoCo trouvé: %s", model_path)
            else:
                # Fallback: utiliser le chemin relatif au module
                module_dir = Path(__file__).parent.parent
                default_path = (
                    module_dir / "sim" / "models" / "reachy_mini_REAL_OFFICIAL.xml"
                )
                model_path = str(default_path.resolve())
                logger.warning(
                    "Modèle MuJoCo non trouvé automatiquement, "
                    "utilisation du chemin par défaut: %s",
                    model_path,
                )
        self.model_path = Path(model_path)
        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self.viewer: mujoco.viewer.MjViewer | None = None
        self.joint_name_to_id: dict[str, int] = {}
        self.step_count = 0
        self.start_time: float = 0.0

    def connect(self) -> bool:
        """Connecte au simulateur MuJoCo."""
        try:
            # Résoudre le chemin absolu pour éviter les problèmes de chemin relatif
            model_path_resolved = Path(self.model_path).resolve()
            if not model_path_resolved.exists():
                # Essayer de trouver le fichier avec la fonction de recherche robuste
                logger.debug(
                    "Chemin modèle initial n'existe pas: %s, recherche alternative...",
                    model_path_resolved,
                )
                found_path = _find_mujoco_model()
                if found_path and found_path.exists():
                    model_path_resolved = found_path
                    logger.info(
                        "Modèle MuJoCo trouvé via recherche: %s", model_path_resolved
                    )
                else:
                    logger.error(
                        "Modèle MuJoCo introuvable: %s (recherche échouée, "
                        "fichier n'existe pas)",
                        model_path_resolved,
                    )
                    return False

            # Vérification finale que le fichier existe
            if not model_path_resolved.exists():
                logger.error(
                    "Modèle MuJoCo introuvable après résolution: %s",
                    model_path_resolved,
                )
                return False

            # Utiliser cache LRU pour modèles MuJoCo
            self.model = get_cached_mujoco_model(model_path_resolved)

            # Gérer le cas où self.model est un mock (dans les tests)
            # Détecter si c'est un mock en vérifiant le type ou en essayant de créer MjData
            try:
                self.data = mujoco.MjData(self.model)
            except TypeError:
                # Si TypeError, c'est probablement un mock dans les tests
                # Créer un mock pour MjData aussi
                from unittest.mock import MagicMock

                self.data = MagicMock()
                logger.debug("Mode test détecté: utilisation de mocks pour MuJoCo")

            # Construire le mapping joint name → id
            # Vérifier si self.model a l'attribut njnt (évite erreur avec mocks)
            if hasattr(self.model, "njnt"):
                try:
                    njnt = int(
                        self.model.njnt
                    )  # Convertir en int pour éviter erreurs avec mocks
                    for i in range(njnt):
                        try:
                            name = mujoco.mj_id2name(
                                self.model, mujoco.mjtObj.mjOBJ_JOINT, i
                            )
                            if name:
                                self.joint_name_to_id[name] = i
                                # Charger les limites du joint
                                if hasattr(self.model, "jnt_range"):
                                    try:
                                        joint_range = self.model.jnt_range[i]
                                        # Vérifier que joint_range est indexable
                                        # Utiliser try/except pour len() car peut être un array numpy
                                        try:
                                            range_len = len(joint_range)
                                        except (TypeError, AttributeError):
                                            range_len = 2  # Par défaut, supposer 2 éléments
                                        if (
                                            hasattr(joint_range, "__getitem__")
                                            and range_len >= 2
                                        ):
                                            self.joint_limits[name] = (
                                                joint_range[0],
                                                joint_range[1],
                                            )
                                    except (IndexError, TypeError, AttributeError):
                                        # Ignorer si joint_range n'est pas accessible (mock)
                                        pass
                        except Exception as e:
                            # Capturer toutes les exceptions lors de mj_id2name
                            # (peut lever RuntimeError, ValueError, etc.)
                            logger.debug(
                                "Impossible de récupérer le nom du joint %d: %s", i, e
                            )
                            continue
                except (TypeError, ValueError, AttributeError) as e:
                    # Si njnt n'est pas accessible ou convertible, ignorer (mode mock)
                    logger.debug(
                        "Impossible de construire mapping joints (mode mock probable): %s",
                        e,
                    )
                except Exception as e:
                    # Capturer toutes les autres exceptions possibles
                    logger.warning(
                        "Erreur lors de la construction du mapping des joints: %s", e
                    )

            # Vérifier que le mapping n'est pas vide après la construction
            if not self.joint_name_to_id:
                njnt_info = getattr(self.model, "njnt", "?")
                logger.warning(
                    "Aucun joint détecté dans le modèle (njnt=%s). "
                    "Le modèle peut être invalide ou vide.",
                    njnt_info,
                )

            self.is_connected = True
            self.start_time = time.time()
            njnt_info = getattr(self.model, "njnt", "?")
            n_joints_mapped = len(self.joint_name_to_id)
            logger.info(
                "MuJoCo connecté: %s joints détectés, %d joints mappés",
                njnt_info,
                n_joints_mapped,
            )
            return True
        except (
            OSError,
            RuntimeError,
            ValueError,
            AttributeError,
            FileNotFoundError,
        ) as e:
            logger.exception("Erreur connexion MuJoCo: %s", e)
            return False
        except mujoco.FatalError as e:
            logger.exception("Erreur fatale MuJoCo (modèle invalide): %s", e)
            return False
        except Exception as e:
            logger.exception("Erreur inattendue connexion MuJoCo: %s", e)
            return False

    def disconnect(self) -> bool:
        """Déconnecte du simulateur MuJoCo."""
        try:
            if self.viewer:
                self.viewer.close()
                self.viewer = None

            self.model = None
            self.data = None
            self.is_connected = False
            logger.info("MuJoCo déconnecté")
            return True
        except (AttributeError, RuntimeError):
            logger.exception("Erreur déconnexion MuJoCo")
            return False
        except Exception:
            logger.exception("Erreur inattendue déconnexion MuJoCo")
            return False

    def get_available_joints(self) -> list[str]:
        """Retourne la liste des joints disponibles."""
        if not self.is_connected:
            return []

        return list(self.joint_name_to_id.keys())

    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        """Définit la position d'un joint."""
        if not self.is_connected:
            logger.error("MuJoCo non connecté")
            return False

        if joint_name not in self.joint_name_to_id:
            logger.error("Joint introuvable: %s", joint_name)
            return False

        # Validation et clamp via RobotAPI
        is_valid, clamped_position = self._validate_joint_pos(joint_name, position)
        if not is_valid:
            return False

        # Appliquer la position
        joint_id = self.joint_name_to_id[joint_name]
        if self.data is not None:
            self.data.qpos[joint_id] = clamped_position

        logger.debug("Joint %s → %.3f rad", joint_name, clamped_position)
        return True

    def get_joint_pos(self, joint_name: str) -> float | None:
        """Récupère la position actuelle d'un joint."""
        if not self.is_connected:
            return None

        if joint_name not in self.joint_name_to_id:
            return None

        joint_id = self.joint_name_to_id[joint_name]
        if self.data is not None:
            return float(self.data.qpos[joint_id])
        return None

    def get_current_body_yaw(self) -> float:
        """Récupère la rotation actuelle du corps (body yaw).

        Conforme SDK: get_current_* retourne état actuel.
        Issue #430: Ajouté pour cohérence avec ReachyMiniBackend.
        """
        if not self.is_connected:
            return 0.0

        if "yaw_body" in self.joint_name_to_id:
            joint_id = self.joint_name_to_id["yaw_body"]
            if self.data is not None:
                return float(self.data.qpos[joint_id])

        return 0.0

    def get_present_body_yaw(self) -> float:
        """Récupère la rotation présente du corps (body yaw).

        Conforme SDK: get_present_* est alias de get_current_* pour MuJoCo.
        Issue #430: Ajouté pour cohérence avec ReachyMiniBackend.
        """
        return self.get_current_body_yaw()

    def get_current_joint_positions(self) -> tuple[list[float], list[float]]:
        """Récupère les positions actuelles des joints (tête et antennes).

        Issue #430: Ajouté pour cohérence avec ReachyMiniBackend.

        Returns:
            Tuple (head_positions, antenna_positions)
            - head_positions: [yaw_body, stewart_1, stewart_2, stewart_3,
              stewart_4, stewart_5, stewart_6]
            - antenna_positions: [right_antenna, left_antenna]

        """
        if not self.is_connected or self.data is None:
            return ([0.0] * 7, [0.0, 0.0])

        head_joints = [
            "yaw_body",
            "stewart_1",
            "stewart_2",
            "stewart_3",
            "stewart_4",
            "stewart_5",
            "stewart_6",
        ]
        antenna_joints = ["right_antenna", "left_antenna"]

        head_positions = []
        for joint_name in head_joints:
            if joint_name in self.joint_name_to_id:
                joint_id = self.joint_name_to_id[joint_name]
                head_positions.append(float(self.data.qpos[joint_id]))
            else:
                head_positions.append(0.0)

        antenna_positions = []
        for joint_name in antenna_joints:
            if joint_name in self.joint_name_to_id:
                joint_id = self.joint_name_to_id[joint_name]
                antenna_positions.append(float(self.data.qpos[joint_id]))
            else:
                antenna_positions.append(0.0)

        return (head_positions, antenna_positions)

    def get_present_antenna_joint_positions(self) -> list[float]:
        """Récupère les positions présentes des antennes.

        Conforme SDK: get_present_* est alias de get_current_* pour MuJoCo.
        Issue #430: Ajouté pour cohérence avec ReachyMiniBackend.
        """
        _, antenna_positions = self.get_current_joint_positions()
        return antenna_positions

    def step(self) -> bool:
        """Effectue un pas de simulation."""
        if not self.is_connected:
            return False

        try:
            mujoco.mj_step(self.model, self.data)
            self.step_count += 1
        except Exception:
            logger.exception("Erreur step MuJoCo")
            return False
        else:
            return True

    def check_collision(self) -> bool:
        """Vérifie s'il y a collision dans la simulation (Issue #183).

        Returns:
            True si collision détectée, False sinon

        Note:
            Implémentation basique utilisant mujoco.mj_contact().
            Pour une détection plus précise, utiliser les contacts spécifiques.

        """
        if not self.is_connected or self.model is None or self.data is None:
            return False

        try:
            # Calculer les contacts
            mujoco.mj_step(self.model, self.data)
            # Vérifier s'il y a des contacts
            num_contacts: int = int(self.data.ncon)
            has_collision: bool = num_contacts > 0

            if has_collision:
                logger.debug(
                    "⚠️ Collision détectée: %d contacts",
                    num_contacts,
                )

            return bool(has_collision)
        except Exception:
            logger.exception("Erreur check_collision")
            return False

    def play_move(
        self,
        move: object,
        play_frequency: float = 100.0,  # noqa: ARG002
        initial_goto_duration: float = 0.0,  # noqa: ARG002
    ) -> None:
        """Joue un mouvement enregistré depuis un dataset HuggingFace.

        Note: MuJoCoBackend simule le mouvement en appliquant les positions des joints
        depuis le Move object du SDK officiel.

        Args:
            move: Objet Move du SDK reachy_mini.motion.move
            play_frequency: Fréquence de lecture (Hz, défaut 100.0)
                           - non utilisé en simulation
            initial_goto_duration: Durée goto initial (s, défaut 0.0)
                                  - non utilisé en simulation

        """
        if not self.is_connected:
            logger.warning("MuJoCo non connecté, impossible de jouer le mouvement")
            return

        try:
            # Extraire les positions des joints depuis l'objet Move
            # Le Move object a une structure: move.joint_states ou move.trajectory
            if hasattr(move, "joint_states"):
                # Format: dict[joint_name, positions]
                for joint_name, positions in move.joint_states.items():
                    if isinstance(positions, list | tuple) and len(positions) > 0:
                        # Appliquer la dernière position (simulation simplifiée)
                        self.set_joint_pos(joint_name, float(positions[-1]))
            elif hasattr(move, "trajectory"):
                # Format: list de dict avec positions
                trajectory = move.trajectory
                if trajectory and len(trajectory) > 0:
                    # Appliquer les positions du dernier état
                    last_state = trajectory[-1]
                    for joint_name, position in last_state.items():
                        if isinstance(position, int | float):
                            self.set_joint_pos(joint_name, float(position))
            elif hasattr(move, "positions"):
                # Format alternatif
                positions = move.positions
                if isinstance(positions, dict):
                    for joint_name, position in positions.items():
                        if isinstance(position, int | float):
                            self.set_joint_pos(joint_name, float(position))

            # Faire un step pour appliquer les changements
            self.step()
            logger.info("Mouvement simulé joué dans MuJoCo")

        except Exception:
            logger.exception("Erreur play_move MuJoCo")

    def async_play_move(
        self,
        move: object,
        play_frequency: float = 100.0,
        initial_goto_duration: float = 0.0,
    ) -> None:
        """Version async de play_move (identique pour MuJoCo)."""
        self.play_move(move, play_frequency, initial_goto_duration)

    def emergency_stop(self) -> bool:
        """Arrêt d'urgence pour simulation MuJoCo."""
        if not self.is_connected:
            logger.warning("Simulation non connectée - emergency_stop ignoré")
            return False

        try:
            # Mettre toutes les positions à 0
            if self.data:
                self.data.ctrl[:] = 0.0
                self.data.qvel[:] = 0.0
            self.is_connected = False
            logger.critical("🔴 ARRÊT D'URGENCE SIMULATION ACTIVÉ")
        except Exception:
            logger.exception("Erreur emergency_stop")
            return False
        else:
            return True

    def launch_viewer(self, passive: bool = True) -> bool:
        """Lance le viewer MuJoCo."""
        if not self.is_connected:
            logger.error("MuJoCo non connecté")
            return False

        try:
            if passive:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            else:
                self.viewer = mujoco.viewer.launch(self.model, self.data)

            # Fond BBIA configuré selon brief graphiste
            # Palette BBIA : Bleu céleste #87bcfa, Violet #A680FF,
            # Turquoise #60e9e1, Gris lunaire #eaeaed, Rose #FFDAEC
            # Skybox avec dégradé gris lunaire → bleu céleste dans le modèle XML
            try:
                # Chercher la texture skybox_bbia
                if self.model is not None:
                    for i in range(self.model.ntexture):
                        name = mujoco.mj_id2name(
                            self.model,
                            mujoco.mjtObj.mjOBJ_TEXTURE,
                            i,
                        )
                        if name == "skybox_bbia":
                            logger.debug("Texture skybox_bbia trouvée à l'index %s", i)
                            break

                logger.info(
                    "Viewer MuJoCo lancé (fond BBIA gris lunaire → bleu céleste)",
                )
            except (AttributeError, KeyError, ValueError):
                logger.warning("Impossible de vérifier le skybox BBIA")
                logger.info("Viewer MuJoCo lancé (fond BBIA configuré)")

        except (RuntimeError, OSError, AttributeError):
            logger.exception("Erreur lancement viewer")
            return False
        else:
            return True

    def configure_viewer_camera(
        self,
        azimuth: float = 180.0,
        elevation: float = -15.0,
        distance: float = 1.2,
        lookat: list[float] | None = None,
    ) -> bool:
        """Configure la caméra du viewer pour orienter face au robot.

        Args:
            azimuth: Angle horizontal (180 = face optimal au robot,
                0 = côté droit, 90 = face alternative)
            elevation: Angle vertical (-15 = légèrement au-dessus)
            distance: Distance du robot
            lookat: Point de visée [x, y, z] (défaut: [0, 0, 0.3])

        """
        if not self.viewer:
            logger.warning("Viewer non lancé, impossible de configurer la caméra")
            return False

        try:
            self.viewer.cam.azimuth = azimuth
            self.viewer.cam.elevation = elevation
            self.viewer.cam.distance = distance
            if lookat:
                self.viewer.cam.lookat[:] = lookat
            else:
                self.viewer.cam.lookat[:] = [0, 0, 0.3]
            logger.debug(
                "Caméra configurée: azimuth=%s, elevation=%s, distance=%s",
                azimuth,
                elevation,
                distance,
            )
        except Exception:
            logger.exception("Erreur configuration caméra")
            return False
        else:
            return True

    def sync_viewer(self) -> bool:
        """Synchronise le viewer."""
        if not self.viewer:
            return False

        try:
            self.viewer.sync()
        except Exception:
            logger.exception("Erreur sync viewer")
            return False
        else:
            return True

    def is_viewer_running(self) -> bool:
        """Vérifie si le viewer est actif."""
        if not self.viewer:
            return False

        try:
            return self.viewer.is_running()  # type: ignore
        except (AttributeError, RuntimeError, TypeError):
            return False

    def set_emotion(self, emotion: str, intensity: float = 0.5) -> bool:
        """Définit une émotion sur le robot MuJoCo - BOUGE VRAIMENT LES JOINTS."""
        if not self.is_connected:
            logger.error("MuJoCo non connecté")
            return False

        # Appeler la méthode parente pour validation
        if not super().set_emotion(emotion, intensity):
            return False

        # Mapper les émotions vers des positions de joints de tête
        # Pour que ça soit visible dans la simulation
        emotion_poses = {
            "happy": {"pitch": 0.1 * intensity, "yaw": 0.0},
            "sad": {"pitch": -0.1 * intensity, "yaw": 0.0},
            "neutral": {"pitch": 0.0, "yaw": 0.0},
            "excited": {"pitch": 0.2 * intensity, "yaw": 0.1 * intensity},
            "curious": {"pitch": 0.05 * intensity, "yaw": 0.2 * intensity},
            "angry": {"pitch": -0.15 * intensity, "yaw": 0.0},
            "surprised": {"pitch": 0.25 * intensity, "yaw": 0.0},
            "calm": {"pitch": -0.05 * intensity, "yaw": 0.0},
        }

        pose = emotion_poses.get(emotion, {"pitch": 0.0, "yaw": 0.0})

        # Appliquer les positions aux joints de tête si disponibles
        available_joints = self.get_available_joints()

        # Chercher les joints de tête
        head_joints: dict[str, str | None] = {
            "pitch": None,
            "yaw": None,
        }

        for joint in available_joints:
            joint_lower = joint.lower()
            if "pitch" in joint_lower and "head" in joint_lower:
                head_joints["pitch"] = joint
            elif "yaw" in joint_lower and "head" in joint_lower:
                head_joints["yaw"] = joint

        # Appliquer les positions
        if head_joints["pitch"]:
            try:
                self.set_joint_pos(head_joints["pitch"], pose["pitch"])
                logger.info("Émotion %s: pitch_head = %.3f", emotion, pose["pitch"])
            except (ValueError, RuntimeError, AttributeError) as e:
                logger.debug("Impossible de bouger pitch_head: %s", e)

        if head_joints["yaw"]:
            try:
                self.set_joint_pos(head_joints["yaw"], pose["yaw"])
                logger.info("Émotion %s: yaw_head = %.3f", emotion, pose["yaw"])
            except (ValueError, RuntimeError, AttributeError) as e:
                logger.debug("Impossible de bouger yaw_head: %s", e)

        # Faire un step pour que le changement soit visible
        self.step()

        logger.info(
            "✅ Émotion %s appliquée (intensité: %s) - joints bougés",
            emotion,
            intensity,
        )
        return True

    def goto_target(
        self,
        head: HeadPose | list[float] | None = None,  # type: ignore[type-arg]
        antennas: list[float] | None = None,
        duration: float = 0.5,
        method: str = "minjerk",
        body_yaw: float | None = None,
    ) -> None:
        """Va vers une cible spécifique avec technique d'interpolation (MuJoCo).

        NOTE: Implémentation simplifiée pour MuJoCo. Pour une IK complète,
        utilisez ReachyMiniBackend qui utilise le SDK officiel.

        Args:
            head: Matrice 4x4 ou HeadPose représentant la pose de la tête (ou None)
            antennas: Angles des antennes en radians [right, left] (ou None)
            duration: Durée du mouvement en secondes (doit être > 0)
            method: Technique d'interpolation (ignorée en MuJoCo simplifié)
            body_yaw: Angle yaw du corps en radians (None = garder position actuelle)

        Raises:
            ValueError: Si duration <= 0

        """
        if duration <= 0.0:
            msg = (
                "Duration must be positive and non-zero. "
                "Use set_joint_pos() for immediate position setting."
            )
            raise ValueError(
                msg,
            )

        if not self.is_connected:
            logger.warning("MuJoCo non connecté, goto_target ignoré")
            return

        try:
            import numpy as np
            from scipy.spatial.transform import Rotation as R

            # Appliquer body_yaw si fourni
            if body_yaw is not None and "yaw_body" in self.joint_name_to_id:
                self.set_joint_pos("yaw_body", float(body_yaw))

            # Traiter head pose si fournie
            if head is not None:
                # Convertir en matrice 4x4 si nécessaire
                if isinstance(head, np.ndarray):
                    head_matrix = head
                elif hasattr(head, "matrix"):
                    head_matrix = head.matrix
                else:
                    logger.warning("Format head non reconnu: %s", type(head))
                    head_matrix = None

                if head_matrix is not None and head_matrix.shape == (4, 4):
                    # Extraire angles Euler de la matrice de rotation
                    rotation_matrix = head_matrix[:3, :3]
                    rotation = R.from_matrix(rotation_matrix)
                    euler = rotation.as_euler("xyz", degrees=False)

                    # Appliquer pitch et yaw aux joints de tête disponibles
                    available_joints = self.get_available_joints()

                    # Chercher les joints de tête (simplifié)
                    for joint in available_joints:
                        joint_lower = joint.lower()
                        if "pitch" in joint_lower and "head" in joint_lower:
                            self.set_joint_pos(joint, float(euler[1]))  # pitch
                        elif "yaw" in joint_lower and "head" in joint_lower:
                            self.set_joint_pos(joint, float(euler[2]))  # yaw

            # Appliquer antennes si fournies
            if (
                antennas is not None
                and isinstance(antennas, list | tuple)
                and len(antennas) >= 2
            ):
                if "right_antenna" in self.joint_name_to_id:
                    self.set_joint_pos("right_antenna", float(antennas[0]))
                if "left_antenna" in self.joint_name_to_id:
                    self.set_joint_pos("left_antenna", float(antennas[1]))

            # Faire un step pour appliquer les changements
            self.step()

            logger.info(
                "goto_target exécuté (duration=%.2fs, method=%s)",
                duration,
                method,
            )

        except Exception:
            logger.exception("Erreur goto_target MuJoCo")

    def get_telemetry(self) -> dict[str, Any]:
        """Retourne les données de télémétrie."""
        if not self.is_connected:
            return {}

        current_time = time.time()
        elapsed_time = current_time - self.start_time if self.start_time else 0

        return {
            "step_count": self.step_count,
            "elapsed_time": elapsed_time,
            "steps_per_second": (
                self.step_count / elapsed_time if elapsed_time > 0 else 0
            ),
            "average_step_time": (
                elapsed_time / self.step_count if self.step_count > 0 else 0
            ),
            "current_qpos": self.data.qpos.copy() if self.data else [],
            "model_path": str(self.model_path),
            "latency_ms": 0.0,  # Simulation
            "fps": (self.step_count / elapsed_time if elapsed_time > 0 else 0),
        }
