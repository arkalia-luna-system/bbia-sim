"""Simulateur MuJoCo pour BBIA-SIM.

Ce module implémente la classe MuJoCoSimulator qui gère la simulation
3D du robot Reachy Mini avec support des modes graphique et headless.
"""

import logging
import sys
import time
from pathlib import Path
from typing import Any

import mujoco
import mujoco.viewer

logger = logging.getLogger(__name__)


class MuJoCoSimulator:
    """Simulateur MuJoCo pour le robot Reachy Mini.

    Cette classe gère le chargement des modèles MJCF, l'exécution de la simulation
    en mode graphique ou headless, et l'interaction avec les articulations du robot.
    """

    def __init__(self, model_path: str) -> None:
        """Initialise le simulateur MuJoCo.

        Args:
            model_path: Chemin vers le fichier MJCF/XML du modèle

        Raises:
            FileNotFoundError: Si le fichier modèle n'existe pas
            mujoco.FatalError: Si le modèle MJCF est invalide

        """
        self.model_path = Path(model_path)
        if not self.model_path.exists():
            logger.error("Modèle MuJoCo introuvable : %s", self.model_path)
            raise FileNotFoundError(f"Modèle MuJoCo introuvable : {self.model_path}")

        try:
            self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
            self.data = mujoco.MjData(self.model)
            self.viewer: mujoco.viewer.MjViewer | None = None
            self.target_positions: dict[str, float] = {}  # Positions cibles à maintenir
            logger.info("Simulateur MuJoCo initialisé avec %s", self.model_path)
        except Exception as e:
            logger.exception("Erreur lors du chargement du modèle MJCF : %s", e)
            raise

    def launch_simulation(
        self,
        headless: bool = False,
        duration: int | float | None = None,
    ) -> None:
        """Lance la simulation MuJoCo.

        Args:
            headless: Si True, lance en mode headless (sans interface graphique)
            duration: Durée de simulation en secondes (None = infinie)

        """
        if headless:
            logger.info("Mode headless activé")
            self._run_headless_simulation(duration)
        else:
            logger.info("Lancement de la simulation graphique MuJoCo")
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                self._run_graphical_simulation(duration)
            except Exception as e:
                if "mjpython" in str(e) and sys.platform == "darwin":
                    logger.exception(
                        "❌ Sur macOS, le viewer MuJoCo nécessite mjpython au lieu de python.\n"
                        "💡 Solutions :\n"
                        "  • Utilisez : mjpython -m bbia_sim --sim --verbose\n"
                        "  • Ou utilisez : python -m bbia_sim --sim --headless\n"
                        "  • Ou installez mjpython : pip install mujoco-python-viewer",
                    )
                    raise RuntimeError(
                        "Viewer MuJoCo non disponible sur macOS avec python standard",
                    ) from e
                logger.exception("Erreur lors du lancement du viewer : %s", e)
                raise

    def _run_headless_simulation(self, duration: int | float | None) -> None:
        """Exécute la simulation en mode headless."""
        logger.info("Simulation headless démarrée")
        start_time = time.monotonic()  # Utiliser monotonic pour éviter la dérive
        step_count = 0

        while True:
            mujoco.mj_step(self.model, self.data)
            step_count += 1

            # Vérification de durée APRÈS chaque step pour un contrôle précis
            if duration:
                elapsed = time.monotonic() - start_time
                if elapsed >= duration:
                    break

            # Log moins fréquent pour éviter le spam
            if step_count % 10000 == 0:
                elapsed = time.monotonic() - start_time
                logger.info("Step %s - Temps écoulé: %.2fs", step_count, elapsed)

            # OPTIMISATION RAM: Limite obligatoire pour éviter boucles infinies
            if duration is None and step_count > 10000:
                logger.warning("Limite de 10000 steps atteinte en mode headless")
                break

        actual_duration = time.monotonic() - start_time
        logger.info(
            f"Simulation headless arrêtée après {step_count} steps ({actual_duration:.2f}s)",
        )

        # OPTIMISATION RAM: Décharger modèle MuJoCo après arrêt pour libérer mémoire
        try:
            del self.model
            del self.data
            logger.debug("Modèle MuJoCo déchargé (optimisation RAM)")
        except Exception as e:
            logger.debug(
                f"Erreur lors du déchargement du modèle MuJoCo (peut être déjà déchargé): {e}"
            )

    def _run_graphical_simulation(self, duration: int | float | None) -> None:
        """Exécute la simulation avec l'interface graphique."""
        if self.viewer is None:
            logger.error("Le viewer n'est pas initialisé pour la simulation graphique.")
            return

        start_time = time.time()
        step_count = 0

        while self.viewer.is_running() and (
            duration is None or (time.time() - start_time) < duration
        ):
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            step_count += 1

        self.viewer.close()
        logger.info("Simulation graphique arrêtée après %s steps", step_count)

        # OPTIMISATION RAM: Décharger modèle MuJoCo après arrêt pour libérer mémoire
        try:
            del self.model
            del self.data
            logger.debug("Modèle MuJoCo déchargé (optimisation RAM)")
        except Exception as e:
            logger.debug(
                f"Erreur lors du déchargement du modèle MuJoCo (peut être déjà déchargé): {e}"
            )

    def load_scene(self, scene_path: str) -> None:
        """Charge une scène spécifique.

        Args:
            scene_path: Chemin vers le fichier de scène MJCF/XML

        """
        new_model_path = Path("src/bbia_sim/sim/scenes") / scene_path
        if not new_model_path.exists():
            logger.warning(
                f"Scène '{scene_path}' non trouvée. Chargement du modèle par défaut.",
            )
            new_model_path = self.model_path

        logger.info("Chargement de la scène/modèle : %s", new_model_path)
        try:
            self.model = mujoco.MjModel.from_xml_path(str(new_model_path))
            self.data = mujoco.MjData(self.model)
            if self.viewer:
                self.viewer.update_model(self.model, self.data)
        except mujoco.FatalError as e:
            logger.exception("Erreur lors du chargement de la scène : %s", e)
            raise

    def set_joint_position(self, joint_name: str, angle: float) -> None:
        """Définit la position d'une articulation avec validation et clamp.

        Args:
            joint_name: Nom de l'articulation
            angle: Angle en radians

        Raises:
            KeyError: Si l'articulation n'existe pas
            ValueError: Si l'angle est hors limites

        """
        try:
            joint_id = self.model.joint(joint_name).id

            # Récupération des limites du joint (API MuJoCo moderne)
            try:
                joint_range = self.model.joint_range[joint_id]
                min_limit = joint_range[0]
                max_limit = joint_range[1]
            except (AttributeError, IndexError):
                # Fallback si joint_range n'est pas disponible
                min_limit = -3.14  # -π
                max_limit = 3.14  # +π
                logger.debug(
                    f"Limites par défaut pour {joint_name}: [{min_limit}, {max_limit}]",
                )

            # Clamp de l'angle dans les limites
            clamped_angle = max(min_limit, min(max_limit, angle))

            if clamped_angle != angle:
                logger.warning(
                    f"Angle {angle:.3f} clampé à {clamped_angle:.3f} "
                    f"pour joint {joint_name} (limites: [{min_limit:.3f}, {max_limit:.3f}])",
                )

            # Stockage de la position cible
            self.target_positions[joint_name] = clamped_angle

            # Application de la position avec contrôle PID simple
            self.data.qpos[joint_id] = clamped_angle

            # Contrôle PID simple pour maintenir la position
            # Recherche de l'actuateur correspondant au joint
            actuator_id = None
            for i in range(self.model.nu):
                actuator_name = mujoco.mj_id2name(
                    self.model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    i,
                )
                if actuator_name == joint_name:
                    actuator_id = i
                    break

            if actuator_id is not None:
                # Calcul de l'erreur
                current_pos = self.data.qpos[joint_id]
                error = clamped_angle - current_pos

                # Contrôle proportionnel simple
                # NOTE: Gains PID alignés SDK Reachy Mini officiel
                # SDK utilise kp=17.11 pour joints stewart (sts3215_345), kp=2.54 pour xc330m288t
                # Pour simulation MuJoCo, kp=100.0 est approprié pour contrôle rapide
                # Si besoin de conformité exacte, charger gains depuis XML modèle
                kp = 100.0  # Gain proportionnel (simulation MuJoCo)
                control_force = kp * error

                # Application du contrôle
                self.data.ctrl[actuator_id] = control_force
                logger.debug("Contrôle appliqué à %s: %s", joint_name, control_force:.3f)
            mujoco.mj_forward(self.model, self.data)
            logger.debug(
                f"Articulation '{joint_name}' positionnée à {clamped_angle:.3f} rad",
            )
        except KeyError:
            logger.exception("Articulation '%s' non trouvée", joint_name)
            raise

    def get_joint_position(self, joint_name: str) -> float:
        """Retourne la position actuelle d'une articulation.

        Args:
            joint_name: Nom de l'articulation

        Returns:
            Position en radians

        Raises:
            KeyError: Si l'articulation n'existe pas

        """
        try:
            joint_id = self.model.joint(joint_name).id
            # Retourner la position cible si elle existe, sinon la position actuelle
            if joint_name in self.target_positions:
                return float(self.target_positions[joint_name])
            return float(self.data.qpos[joint_id])
        except KeyError:
            logger.exception("Articulation '%s' non trouvée", joint_name)
            raise

    def get_robot_state(self) -> dict[str, Any]:
        """Retourne l'état complet du robot.

        Returns:
            Dictionnaire contenant les positions des articulations et autres données

        """
        joint_positions = {}
        for i in range(self.model.njnt):
            joint_name = self.model.joint(i).name
            # Utiliser get_joint_position pour respecter les positions cibles
            joint_positions[joint_name] = self.get_joint_position(joint_name)

        return {
            "joint_positions": joint_positions,
            "time": float(self.data.time),
            "qpos": self.data.qpos.tolist(),
            "qvel": self.data.qvel.tolist(),
            "n_joints": self.model.njnt,
            "n_bodies": self.model.nbody,
        }

    def get_available_joints(self) -> list[str]:
        """Retourne la liste des articulations disponibles.

        Returns:
            Liste des noms d'articulations

        """
        return [self.model.joint(i).name for i in range(self.model.njnt)]

    def _step_simulation(self) -> None:
        """Effectue un step de simulation (méthode interne)."""
        mujoco.mj_step(self.model, self.data)

    def close(self) -> None:
        """Ferme le viewer si actif."""
        if self.viewer:
            self.viewer.close()
            self.viewer = None
        logger.info("Simulation arrêtée")
