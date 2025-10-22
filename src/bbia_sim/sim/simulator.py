"""
Simulateur MuJoCo pour BBIA-SIM.

Ce module implémente la classe MuJoCoSimulator qui gère la simulation
3D du robot Reachy Mini avec support des modes graphique et headless.
"""

import logging
import time
from pathlib import Path
from typing import Any, Optional

import mujoco
import mujoco.viewer

logger = logging.getLogger(__name__)


class MuJoCoSimulator:
    """
    Simulateur MuJoCo pour le robot Reachy Mini.

    Cette classe gère le chargement des modèles MJCF, l'exécution de la simulation
    en mode graphique ou headless, et l'interaction avec les articulations du robot.
    """

    def __init__(self, model_path: str) -> None:
        """
        Initialise le simulateur MuJoCo.

        Args:
            model_path: Chemin vers le fichier MJCF/XML du modèle

        Raises:
            FileNotFoundError: Si le fichier modèle n'existe pas
            mujoco.FatalError: Si le modèle MJCF est invalide
        """
        self.model_path = Path(model_path)
        if not self.model_path.exists():
            logger.error(f"Modèle MuJoCo introuvable : {self.model_path}")
            raise FileNotFoundError(f"Modèle MuJoCo introuvable : {self.model_path}")

        try:
            self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
            self.data = mujoco.MjData(self.model)
            self.viewer: Optional[mujoco.viewer.MjViewer] = None
            logger.info(f"Simulateur MuJoCo initialisé avec {self.model_path}")
        except mujoco.FatalError as e:
            logger.error(f"Erreur lors du chargement du modèle MJCF : {e}")
            raise

    def launch_simulation(
        self, headless: bool = False, duration: Optional[int] = None
    ) -> None:
        """
        Lance la simulation MuJoCo.

        Args:
            headless: Si True, lance en mode headless (sans interface graphique)
            duration: Durée de simulation en secondes (None = infinie)
        """
        if headless:
            logger.info("Mode headless activé")
            self._run_headless_simulation(duration)
        else:
            logger.info("Lancement de la simulation graphique MuJoCo")
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._run_graphical_simulation(duration)

    def _run_headless_simulation(self, duration: Optional[int]) -> None:
        """Exécute la simulation en mode headless."""
        logger.info("Simulation headless démarrée")
        start_time = time.time()
        step_count = 0

        while True:
            mujoco.mj_step(self.model, self.data)
            step_count += 1

            if step_count % 100 == 0:
                logger.info(f"Step {step_count}")

            if duration and (time.time() - start_time) > duration:
                break

            # Limite de sécurité pour éviter les boucles infinies
            if duration is None and step_count > 10000:
                logger.warning("Limite de 10000 steps atteinte en mode headless")
                break

        logger.info(f"Simulation headless arrêtée après {step_count} steps")

    def _run_graphical_simulation(self, duration: Optional[int]) -> None:
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
        logger.info(f"Simulation graphique arrêtée après {step_count} steps")

    def load_scene(self, scene_path: str) -> None:
        """
        Charge une scène spécifique.

        Args:
            scene_path: Chemin vers le fichier de scène MJCF/XML
        """
        new_model_path = Path("src/bbia_sim/sim/scenes") / scene_path
        if not new_model_path.exists():
            logger.warning(
                f"Scène '{scene_path}' non trouvée. Chargement du modèle par défaut."
            )
            new_model_path = self.model_path

        logger.info(f"Chargement de la scène/modèle : {new_model_path}")
        try:
            self.model = mujoco.MjModel.from_xml_path(str(new_model_path))
            self.data = mujoco.MjData(self.model)
            if self.viewer:
                self.viewer.update_model(self.model, self.data)
        except mujoco.FatalError as e:
            logger.error(f"Erreur lors du chargement de la scène : {e}")
            raise

    def set_joint_position(self, joint_name: str, angle: float) -> None:
        """
        Définit la position d'une articulation.

        Args:
            joint_name: Nom de l'articulation
            angle: Angle en radians

        Raises:
            KeyError: Si l'articulation n'existe pas
        """
        try:
            joint_id = self.model.joint(joint_name).id
            self.data.qpos[joint_id] = angle
            mujoco.mj_forward(self.model, self.data)
            logger.debug(f"Articulation '{joint_name}' positionnée à {angle} rad")
        except KeyError:
            logger.error(f"Articulation '{joint_name}' non trouvée")
            raise

    def get_joint_position(self, joint_name: str) -> float:
        """
        Retourne la position actuelle d'une articulation.

        Args:
            joint_name: Nom de l'articulation

        Returns:
            Position en radians

        Raises:
            KeyError: Si l'articulation n'existe pas
        """
        try:
            joint_id = self.model.joint(joint_name).id
            return float(self.data.qpos[joint_id])
        except KeyError:
            logger.error(f"Articulation '{joint_name}' non trouvée")
            raise

    def get_robot_state(self) -> dict[str, Any]:
        """
        Retourne l'état complet du robot.

        Returns:
            Dictionnaire contenant les positions des articulations et autres données
        """
        joint_positions = {}
        for i in range(self.model.njnt):
            joint_name = self.model.joint(i).name
            joint_positions[joint_name] = float(self.data.qpos[i])

        return {
            "joint_positions": joint_positions,
            "time": float(self.data.time),
            "qpos": self.data.qpos.tolist(),
            "qvel": self.data.qvel.tolist(),
            "n_joints": self.model.njnt,
            "n_bodies": self.model.nbody,
        }

    def get_available_joints(self) -> list[str]:
        """
        Retourne la liste des articulations disponibles.

        Returns:
            Liste des noms d'articulations
        """
        return [self.model.joint(i).name for i in range(self.model.njnt)]

    def close(self) -> None:
        """Ferme le viewer si actif."""
        if self.viewer:
            self.viewer.close()
            self.viewer = None
        logger.info("Simulation arrêtée")
