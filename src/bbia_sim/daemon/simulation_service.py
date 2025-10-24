"""Service de simulation MuJoCo pour BBIA-SIM."""

import asyncio
import contextlib
import logging
from typing import Any, Union

from ..sim.simulator import MuJoCoSimulator

logger = logging.getLogger(__name__)


class SimulationService:
    """Service de gestion de la simulation MuJoCo."""

    def __init__(self, model_path: Union[str, None] = None):
        """Initialise le service de simulation.

        Args:
            model_path: Chemin vers le modèle MJCF/XML

        """
        self.model_path = (
            model_path or "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
        )
        self.simulator: Union[MuJoCoSimulator, None] = None
        self.is_running = False
        self._simulation_task: Union[asyncio.Task, None] = None

    async def start_simulation(self, headless: bool = True) -> bool:
        """Démarre la simulation MuJoCo.

        Args:
            headless: Si True, lance en mode headless

        Returns:
            True si la simulation a démarré avec succès

        """
        try:
            if self.is_running:
                logger.warning("La simulation est déjà en cours")
                return True

            logger.info(f"Démarrage de la simulation MuJoCo avec {self.model_path}")
            self.simulator = MuJoCoSimulator(self.model_path)

            if headless:
                # Mode headless asynchrone
                self._simulation_task = asyncio.create_task(
                    self._run_headless_simulation()
                )
            else:
                # Mode graphique (nécessite un thread séparé)
                self._simulation_task = asyncio.create_task(
                    self._run_graphical_simulation()
                )

            self.is_running = True
            logger.info("Simulation MuJoCo démarrée avec succès")
            return True

        except Exception as e:
            logger.error(f"Erreur lors du démarrage de la simulation : {e}")
            return False

    async def stop_simulation(self) -> bool:
        """Arrête la simulation MuJoCo.

        Returns:
            True si la simulation a été arrêtée avec succès

        """
        try:
            if not self.is_running:
                logger.warning("Aucune simulation en cours")
                return True

            logger.info("Arrêt de la simulation MuJoCo")
            self.is_running = False

            if self._simulation_task:
                self._simulation_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await self._simulation_task

            if self.simulator:
                self.simulator.close()
                self.simulator = None

            logger.info("Simulation MuJoCo arrêtée avec succès")
            return True

        except Exception as e:
            logger.error(f"Erreur lors de l'arrêt de la simulation : {e}")
            return False

    async def _run_headless_simulation(self) -> None:
        """Exécute la simulation en mode headless."""
        if not self.simulator:
            return

        logger.info("Simulation headless démarrée")
        step_count = 0

        while self.is_running:
            try:
                # Simulation d'un step MuJoCo
                if self.simulator:
                    # Note: mj_step est synchrone, on l'exécute dans un thread
                    await asyncio.get_event_loop().run_in_executor(
                        None, self.simulator._step_simulation
                    )
                    step_count += 1

                    if step_count % 1000 == 0:
                        logger.debug(f"Step {step_count}")

                # Attente avant le prochain step
                await asyncio.sleep(0.001)  # ~1000 Hz

            except Exception as e:
                logger.error(f"Erreur dans la simulation headless : {e}")
                await asyncio.sleep(0.1)

    async def _run_graphical_simulation(self) -> None:
        """Exécute la simulation en mode graphique."""
        if not self.simulator:
            return

        logger.info("Simulation graphique démarrée")
        try:
            # Pour le mode graphique, on utilise le viewer MuJoCo
            # qui doit s'exécuter dans le thread principal
            await asyncio.get_event_loop().run_in_executor(
                None, self.simulator.launch_simulation, False, None
            )
        except Exception as e:
            logger.error(f"Erreur simulation graphique : {e}")
            # Fallback vers headless si le viewer échoue
            logger.info("Fallback vers simulation headless")
            await self._run_headless_simulation()

    def get_robot_state(self) -> dict[str, Any]:
        """Récupère l'état actuel du robot depuis la simulation.

        Returns:
            État du robot

        """
        if not self.simulator or not self.is_running:
            return self._get_default_state()

        try:
            return self.simulator.get_robot_state()
        except Exception as e:
            logger.error(f"Erreur lors de la récupération de l'état : {e}")
            return self._get_default_state()

    def get_joint_positions(self) -> dict[str, float]:
        """Récupère les positions des articulations.

        Returns:
            Positions des articulations

        """
        if not self.simulator or not self.is_running:
            return self._get_default_joint_positions()

        try:
            state = self.simulator.get_robot_state()
            return state.get("joint_positions", self._get_default_joint_positions())
        except Exception as e:
            logger.error(f"Erreur lors de la récupération des positions : {e}")
            return self._get_default_joint_positions()

    def set_joint_position(self, joint_name: str, position: float) -> bool:
        """Définit la position d'une articulation.

        Args:
            joint_name: Nom de l'articulation
            position: Position en radians

        Returns:
            True si la position a été définie avec succès

        """
        if not self.simulator or not self.is_running:
            logger.warning("Simulation non disponible")
            return False

        try:
            self.simulator.set_joint_position(joint_name, position)
            logger.info(f"Position de {joint_name} définie à {position}")
            return True
        except Exception as e:
            logger.error(f"Erreur lors de la définition de la position : {e}")
            return False

    def get_available_joints(self) -> list[str]:
        """Récupère la liste des articulations disponibles.

        Returns:
            Liste des noms d'articulations

        """
        if not self.simulator:
            return self._get_default_joint_names()

        try:
            return self.simulator.get_available_joints()
        except Exception as e:
            logger.error(f"Erreur lors de la récupération des articulations : {e}")
            return self._get_default_joint_names()

    def _get_default_state(self) -> dict[str, Any]:
        """Retourne un état par défaut."""
        return {
            "joint_positions": self._get_default_joint_positions(),
            "time": 0.0,
            "qpos": [0.0] * 8,
            "qvel": [0.0] * 8,
        }

    def _get_default_joint_positions(self) -> dict[str, float]:
        """Retourne des positions d'articulations par défaut."""
        return {
            "neck_yaw": 0.0,
            "right_shoulder_pitch": 0.0,
            "right_elbow_pitch": 0.0,
            "right_gripper_joint": 0.0,
            "left_shoulder_pitch": 0.0,
            "left_elbow_pitch": 0.0,
            "left_gripper_joint": 0.0,
        }

    def _get_default_joint_names(self) -> list[str]:
        """Retourne les noms d'articulations par défaut."""
        return list(self._get_default_joint_positions().keys())

    def is_simulation_ready(self) -> bool:
        """Vérifie si la simulation est prête."""
        return self.simulator is not None and self.is_running


# Instance globale du service de simulation
simulation_service = SimulationService()
