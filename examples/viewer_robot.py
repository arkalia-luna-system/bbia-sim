#!/usr/bin/env python3
"""
Viewer MuJoCo simple - Pour capture d'écran
"""

import sys
import time
from pathlib import Path

import mujoco

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory


def viewer_simple():
    """Lance le viewer avec le robot."""

    print("=" * 60)
    print("🤖 MUJOCO VIEWER - Capture d'écran")
    print("=" * 60)

    # Charger modèle
    model_path = Path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
    print(f"\n📂 Chargement: {model_path}")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    # Créer robot
    robot = RobotFactory.create_backend("reachy_mini")
    assert robot is not None, "Robot ne peut pas être None"
    robot.connect()

    # Pose pour screenshot
    robot.set_emotion("happy", 0.9)
    robot.set_joint_pos("yaw_body", 0.2)
    robot.step()

    print("\n🖥️  Lancement viewer MuJoCo...")
    print("📸 Faites ⌘ + ⇧ + 4 pour capturer")
    print("❌ Fermez pour quitter")
    print()

    # Essayer avec mujoco.viewer direct
    try:
        # Créer une fonction d'animation
        def animate():
            step = 0
            while True:
                # Petit mouvement
                data.qpos[0] = 0.1 * (step % 100) / 100  # yaw_body

                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)

                step += 1
                yield

        # Lancer viewer et configurer la caméra
        # Note: mujoco.viewer.launch() ouvre une fenêtre interactive
        # Pour configurer la caméra, il faut utiliser launch_passive
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Configurer la caméra à 180° (face optimal) immédiatement
            viewer.cam.azimuth = 180.0
            viewer.cam.elevation = -15.0
            viewer.cam.distance = 1.2  # Rapproché de 20%
            viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
            viewer.sync()

            # Garder le viewer ouvert
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.01)

    except Exception as e:
        print(f"❌ Erreur: {e}")
        print("\n💡 Essayez avec mjpython au lieu de python3:")
        print("   mjpython examples/viewer_robot.py")


if __name__ == "__main__":
    viewer_simple()
