#!/usr/bin/env python3
"""
Viewer MuJoCo simple - Pour capture d'écran
"""

import sys
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

        # Lancer viewer
        mujoco.viewer.launch(model, data)

    except Exception as e:
        print(f"❌ Erreur: {e}")
        print("\n💡 Essayez avec mjpython au lieu de python3:")
        print("   mjpython examples/viewer_robot.py")


if __name__ == "__main__":
    viewer_simple()
