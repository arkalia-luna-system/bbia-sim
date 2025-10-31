#!/usr/bin/env python3
"""Test pour vérifier que le skybox fonctionne"""

import sys
from pathlib import Path

import mujoco

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

model_path = Path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
print(f"📂 Chargement: {model_path}")

model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)

# Vérifier les paramètres visuels
print("\n🎨 Paramètres visuels du modèle:")
if hasattr(model, "vis"):
    if hasattr(model.vis, "rgba"):
        print(f"   Sky rgba: {model.vis.rgba.sky}")
        print(f"   Fog rgba: {model.vis.rgba.fog}")
        print(f"   Haze rgba: {model.vis.rgba.haze}")

# Vérifier les textures
print("\n🖼️  Textures dans le modèle:")
print(f"   Nombre de textures: {model.ntexture}")
for i in range(model.ntexture):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TEXTURE, i)
    print(f"   Texture {i}: {name}")

# Lancer viewer pour vérifier
print("\n🖥️  Lancement viewer pour vérification...")
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("✅ Viewer ouvert - Vérifiez le fond!")
    print("❌ Fermez la fenêtre pour quitter")

    while viewer.is_running():
        viewer.sync()
        import time

        time.sleep(0.01)
