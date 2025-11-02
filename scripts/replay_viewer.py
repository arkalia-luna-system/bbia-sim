#!/usr/bin/env python3
"""Record & Replay pour BBIA
Enregistrement et rejeu des animations
"""

import json
import time
from pathlib import Path
from typing import Any

import mujoco
import mujoco.viewer


class AnimationRecorder:
    """Enregistreur d'animations BBIA."""

    def __init__(self, record_path: str):
        self.record_path = Path(record_path)
        self.record_path.parent.mkdir(parents=True, exist_ok=True)
        self.records: list[dict[str, Any]] = []
        self.start_time = None
        self.is_recording = False

    def start_recording(self):
        """Démarre l'enregistrement."""
        self.records = []
        self.start_time = time.time()
        self.is_recording = True
        print(f"🎬 Enregistrement démarré: {self.record_path}")

    def record_frame(self, joint_name: str, qpos: float, step: int):
        """Enregistre une frame."""
        if not self.is_recording:
            return

        elapsed = time.time() - self.start_time
        record = {
            "t": elapsed,
            "step": step,
            "joint": joint_name,
            "qpos": qpos,
            "timestamp": time.time(),
        }
        self.records.append(record)

    def stop_recording(self):
        """Arrête l'enregistrement et sauvegarde."""
        if not self.is_recording:
            return None

        self.is_recording = False

        # Sauvegarder en JSONL
        with open(self.record_path, "w") as f:
            f.writelines(json.dumps(record) + "\n" for record in self.records)

        print(
            f"💾 Enregistrement sauvegardé: {len(self.records)} frames → {self.record_path}",
        )
        return len(self.records)


class AnimationReplayer:
    """Rejeu d'animations BBIA."""

    def __init__(self, record_path: str):
        self.record_path = Path(record_path)
        self.records: list[dict[str, Any]] = []
        self.current_frame = 0
        self.start_time = None

    def load_recording(self) -> bool:
        """Charge l'enregistrement."""
        if not self.record_path.exists():
            print(f"❌ Fichier d'enregistrement introuvable: {self.record_path}")
            return False

        self.records = []
        with open(self.record_path) as f:
            for line in f:
                record = json.loads(line.strip())
                self.records.append(record)

        print(f"📁 Enregistrement chargé: {len(self.records)} frames")
        return True

    def replay_with_viewer(self, model_path: str, speed_factor: float = 1.0):
        """Rejoue l'animation avec le viewer MuJoCo."""
        if not self.load_recording():
            return False

        # Charger le modèle MuJoCo
        try:
            model = mujoco.MjModel.from_xml_path(model_path)
            data = mujoco.MjData(model)
            print(f"✅ Modèle chargé: {model.njnt} joints")
        except Exception as e:
            print(f"❌ Erreur chargement modèle: {e}")
            return False

        # Trouver le joint
        joint_id = None
        joint_name = self.records[0]["joint"] if self.records else "yaw_body"

        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name == joint_name:
                joint_id = i
                break

        if joint_id is None:
            print(f"❌ Joint '{joint_name}' introuvable")
            return False

        print(f"🎬 Rejeu de l'animation: {joint_name}")
        print(f"   • Frames: {len(self.records)}")
        print(f"   • Durée: {self.records[-1]['t']:.1f}s")
        print(f"   • Vitesse: {speed_factor}x")

        # Rejeu avec viewer
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                # Configurer la caméra à 180° (face optimal) immédiatement
                viewer.cam.azimuth = 180.0
                viewer.cam.elevation = -15.0
                viewer.cam.distance = 1.2
                viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
                viewer.sync()

                self.start_time = time.time()
                self.current_frame = 0

                while viewer.is_running() and self.current_frame < len(self.records):
                    record = self.records[self.current_frame]

                    # Appliquer la position
                    data.qpos[joint_id] = record["qpos"]
                    mujoco.mj_step(model, data)
                    viewer.sync()

                    # Contrôle de vitesse
                    if self.current_frame < len(self.records) - 1:
                        next_record = self.records[self.current_frame + 1]
                        dt = (next_record["t"] - record["t"]) / speed_factor
                        time.sleep(dt)

                    self.current_frame += 1

                print(f"✅ Rejeu terminé: {self.current_frame} frames")

        except Exception as e:
            print(f"❌ Erreur rejeu: {e}")
            return False

        return True

    def get_recording_info(self) -> dict[str, Any]:
        """Retourne les informations sur l'enregistrement."""
        if not self.records:
            return {}

        return {
            "total_frames": len(self.records),
            "duration": self.records[-1]["t"] if self.records else 0,
            "joint": self.records[0]["joint"] if self.records else None,
            "start_time": self.records[0]["timestamp"] if self.records else None,
            "end_time": self.records[-1]["timestamp"] if self.records else None,
            "file_path": str(self.record_path),
        }


def main():
    """Script de rejeu standalone."""
    import argparse

    parser = argparse.ArgumentParser(description="Rejeu d'animation BBIA")
    parser.add_argument("record_file", help="Fichier d'enregistrement (.jsonl)")
    parser.add_argument(
        "--model",
        default="src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml",
        help="Modèle MuJoCo",
    )
    parser.add_argument("--speed", type=float, default=1.0, help="Facteur de vitesse")

    args = parser.parse_args()

    replayer = AnimationReplayer(args.record_file)

    if replayer.load_recording():
        info = replayer.get_recording_info()
        print("📊 Informations enregistrement:")
        for key, value in info.items():
            print(f"   • {key}: {value}")

        replayer.replay_with_viewer(args.model, args.speed)


if __name__ == "__main__":
    main()
