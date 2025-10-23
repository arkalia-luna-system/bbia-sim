#!/usr/bin/env python3
"""
Visualisation des mouvements BBIA en temps réel
Alternative à la fenêtre 3D pour voir les mouvements du robot
"""

import argparse
import asyncio
import time
from datetime import datetime

import httpx


class BBIAVisualizer:
    """Visualiseur des mouvements BBIA en temps réel."""

    def __init__(self, api_url: str, token: str):
        self.api_url = api_url
        self.token = token
        self.headers = {"Authorization": f"Bearer {token}"}

    async def get_joint_state(self) -> dict:
        """Récupère l'état actuel des articulations."""
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"{self.api_url}/api/state/joints", headers=self.headers
            )
            response.raise_for_status()
            return response.json()

    async def set_joint_position(self, joint_name: str, position: float) -> bool:
        """Définit la position d'une articulation."""
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.api_url}/api/motion/joints",
                headers=self.headers,
                json=[{"joint_name": joint_name, "position": position}],
            )
            return response.status_code == 200

    def display_joint_state(self, state: dict):
        """Affiche l'état des articulations de manière visuelle."""
        print("\n" + "=" * 80)
        print(f"🤖 ÉTAT BBIA - {datetime.now().strftime('%H:%M:%S')}")
        print("=" * 80)

        joints = state.get("joints", {})

        # Affichage des articulations principales
        main_joints = [
            "neck_yaw",
            "right_shoulder_pitch",
            "right_elbow_pitch",
            "left_shoulder_pitch",
            "left_elbow_pitch",
        ]

        for joint in main_joints:
            if joint in joints:
                pos = joints[joint]["position"]
                # Créer une barre visuelle
                bar_length = 20
                normalized_pos = (pos + 1.57) / 3.14  # Normaliser entre 0 et 1
                filled_length = int(normalized_pos * bar_length)
                bar = "█" * filled_length + "░" * (bar_length - filled_length)

                print(f"{joint:20} |{bar}| {pos:6.3f} rad")

        print("=" * 80)

    async def demo_wave_movement(self):
        """Démo de mouvement de salutation avec visualisation."""
        print("\n🎭 DÉMO SALUTATION AVEC VISUALISATION")
        print("=" * 50)

        # Séquence de mouvements pour le salut
        movements = [
            (
                "Position initiale",
                {"right_shoulder_pitch": -0.3, "right_elbow_pitch": 0.0},
            ),
            (
                "Lever le bras",
                {"right_shoulder_pitch": -0.8, "right_elbow_pitch": -0.5},
            ),
            (
                "Mouvement salut",
                {"right_shoulder_pitch": -0.6, "right_elbow_pitch": -0.3},
            ),
            (
                "Retour initial",
                {"right_shoulder_pitch": -0.3, "right_elbow_pitch": 0.0},
            ),
        ]

        for step_name, positions in movements:
            print(f"\n📝 {step_name}")

            # Appliquer les positions
            for joint, pos in positions.items():
                await self.set_joint_position(joint, pos)
                await asyncio.sleep(0.1)  # Petit délai pour voir le mouvement

            # Afficher l'état
            state = await self.get_joint_state()
            self.display_joint_state(state)
            await asyncio.sleep(1.0)  # Pause pour observer

    async def demo_head_movement(self):
        """Démo de mouvement de tête avec visualisation."""
        print("\n🎭 DÉMO MOUVEMENT DE TÊTE")
        print("=" * 50)

        # Mouvement oscillant de la tête
        positions = [0.0, 0.4, -0.4, 0.2, -0.2, 0.0]

        for i, pos in enumerate(positions):
            print(f"\n📝 Étape {i+1}/6 - Position tête: {pos:.1f} rad")

            await self.set_joint_position("neck_yaw", pos)
            await asyncio.sleep(0.2)

            # Afficher l'état
            state = await self.get_joint_state()
            self.display_joint_state(state)
            await asyncio.sleep(0.8)

    async def real_time_monitoring(self, duration: int = 10):
        """Surveillance en temps réel des mouvements."""
        print(f"\n📊 SURVEILLANCE TEMPS RÉEL ({duration}s)")
        print("=" * 50)

        start_time = time.time()
        while time.time() - start_time < duration:
            state = await self.get_joint_state()
            self.display_joint_state(state)
            await asyncio.sleep(0.5)  # Mise à jour toutes les 500ms

    async def run(self, demo_type: str = "wave", duration: int = 10):
        """Lance la démonstration choisie."""
        try:
            # Vérifier la connexion
            print("🔍 Vérification de la connexion API...")
            await self.get_joint_state()
            print("✅ API BBIA-SIM accessible")

            if demo_type == "wave":
                await self.demo_wave_movement()
            elif demo_type == "head":
                await self.demo_head_movement()
            elif demo_type == "monitor":
                await self.real_time_monitoring(duration)
            else:
                print(f"❌ Type de démo inconnu: {demo_type}")
                return

            print("\n🎉 Démonstration terminée!")

        except Exception as e:
            print(f"❌ Erreur: {e}")


async def main():
    parser = argparse.ArgumentParser(description="Visualiseur BBIA")
    parser.add_argument(
        "--api-url", default="http://localhost:8000", help="URL de l'API"
    )
    parser.add_argument(
        "--token", default="bbia-secret-key-dev", help="Token d'authentification"
    )
    parser.add_argument(
        "--demo",
        choices=["wave", "head", "monitor"],
        default="wave",
        help="Type de démo",
    )
    parser.add_argument("--duration", type=int, default=10, help="Durée en secondes")

    args = parser.parse_args()

    visualizer = BBIAVisualizer(args.api_url, args.token)
    await visualizer.run(args.demo, args.duration)


if __name__ == "__main__":
    asyncio.run(main())
