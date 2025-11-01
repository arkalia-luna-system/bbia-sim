#!/usr/bin/env python3
"""
Demo outils LLM - BBIA-SIM
Démontre l'utilisation des outils LLM pour contrôle robot.

Usage:
    python examples/demo_tools_llm.py
"""

import sys
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_tools import BBIATools
from bbia_sim.bbia_vision import BBIAVision
from bbia_sim.robot_factory import RobotFactory


def main() -> None:
    """Demo outils LLM."""
    print("🤖 Demo Outils LLM - BBIA-SIM")
    print("=" * 50)

    # Créer backend robot (simulation)
    print("\n📡 Connexion robot (simulation)...")
    robot_api = RobotFactory.create_backend("mujoco")
    if not robot_api:
        print("❌ Impossible de créer backend")
        return

    try:
        robot_api.connect()
        print("✅ Robot connecté")

        # Initialiser modules BBIA
        print("\n🔧 Initialisation modules BBIA...")
        vision = BBIAVision(robot_api=robot_api)
        tools = BBIATools(robot_api=robot_api, vision=vision)

        # Afficher outils disponibles
        print("\n📋 Outils disponibles:")
        available_tools = tools.get_tools()
        for tool in available_tools:
            tool_func = tool.get("function", {})
            print(
                f"  - {tool_func.get('name', 'unknown')}: {tool_func.get('description', '')}"
            )

        # Exemple 1: Mouvement tête
        print("\n🎯 Exemple 1: Mouvement tête (gauche)")
        result = tools.execute_tool(
            "move_head", {"direction": "left", "intensity": 0.5}
        )
        print(f"  Résultat: {result}")

        # Exemple 2: Capture caméra
        print("\n📷 Exemple 2: Capture caméra")
        result = tools.execute_tool("camera", {"analyze": True})
        print(f"  Résultat: {result.get('status', 'unknown')}")
        if "summary" in result:
            print(f"  Résumé: {result['summary']}")

        # Exemple 3: Jouer émotion
        print("\n😊 Exemple 3: Jouer émotion")
        result = tools.execute_tool(
            "play_emotion", {"emotion": "happy", "intensity": 0.7}
        )
        print(f"  Résultat: {result}")

        # Exemple 4: Activer/désactiver suivi visage
        print("\n👁️ Exemple 4: Suivi visage")
        result = tools.execute_tool("head_tracking", {"enabled": True})
        print(f"  Résultat: {result}")

        # Afficher structure outils pour LLM
        print("\n📊 Structure outils (format LLM):")
        print(f"  Nombre d'outils: {len(available_tools)}")
        print("  Format JSON Schema prêt pour function calling")

        print("\n✅ Demo terminée")

    except Exception as e:
        print(f"\n❌ Erreur: {e}")
        import traceback

        traceback.print_exc()
    finally:
        if robot_api:
            robot_api.disconnect()
            print("\n🔌 Robot déconnecté")


if __name__ == "__main__":
    main()
