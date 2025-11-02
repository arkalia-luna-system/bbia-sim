#!/usr/bin/env python3
"""plot_trace.py - Génère des graphiques à partir des traces BBIA
Crée des visualisations des mouvements de joints pour les démos
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Any

# import matplotlib.pyplot as plt
# import numpy as np

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def load_trace(trace_file: str) -> list[dict[str, Any]]:
    """Charge une trace JSONL."""
    trace_data = []

    with open(trace_file, encoding="utf-8") as f:
        for line in f:
            if line.strip():
                try:
                    data = json.loads(line.strip())
                    trace_data.append(data)
                except json.JSONDecodeError:
                    print(f"⚠️ Ligne ignorée (JSON invalide): {line.strip()}")

    return trace_data


def plot_joint_movements(
    trace_data: list[dict[str, Any]],
    output_file: str,
    title: str = "Mouvements joints BBIA",
) -> bool:
    """Génère un rapport textuel des mouvements de joints."""
    if not trace_data:
        print("❌ Aucune donnée dans la trace")
        return False

    # Extraire les données
    times = [d["t"] for d in trace_data]
    joints = [d["joint"] for d in trace_data]
    positions = [d["qpos"] for d in trace_data]

    # Trouver les joints uniques
    unique_joints = list(set(joints))

    # Créer un rapport textuel
    report_file = output_file.replace(".png", ".txt")

    with open(report_file, "w", encoding="utf-8") as f:
        f.write(f"{title}\n")
        f.write("=" * 50 + "\n\n")

        f.write("📊 STATISTIQUES GÉNÉRALES:\n")
        f.write(f"  - Durée totale: {max(times) - min(times):.2f}s\n")
        f.write(f"  - Nombre de mesures: {len(trace_data)}\n")
        f.write(f"  - Joints testés: {len(unique_joints)}\n")
        f.write(
            f"  - Fréquence moyenne: {len(trace_data) / (max(times) - min(times)):.1f} Hz\n\n",
        )

        for joint in unique_joints:
            # Filtrer les données pour ce joint
            joint_positions = [
                p for j, p in zip(joints, positions, strict=False) if j == joint
            ]

            if joint_positions:
                min_pos = min(joint_positions)
                max_pos = max(joint_positions)
                avg_pos = sum(joint_positions) / len(joint_positions)

                f.write(f"🔧 JOINT {joint.upper()}:\n")
                f.write(f"  - Position min: {min_pos:.3f} rad\n")
                f.write(f"  - Position max: {max_pos:.3f} rad\n")
                f.write(f"  - Position moyenne: {avg_pos:.3f} rad\n")
                f.write(f"  - Amplitude: {max_pos - min_pos:.3f} rad\n")

                # Vérifier limites de sécurité
                if max_pos > 0.3 or min_pos < -0.3:
                    f.write("  ⚠️  ATTENTION: Limite sécurité dépassée!\n")
                else:
                    f.write("  ✅ Limites sécurité respectées\n")
                f.write("\n")

    print(f"✅ Rapport sauvegardé: {report_file}")
    return True


def plot_latency_analysis(trace_data: list[dict[str, Any]], output_file: str) -> bool:
    """Génère un rapport d'analyse de latence."""
    if not trace_data:
        print("❌ Aucune donnée dans la trace")
        return False

    # Calculer les intervalles de temps
    times = [d["t"] for d in trace_data]
    dt = [times[i + 1] - times[i] for i in range(len(times) - 1)]

    # Statistiques
    mean_dt = sum(dt) / len(dt) if dt else 0
    min_dt = min(dt) if dt else 0
    max_dt = max(dt) if dt else 0
    fps = 1.0 / mean_dt if mean_dt > 0 else 0

    # Créer un rapport textuel
    report_file = output_file.replace(".png", "_latency.txt")

    with open(report_file, "w", encoding="utf-8") as f:
        f.write("ANALYSE DE LATENCE BBIA\n")
        f.write("=" * 50 + "\n\n")

        f.write("📊 STATISTIQUES TEMPORELLES:\n")
        f.write(f"  - Nombre de mesures: {len(trace_data)}\n")
        f.write(f"  - Durée totale: {max(times) - min(times):.3f}s\n")
        f.write(f"  - Intervalle moyen: {mean_dt * 1000:.1f}ms\n")
        f.write(f"  - Intervalle min: {min_dt * 1000:.1f}ms\n")
        f.write(f"  - Intervalle max: {max_dt * 1000:.1f}ms\n")
        f.write(f"  - FPS moyen: {fps:.1f}\n\n")

        # Analyse de stabilité
        target_fps = 60
        target_dt = 1.0 / target_fps

        f.write("🎯 ANALYSE DE STABILITÉ:\n")
        f.write(f"  - FPS cible: {target_fps}\n")
        f.write(f"  - Intervalle cible: {target_dt * 1000:.1f}ms\n")

        if fps >= target_fps * 0.9:  # 90% du FPS cible
            f.write(f"  ✅ Performance excellente ({fps:.1f} FPS)\n")
        elif fps >= target_fps * 0.7:  # 70% du FPS cible
            f.write(f"  ⚠️  Performance correcte ({fps:.1f} FPS)\n")
        else:
            f.write(f"  ❌ Performance insuffisante ({fps:.1f} FPS)\n")

    print(f"✅ Analyse latence sauvegardée: {report_file}")
    print(f"📊 FPS moyen: {fps:.1f}")
    print(f"📊 Intervalle moyen: {mean_dt * 1000:.1f}ms")

    return True


def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description="Génère des graphiques à partir des traces BBIA",
    )
    parser.add_argument("--input", required=True, help="Fichier trace JSONL")
    parser.add_argument("--output", required=True, help="Fichier de sortie PNG")
    parser.add_argument(
        "--title",
        default="Mouvements joints BBIA",
        help="Titre du graphique",
    )
    parser.add_argument(
        "--type",
        choices=["movements", "latency"],
        default="movements",
        help="Type de graphique",
    )

    args = parser.parse_args()

    # Vérifier que le fichier d'entrée existe
    if not Path(args.input).exists():
        print(f"❌ Fichier d'entrée non trouvé: {args.input}")
        return 1

    # Charger la trace
    print(f"📊 Chargement trace: {args.input}")
    trace_data = load_trace(args.input)

    if not trace_data:
        print("❌ Aucune donnée valide dans la trace")
        return 1

    print(f"📈 {len(trace_data)} points de données chargés")

    # Créer le répertoire de sortie
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)

    # Générer le graphique
    if args.type == "movements":
        success = plot_joint_movements(trace_data, args.output, args.title)
    elif args.type == "latency":
        success = plot_latency_analysis(trace_data, args.output)
    else:
        print(f"❌ Type de graphique non supporté: {args.type}")
        return 1

    if success:
        print(f"🎉 Graphique généré avec succès: {args.output}")
        return 0
    print("❌ Erreur génération graphique")
    return 1


if __name__ == "__main__":
    sys.exit(main())
