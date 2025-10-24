#!/usr/bin/env python3
"""
Télémétrie minimale pour BBIA
Compteur steps/s, temps moyen step, drift max
Export .csv dans artifacts/
"""

import csv
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


class TelemetryCollector:
    """Collecteur de télémétrie BBIA."""
    
    def __init__(self, output_dir: str = "artifacts"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.step_times: List[float] = []
        self.joint_positions: List[Dict[str, float]] = []
        self.start_time = None
        self.last_step_time = None
        
    def start_collection(self):
        """Démarre la collecte de télémétrie."""
        self.step_times = []
        self.joint_positions = []
        self.start_time = time.time()
        self.last_step_time = self.start_time
        print(f"📊 Télémétrie démarrée")
    
    def record_step(self, joint_positions: Dict[str, float]):
        """Enregistre un pas de simulation."""
        current_time = time.time()
        
        # Temps de step
        if self.last_step_time:
            step_time = current_time - self.last_step_time
            self.step_times.append(step_time)
        
        # Positions des joints
        self.joint_positions.append({
            "timestamp": current_time,
            "elapsed": current_time - self.start_time,
            **joint_positions
        })
        
        self.last_step_time = current_time
    
    def stop_collection(self) -> Dict[str, Any]:
        """Arrête la collecte et retourne les statistiques."""
        if not self.step_times:
            return {}
        
        # Calculs statistiques
        total_time = time.time() - self.start_time
        avg_step_time = sum(self.step_times) / len(self.step_times)
        steps_per_second = len(self.step_times) / total_time
        
        # Drift max (variation des positions)
        max_drift = 0.0
        if len(self.joint_positions) > 1:
            for joint in self.joint_positions[0].keys():
                if joint not in ["timestamp", "elapsed"]:
                    positions = [pos[joint] for pos in self.joint_positions]
                    max_drift = max(max_drift, max(positions) - min(positions))
        
        stats = {
            "total_steps": len(self.step_times),
            "total_time": total_time,
            "steps_per_second": steps_per_second,
            "average_step_time": avg_step_time,
            "max_step_time": max(self.step_times),
            "min_step_time": min(self.step_times),
            "max_drift": max_drift,
            "start_time": self.start_time,
            "end_time": time.time(),
        }
        
        print(f"📊 Télémétrie arrêtée: {stats['total_steps']} steps, {stats['steps_per_second']:.1f} steps/s")
        return stats
    
    def export_csv(self, filename: str, stats: Dict[str, Any]) -> str:
        """Exporte les données en CSV."""
        csv_path = self.output_dir / filename
        
        # Export des statistiques
        stats_path = self.output_dir / f"stats_{filename}"
        with open(stats_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Metric", "Value"])
            for key, value in stats.items():
                writer.writerow([key, value])
        
        # Export des positions des joints
        if self.joint_positions:
            with open(csv_path, 'w', newline='') as f:
                fieldnames = self.joint_positions[0].keys()
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.joint_positions)
        
        print(f"💾 Télémétrie exportée: {csv_path} + {stats_path}")
        return str(csv_path)
    
    def get_live_stats(self) -> Dict[str, Any]:
        """Retourne les statistiques en temps réel."""
        if not self.step_times:
            return {}
        
        current_time = time.time()
        total_time = current_time - self.start_time
        
        return {
            "current_steps": len(self.step_times),
            "current_time": total_time,
            "current_sps": len(self.step_times) / total_time if total_time > 0 else 0,
            "last_step_time": self.step_times[-1] if self.step_times else 0,
        }
