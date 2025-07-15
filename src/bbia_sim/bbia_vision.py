#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
BBIA Vision - Module de vision avancé pour Reachy Mini Wireless
Reconnaissance d'objets, détection de visages, suivi d'objets
"""

import time
import random
from typing import List, Dict, Tuple, Optional, Any
from datetime import datetime

class BBIAVision:
    """Module de vision avancé pour BBIA"""
    
    def __init__(self):
        self.camera_active = True
        self.vision_quality = "HD"
        self.detection_range = 3.0  # mètres
        self.objects_detected = []
        self.faces_detected = []
        self.tracking_active = False
        self.current_focus = None
        
        # Spécifications hardware réelles
        self.specs = {
            "camera": "Grand angle",
            "resolution": "1080p",
            "fov": "120°",
            "focus": "Auto",
            "night_vision": False
        }
        
        print("📷 BBIA Vision initialisé")
        print(f"   • Caméra : {self.specs['camera']}")
        print(f"   • Résolution : {self.specs['resolution']}")
        print(f"   • Champ de vision : {self.specs['fov']}")
    
    def scan_environment(self) -> Dict[str, Any]:
        """Scanne l'environnement et détecte les objets"""
        print("🔍 Scan de l'environnement...")
        
        # Simulation de détection d'objets
        objects = [
            {"name": "chaise", "distance": 1.2, "confidence": 0.95, "position": (0.5, 0.3)},
            {"name": "table", "distance": 2.1, "confidence": 0.92, "position": (0.8, 0.1)},
            {"name": "livre", "distance": 0.8, "confidence": 0.88, "position": (0.2, 0.4)},
            {"name": "fenêtre", "distance": 3.0, "confidence": 0.97, "position": (1.0, 0.5)},
            {"name": "plante", "distance": 1.5, "confidence": 0.85, "position": (0.6, 0.2)}
        ]
        
        # Simulation de détection de visages
        faces = [
            {"name": "humain", "distance": 1.8, "confidence": 0.94, "emotion": "neutral", "position": (0.4, 0.3)},
            {"name": "humain", "distance": 2.3, "confidence": 0.91, "emotion": "happy", "position": (0.7, 0.2)}
        ]
        
        self.objects_detected = objects
        self.faces_detected = faces
        
        print(f"✅ {len(objects)} objets détectés")
        print(f"✅ {len(faces)} visages détectés")
        
        return {
            "objects": objects,
            "faces": faces,
            "timestamp": datetime.now().isoformat()
        }
    
    def recognize_object(self, object_name: str) -> Optional[Dict]:
        """Reconnaît un objet spécifique"""
        print(f"🔍 Reconnaissance de l'objet : {object_name}")
        
        for obj in self.objects_detected:
            if obj["name"] == object_name:
                print(f"✅ Objet reconnu : {object_name}")
                print(f"   • Distance : {obj['distance']}m")
                print(f"   • Confiance : {obj['confidence']*100:.1f}%")
                print(f"   • Position : {obj['position']}")
                return obj
        
        print(f"❌ Objet non trouvé : {object_name}")
        return None
    
    def detect_faces(self) -> List[Dict]:
        """Détecte les visages dans le champ de vision"""
        print("👥 Détection de visages...")
        
        if not self.faces_detected:
            self.scan_environment()
        
        for face in self.faces_detected:
            print(f"👤 Visage détecté :")
            print(f"   • Distance : {face['distance']}m")
            print(f"   • Émotion : {face['emotion']}")
            print(f"   • Confiance : {face['confidence']*100:.1f}%")
        
        return self.faces_detected
    
    def track_object(self, object_name: str) -> bool:
        """Active le suivi d'un objet"""
        print(f"🎯 Activation du suivi : {object_name}")
        
        obj = self.recognize_object(object_name)
        if obj:
            self.tracking_active = True
            self.current_focus = obj
            print(f"✅ Suivi activé pour : {object_name}")
            return True
        else:
            print(f"❌ Impossible de suivre : {object_name} (non détecté)")
            return False
    
    def stop_tracking(self):
        """Arrête le suivi d'objet"""
        if self.tracking_active:
            print(f"⏹️ Arrêt du suivi : {self.current_focus['name'] if self.current_focus else 'Aucun'}")
            self.tracking_active = False
            self.current_focus = None
        else:
            print("ℹ️ Aucun suivi actif")
    
    def get_focus_status(self) -> Dict:
        """Retourne le statut du focus actuel"""
        return {
            "tracking_active": self.tracking_active,
            "current_focus": self.current_focus,
            "objects_count": len(self.objects_detected),
            "faces_count": len(self.faces_detected)
        }
    
    def analyze_emotion(self, face_data: Dict) -> str:
        """Analyse l'émotion d'un visage"""
        emotions = ["neutral", "happy", "sad", "angry", "surprised", "fearful"]
        detected_emotion = face_data.get("emotion", "neutral")
        
        print(f"🎭 Analyse d'émotion : {detected_emotion}")
        return detected_emotion
    
    def calculate_distance(self, object_position: Tuple[float, float]) -> float:
        """Calcule la distance d'un objet"""
        # Simulation simple basée sur la position
        x, y = object_position
        distance = (x**2 + y**2)**0.5
        return distance
    
    def get_vision_stats(self) -> Dict:
        """Retourne les statistiques de vision"""
        return {
            "camera_active": self.camera_active,
            "vision_quality": self.vision_quality,
            "detection_range": self.detection_range,
            "objects_detected": len(self.objects_detected),
            "faces_detected": len(self.faces_detected),
            "tracking_active": self.tracking_active,
            "specs": self.specs
        }

def main():
    """Test du module BBIA Vision"""
    print("🧪 Test du module BBIA Vision")
    print("=" * 50)
    
    # Créer l'instance
    vision = BBIAVision()
    
    # Test scan environnement
    print("\n1️⃣ Test scan environnement")
    result = vision.scan_environment()
    
    # Test reconnaissance objet
    print("\n2️⃣ Test reconnaissance objet")
    vision.recognize_object("chaise")
    
    # Test détection visages
    print("\n3️⃣ Test détection visages")
    faces = vision.detect_faces()
    
    # Test suivi objet
    print("\n4️⃣ Test suivi objet")
    vision.track_object("livre")
    
    # Test statuts
    print("\n5️⃣ Test statuts")
    focus_status = vision.get_focus_status()
    vision_stats = vision.get_vision_stats()
    
    print(f"Focus status : {focus_status}")
    print(f"Vision stats : {vision_stats}")
    
    # Arrêt suivi
    print("\n6️⃣ Arrêt suivi")
    vision.stop_tracking()
    
    print("\n✅ Test BBIA Vision terminé")

if __name__ == "__main__":
    main() 