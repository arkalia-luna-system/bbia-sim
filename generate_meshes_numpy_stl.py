#!/usr/bin/env python3
"""
Générateur de meshes STL avec numpy-stl pour Reachy Mini.
Version finale avec des meshes STL valides pour MuJoCo.
"""

import numpy as np
from stl import mesh
import os

def create_box_stl(filename, size):
    """Crée un mesh STL de boîte."""
    w, h, d = size
    
    # Définir les 8 sommets de la boîte
    vertices = np.array([
        [-w/2, -h/2, -d/2],  # 0
        [ w/2, -h/2, -d/2],  # 1
        [ w/2,  h/2, -d/2],  # 2
        [-w/2,  h/2, -d/2],  # 3
        [-w/2, -h/2,  d/2],  # 4
        [ w/2, -h/2,  d/2],  # 5
        [ w/2,  h/2,  d/2],  # 6
        [-w/2,  h/2,  d/2],  # 7
    ])
    
    # Définir les 12 triangles (2 par face)
    faces = np.array([
        # Face avant (z-)
        [0, 1, 2], [0, 2, 3],
        # Face arrière (z+)
        [4, 6, 5], [4, 7, 6],
        # Face gauche (x-)
        [0, 3, 7], [0, 7, 4],
        # Face droite (x+)
        [1, 5, 6], [1, 6, 2],
        # Face bas (y-)
        [0, 4, 5], [0, 5, 1],
        # Face haut (y+)
        [3, 2, 6], [3, 6, 7],
    ])
    
    # Créer le mesh
    box_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, face in enumerate(faces):
        for j in range(3):
            box_mesh.vectors[i][j] = vertices[face[j], :]
    
    # Sauvegarder
    box_mesh.save(filename)

def create_sphere_stl(filename, radius, resolution=8):
    """Crée un mesh STL de sphère."""
    # Générer les sommets
    vertices = []
    faces = []
    
    # Pôles
    vertices.append([0, 0, radius])   # Pôle nord
    vertices.append([0, 0, -radius])  # Pôle sud
    
    # Cercles intermédiaires
    for i in range(1, resolution):
        phi = np.pi * i / resolution
        for j in range(resolution):
            theta = 2 * np.pi * j / resolution
            x = radius * np.sin(phi) * np.cos(theta)
            y = radius * np.sin(phi) * np.sin(theta)
            z = radius * np.cos(phi)
            vertices.append([x, y, z])
    
    vertices = np.array(vertices)
    
    # Générer les faces
    # Faces autour du pôle nord
    for j in range(resolution):
        next_j = (j + 1) % resolution
        faces.append([0, 2 + j, 2 + next_j])
    
    # Faces autour du pôle sud
    for j in range(resolution):
        next_j = (j + 1) % resolution
        last_circle_start = 2 + (resolution - 2) * resolution
        faces.append([1, last_circle_start + next_j, last_circle_start + j])
    
    # Faces intermédiaires
    for i in range(resolution - 2):
        for j in range(resolution):
            next_j = (j + 1) % resolution
            
            v1 = 2 + i * resolution + j
            v2 = 2 + i * resolution + next_j
            v3 = 2 + (i + 1) * resolution + next_j
            v4 = 2 + (i + 1) * resolution + j
            
            faces.append([v1, v2, v3])
            faces.append([v1, v3, v4])
    
    faces = np.array(faces)
    
    # Créer le mesh
    sphere_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, face in enumerate(faces):
        for j in range(3):
            sphere_mesh.vectors[i][j] = vertices[face[j], :]
    
    # Sauvegarder
    sphere_mesh.save(filename)

def main():
    """Génère tous les meshes STL pour Reachy Mini."""
    output_dir = "/Volumes/T7/bbia-reachy-sim/src/bbia_sim/sim/assets/meshes"
    
    print("🎨 Génération des meshes STL avec numpy-stl...")
    
    # 1. Torse (boîte)
    print("  📦 Génération du torse...")
    create_box_stl(os.path.join(output_dir, "torso.stl"), [0.2, 0.2, 0.4])
    
    # 2. Tête (sphère)
    print("  🧠 Génération de la tête...")
    create_sphere_stl(os.path.join(output_dir, "head.stl"), 0.08, resolution=6)
    
    # 3. Bras supérieur (boîte allongée)
    print("  💪 Génération du bras supérieur...")
    create_box_stl(os.path.join(output_dir, "upper_arm.stl"), [0.06, 0.06, 0.15])
    
    # 4. Avant-bras (boîte allongée plus fine)
    print("  🤲 Génération de l'avant-bras...")
    create_box_stl(os.path.join(output_dir, "forearm.stl"), [0.05, 0.05, 0.15])
    
    # 5. Pince (petite boîte)
    print("  ✋ Génération de la pince...")
    create_box_stl(os.path.join(output_dir, "gripper.stl"), [0.04, 0.02, 0.02])
    
    print("✅ Génération terminée !")
    print(f"📁 Meshes STL créés dans : {output_dir}")

if __name__ == "__main__":
    main()
