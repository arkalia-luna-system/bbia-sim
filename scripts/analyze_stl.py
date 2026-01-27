#!/usr/bin/env python3
"""Script d'analyse des fichiers STL pour impression 3D.

Usage:
    python scripts/analyze_stl.py <fichier.stl>
    python scripts/analyze_stl.py --inventory  # Liste tous les STL du projet
"""

import argparse
import struct
import sys
from pathlib import Path


def analyze_stl_binary(filepath: Path) -> dict:
    """Analyse un fichier STL binaire."""
    with open(filepath, 'rb') as f:
        header = f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]
        
        f.seek(84)
        min_coords = [float('inf')] * 3
        max_coords = [float('-inf')] * 3
        
        for _ in range(num_triangles):
            f.read(12)  # normal
            for _ in range(3):
                v = struct.unpack('<3f', f.read(12))
                for i in range(3):
                    min_coords[i] = min(min_coords[i], v[i])
                    max_coords[i] = max(max_coords[i], v[i])
            f.read(2)  # attribute
        
        dimensions = [max_coords[i] - min_coords[i] for i in range(3)]
        center = [(min_coords[i] + max_coords[i]) / 2 for i in range(3)]
        
        return {
            'num_triangles': num_triangles,
            'dimensions_m': dimensions,
            'dimensions_mm': [d * 1000 for d in dimensions],
            'dimensions_inches': [d * 39.3701 for d in dimensions],
            'min_coords': min_coords,
            'max_coords': max_coords,
            'center': center,
            'file_size': filepath.stat().st_size,
        }


def check_h2s_compatibility(dimensions_mm: list[float]) -> dict:
    """Vérifie la compatibilité avec imprimante H2S."""
    h2s_bed = [220, 220, 250]  # mm
    fits = all(dimensions_mm[i] < h2s_bed[i] for i in range(3))
    
    margins = [h2s_bed[i] - dimensions_mm[i] for i in range(3)]
    
    return {
        'compatible': fits,
        'margins_mm': margins,
        'bed_size': h2s_bed,
    }


def print_analysis(filepath: Path) -> None:
    """Affiche l'analyse d'un fichier STL."""
    print(f'\n{"="*60}')
    print(f'📦 ANALYSE: {filepath.name}')
    print(f'{"="*60}\n')
    
    try:
        data = analyze_stl_binary(filepath)
        
        print(f'📊 Informations:')
        print(f'   Triangles: {data["num_triangles"]:,}')
        print(f'   Taille fichier: {data["file_size"]:,} bytes')
        
        print(f'\n📐 Dimensions (fichier en MÈTRES):')
        print(f'   X: {data["dimensions_m"][0]:.6f} m = {data["dimensions_mm"][0]:.2f} mm')
        print(f'   Y: {data["dimensions_m"][1]:.6f} m = {data["dimensions_mm"][1]:.2f} mm')
        print(f'   Z: {data["dimensions_m"][2]:.6f} m = {data["dimensions_mm"][2]:.2f} mm')
        
        print(f'\n🖨️  Compatibilité H2S:')
        h2s = check_h2s_compatibility(data['dimensions_mm'])
        status = '✅ OUI' if h2s['compatible'] else '❌ NON'
        print(f'   {status}')
        
        if h2s['compatible']:
            print(f'   Marges disponibles:')
            for i, axis in enumerate(['X', 'Y', 'Z']):
                print(f'   {axis}: {h2s["margins_mm"][i]:.1f} mm')
            
            print(f'\n📋 Orientation recommandée:')
            dims = data['dimensions_mm']
            max_dim = max(dims)
            min_dim = min(dims)
            print(f'   - Face large: {max_dim:.1f} mm')
            print(f'   - Hauteur: {min_dim:.1f} mm')
            print(f'   - Support: {"NON" if min_dim > 2 else "OUI (surfaces fines)"}')
        
        print()
        
    except Exception as e:
        print(f'❌ Erreur: {e}')
        sys.exit(1)


def inventory_stl_files(project_root: Path) -> None:
    """Liste tous les fichiers STL du projet."""
    print(f'\n{"="*60}')
    print(f'📋 INVENTAIRE FICHIERS STL')
    print(f'{"="*60}\n')
    
    stl_files = list(project_root.rglob('*.stl'))
    
    print(f'Total: {len(stl_files)} fichiers STL\n')
    
    # Grouper par dossier
    by_dir: dict[str, list[Path]] = {}
    for stl_file in stl_files:
        rel_path = stl_file.relative_to(project_root)
        dir_name = str(rel_path.parent)
        if dir_name not in by_dir:
            by_dir[dir_name] = []
        by_dir[dir_name].append(stl_file)
    
    for dir_name in sorted(by_dir.keys()):
        print(f'📁 {dir_name}/')
        for stl_file in sorted(by_dir[dir_name]):
            try:
                data = analyze_stl_binary(stl_file)
                name = stl_file.name
                dims = data['dimensions_mm']
                print(f'   - {name}')
                print(f'     {dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f} mm')
            except:
                print(f'   - {stl_file.name} (erreur analyse)')
        print()


def main() -> None:
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(description='Analyse fichiers STL')
    parser.add_argument('file', nargs='?', help='Fichier STL à analyser')
    parser.add_argument(
        '--inventory',
        action='store_true',
        help='Liste tous les fichiers STL du projet',
    )
    
    args = parser.parse_args()
    
    project_root = Path(__file__).parent.parent
    
    if args.inventory:
        inventory_stl_files(project_root)
    elif args.file:
        filepath = Path(args.file)
        if not filepath.exists():
            print(f'❌ Fichier non trouvé: {filepath}')
            sys.exit(1)
        print_analysis(filepath)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
