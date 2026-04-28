#!/usr/bin/env python3
"""Liste tous les fichiers SVG disponibles"""
import os
from pathlib import Path

print("=" * 70)
print("📋 TOUS LES FICHIERS SVG DISPONIBLES")
print("=" * 70)

current_dir = Path(".")
all_svg = list(current_dir.rglob("*.svg"))

if not all_svg:
    print("❌ Aucun fichier SVG trouvé")
else:
    print(f"\n✅ {len(all_svg)} fichier(s) SVG trouvé(s) :\n")
    for i, svg_path in enumerate(all_svg, 1):
        size = svg_path.stat().st_size / 1024
        rel_path = svg_path.relative_to(current_dir)
        print(f"{i}. {rel_path}")
        print(f"   Taille: {size:.1f}K")
        print()

print("\n💡 Indique-moi quels fichiers sont les BONS logos !")
