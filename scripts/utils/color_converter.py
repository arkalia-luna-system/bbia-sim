#!/usr/bin/env python3
"""
Utilitaire de conversion de couleurs pour MuJoCo/Reachy Mini
Convertit entre formats HEX, RGB (0-255), et RGBA MuJoCo (0-1)
"""

import sys


def hex_to_rgb(hex_color: str) -> tuple[int, int, int]:
    """Convertit HEX en RGB (0-255)"""
    hex_color = hex_color.lstrip('#')
    if len(hex_color) == 3:
        # Format court #RGB -> #RRGGBB
        hex_color = ''.join([c*2 for c in hex_color])
    r = int(hex_color[0:2], 16)
    g = int(hex_color[2:4], 16)
    b = int(hex_color[4:6], 16)
    return (r, g, b)


def rgb_to_hex(r: int, g: int, b: int) -> str:
    """Convertit RGB (0-255) en HEX"""
    return f"#{r:02x}{g:02x}{b:02x}".upper()


def hex_to_mujoco_rgba(hex_color: str, alpha: float = 1.0) -> str:
    """Convertit HEX en RGBA MuJoCo (0-1)"""
    r, g, b = hex_to_rgb(hex_color)
    r_norm = r / 255.0
    g_norm = g / 255.0
    b_norm = b / 255.0
    return f"{r_norm:.6f} {g_norm:.6f} {b_norm:.6f} {alpha}"


def mujoco_rgba_to_hex(rgba_str: str) -> tuple[str, float]:
    """Convertit RGBA MuJoCo (0-1) en HEX et alpha"""
    parts = rgba_str.strip().split()
    if len(parts) < 3:
        raise ValueError("Format RGBA invalide, attendu: 'r g b [a]'")

    r = float(parts[0]) * 255
    g = float(parts[1]) * 255
    b = float(parts[2]) * 255
    alpha = float(parts[3]) if len(parts) > 3 else 1.0

    hex_color = rgb_to_hex(int(r), int(g), int(b))
    return (hex_color, alpha)


def print_color_info(hex_color: str | None = None,
                     rgb: tuple[int, int, int] | None = None,
                     mujoco_rgba: str | None = None):
    """Affiche les informations de couleur dans tous les formats"""

    if hex_color:
        r, g, b = hex_to_rgb(hex_color)
        mujoco = hex_to_mujoco_rgba(hex_color)
        print(f"\n🎨 Couleur: {hex_color}")
        print(f"   RGB (0-255): ({r}, {g}, {b})")
        print(f"   MuJoCo RGBA: {mujoco}")

    elif rgb:
        hex_color = rgb_to_hex(*rgb)
        mujoco = hex_to_mujoco_rgba(hex_color)
        print(f"\n🎨 Couleur RGB: {rgb}")
        print(f"   HEX: {hex_color}")
        print(f"   MuJoCo RGBA: {mujoco}")

    elif mujoco_rgba:
        hex_color, alpha = mujoco_rgba_to_hex(mujoco_rgba)
        r, g, b = hex_to_rgb(hex_color)
        print(f"\n🎨 Couleur MuJoCo: {mujoco_rgba}")
        print(f"   HEX: {hex_color}")
        print(f"   RGB (0-255): ({r}, {g}, {b})")
        print(f"   Alpha: {alpha}")


def main():
    """Point d'entrée principal"""
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python color_converter.py <HEX>              # Ex: #87bcfa")
        print("  python color_converter.py <RGB>              # Ex: 135,188,250")
        print("  python color_converter.py <MuJoCo>           # Ex: '0.529 0.737 0.980'")
        print("\nExemples:")
        print("  python color_converter.py #87bcfa")
        print("  python color_converter.py 135,188,250")
        print("  python color_converter.py '0.529 0.737 0.980'")
        print("\nPalette BBIA:")
        print("  python color_converter.py #87bcfa  # Bleu céleste")
        print("  python color_converter.py #eaeaed  # Gris lunaire")
        print("  python color_converter.py #A680FF  # Violet électrique")
        print("  python color_converter.py #60e9e1  # Turquoise éthéré")
        sys.exit(1)

    input_str = sys.argv[1].strip()

    # Détection du format
    if input_str.startswith('#'):
        # Format HEX
        print_color_info(hex_color=input_str)

    elif ',' in input_str:
        # Format RGB (0-255)
        parts = [int(x.strip()) for x in input_str.split(',')]
        if len(parts) == 3:
            print_color_info(rgb=(parts[0], parts[1], parts[2]))
        else:
            print("❌ Format RGB invalide. Utilisez: r,g,b (ex: 135,188,250)")
            sys.exit(1)

    else:
        # Format MuJoCo RGBA
        try:
            print_color_info(mujoco_rgba=input_str)
        except ValueError as e:
            print(f"❌ Erreur: {e}")
            sys.exit(1)


if __name__ == "__main__":
    main()

