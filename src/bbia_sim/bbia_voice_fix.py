#!/usr/bin/env python3
"""Script temporaire pour fixer l'indentation dans bbia_voice.py"""

# Lire le fichier
with open("src/bbia_sim/bbia_voice.py", "r", encoding="utf-8") as f:
    lines = f.readlines()

# Corriger l'indentation ligne 279-282
for i in range(278, 283):  # Lignes 279-282 (0-indexed)
    if i < len(lines):
        line = lines[i]
        # Si la ligne commence sans indentation après un 'with:', l'indenter
        if i == 278:  # ligne 279 (avec _wave.open)
            if not line.startswith("                        "):  # 24 espaces
                lines[i] = "                        " + line.lstrip()
        elif i in [279, 280, 281]:  # lignes 280-282
            if not line.startswith("                        "):  # 24 espaces
                lines[i] = "                        " + line.lstrip()

# Écrire le fichier corrigé
with open("src/bbia_sim/bbia_voice.py", "w", encoding="utf-8") as f:
    f.writelines(lines)

print("✅ Indentation corrigée")

