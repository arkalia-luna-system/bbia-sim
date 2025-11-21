#!/usr/bin/env python3
"""Script pour corriger automatiquement les f-strings dans les appels de logging.

Convertit logger.info(f"...") en logger.info("...", ...) avec format %s.
"""

import re
import sys
from pathlib import Path


def fix_logging_fstring(content: str) -> str:
    """Corrige les f-strings dans les appels de logging."""
    lines = content.split("\n")
    fixed_lines = []
    i = 0

    while i < len(lines):
        line = lines[i]

        # Détecter logger.xxx(f"...")
        pattern = r'(logger\.(?:debug|info|warning|error|exception|critical))\s*\(\s*f"([^"]+)"\s*\)'
        match = re.search(pattern, line)

        if match:
            logger_call = match.group(1)
            fstring_content = match.group(2)

            # Extraire les variables {var} de la f-string
            var_pattern = r"\{([^}]+)\}"
            vars_found = re.findall(var_pattern, fstring_content)

            if vars_found:
                # Construire le nouveau format string avec %s
                format_string = re.sub(var_pattern, "%s", fstring_content)
                # Construire les arguments
                args = ", ".join(vars_found)

                # Remplacer
                new_line = re.sub(
                    pattern,
                    f'{logger_call}("{format_string}", {args})',
                    line,
                )
                fixed_lines.append(new_line)
                i += 1
                continue

        # Détecter logger.xxx(f"...") sur plusieurs lignes
        if "logger." in line and 'f"' in line:
            # Chercher la fin de l'appel (peut être sur plusieurs lignes)
            full_call = line
            j = i + 1
            paren_count = line.count("(") - line.count(")")

            while j < len(lines) and paren_count > 0:
                full_call += "\n" + lines[j]
                paren_count += lines[j].count("(") - lines[j].count(")")
                j += 1

            # Essayer de corriger si c'est une f-string
            if 'f"' in full_call or "f'" in full_call:
                # Pattern plus complexe pour multi-lignes
                multiline_pattern = r'(logger\.(?:debug|info|warning|error|exception|critical))\s*\(\s*f["\']([^"\']+)["\']\s*\)'
                multiline_match = re.search(multiline_pattern, full_call, re.DOTALL)

                if multiline_match:
                    logger_call = multiline_match.group(1)
                    fstring_content = multiline_match.group(2).strip()

                    # Extraire variables
                    var_pattern = r"\{([^}]+)\}"
                    vars_found = re.findall(var_pattern, fstring_content)

                    if vars_found:
                        format_string = re.sub(var_pattern, "%s", fstring_content)
                        args = ", ".join(vars_found)

                        # Remplacer dans full_call
                        new_call = re.sub(
                            multiline_pattern,
                            f'{logger_call}("{format_string}", {args})',
                            full_call,
                            flags=re.DOTALL,
                        )

                        # Ajouter les lignes corrigées
                        fixed_lines.extend(new_call.split("\n"))
                        i = j
                        continue

        fixed_lines.append(line)
        i += 1

    return "\n".join(fixed_lines)


def fix_file(file_path: Path) -> bool:
    """Corrige un fichier."""
    try:
        content = file_path.read_text(encoding="utf-8")
        fixed_content = fix_logging_fstring(content)

        if content != fixed_content:
            file_path.write_text(fixed_content, encoding="utf-8")
            return True
        return False
    except Exception as e:
        print(f"Erreur sur {file_path}: {e}", file=sys.stderr)
        return False


def main():
    """Point d'entrée principal."""
    src_dir = Path("src/bbia_sim")
    if not src_dir.exists():
        print("Erreur: src/bbia_sim n'existe pas", file=sys.stderr)
        sys.exit(1)

    fixed_count = 0
    for py_file in src_dir.rglob("*.py"):
        if fix_file(py_file):
            fixed_count += 1
            print(f"✅ Corrigé: {py_file}")

    print(f"\n✅ {fixed_count} fichiers corrigés")


if __name__ == "__main__":
    main()
