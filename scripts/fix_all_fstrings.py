#!/usr/bin/env python3
"""Script robuste pour corriger toutes les f-strings dans les appels de logging."""

import re
import sys
from pathlib import Path


def extract_fstring_vars(fstring_content: str) -> list[tuple[str, str]]:
    """Extrait les variables d'une f-string et retourne (pattern, var_name)."""
    vars_found = []
    pattern = r"\{([^}]+)\}"

    for match in re.finditer(pattern, fstring_content):
        var_expr = match.group(1)
        # Simplifier les expressions complexes
        if "." in var_expr or "[" in var_expr or "(" in var_expr:
            # Expression complexe, garder tel quel pour l'instant
            vars_found.append((match.group(0), var_expr))
        else:
            # Variable simple
            vars_found.append((match.group(0), var_expr.strip()))

    return vars_found


def fix_fstring_in_line(line: str) -> str | None:
    """Corrige une f-string dans une ligne de logging."""
    # Pattern pour logger.xxx(f"...")
    pattern = r'(logger\.(?:debug|info|warning|error|exception|critical))\s*\(\s*f["\']([^"\']+)["\']\s*\)'

    match = re.search(pattern, line)
    if not match:
        return None

    logger_call = match.group(1)
    fstring_content = match.group(2)

    # Extraire variables
    vars_found = extract_fstring_vars(fstring_content)

    if not vars_found:
        # Pas de variables, juste enlever le f
        format_string = fstring_content
        new_line = re.sub(
            pattern,
            f'{logger_call}("{format_string}")',
            line,
        )
        return new_line

    # Construire format string avec %s
    format_string = fstring_content
    for pattern_match, _var_name in vars_found:
        format_string = format_string.replace(pattern_match, "%s")

    # Construire arguments
    args = ", ".join(var_name for _, var_name in vars_found)

    # Remplacer
    new_line = re.sub(
        pattern,
        f'{logger_call}("{format_string}", {args})',
        line,
    )

    return new_line


def fix_multiline_fstring(lines: list[str], start_idx: int) -> tuple[int, list[str]]:
    """Corrige une f-string sur plusieurs lignes."""
    # Trouver la fin de l'appel
    full_call = lines[start_idx]
    i = start_idx + 1
    paren_count = full_call.count("(") - full_call.count(")")

    while i < len(lines) and paren_count > 0:
        full_call += "\n" + lines[i]
        paren_count += lines[i].count("(") - lines[i].count(")")
        i += 1

    # Vérifier si c'est une f-string
    if 'f"' not in full_call and "f'" not in full_call:
        return start_idx + 1, lines

    # Extraire le contenu f-string (peut être sur plusieurs lignes)
    fstring_match = re.search(r'f["\']([^"\']+)["\']', full_call, re.DOTALL)
    if not fstring_match:
        return start_idx + 1, lines

    fstring_content = fstring_match.group(1).strip()

    # Extraire variables
    vars_found = extract_fstring_vars(fstring_content)

    if not vars_found:
        format_string = fstring_content
        new_call = full_call.replace('f"', '"').replace("f'", "'")
    else:
        # Construire format string
        format_string = fstring_content
        for pattern_match, _var_name in vars_found:
            format_string = format_string.replace(pattern_match, "%s")

        # Construire arguments
        args = ", ".join(var_name for _, var_name in vars_found)

        # Remplacer dans full_call
        new_call = re.sub(
            r'f["\']' + re.escape(fstring_content) + r'["\']',
            f'"{format_string}", {args}',
            full_call,
            flags=re.DOTALL,
        )

    # Remplacer les lignes
    new_lines = lines[:start_idx] + new_call.split("\n") + lines[i:]

    return i, new_lines


def fix_file(file_path: Path) -> int:
    """Corrige un fichier et retourne le nombre de corrections."""
    try:
        content = file_path.read_text(encoding="utf-8")
        lines = content.split("\n")
        fixed_lines = []
        i = 0
        fixed_count = 0

        while i < len(lines):
            line = lines[i]

            # Essayer correction simple
            fixed_line = fix_fstring_in_line(line)
            if fixed_line and fixed_line != line:
                fixed_lines.append(fixed_line)
                fixed_count += 1
                i += 1
                continue

            # Vérifier si c'est le début d'une f-string multi-ligne
            if (
                "logger." in line
                and ('f"' in line or "f'" in line)
                and line.count("(") > line.count(")")
            ):
                new_i, new_lines = fix_multiline_fstring(lines, i)
                if new_i > i + 1:
                    # Correction multi-ligne réussie
                    fixed_lines = new_lines[:i] + new_lines[i:]
                    fixed_count += 1
                    i = new_i
                    continue

            fixed_lines.append(line)
            i += 1

        if fixed_count > 0:
            new_content = "\n".join(fixed_lines)
            file_path.write_text(new_content, encoding="utf-8")

        return fixed_count
    except Exception as e:
        print(f"Erreur sur {file_path}: {e}", file=sys.stderr)
        return 0


def main():
    """Point d'entrée principal."""
    src_dir = Path("src/bbia_sim")
    if not src_dir.exists():
        print("Erreur: src/bbia_sim n'existe pas", file=sys.stderr)
        sys.exit(1)

    total_fixed = 0
    files_fixed = 0

    # Traiter les fichiers avec le plus d'erreurs d'abord
    py_files = list(src_dir.rglob("*.py"))

    for py_file in py_files:
        fixed = fix_file(py_file)
        if fixed > 0:
            total_fixed += fixed
            files_fixed += 1
            print(f"✅ {py_file.name}: {fixed} corrections")

    print(f"\n✅ {files_fixed} fichiers corrigés, {total_fixed} f-strings converties")


if __name__ == "__main__":
    main()
