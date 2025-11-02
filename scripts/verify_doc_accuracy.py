#!/usr/bin/env python3
"""
Script pour vérifier l'exactitude de la documentation contre le code réel.
"""

import re
import subprocess
from pathlib import Path
from typing import Any

def count_tests():
    """Compte le nombre réel de tests."""
    result = subprocess.run(
        ['find', 'tests', '-name', '*.py', '-exec', 'grep', '-h', 'def test_', '{}', ';'],
        capture_output=True, text=True, cwd='/Volumes/T7/bbia-reachy-sim'
    )
    return len([l for l in result.stdout.split('\n') if l.strip() and l.strip().startswith('def test_')])

def count_docs():
    """Compte le nombre réel de fichiers MD."""
    result = subprocess.run(
        ['find', 'docs', '-name', '*.md', '-type', 'f'],
        capture_output=True, text=True, cwd='/Volumes/T7/bbia-reachy-sim'
    )
    return len([f for f in result.stdout.split('\n') if f.strip()])

def count_emotions():
    """Compte le nombre réel d'émotions dans bbia_emotions.py."""
    emotions_file = Path('/Volumes/T7/bbia-reachy-sim/src/bbia_sim/bbia_emotions.py')
    if not emotions_file.exists():
        return 0
    
    content = emotions_file.read_text()
    # Chercher toutes les clés dans self.emotions = {...}
    emotions = set()
    in_emotions_dict = False
    brace_count = 0
    
    for line in content.split('\n'):
        if 'self.emotions = {' in line:
            in_emotions_dict = True
            brace_count = line.count('{') - line.count('}')
            continue
        
        if in_emotions_dict:
            brace_count += line.count('{') - line.count('}')
            
            # Extraire les clés d'émotion
            if '":' in line or '" :' in line:
                match = re.search(r'["\']([a-z_]+)["\']\s*:', line)
                if match:
                    emotions.add(match.group(1))
            
            if brace_count <= 0 and '}' in line:
                break
    
    return len(emotions)

def verify_ci_cd_tools():
    """Vérifie que les outils CI/CD mentionnés existent vraiment."""
    ci_file = Path('/Volumes/T7/bbia-reachy-sim/.github/workflows/ci.yml')
    if not ci_file.exists():
        return {}
    
    content = ci_file.read_text()
    tools = {
        'black': 'black' in content.lower(),
        'ruff': 'ruff' in content.lower(),
        'mypy': 'mypy' in content.lower(),
        'bandit': 'bandit' in content.lower(),
        'pip-audit': 'pip-audit' in content.lower() or 'pip audit' in content.lower(),
    }
    return tools

def verify_architecture():
    """Vérifie que Factory et ABC sont bien présents."""
    factory_file = Path('/Volumes/T7/bbia-reachy-sim/src/bbia_sim/robot_factory.py')
    api_file = Path('/Volumes/T7/bbia-reachy-sim/src/bbia_sim/robot_api.py')
    
    factory_exists = factory_file.exists() and 'class RobotFactory' in factory_file.read_text()
    abc_exists = api_file.exists() and ('ABC' in api_file.read_text() and '@abstractmethod' in api_file.read_text())
    
    return {
        'factory': factory_exists,
        'abc': abc_exists,
    }

def main():
    """Fonction principale."""
    print("🔍 Vérification Documentation vs Code Réel\n")
    
    # Tests
    real_tests = count_tests()
    print(f"✅ Tests réels: {real_tests}")
    print(f"   README dit: 1200+")
    if real_tests < 1200:
        print(f"   ⚠️  Différence: {1200 - real_tests}")
    
    # Docs
    real_docs = count_docs()
    print(f"\n✅ Docs réels: {real_docs}")
    print(f"   README dit: 280")
    if real_docs != 280:
        print(f"   ⚠️  Différence: {abs(real_docs - 280)}")
    
    # Émotions
    real_emotions = count_emotions()
    print(f"\n✅ Émotions réelles: {real_emotions}")
    print(f"   README dit: 12")
    if real_emotions != 12:
        print(f"   ❌ ERREUR: {real_emotions} émotions trouvées, pas 12!")
        print(f"   ⚠️  Correction nécessaire dans README.md")
    
    # CI/CD
    ci_tools = verify_ci_cd_tools()
    print(f"\n✅ Outils CI/CD:")
    for tool, exists in ci_tools.items():
        status = "✅" if exists else "❌"
        print(f"   {status} {tool}: {exists}")
    
    # Architecture
    arch = verify_architecture()
    print(f"\n✅ Architecture:")
    print(f"   {'✅' if arch['factory'] else '❌'} Factory: {arch['factory']}")
    print(f"   {'✅' if arch['abc'] else '❌'} ABC: {arch['abc']}")
    
    print("\n" + "="*50)
    if real_emotions != 12:
        print("❌ ERREUR DÉTECTÉE: Nombre d'émotions incorrect!")
        return 1
    print("✅ Documentation globalement cohérente")
    return 0

if __name__ == "__main__":
    exit(main())

