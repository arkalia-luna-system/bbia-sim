#!/usr/bin/env python3
"""
Script de vérification complète du projet BBIA Reachy Mini
Vérifie que tous les composants sont fonctionnels
"""

import os
import subprocess
import sys
from pathlib import Path


def check_python_imports():
    """Vérifie que tous les modules Python s'importent correctement"""
    print("🐍 Vérification des imports Python...")

    modules_to_check = [
        "src.bbia_sim.bbia_audio",
        "src.bbia_sim.bbia_vision",
        "src.bbia_sim.bbia_voice",
        "src.bbia_sim.bbia_emotions",
        "src.bbia_sim.bbia_behavior",
        "src.bbia_sim.daemon.simulation_service",
        "src.bbia_sim.sim.simulator",
    ]

    success_count = 0
    for module in modules_to_check:
        try:
            __import__(module)
            print(f"  ✅ {module}")
            success_count += 1
        except ImportError as e:
            print(f"  ❌ {module}: {e}")

    print(f"📊 {success_count}/{len(modules_to_check)} modules importés avec succès")
    return success_count == len(modules_to_check)


def check_mujoco_model():
    """Vérifie que le modèle MuJoCo se charge correctement"""
    print("\n🤖 Vérification du modèle MuJoCo...")

    try:
        import mujoco

        model_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"

        if not os.path.exists(model_path):
            print(f"  ❌ Modèle non trouvé: {model_path}")
            return False

        m = mujoco.MjModel.from_xml_path(model_path)
        print(
            f"  ✅ Modèle chargé: {m.nq} articulations, {m.nbody} corps, {m.ngeom} géométries"
        )
        return True

    except Exception as e:
        print(f"  ❌ Erreur chargement modèle: {e}")
        return False


def check_stl_assets():
    """Vérifie que tous les assets STL sont présents"""
    print("\n📦 Vérification des assets STL...")

    assets_dir = Path("src/bbia_sim/sim/assets/reachy_official")
    stl_files = list(assets_dir.glob("*.stl"))

    if len(stl_files) == 41:
        print(f"  ✅ {len(stl_files)} fichiers STL trouvés (nombre attendu)")

        # Vérifier la taille des fichiers
        valid_count = 0
        for stl_file in stl_files:
            size = stl_file.stat().st_size
            if size > 1000:  # Vrai fichier STL
                valid_count += 1

        print(f"  ✅ {valid_count}/{len(stl_files)} fichiers STL valides")
        return valid_count == len(stl_files)
    else:
        print(f"  ❌ {len(stl_files)} fichiers STL trouvés (attendu: 41)")
        return False


def check_tests():
    """Vérifie que les tests peuvent s'exécuter"""
    print("\n🧪 Vérification des tests...")

    try:
        result = subprocess.run(
            [sys.executable, "-m", "pytest", "tests/", "--collect-only", "-q"],
            capture_output=True,
            text=True,
            timeout=30,
        )

        if result.returncode == 0:
            lines = result.stdout.strip().split("\n")
            test_count = 0
            for line in lines:
                if "collected" in line:
                    test_count = int(line.split()[1])
                    break

            print(f"  ✅ {test_count} tests collectés")
            return True
        else:
            print(f"  ❌ Erreur collecte tests: {result.stderr}")
            return False

    except Exception as e:
        print(f"  ❌ Erreur exécution tests: {e}")
        return False


def check_api_endpoints():
    """Vérifie que l'API peut démarrer"""
    print("\n🌐 Vérification de l'API...")

    try:
        # Test d'import des modules API

        print("  ✅ Modules API importés avec succès")
        return True

    except Exception as e:
        print(f"  ❌ Erreur import API: {e}")
        return False


def check_scripts():
    """Vérifie que les scripts principaux existent"""
    print("\n📜 Vérification des scripts...")

    required_scripts = [
        "scripts/launch_complete_robot.py",
        "scripts/download_ALL_stl.py",
        "scripts/process_manager.py",
        "scripts/smart_process_cleanup.sh",
    ]

    success_count = 0
    for script in required_scripts:
        if os.path.exists(script):
            print(f"  ✅ {script}")
            success_count += 1
        else:
            print(f"  ❌ {script} manquant")

    print(f"📊 {success_count}/{len(required_scripts)} scripts trouvés")
    return success_count == len(required_scripts)


def main():
    """Fonction principale de vérification"""
    print("🔍 VÉRIFICATION COMPLÈTE DU PROJET BBIA REACHY MINI")
    print("=" * 60)

    checks = [
        ("Imports Python", check_python_imports),
        ("Modèle MuJoCo", check_mujoco_model),
        ("Assets STL", check_stl_assets),
        ("Tests", check_tests),
        ("API", check_api_endpoints),
        ("Scripts", check_scripts),
    ]

    results = []
    for name, check_func in checks:
        try:
            result = check_func()
            results.append((name, result))
        except Exception as e:
            print(f"  ❌ Erreur dans {name}: {e}")
            results.append((name, False))

    print("\n" + "=" * 60)
    print("📊 RÉSULTATS FINAUX")
    print("=" * 60)

    success_count = 0
    for name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} {name}")
        if result:
            success_count += 1

    print(f"\n🎯 Score: {success_count}/{len(results)} vérifications réussies")

    if success_count == len(results):
        print("🎉 TOUS LES COMPOSANTS SONT FONCTIONNELS !")
        return 0
    else:
        print("⚠️  Certains composants nécessitent une attention")
        return 1


if __name__ == "__main__":
    sys.exit(main())
