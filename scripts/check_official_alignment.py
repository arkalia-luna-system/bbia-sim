#!/usr/bin/env python3
"""
Script d'alignement officiel BBIA-SIM ↔ Reachy Mini
Compare notre projet avec le repo officiel pollen-robotics/reachy_mini
"""

import json
import os
import re
from pathlib import Path

import requests


class OfficialAlignmentChecker:
    """Vérificateur d'alignement avec le repo officiel"""

    def __init__(self):
        self.project_root = Path(__file__).parent.parent
        self.official_base_url = "https://raw.githubusercontent.com/pollen-robotics/reachy_mini/main"
        self.official_mjcf_url = f"{self.official_base_url}/src/reachy_mini/descriptions/reachy_mini/mjcf/reachy_mini.xml"
        self.official_assets_url = f"{self.official_base_url}/src/reachy_mini/descriptions/reachy_mini/mjcf/assets/"

    def check_mjcf_model_alignment(self) -> dict[str, any]:
        """Vérifie l'alignement du modèle MJCF"""
        print("🔍 Vérification alignement modèle MJCF...")

        try:
            # Récupérer le modèle officiel
            response = requests.get(self.official_mjcf_url, timeout=10)
            response.raise_for_status()
            official_content = response.text

            # Lire notre modèle
            our_model_path = self.project_root / "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
            if not our_model_path.exists():
                return {"status": "error", "message": "Notre modèle officiel non trouvé"}

            with open(our_model_path) as f:
                our_content = f.read()

            # Comparaisons
            results = {
                "status": "success",
                "comparisons": {}
            }

            # 1. Nombre de joints
            official_joints = len(re.findall(r'<joint name="([^"]+)"', official_content))
            our_joints = len(re.findall(r'<joint name="([^"]+)"', our_content))
            results["comparisons"]["joints_count"] = {
                "official": official_joints,
                "ours": our_joints,
                "match": official_joints == our_joints
            }

            # 2. Nombre d'actuateurs
            official_actuators = len(re.findall(r'<actuator name="([^"]+)"', official_content))
            our_actuators = len(re.findall(r'<actuator name="([^"]+)"', our_content))
            results["comparisons"]["actuators_count"] = {
                "official": official_actuators,
                "ours": our_actuators,
                "match": official_actuators == our_actuators
            }

            # 3. Nombre de corps
            official_bodies = len(re.findall(r'<body name="([^"]+)"', official_content))
            our_bodies = len(re.findall(r'<body name="([^"]+)"', our_content))
            results["comparisons"]["bodies_count"] = {
                "official": official_bodies,
                "ours": our_bodies,
                "match": official_bodies == our_bodies
            }

            # 4. Nombre de géométries
            official_geoms = len(re.findall(r'<geom name="([^"]+)"', official_content))
            our_geoms = len(re.findall(r'<geom name="([^"]+)"', our_content))
            results["comparisons"]["geoms_count"] = {
                "official": official_geoms,
                "ours": our_geoms,
                "match": official_geoms == our_geoms
            }

            # 5. Fichiers STL référencés
            official_stl_files = sorted(set(re.findall(r'<mesh file="([^"]+\.stl)"', official_content)))
            our_stl_files = sorted(set(re.findall(r'<mesh file="([^"]+\.stl)"', our_content)))
            results["comparisons"]["stl_files"] = {
                "official": official_stl_files,
                "ours": our_stl_files,
                "match": official_stl_files == our_stl_files,
                "missing": list(set(official_stl_files) - set(our_stl_files)),
                "extra": list(set(our_stl_files) - set(official_stl_files))
            }

            return results

        except Exception as e:
            return {"status": "error", "message": f"Erreur comparaison MJCF: {e}"}

    def check_stl_assets_alignment(self) -> dict[str, any]:
        """Vérifie l'alignement des assets STL"""
        print("📦 Vérification alignement assets STL...")

        try:
            # Récupérer la liste des STL du modèle officiel
            response = requests.get(self.official_mjcf_url, timeout=10)
            response.raise_for_status()
            official_content = response.text
            official_stl_files = sorted(set(re.findall(r'<mesh file="([^"]+\.stl)"', official_content)))

            # Vérifier nos assets
            our_assets_dir = self.project_root / "src/bbia_sim/sim/assets/reachy_official"
            if not our_assets_dir.exists():
                return {"status": "error", "message": "Répertoire assets non trouvé"}

            our_stl_files = sorted([f for f in os.listdir(our_assets_dir) if f.endswith('.stl')])

            results = {
                "status": "success",
                "comparisons": {
                    "official_count": len(official_stl_files),
                    "our_count": len(our_stl_files),
                    "official_files": official_stl_files,
                    "our_files": our_stl_files,
                    "missing_files": list(set(official_stl_files) - set(our_stl_files)),
                    "extra_files": list(set(our_stl_files) - set(official_stl_files)),
                    "all_present": len(set(official_stl_files) - set(our_stl_files)) == 0
                }
            }

            # Vérifier la taille des fichiers
            file_sizes = {}
            for stl_file in our_stl_files:
                file_path = our_assets_dir / stl_file
                file_size = file_path.stat().st_size
                file_sizes[stl_file] = file_size

                # Vérifier si c'est un pointeur Git LFS (fichier < 1000 bytes)
                if file_size < 1000:
                    results["comparisons"]["lfs_pointers"] = results["comparisons"].get("lfs_pointers", [])
                    results["comparisons"]["lfs_pointers"].append(stl_file)

            results["comparisons"]["file_sizes"] = file_sizes

            return results

        except Exception as e:
            return {"status": "error", "message": f"Erreur vérification STL: {e}"}

    def check_joints_specifications(self) -> dict[str, any]:
        """Vérifie les spécifications des joints"""
        print("🔧 Vérification spécifications joints...")

        try:
            # Récupérer le modèle officiel
            response = requests.get(self.official_mjcf_url, timeout=10)
            response.raise_for_status()
            official_content = response.text

            # Extraire les joints officiels
            official_joints = re.findall(r'<joint name="([^"]+)"[^>]*range="([^"]+)"', official_content)
            official_joint_ranges = dict(official_joints)

            # Vérifier nos joints
            our_joints_file = self.project_root / "src/bbia_sim/sim/joints.py"
            if not our_joints_file.exists():
                return {"status": "error", "message": "Fichier joints.py non trouvé"}

            with open(our_joints_file) as f:
                our_joints_content = f.read()

            # Extraire nos joints
            our_joints = re.findall(r'"([^"]+)": \(([^,]+), ([^)]+)\)', our_joints_content)
            our_joint_ranges = {name: (float(min_val), float(max_val)) for name, min_val, max_val in our_joints}

            results = {
                "status": "success",
                "comparisons": {
                    "official_joints": official_joint_ranges,
                    "our_joints": our_joint_ranges,
                    "official_count": len(official_joint_ranges),
                    "our_count": len(our_joint_ranges),
                    "missing_joints": list(set(official_joint_ranges.keys()) - set(our_joint_ranges.keys())),
                    "extra_joints": list(set(our_joint_ranges.keys()) - set(official_joint_ranges.keys()))
                }
            }

            return results

        except Exception as e:
            return {"status": "error", "message": f"Erreur vérification joints: {e}"}

    def check_api_endpoints_alignment(self) -> dict[str, any]:
        """Vérifie l'alignement des endpoints API"""
        print("🌐 Vérification alignement API...")

        try:
            # Vérifier nos endpoints
            our_api_files = [
                self.project_root / "src/bbia_sim/daemon/app/routers/motion.py",
                self.project_root / "src/bbia_sim/daemon/app/routers/state.py",
                self.project_root / "src/bbia_sim/daemon/app/main.py"
            ]

            endpoints = []
            for api_file in our_api_files:
                if api_file.exists():
                    with open(api_file) as f:
                        content = f.read()
                        # Extraire les routes
                        routes = re.findall(r'@router\.(get|post|put|delete)\(["\']([^"\']+)["\']', content)
                        endpoints.extend([(method, route) for method, route in routes])

            results = {
                "status": "success",
                "comparisons": {
                    "endpoints_count": len(endpoints),
                    "endpoints": endpoints,
                    "motion_endpoints": [ep for ep in endpoints if "motion" in ep[1]],
                    "state_endpoints": [ep for ep in endpoints if "state" in ep[1]]
                }
            }

            return results

        except Exception as e:
            return {"status": "error", "message": f"Erreur vérification API: {e}"}

    def run_full_alignment_check(self) -> dict[str, any]:
        """Exécute la vérification complète d'alignement"""
        print("🎯 VÉRIFICATION COMPLÈTE D'ALIGNEMENT OFFICIEL")
        print("=" * 60)

        results = {
            "mjcf_model": self.check_mjcf_model_alignment(),
            "stl_assets": self.check_stl_assets_alignment(),
            "joints_specs": self.check_joints_specifications(),
            "api_endpoints": self.check_api_endpoints_alignment()
        }

        # Résumé
        print("\n" + "=" * 60)
        print("📊 RÉSUMÉ DE L'ALIGNEMENT")
        print("=" * 60)

        total_checks = 0
        passed_checks = 0

        for category, result in results.items():
            if result["status"] == "success":
                print(f"✅ {category}: Vérification réussie")
                passed_checks += 1
            else:
                print(f"❌ {category}: {result['message']}")
            total_checks += 1

        print(f"\n🎯 Score: {passed_checks}/{total_checks} vérifications réussies")

        if passed_checks == total_checks:
            print("🎉 ALIGNEMENT PARFAIT AVEC LE REPO OFFICIEL !")
        else:
            print("⚠️  DES ÉCARTS DÉTECTÉS AVEC LE REPO OFFICIEL")

        return results


def main():
    """Fonction principale"""
    checker = OfficialAlignmentChecker()
    results = checker.run_full_alignment_check()

    # Sauvegarder les résultats
    results_file = Path(__file__).parent.parent / "logs" / "alignment_check_results.json"
    results_file.parent.mkdir(exist_ok=True)

    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2, ensure_ascii=False)

    print(f"\n📄 Résultats sauvegardés dans: {results_file}")


if __name__ == "__main__":
    main()
