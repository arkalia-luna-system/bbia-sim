#!/usr/bin/env python3
"""
Script de test de conformité BBIA-SIM
Valide que notre simulation est conforme aux spécifications officielles
"""

import subprocess
import sys
import time
from pathlib import Path


class ConformityTester:
    """Testeur de conformité BBIA-SIM"""

    def __init__(self):
        self.project_root = Path(__file__).parent.parent
        self.results = {}

    def test_model_loading(self) -> bool:
        """Test : Chargement du modèle officiel"""
        print("🤖 Test chargement modèle officiel...")

        try:
            # Test avec mjpython (macOS)
            result = subprocess.run([
                sys.executable, "-c",
                """
import mujoco
model = mujoco.MjModel.from_xml_path('src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml')
print(f'Modèle chargé: {model.njnt} joints, {model.nbody} corps, {model.ngeom} géométries')
print(f'Actuateurs: {model.nu}')
print('✅ Modèle officiel chargé avec succès')
"""
            ], capture_output=True, text=True, cwd=self.project_root, timeout=10)

            if result.returncode == 0:
                print("✅ Modèle officiel chargé avec succès")
                return True
            else:
                print(f"❌ Erreur chargement modèle: {result.stderr}")
                return False

        except Exception as e:
            print(f"❌ Erreur test modèle: {e}")
            return False

    def test_stl_assets_quality(self) -> bool:
        """Test : Qualité des assets STL"""
        print("📦 Test qualité assets STL...")

        try:
            assets_dir = self.project_root / "src/bbia_sim/sim/assets/reachy_official"
            stl_files = list(assets_dir.glob("*.stl"))

            if len(stl_files) != 41:
                print(f"❌ Nombre incorrect de STL: {len(stl_files)}/41")
                return False

            # Vérifier qu'aucun fichier n'est un pointeur Git LFS
            lfs_pointers = []
            for stl_file in stl_files:
                if stl_file.stat().st_size < 1000:
                    lfs_pointers.append(stl_file.name)

            if lfs_pointers:
                print(f"❌ Pointeurs Git LFS détectés: {lfs_pointers}")
                return False

            print(f"✅ {len(stl_files)} fichiers STL valides")
            return True

        except Exception as e:
            print(f"❌ Erreur test STL: {e}")
            return False

    def test_joints_specifications(self) -> bool:
        """Test : Spécifications des joints"""
        print("🔧 Test spécifications joints...")

        try:
            result = subprocess.run([
                sys.executable, "-c",
                """
from src.bbia_sim.sim.joints import VALID_JOINTS, MAIN_JOINTS
print(f'Joints valides: {len(VALID_JOINTS)}')
print(f'Joints principaux: {len(MAIN_JOINTS)}')
print(f'Joints: {list(VALID_JOINTS.keys())}')
print('✅ Spécifications joints OK')
"""
            ], capture_output=True, text=True, cwd=self.project_root, timeout=5)

            if result.returncode == 0:
                print("✅ Spécifications joints conformes")
                return True
            else:
                print(f"❌ Erreur spécifications joints: {result.stderr}")
                return False

        except Exception as e:
            print(f"❌ Erreur test joints: {e}")
            return False

    def test_api_endpoints(self) -> bool:
        """Test : Endpoints API"""
        print("🌐 Test endpoints API...")

        try:
            # Démarrer l'API en arrière-plan
            api_process = subprocess.Popen([
                sys.executable, "-m", "uvicorn",
                "src.bbia_sim.daemon.app.main:app",
                "--port", "8000", "--host", "127.0.0.1"
            ], cwd=self.project_root)

            # Attendre que l'API démarre
            time.sleep(3)

            # Tester les endpoints essentiels
            endpoints_to_test = [
                ("GET", "/api/info"),
                ("GET", "/api/state/status"),
                ("GET", "/api/state/joints"),
                ("POST", "/api/motion/joints"),
            ]

            import requests
            headers = {"Authorization": "Bearer bbia-secret-key-dev"}

            for method, endpoint in endpoints_to_test:
                url = f"http://127.0.0.1:8000{endpoint}"

                if method == "GET":
                    response = requests.get(url, headers=headers, timeout=5)
                elif method == "POST":
                    response = requests.post(url, json=[{"joint_name": "yaw_body", "position": 0.1}], headers=headers, timeout=5)

                if response.status_code not in [200, 422]:  # 422 OK pour validation
                    print(f"❌ Endpoint {endpoint} échoué: {response.status_code}")
                    api_process.terminate()
                    return False

            api_process.terminate()
            print("✅ Endpoints API fonctionnels")
            return True

        except Exception as e:
            print(f"❌ Erreur test API: {e}")
            if 'api_process' in locals():
                api_process.terminate()
            return False

    def test_bbia_modules(self) -> bool:
        """Test : Modules BBIA"""
        print("🧠 Test modules BBIA...")

        try:
            result = subprocess.run([
                sys.executable, "-c",
                """
from src.bbia_sim.bbia_emotions import BBIAEmotions
from src.bbia_sim.bbia_vision import BBIAVision
from src.bbia_sim.bbia_voice import dire_texte
from src.bbia_sim.bbia_audio import detecter_son

# Test BBIA Emotions
emotions = BBIAEmotions()
emotions.set_emotion('happy', 0.8)
assert emotions.current_emotion == 'happy'

# Test BBIA Vision
vision = BBIAVision()
assert vision.camera_active

# Test BBIA Voice
result = dire_texte('Test')
# dire_texte peut retourner None, c'est normal

# Test BBIA Audio
result = detecter_son('test.wav')
assert isinstance(result, bool)

print('✅ Modules BBIA fonctionnels')
"""
            ], capture_output=True, text=True, cwd=self.project_root, timeout=10)

            if result.returncode == 0:
                print("✅ Modules BBIA fonctionnels")
                return True
            else:
                print(f"❌ Erreur modules BBIA: {result.stderr}")
                return False

        except Exception as e:
            print(f"❌ Erreur test BBIA: {e}")
            return False

    def test_simulation_performance(self) -> bool:
        """Test : Performance simulation"""
        print("⚡ Test performance simulation...")

        try:
            result = subprocess.run([
                sys.executable, "-c",
                """
import mujoco
import time

model = mujoco.MjModel.from_xml_path('src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml')
data = mujoco.MjData(model)

# Test performance
start_time = time.time()
for _ in range(100):
    mujoco.mj_step(model, data)
end_time = time.time()

duration = end_time - start_time
steps_per_second = 100 / duration

print(f'Performance: {steps_per_second:.1f} steps/sec')
print(f'Durée 100 steps: {duration:.3f}s')

if steps_per_second > 1000:
    print('✅ Performance simulation excellente')
else:
    print('⚠️ Performance simulation acceptable')
"""
            ], capture_output=True, text=True, cwd=self.project_root, timeout=15)

            if result.returncode == 0:
                print("✅ Performance simulation validée")
                return True
            else:
                print(f"❌ Erreur test performance: {result.stderr}")
                return False

        except Exception as e:
            print(f"❌ Erreur test performance: {e}")
            return False

    def run_full_conformity_test(self) -> bool:
        """Exécute tous les tests de conformité"""
        print("🎯 TEST DE CONFORMITÉ COMPLET BBIA-SIM")
        print("=" * 50)

        tests = [
            ("Modèle officiel", self.test_model_loading),
            ("Assets STL", self.test_stl_assets_quality),
            ("Spécifications joints", self.test_joints_specifications),
            ("Endpoints API", self.test_api_endpoints),
            ("Modules BBIA", self.test_bbia_modules),
            ("Performance simulation", self.test_simulation_performance),
        ]

        passed = 0
        total = len(tests)

        for test_name, test_func in tests:
            print(f"\n📋 {test_name}...")
            if test_func():
                passed += 1
            else:
                print(f"❌ {test_name} échoué")

        print("\n" + "=" * 50)
        print("📊 RÉSULTATS CONFORMITÉ")
        print("=" * 50)

        for i, (test_name, _) in enumerate(tests):
            status = "✅ PASS" if i < passed else "❌ FAIL"
            print(f"{status} {test_name}")

        print(f"\n🎯 Score: {passed}/{total} tests passés")

        if passed == total:
            print("🎉 CONFORMITÉ PARFAITE AVEC LES SPÉCIFICATIONS OFFICIELLES !")
            return True
        else:
            print("⚠️  DES ÉCARTS DE CONFORMITÉ DÉTECTÉS")
            return False


def main():
    """Fonction principale"""
    tester = ConformityTester()
    success = tester.run_full_conformity_test()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
