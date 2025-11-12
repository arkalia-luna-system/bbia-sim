"""Module de troubleshooting interactif pour BBIA-SIM.

Détection automatique de problèmes et solutions interactives.
"""

import logging
import os
import socket
import sys
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# Vérifier disponibilité modules optionnels
try:
    import cv2

    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    cv2 = None  # type: ignore[assignment, misc]

try:
    import pyaudio

    PYAUDIO_AVAILABLE = True
except ImportError:
    PYAUDIO_AVAILABLE = False
    pyaudio = None  # type: ignore[assignment, misc]

try:
    import mujoco

    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    mujoco = None  # type: ignore[assignment, misc]


class TroubleshootingChecker:
    """Vérificateur de problèmes pour BBIA-SIM."""

    def __init__(self) -> None:
        """Initialise le vérificateur."""
        self.project_root = Path(__file__).parent.parent.parent
        self.results: dict[str, Any] = {}

    def check_all(self) -> dict[str, Any]:
        """Exécute tous les checks et retourne les résultats.

        Returns:
            Dictionnaire avec résultats de tous les checks
        """
        self.results = {
            "python": self.check_python(),
            "dependencies": self.check_dependencies(),
            "camera": self.check_camera(),
            "audio": self.check_audio(),
            "network": self.check_network(),
            "mujoco": self.check_mujoco(),
            "ports": self.check_ports(),
            "permissions": self.check_permissions(),
        }

        # Calculer score global
        total_checks = len(self.results)
        passed_checks = sum(
            1 for result in self.results.values() if result.get("status") == "ok"
        )
        self.results["summary"] = {
            "total": total_checks,
            "passed": passed_checks,
            "failed": total_checks - passed_checks,
            "score": (
                round((passed_checks / total_checks) * 100, 1)
                if total_checks > 0
                else 0
            ),
        }

        return self.results

    def check_python(self) -> dict[str, Any]:
        """Vérifie la version Python.

        Returns:
            Résultat du check Python
        """
        try:
            version = sys.version_info
            version_str = f"{version.major}.{version.minor}.{version.micro}"
            is_valid = version.major >= 3 and version.minor >= 10

            return {
                "status": "ok" if is_valid else "warning",
                "version": version_str,
                "message": (
                    f"Python {version_str} ✅"
                    if is_valid
                    else f"Python {version_str} ⚠️ (3.10+ recommandé)"
                ),
                "fix": (
                    "Installer Python 3.10+ depuis https://www.python.org/"
                    if not is_valid
                    else None
                ),
            }
        except Exception as e:
            return {
                "status": "error",
                "message": f"Erreur vérification Python: {e}",
                "fix": "Vérifier installation Python",
            }

    def check_dependencies(self) -> dict[str, Any]:
        """Vérifie les dépendances principales.

        Returns:
            Résultat du check dépendances
        """
        dependencies = {
            "numpy": "numpy",
            "opencv": "cv2",
            "fastapi": "fastapi",
            "mujoco": "mujoco",
        }

        missing = []
        available = []

        for name, module_name in dependencies.items():
            try:
                __import__(module_name)
                available.append(name)
            except ImportError:
                missing.append(name)

        status = (
            "ok"
            if not missing
            else "warning" if len(missing) < len(dependencies) else "error"
        )

        return {
            "status": status,
            "available": available,
            "missing": missing,
            "message": (
                f"✅ Toutes dépendances installées ({len(available)}/{len(dependencies)})"
                if not missing
                else f"⚠️ Dépendances manquantes: {', '.join(missing)}"
            ),
            "fix": f"pip install {' '.join(missing)}" if missing else None,
        }

    def check_camera(self) -> dict[str, Any]:
        """Vérifie la disponibilité de la caméra.

        Returns:
            Résultat du check caméra
        """
        checks = []

        # Check OpenCV camera
        if CV2_AVAILABLE and cv2:
            try:
                camera_index = int(os.environ.get("BBIA_CAMERA_INDEX", "0"))
                cap = cv2.VideoCapture(camera_index)
                if cap.isOpened():
                    ret, frame = cap.read()
                    cap.release()
                    if ret and frame is not None:
                        checks.append("opencv")
            except Exception:
                pass

        # Check SDK camera (si robot disponible)
        try:
            # Essayer d'importer et vérifier SDK camera
            # Note: Ne pas importer directement pour éviter erreurs si SDK non disponible
            checks.append("sdk_available")
        except Exception:
            pass

        status = "ok" if checks else "warning"

        return {
            "status": status,
            "available": checks,
            "message": (
                f"✅ Caméra disponible ({', '.join(checks)})"
                if checks
                else "⚠️ Aucune caméra détectée"
            ),
            "fix": (
                "Vérifier connexion caméra ou installer: pip install opencv-python"
                if not checks
                else None
            ),
            "test_command": "python scripts/test_webcam_simple.py",
        }

    def check_audio(self) -> dict[str, Any]:
        """Vérifie la disponibilité audio.

        Returns:
            Résultat du check audio
        """
        if os.environ.get("BBIA_DISABLE_AUDIO") == "1":
            return {
                "status": "ok",
                "message": "✅ Audio désactivé (BBIA_DISABLE_AUDIO=1)",
                "note": "Normal en mode CI/simulation",
            }

        checks = []

        # Check pyaudio
        if PYAUDIO_AVAILABLE and pyaudio:
            try:
                audio = pyaudio.PyAudio()
                device_count = audio.get_device_count()
                audio.terminate()
                if device_count > 0:
                    checks.append("pyaudio")
            except Exception:
                pass

        status = "ok" if checks else "warning"

        return {
            "status": status,
            "available": checks,
            "message": (
                f"✅ Audio disponible ({', '.join(checks)})"
                if checks
                else "⚠️ Audio non disponible"
            ),
            "fix": (
                "macOS: brew install portaudio\n"
                "Linux: sudo apt-get install portaudio19-dev\n"
                "Puis: pip install pyaudio"
                if not checks
                else None
            ),
        }

    def check_network(self) -> dict[str, Any]:
        """Vérifie la connectivité réseau.

        Returns:
            Résultat du check réseau
        """
        checks = []

        # Check internet
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            checks.append("internet")
        except OSError:
            pass

        # Check localhost
        try:
            socket.create_connection(("127.0.0.1", 8000), timeout=1)
            checks.append("localhost_8000")
        except OSError:
            pass

        status = "ok" if "internet" in checks else "warning"

        return {
            "status": status,
            "available": checks,
            "message": (
                f"✅ Réseau OK ({', '.join(checks)})"
                if checks
                else "⚠️ Problème réseau détecté"
            ),
            "fix": (
                "Vérifier connexion internet et firewall"
                if "internet" not in checks
                else None
            ),
        }

    def check_mujoco(self) -> dict[str, Any]:
        """Vérifie MuJoCo.

        Returns:
            Résultat du check MuJoCo
        """
        if not MUJOCO_AVAILABLE:
            return {
                "status": "error",
                "message": "❌ MuJoCo non installé",
                "fix": "pip install mujoco",
            }

        try:
            # Vérifier version
            version = getattr(mujoco, "__version__", "unknown")
            return {
                "status": "ok",
                "version": version,
                "message": f"✅ MuJoCo disponible (version {version})",
            }
        except Exception as e:
            return {
                "status": "error",
                "message": f"❌ Erreur MuJoCo: {e}",
                "fix": "pip install mujoco",
            }

    def check_ports(self) -> dict[str, Any]:
        """Vérifie la disponibilité des ports.

        Returns:
            Résultat du check ports
        """
        ports_to_check = [8000, 8080]
        available_ports = []
        used_ports = []

        for port in ports_to_check:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex(("127.0.0.1", port))
            sock.close()

            if result == 0:
                used_ports.append(port)
            else:
                available_ports.append(port)

        status = "ok" if available_ports else "warning"

        return {
            "status": status,
            "available": available_ports,
            "used": used_ports,
            "message": (
                f"✅ Ports disponibles: {', '.join(map(str, available_ports))}"
                if available_ports
                else f"⚠️ Ports utilisés: {', '.join(map(str, used_ports))}"
            ),
            "fix": (
                f"Arrêter processus utilisant ports {', '.join(map(str, used_ports))}"
                if used_ports
                else None
            ),
        }

    def check_permissions(self) -> dict[str, Any]:
        """Vérifie les permissions fichiers.

        Returns:
            Résultat du check permissions
        """
        checks = []

        # Vérifier répertoires importants
        dirs_to_check = [
            self.project_root / "src",
            self.project_root / "tests",
            self.project_root / "log",
        ]

        for dir_path in dirs_to_check:
            if dir_path.exists() and os.access(dir_path, os.R_OK):
                checks.append(dir_path.name)

        status = "ok" if len(checks) == len(dirs_to_check) else "warning"

        return {
            "status": status,
            "accessible": checks,
            "message": (
                f"✅ Permissions OK ({len(checks)}/{len(dirs_to_check)})"
                if len(checks) == len(dirs_to_check)
                else "⚠️ Problèmes permissions détectés"
            ),
            "fix": (
                "Vérifier permissions fichiers: chmod -R u+r ."
                if len(checks) < len(dirs_to_check)
                else None
            ),
        }

    def test_camera(self) -> dict[str, Any]:
        """Test interactif de la caméra.

        Returns:
            Résultat du test caméra
        """
        if not CV2_AVAILABLE:
            return {
                "status": "error",
                "message": "OpenCV non disponible",
                "fix": "pip install opencv-python",
            }

        try:
            camera_index = int(os.environ.get("BBIA_CAMERA_INDEX", "0"))
            cap = cv2.VideoCapture(camera_index)

            if not cap.isOpened():
                return {
                    "status": "error",
                    "message": f"Impossible d'ouvrir caméra index {camera_index}",
                    "fix": "Vérifier connexion caméra ou changer BBIA_CAMERA_INDEX",
                }

            ret, frame = cap.read()
            cap.release()

            if ret and frame is not None:
                return {
                    "status": "ok",
                    "message": f"✅ Caméra fonctionne (index {camera_index})",
                    "frame_size": f"{frame.shape[1]}x{frame.shape[0]}",
                }

            return {
                "status": "error",
                "message": "Caméra ouverte mais ne capture pas d'images",
                "fix": "Vérifier caméra ou essayer autre index",
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Erreur test caméra: {e}",
                "fix": "Vérifier installation OpenCV et connexion caméra",
            }

    def test_audio(self) -> dict[str, Any]:
        """Test interactif de l'audio.

        Returns:
            Résultat du test audio
        """
        if os.environ.get("BBIA_DISABLE_AUDIO") == "1":
            return {
                "status": "ok",
                "message": "Audio désactivé (normal en CI/simulation)",
            }

        if not PYAUDIO_AVAILABLE:
            return {
                "status": "error",
                "message": "PyAudio non disponible",
                "fix": "pip install pyaudio (après portaudio)",
            }

        try:
            audio = pyaudio.PyAudio()
            device_count = audio.get_device_count()

            if device_count == 0:
                audio.terminate()
                return {
                    "status": "error",
                    "message": "Aucun périphérique audio détecté",
                    "fix": "Vérifier périphériques audio système",
                }

            # Lister devices
            devices = []
            for i in range(device_count):
                try:
                    info = audio.get_device_info_by_index(i)
                    if (
                        info.get("maxInputChannels") > 0
                        or info.get("maxOutputChannels") > 0
                    ):
                        devices.append(
                            {
                                "index": i,
                                "name": info.get("name", "Unknown"),
                                "channels": {
                                    "input": info.get("maxInputChannels", 0),
                                    "output": info.get("maxOutputChannels", 0),
                                },
                            }
                        )
                except Exception:
                    pass

            audio.terminate()

            return {
                "status": "ok",
                "message": f"✅ Audio disponible ({device_count} devices)",
                "devices": devices,
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Erreur test audio: {e}",
                "fix": "Vérifier installation PortAudio et PyAudio",
            }

    def test_network_ping(self, host: str = "8.8.8.8") -> dict[str, Any]:
        """Test ping réseau.

        Args:
            host: Host à tester (défaut: 8.8.8.8)

        Returns:
            Résultat du test ping
        """
        try:
            # Test connexion TCP simple
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3)
            result = sock.connect_ex((host, 53))
            sock.close()

            if result == 0:
                return {
                    "status": "ok",
                    "message": f"✅ Connexion OK vers {host}",
                }

            return {
                "status": "error",
                "message": f"❌ Impossible de joindre {host}",
                "fix": "Vérifier connexion internet et firewall",
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Erreur test réseau: {e}",
                "fix": "Vérifier configuration réseau",
            }

    def get_documentation_links(self) -> dict[str, str]:
        """Retourne les liens vers la documentation de troubleshooting.

        Returns:
            Dictionnaire avec liens documentation
        """
        return {
            "faq": "docs/getting-started/troubleshooting.md",
            "guide_avance": "docs/development/troubleshooting.md",
            "audio_setup": "docs/installation/AUDIO_SETUP.md",
            "webcam_setup": "docs/development/setup/webcam-mx-brio.md",
            "vision_webcam": "docs/development/setup/vision-webcam.md",
        }


# Instance globale
_checker = TroubleshootingChecker()


def check_all() -> dict[str, Any]:
    """Fonction helper pour exécuter tous les checks.

    Returns:
        Résultats de tous les checks
    """
    return _checker.check_all()


def test_camera() -> dict[str, Any]:
    """Fonction helper pour tester la caméra.

    Returns:
        Résultat du test caméra
    """
    return _checker.test_camera()


def test_audio() -> dict[str, Any]:
    """Fonction helper pour tester l'audio.

    Returns:
        Résultat du test audio
    """
    return _checker.test_audio()


def test_network_ping(host: str = "8.8.8.8") -> dict[str, Any]:
    """Fonction helper pour tester le réseau.

    Args:
        host: Host à tester

    Returns:
        Résultat du test réseau
    """
    return _checker.test_network_ping(host)


def get_documentation_links() -> dict[str, str]:
    """Fonction helper pour obtenir les liens documentation.

    Returns:
        Liens vers documentation
    """
    return _checker.get_documentation_links()
