# 🧪 Tests de Préparation - Avant Arrivée Robot

**Date** : 8 Décembre 2025  
**Objectif** : Tester l'environnement local avant l'arrivée du robot Reachy Mini  
**Durée estimée** : 30-60 minutes

> **Note** : Ces tests peuvent être effectués **MAINTENANT** (sans robot physique) pour valider que tout est prêt.

---

## 📋 Checklist Rapide

- [ ] ✅ Test connexion Zenoh locale
- [ ] ✅ Validation daemon `reachy-mini-daemon` (mode simulation)
- [ ] ✅ Vérification communication réseau (préparation WiFi)
- [ ] ✅ Tests SDK officiel (imports et compatibilité)
- [ ] ✅ Tests backend BBIA-SIM

> **💡 Astuce** : Utilisez `python scripts/bbia_doctor.py` pour un diagnostic automatique complet de tous ces éléments !

---

## 1. 🔌 Test Connexion Zenoh Locale

### Objectif
Vérifier que Zenoh est installé et peut créer une session locale.

### Test 1 : Import Zenoh

```bash
# Activer l'environnement virtuel
source venv/bin/activate

# Test import Zenoh
python -c "import zenoh; print('✅ Zenoh installé:', zenoh.__version__)"
```

**Résultat attendu :**
```
✅ Zenoh installé: 1.x.x
```

### Test 2 : Session Zenoh Locale

```bash
# Utiliser le script de démonstration existant
python scripts/demo_bridge_zenoh.py
```

**Résultat attendu :**
```
🔌 Test de connexion Bridge Zenoh...
✅ Bridge Zenoh connecté avec succès
✅ Commande envoyée avec succès
📊 État robot: {...}
```

### Test 3 : Test Zenoh Direct (Script Manuel)

```python
# test_zenoh_local.py
import zenoh
from zenoh import Config

# Créer une session Zenoh locale
config = Config()
config.insert_json5("mode", '"client"')
config.insert_json5("connect", '["tcp://localhost:7447"]')

session = zenoh.open(config)
print("✅ Session Zenoh créée avec succès")

# Publier un message de test
pub = session.declare_publisher("test/zenoh")
pub.put("Hello Zenoh!")
print("✅ Message publié")

# Fermer la session
session.close()
print("✅ Session fermée")
```

**Exécuter :**
```bash
python test_zenoh_local.py
```

---

## 2. 🟣 Validation Daemon `reachy-mini-daemon`

### Objectif
Vérifier que le daemon officiel peut être lancé en mode simulation.

### Prérequis

```bash
# Installer le SDK officiel (si pas déjà fait)
pip install -U reachy-mini

# Installer MuJoCo pour simulation (optionnel mais recommandé)
pip install -U "reachy-mini[mujoco]"
```

### Test 1 : Vérifier Installation Daemon

```bash
# Vérifier que la commande existe
which reachy-mini-daemon

# Vérifier version
reachy-mini-daemon --version
```

**Résultat attendu :**
```
/usr/local/bin/reachy-mini-daemon
# ou
~/.local/bin/reachy-mini-daemon
```

### Test 2 : Lancer Daemon en Mode Simulation

```bash
# Terminal 1 : Lancer le daemon en simulation
reachy-mini-daemon --sim

# Ou sur macOS avec MuJoCo :
mjpython -m reachy_mini.daemon.app.main --sim --scene minimal
```

**Résultat attendu :**
```
🚀 Daemon Reachy Mini démarré (mode simulation)
📡 API disponible sur http://localhost:8000
```

### Test 3 : Vérifier API du Daemon

```bash
# Terminal 2 : Tester l'API (pendant que le daemon tourne)
curl http://localhost:8000/api/state/full

# Ou ouvrir dans le navigateur :
# http://localhost:8000/docs
```

**Résultat attendu :**
```json
{
  "head": {...},
  "body": {...},
  "antennas": {...},
  "timestamp": "..."
}
```

### Test 4 : Health Checks

```bash
# Health check (liveness)
curl http://localhost:8000/metrics/healthz

# Readiness check
curl http://localhost:8000/metrics/readyz

# Health détaillé
curl http://localhost:8000/metrics/health
```

**Résultat attendu :**
```
200 OK
```

---

## 3. 📡 Vérification Communication Réseau (Préparation WiFi)

### Objectif
Préparer la configuration réseau pour quand le robot arrivera.

### Test 1 : Vérifier Configuration Réseau Locale

```bash
# Vérifier IP locale
ifconfig | grep "inet "  # macOS/Linux
# ou
ipconfig getifaddr en0  # macOS
# ou
hostname -I  # Linux

# Vérifier ports disponibles
netstat -an | grep LISTEN | grep -E "8000|7447"
```

**Résultat attendu :**
```
inet 192.168.x.x  # Votre IP locale
```

### Test 2 : Test Connexion Locale (Simulation)

```bash
# Tester connexion localhost
ping localhost

# Tester port 8000 (daemon)
curl http://localhost:8000/api/state/full

# Tester port 7447 (Zenoh par défaut)
# (nécessite un serveur Zenoh en cours d'exécution)
```

### Test 3 : Script de Préparation WiFi (Pour Plus Tard)

```python
# scripts/prepare_wifi_connection.py
#!/usr/bin/env python3
"""Script de préparation WiFi pour robot Reachy Mini"""

import socket
import subprocess
import sys

def get_local_ip():
    """Récupère l'IP locale."""
    try:
        # Créer une connexion socket pour déterminer l'IP locale
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

def check_port_open(host, port):
    """Vérifie si un port est ouvert."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1)
    result = sock.connect_ex((host, port))
    sock.close()
    return result == 0

def main():
    print("📡 Préparation WiFi pour Reachy Mini")
    print("=" * 50)
    
    # IP locale
    local_ip = get_local_ip()
    print(f"✅ IP locale: {local_ip}")
    
    # Ports à vérifier
    ports = {
        8000: "API Daemon",
        7447: "Zenoh",
    }
    
    print("\n🔍 Vérification ports:")
    for port, name in ports.items():
        if check_port_open("localhost", port):
            print(f"  ✅ Port {port} ({name}): OUVERT")
        else:
            print(f"  ⚠️  Port {port} ({name}): FERMÉ (normal si daemon non lancé)")
    
    print("\n📋 Checklist WiFi (pour robot réel):")
    print("  [ ] Robot et PC sur même réseau WiFi")
    print(f"  [ ] IP robot connue (scanner réseau ou configurée)")
    print(f"  [ ] Firewall autorise ports 8000 et 7447")
    print(f"  [ ] Test ping: ping <robot_ip>")
    print(f"  [ ] Test API: curl http://<robot_ip>:8000/api/state/full")
    
    print("\n✅ Préparation réseau terminée")

if __name__ == "__main__":
    main()
```

**Exécuter :**
```bash
python scripts/prepare_wifi_connection.py
```

---

## 4. 🧪 Tests SDK Officiel

### Objectif
Vérifier que le SDK officiel est installé et compatible.

### Test 1 : Import SDK

```bash
python -c "from reachy_mini import ReachyMini; print('✅ SDK Reachy Mini installé')"
```

**Résultat attendu :**
```
✅ SDK Reachy Mini installé
```

### Test 2 : Test Compatibilité (Sans Robot)

```bash
# Utiliser le script de test existant
python -m pytest tests/test_sdk_dependencies.py -v
```

**Résultat attendu :**
```
test_reachy_mini_import ... ✅
test_zenoh_import ... ✅
test_motor_controller_import ... ✅
...
```

### Test 3 : Test Backend BBIA-SIM

```python
# test_backend_preparation.py
from bbia_sim.robot_factory import RobotFactory

# Créer backend reachy_mini (mode simulation)
robot = RobotFactory.create_backend("reachy_mini", use_sim=True)

# Tester connexion (mode simulation)
if robot.connect():
    print("✅ Backend reachy_mini créé avec succès")
    print(f"✅ Connexion: {robot.is_connected}")
    print(f"✅ Méthodes disponibles: {[m for m in dir(robot) if not m.startswith('_')]}")
else:
    print("⚠️ Connexion échouée (normal en simulation sans daemon)")
```

**Exécuter :**
```bash
python test_backend_preparation.py
```

---

## 5. ✅ Tests Complets (Script Unifié)

### Option 1 : BBIA Doctor (Recommandé)

Le script `bbia_doctor.py` vérifie automatiquement tous les éléments de préparation :

```bash
# Diagnostic complet automatique
python scripts/bbia_doctor.py
# ou
python -m bbia_sim --doctor
```

**Vérifie automatiquement :**
- ✅ Zenoh (installation + session locale)
- ✅ Daemon `reachy-mini-daemon`
- ✅ Réseau (IP locale + ports)
- ✅ Dépendances (zenoh, reachy_mini, etc.)
- ✅ Configuration projet

### Option 2 : Script de Test Complet (Manuel)

```python
#!/usr/bin/env python3
# scripts/test_preparation_robot.py
"""Script de test complet avant arrivée robot"""

import sys
import subprocess
from pathlib import Path

def test_zenoh():
    """Test 1: Zenoh."""
    print("🔌 Test 1: Zenoh")
    try:
        import zenoh
        print(f"  ✅ Zenoh installé: {zenoh.__version__}")
        return True
    except ImportError:
        print("  ❌ Zenoh non installé")
        return False

def test_daemon():
    """Test 2: Daemon."""
    print("\n🟣 Test 2: Daemon reachy-mini-daemon")
    try:
        result = subprocess.run(
            ["which", "reachy-mini-daemon"],
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            print(f"  ✅ Daemon trouvé: {result.stdout.strip()}")
            return True
        else:
            print("  ⚠️  Daemon non trouvé (installer: pip install reachy-mini)")
            return False
    except Exception as e:
        print(f"  ❌ Erreur: {e}")
        return False

def test_sdk():
    """Test 3: SDK officiel."""
    print("\n📦 Test 3: SDK Reachy Mini")
    try:
        from reachy_mini import ReachyMini
        print("  ✅ SDK Reachy Mini importé")
        return True
    except ImportError:
        print("  ⚠️  SDK non installé (installer: pip install reachy-mini)")
        return False

def test_backend():
    """Test 4: Backend BBIA-SIM."""
    print("\n🤖 Test 4: Backend BBIA-SIM")
    try:
        sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
        from bbia_sim.robot_factory import RobotFactory
        
        robot = RobotFactory.create_backend("reachy_mini", use_sim=True)
        print("  ✅ Backend reachy_mini créé")
        return True
    except Exception as e:
        print(f"  ❌ Erreur: {e}")
        return False

def test_network():
    """Test 5: Réseau."""
    print("\n📡 Test 5: Configuration réseau")
    import socket
    
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        print(f"  ✅ IP locale: {ip}")
        return True
    except Exception:
        print("  ⚠️  Impossible de déterminer IP locale")
        return False

def main():
    print("🧪 Tests de Préparation - Avant Arrivée Robot")
    print("=" * 60)
    
    results = {
        "Zenoh": test_zenoh(),
        "Daemon": test_daemon(),
        "SDK": test_sdk(),
        "Backend": test_backend(),
        "Réseau": test_network(),
    }
    
    print("\n" + "=" * 60)
    print("📊 Résumé:")
    for name, result in results.items():
        status = "✅" if result else "❌"
        print(f"  {status} {name}")
    
    all_ok = all(results.values())
    if all_ok:
        print("\n🎉 Tous les tests passent ! Vous êtes prêt pour le robot.")
    else:
        print("\n⚠️  Certains tests ont échoué. Vérifiez les erreurs ci-dessus.")
    
    return 0 if all_ok else 1

if __name__ == "__main__":
    sys.exit(main())
```

**Exécuter :**
```bash
python scripts/test_preparation_robot.py
```

---

## 6. 📋 Checklist Finale

### Avant l'Arrivée du Robot

- [ ] ✅ Zenoh installé et testé localement
- [ ] ✅ Daemon `reachy-mini-daemon` peut être lancé en simulation
- [ ] ✅ SDK officiel `reachy-mini` installé et importable
- [ ] ✅ Backend BBIA-SIM `reachy_mini` fonctionne en simulation
- [ ] ✅ IP locale connue et ports vérifiés
- [ ] ✅ Scripts de test exécutés avec succès

### Le Jour de l'Arrivée

- [ ] 🔌 Robot allumé (LED verte)
- [ ] 📡 Robot connecté au WiFi (même réseau que PC)
- [ ] 🔍 IP robot identifiée (scan réseau ou configurée)
- [ ] 🟣 Daemon lancé sur robot (ou PC selon configuration)
- [ ] ✅ Test connexion: `curl http://<robot_ip>:8000/api/state/full`
- [ ] ✅ Test BBIA-SIM: `python examples/reachy_mini/minimal_demo.py` (⚠️ `demo_reachy_mini_corrigee.py` est déprécié)

---

## 🚀 Commandes Rapides

### Test Complet en Une Ligne (Recommandé)

```bash
# Activer venv + diagnostic automatique complet
source venv/bin/activate && python scripts/bbia_doctor.py
# ou
python -m bbia_sim --doctor
```

**Vérifie automatiquement :** Zenoh, daemon, WiFi, dépendances, configuration

### Lancer Daemon + Test API

```bash
# Terminal 1
reachy-mini-daemon --sim

# Terminal 2
curl http://localhost:8000/api/state/full
```

### Test Zenoh Bridge

```bash
python scripts/demo_bridge_zenoh.py
```

---

## 📚 Références

- **Guide complet** : `docs/guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md`
- **Checklist hardware** : `docs/hardware/CHECKLIST_VALIDATION_HARDWARE_DECEMBRE_2025.md`
- **Script bridge Zenoh** : `scripts/demo_bridge_zenoh.py`
- **Tests SDK** : `tests/test_sdk_dependencies.py`
- **Diagnostic automatique** : `python scripts/bbia_doctor.py` (vérifie Zenoh, daemon, WiFi)

---

**Date création :** Oct / Nov. 2025  
**Statut :** ✅ Prêt pour tests locaux (sans robot physique)  
**Dernière mise à jour :** Oct / Nov. 2025 (ajout tests Zenoh/daemon/WiFi dans bbia_doctor)

