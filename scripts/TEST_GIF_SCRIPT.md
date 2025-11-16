# 🧪 Commandes pour tester le script GIF

## 1. Activer le venv

```bash
# Depuis la racine du projet
cd /Volumes/T7/bbia-reachy-sim

# Activer le venv (selon votre système)
source venv/bin/activate  # Linux/macOS
# OU
source .venv/bin/activate  # Si le venv est nommé .venv
```

## 2. Vérifier les dépendances

```bash
# Vérifier que MuJoCo est installé
python3 -c "import mujoco; import logging; logging.info('✅ MuJoCo version:', mujoco.__version__)"

# OU avec mjpython si disponible
mjpython -c "import mujoco; import logging; logging.info('✅ MuJoCo version:', mujoco.__version__)"
```

## 3. Tester le script (sans lancer - vérification syntaxe)

```bash
# Vérifier la syntaxe Python
python3 -m py_compile scripts/generate_emotions_sequence_for_gif.py
echo "✅ Syntaxe OK" || echo "❌ Erreur de syntaxe"
```

## 4. Tester avec différents angles (pour trouver le bon azimuth)

```bash
# Test avec azimuth 90° (par défaut - face)
mjpython scripts/generate_emotions_sequence_for_gif.py --azimuth 90

# Si le robot est de côté, tester d'autres valeurs :
mjpython scripts/generate_emotions_sequence_for_gif.py --azimuth 0
mjpython scripts/generate_emotions_sequence_for_gif.py --azimuth 180
mjpython scripts/generate_emotions_sequence_for_gif.py --azimuth 270
mjpython scripts/generate_emotions_sequence_for_gif.py --azimuth -90
```

## 5. Vérifier qu'il n'y a pas de viewers bloqués après

```bash
# Vérifier les processus MuJoCo
ps aux | grep -i mujoco | grep -v grep || echo "✅ Aucun processus MuJoCo"

# Si des processus persistent
pkill -9 -f mujoco 2>/dev/null
pkill -9 -f mjpython 2>/dev/null
```

## 6. Test complet (avec arrêt après 2 secondes pour vérifier)

```bash
# Lancer le script puis arrêter avec Ctrl+C après avoir vu la caméra
mjpython scripts/generate_emotions_sequence_for_gif.py --azimuth 90
# Appuyer sur Ctrl+C après avoir vérifié la position de la caméra
```

## 7. Vérifier les imports

```bash
# Vérifier que tous les imports fonctionnent
python3 -c "
import sys
from pathlib import Path
sys.path.insert(0, str(Path('src')))
from bbia_sim.robot_factory import RobotFactory
logging.info('✅ Tous les imports OK')
"
```

## 8. Commandes de nettoyage (si problème)

```bash
# Fermer tous les viewers MuJoCo
./scripts/kill_mujoco_viewers.sh

# OU
pkill -9 -f mujoco
pkill -9 -f mjpython
```

## 9. Test rapide (vérification que le script se lance)

```bash
# Test rapide : lancer et arrêter immédiatement avec Ctrl+C
# Cela vérifie que le viewer se lance correctement
timeout 5 mjpython scripts/generate_emotions_sequence_for_gif.py --azimuth 90 || true
```

## 📝 Notes importantes

- **Fond noir** : C'est normal, le viewer MuJoCo ne peut pas changer le fond. Utilisez post-production vidéo.
- **Caméra de côté** : Testez différentes valeurs d'`--azimuth` pour trouver la bonne orientation.
- **Viewer bloqué** : Si une fenêtre reste ouverte, fermez-la avec `Cmd+Q` ou utilisez `kill_mujoco_viewers.sh`.
