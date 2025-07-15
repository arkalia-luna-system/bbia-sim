# 🚀 Premiers pas Reachy Mini avec BBIA

Bienvenue ! Ce guide vous explique comment utiliser BBIA pour simuler et programmer un Reachy Mini Wireless, en respectant les standards et comportements officiels.

---

## 1. Installation rapide

```bash
# Cloner le dépôt
 git clone https://github.com/arkalia-luna-system/bbia-sim.git
 cd bbia-sim

# Installer les dépendances Python
 python -m venv venv
 source venv/bin/activate
 pip install -r requirements.txt
```

---

## 2. Lancer la simulation BBIA

```bash
# Lancer la démo complète (tous modules)
python tests/demo_bbia_complete.py

# Lancer un test de comportement individuel
python src/bbia_sim/bbia_behavior.py
```

---

## 3. Exemples de comportements Reachy Mini (BBIA)

| Comportement Reachy Mini | Exemple de code BBIA |
|-------------------------|----------------------|
| Saluer (wave)           | `manager.execute_behavior("greeting")` |
| Tourner la tête (look)  | `manager.execute_behavior("wake_up")` ou `manager.execute_behavior("vision_tracking")` |
| Bouger les antennes     | `manager.execute_behavior("antenna_animation", {"emotion": "happy"})` |
| Écouter (listen)        | `manager.execute_behavior("conversation")` |
| Répondre (answer)       | `manager.execute_behavior("conversation")` |
| Reconnaître un visage   | `manager.execute_behavior("vision_tracking")` |
| Réagir à une émotion    | `manager.execute_behavior("emotional_response", {"stimulus": "compliment"})` |
| Mode simulation         | `python tests/demo_bbia_complete.py` |

---

## 4. Ajouter un nouveau comportement

1. Créez une nouvelle classe héritant de `BBIABehavior` dans `bbia_behavior.py`.
2. Ajoutez-la dans la méthode `_register_default_behaviors()` du `BBIABehaviorManager`.
3. Testez avec :
```python
manager.execute_behavior("nom_du_comportement")
```

---

## 5. Ressources utiles
- [Documentation complète](📋_INDEX_DOCUMENTATION.md)
- [Tableau de compatibilité Reachy Mini](📋_ETAT_ACTUEL_FINAL.md)
- [Communauté Reachy](https://pollen-robotics.com/reachy)

---

**BBIA est 100% open source et compatible Reachy Mini Wireless.**
N’hésitez pas à contribuer ou à proposer de nouveaux comportements ! 

---

## 🛠️ Conseils essentiels pour réussir avec BBIA

- Teste tout régulièrement (tests unitaires, démo)
- Sauvegarde et versionne ton code
- Documente chaque étape et limitation
- Prends soin de la sécurité (arrêt d’urgence, limites de mouvement)
- Prépare la calibration et la configuration
- Loggue toutes les erreurs et actions
- Range et nettoie les fichiers
- Prépare l’arrivée du vrai robot
- Note et partage tes solutions
- Demande de l’aide à la communauté si besoin

--- 