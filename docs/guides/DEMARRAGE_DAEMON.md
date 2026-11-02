# 🚀 Guide de Démarrage du Daemon BBIA-SIM

**Dernière mise à jour** : 26 Janvier 2026
**Important** : Fonctionne en mode simulation (MuJoCo) sans robot physique

---

## ✅ Démarrage Simple

### Méthode 1 : Module Python (Recommandé)

```bash
python -m bbia_sim.daemon.app.main

```

**Résultat attendu :**

- Dashboard accessible sur http://127.0.0.1:8000/
- API Docs sur http://127.0.0.1:8000/docs
- Simulation MuJoCo démarrée automatiquement (mode headless)
- ✅ **Aucun robot physique requis !**

### Méthode 2 : Script de démarrage

```bash
python scripts/start_public_api.py

```

**Options disponibles :**

```bash
# Mode développement (reload activé)
python scripts/start_public_api.py --dev

# Mode production
python scripts/start_public_api.py --prod --port 8000

# Logs détaillés
python scripts/start_public_api.py --log-level debug

```

---

## 🎯 Mode Simulation (Sans Robot Physique)

**Le daemon fonctionne en mode simulation sans robot physique.**

### Ce qui se passe au démarrage :

1. **Simulation MuJoCo** : Démarre automatiquement en mode headless
   - Utilise le modèle `reachy_mini_REAL_OFFICIAL.xml`
   - Mode simulation pure (pas de hardware requis)

2. **API FastAPI** : Accessible sur http://127.0.0.1:8000
   - Dashboard officiel-like sur `/`
   - API REST complète sur `/development/api/*`
   - WebSocket sur `/ws/*`

3. **Dashboard** : Interface web complète disponible
   - Contrôle daemon ON/OFF
   - Applications
   - App Store Hugging Face
   - Lecteur de mouvements

4. **Backend MuJoCo** : Utilisé automatiquement pour les commandes robot
   - Toutes les commandes fonctionnent en simulation
   - Mouvements, émotions, comportements

### Aucun robot physique requis pour :

- ✅ Voir le dashboard
- ✅ Tester les API endpoints
- ✅ Utiliser les émotions BBIA
- ✅ Contrôler les mouvements (simulation)
- ✅ Tester les comportements
- ✅ Jouer des mouvements enregistrés (simulation)

---

## 🔧 Dépannage

### Erreur "Could not import module 'main'"

**Solution :** Utiliser la méthode recommandée :

```bash
python -m bbia_sim.daemon.app.main

```

Ou via le script :

```bash
python scripts/start_public_api.py

```

Le fichier `__main__.py` a été corrigé pour importer directement l'app (pas de string d'import).

### Warning "You must pass the application as an import string"

**Solution :** Ce warning est normal en mode dev avec reload. Le script `start_public_api.py` utilise maintenant une string d'import quand `reload=True`, et l'objet app directement sinon.

### Simulation ne démarre pas

**Vérifier :**

- MuJoCo installé : `pip install mujoco`
- Modèle XML présent : `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml`

---

## 📊 Vérification

Après démarrage, vérifier que tout fonctionne :

```bash
# Vérifier que le serveur répond
curl http://127.0.0.1:8000/development/api/ecosystem/info

# Ou ouvrir dans le navigateur
open http://127.0.0.1:8000/

```

---

## 🎯 Conclusion

**Le daemon BBIA-SIM fonctionne en mode simulation sans robot physique.**

Toutes les fonctionnalités sont disponibles :

- ✅ Dashboard officiel-like
- ✅ API REST complète
- ✅ Simulation MuJoCo automatique
- ✅ Émotions et comportements BBIA
- ✅ Contrôle mouvements (simulation)

**Le robot physique est optionnel et n'est requis que pour :**

- Tests hardware réels

---

**Pour utiliser le robot physique :** exporter `BBIA_ALLOW_HARDWARE=1`, puis backend
`reachy_mini` avec `use_sim=False`. Sans ce flag, le mode `auto` reste en MuJoCo
(voir `docs/hardware/PREMIER_JOUR_ROBOT.md`). Aucun mouvement matériel au démarrage
du daemon BBIA.

---

## 🎯 Navigation

**Retour à** : [README Documentation](../README.md)
**Voir aussi** : [Guide de Démarrage](GUIDE_DEMARRAGE.md) • [Guide Technique](GUIDE_AVANCE.md) • [Index Thématique](../reference/INDEX_THEMATIQUE.md)
