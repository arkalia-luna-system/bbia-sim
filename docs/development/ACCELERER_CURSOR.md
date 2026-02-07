# Accélérer Cursor sur bbia-reachy-sim — guide unique

**Document de référence unique** pour la lenteur de Cursor (causes, solutions, étapes).  
Tous les anciens guides (racine) redirigent ici.

---

## Ce qui a été fait dans le projet

1. **Pylance plus léger**  
   Dans `.vscode/settings.json` : `python.analysis.diagnosticMode` = **`openFiles`**.  
   Pylance n’analyse plus tout le projet au démarrage, seulement les fichiers ouverts.

2. **Workspaces « light »**  
   - **`bbia-reachy-sim-light.code-workspace`** : exclut `assets/`, `tests/`, `docs/`, `presentation/`.
   - **`bbia-reachy-sim-SRC-ONLY.code-workspace`** : n’ouvre que le dossier `src` (~511 fichiers au lieu de 127k).

3. **`.cursorignore`**  
   Exclut venv*, log, artifacts, assets, tests. Tu peux ajouter `docs/` et `presentation/` pour alléger encore.

4. **File watcher**  
   Dans `.cursor/settings.json`, `files.watcherExclude` inclut : venv-voice, log, logs, artifacts, assets, tests, docs.

---

## Causes principales de la lenteur

| Cause | Gravité | Action |
|-------|--------|--------|
| **Cache/état Cursor (~5 Go)** | Élevée | Nettoyer avec le script (étape 1 ci-dessous) |
| **Projet sur T7 USB + 127k fichiers** | Élevée | Ouvrir seulement `src` ou workspace SRC-ONLY |
| **Trop de fichiers surveillés** | Moyenne | Workspace light ou dossier `src` seul |
| **Taille du projet (~22 Go)** | Moyenne | Travailler sur un sous-ensemble (ex. `src`) |
| **RAM 16 Go** | Faible | Garder Cursor seul quand tu travailles sur ce projet |

---

## Étapes recommandées (dans l’ordre)

### Étape 1 : Nettoyer le cache Cursor (obligatoire)

Le cache d’état peut faire ~5 Go et tout ralentir.

1. **Ferme Cursor complètement** (Cmd+Q). Vérifie qu’il n’y a plus d’icône Cursor.
2. **Lance le script de nettoyage** :
   ```bash
   cd "/Volumes/T7/bbia-reachy-sim/scripts"
   chmod +x nettoyer_cache_cursor.sh
   ./nettoyer_cache_cursor.sh
   ```
   Si le script dit « Cursor est encore ouvert », ferme Cursor puis relance le script.
3. **Rouvre Cursor**. Il recréera une base d’état vide.

**Alternative manuelle :**  
Aller dans `~/Library/Application Support/Cursor/User/globalStorage/`, renommer `state.vscdb` et `state.vscdb.backup` en `.old`, puis rouvrir Cursor.

### Étape 2 : Ouvrir seulement le code (recommandé)

- **Option A** : Double-cliquer sur **`bbia-reachy-sim-SRC-ONLY.code-workspace`** (Cursor n’ouvre que `src`).
- **Option B** : **Fichier → Ouvrir le dossier** → choisir **`/Volumes/T7/bbia-reachy-sim/src`**.
- **Option C** : **Fichier → Ouvrir l’espace de travail** → **`bbia-reachy-sim-light.code-workspace`**.

Pour Pylance : Cmd+Shift+P → « Python: Select Interpreter » → choisir le venv du projet (ex. `../venv/bin/python` ou chemin vers `venv/bin/python`).

### Étape 3 : Si c’est encore lent — tester sur le disque interne

Pour vérifier si le T7 (USB) est en cause :

1. Créer un dossier ex. `~/Projects/bbia-src`.
2. Copier uniquement le dossier **`src`** dedans :
   ```bash
   mkdir -p ~/Projects/bbia-src
   cp -R /Volumes/T7/bbia-reachy-sim/src/* ~/Projects/bbia-src/
   ```
3. Dans Cursor : **Fichier → Ouvrir le dossier** → `~/Projects/bbia-src`.
4. Sélectionner l’interpréteur Python du projet sur T7 : `/Volumes/T7/bbia-reachy-sim/venv/bin/python`.

Si Cursor est fluide avec ce dossier sur le disque interne, la lenteur vient du T7 + taille du projet.

### Si c’est encore lent

- **Cursor → Clear Editor History** (ou supprimer le cache du projet dans `~/Library/Application Support/Cursor`).
- Désactiver temporairement des extensions non indispensables (GitLens, spell-checker, etc.).

---

## Récap

| Étape | Action |
|-------|--------|
| 1 | Fermer Cursor → `scripts/nettoyer_cache_cursor.sh` → rouvrir Cursor |
| 2 | Ouvrir `bbia-reachy-sim-SRC-ONLY.code-workspace` ou le dossier `src` seul |
| 3 (optionnel) | Copier `src` sur le disque interne et ouvrir ce dossier dans Cursor |

En faisant au minimum les étapes 1 et 2, tu devrais voir une nette amélioration.

---

**Anciens fichiers à la racine** : `CURSOR_LENTEUR_CAUSES_ET_SOLUTIONS.md` et `GUIDE_ETAPES_LENTEUR.md` redirigent vers ce document.
