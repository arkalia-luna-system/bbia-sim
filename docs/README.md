# 📚 Documentation BBIA-SIM

> Compatibilité Python et CI
>
> - Python: 3.11+
> - CI: `.github/workflows/ci.yml`
> - Setup rapide:
>   ```bash
>   pyenv install 3.11.9 && pyenv local 3.11.9
>   python -m pip install --upgrade pip
>   pip install -e .
>   ```

Bienvenue dans la documentation du projet BBIA-SIM.

## 📖 Navigation

- **[Index Principal](INDEX_FINAL.md)** - Vue d'ensemble de toute la documentation
- **[Guide Débutant](guides/GUIDE_DEBUTANT.md)** - Commencer avec BBIA-SIM
- **[Guide Reachy Mini](guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md)** - Utiliser le robot physique

## 📁 Structure

La documentation est organisée en catégories :

- **guides/** - Guides pour utilisateurs et développeurs
- **architecture/** - Architecture du système
- **conformite/** - Conformité avec le SDK officiel
- **corrections/** - Corrections appliquées
- **qualite/** - Validation et qualité
- **analyses/** - Analyses du projet
- **audit/** - Audits et rapports
- **simulations/** - Documentation simulation
- **intelligence/** - Intelligence artificielle
- **performance/** - Optimisations performance
- **archives/** - Documentation historique

Voir [INDEX_FINAL.md](INDEX_FINAL.md) pour la liste complète.

## 🚀 Démarrage Rapide

### 3 actions pour démarrer
```bash
# 1) Lancer le dashboard local (FastAPI + WebSocket)
python src/bbia_sim/dashboard_advanced.py

# 2) Démarrer l’API publique (mode dev)
python deploy/public_api.py --dev

# 3) Essayer une démo MuJoCo (3D)
mjpython examples/demo_emotion_ok.py --emotion happy --duration 5
```

- Besoin de détails ? Voir le [Guide Débutant](guides/GUIDE_DEBUTANT.md)
- Variables d’environnement utiles : `BBIA_DISABLE_AUDIO`, `BBIA_TTS_BACKEND`, `BBIA_STT_BACKEND`, `BBIA_LLM_BACKEND`

### Export One‑Pager (PDF/HTML)
```bash
# Installer la dépendance d’export (si non installée)
pip install pypandoc-binary

# Exporter le One‑Pager (PDF si possible, HTML sinon)
scripts/docs/export_onepager.sh
# Sortie dans artifacts/
```

- **PDF nécessite LaTeX** (moteur `xelatex`). Sur macOS, vous pouvez installer un LaTeX minimal:
```bash
# macOS (Homebrew)
brew install pandoc basictex
sudo tlmgr update --self && sudo tlmgr install xetex
```
- Sans LaTeX, le script génère automatiquement un **HTML fallback**.

1. Lire le [README principal](../README.md)
2. Suivre le [Guide Débutant](guides/GUIDE_DEBUTANT.md)
3. Consulter l'[Index](INDEX_FINAL.md) pour trouver ce dont vous avez besoin
