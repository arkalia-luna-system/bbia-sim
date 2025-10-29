# Déployer l’API publique sur Render.com

Ce guide explique comment publier rapidement l’API publique BBIA-SIM sur Render.com à partir de ce dépôt.

## Prérequis
- Compte Render.com
- Repository GitHub connecté (ce projet)
- Fichier `render.yaml` présent à la racine (déjà fourni)

## 1) Connecter le dépôt
1. Aller sur Render.com → New → Web Service.
2. Connecter votre GitHub et sélectionner ce repository.
3. Render détecte `render.yaml` automatiquement.

## 2) Configuration recommandée
- Branche de déploiement: `future` (ou `main` selon votre flux)
- Région: proche de vos utilisateurs
- Environment: Python 3.10+
- Variables d’environnement (optionnelles):
  - `BBIA_API_TOKEN` (si auth requise)
  - `BBIA_DISABLE_AUDIO=1` (recommandé en cloud)

## 3) Déployer
- Cliquer sur Deploy. Le service se construit à partir de `render.yaml`.
- Une fois UP: ouvrir l’URL Render fournie.

## 4) Vérifications
- Santé: `GET /health` → 200 OK
- Docs: `/docs` (Swagger UI) et `/redoc`
- OpenAPI: `/openapi.json`

## 5) Tests rapides (locaux)
```bash
# Mode dev local
python deploy/public_api.py --dev

# Vérifier santé
curl http://localhost:8000/health
```

## Conseils
- Logs Render: onglet “Logs” du service
- Déploiements auto: activés lorsque la branche choisie reçoit de nouveaux commits
- Sécurité: utiliser `BBIA_API_TOKEN` pour protéger les endpoints sensibles

---

Pour tout problème ou suggestion, ouvrir une issue GitHub.
