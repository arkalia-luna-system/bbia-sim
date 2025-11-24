# 🔒 Politique de Sécurité BBIA-SIM

**Date** : 24 novembre 2025  
**Version** : 1.0

---

## 🛡️ Engagement Sécurité

BBIA-SIM prend la sécurité au sérieux. Nous apprécions les efforts de la communauté pour identifier et signaler les vulnérabilités de sécurité.

---

## 📧 Signalement de Vulnérabilités

### Comment signaler une vulnérabilité

**Email** : [À définir - utiliser issues GitHub pour l'instant]  
**GitHub Issues** : https://github.com/arkalia-luna-system/bbia-sim/issues

**Veuillez inclure** :
- Description détaillée de la vulnérabilité
- Étapes pour reproduire le problème
- Impact potentiel
- Suggestions de correction (si disponibles)

### Processus de traitement

1. **Accusé de réception** : Dans les 48 heures
2. **Évaluation** : Analyse de la vulnérabilité
3. **Correction** : Développement d'un correctif
4. **Publication** : Release de sécurité avec crédit au découvreur

**Délai typique** : 7-14 jours selon la sévérité

---

## 🔐 Bonnes Pratiques de Sécurité

### Pour les Utilisateurs

1. **Mise à jour régulière** : Maintenir BBIA-SIM à jour
2. **Secrets** : Ne jamais commiter de tokens, clés API, ou mots de passe
3. **Environnement** : Utiliser des variables d'environnement pour les secrets
4. **Réseau** : En production, limiter l'accès réseau au robot

### Pour les Développeurs

1. **Validation des entrées** : Toujours valider et sanitizer les entrées utilisateur
2. **Authentification** : Utiliser l'authentification Bearer token en production
3. **Rate limiting** : Activer le rate limiting en production
4. **CORS** : Configurer CORS strictement en production
5. **Dépendances** : Maintenir les dépendances à jour (`pip-audit`, `safety`)

---

## 🔒 Mesures de Sécurité Implémentées

### API REST

- ✅ **Authentification** : Bearer token (HTTPBearer)
- ✅ **Rate limiting** : 100 requêtes/minute par IP (production)
- ✅ **Validation JSON** : Taille max 1MB, validation stricte
- ✅ **Headers sécurité** : X-Content-Type-Options, X-Frame-Options, etc.
- ✅ **CORS** : Configurable (strict en production)

### Robot

- ✅ **Emergency stop** : Arrêt d'urgence implémenté
- ✅ **Watchdog** : Timeout 2s pour sécurité
- ✅ **Limites mécaniques** : Clamping ±0.3 rad par défaut
- ✅ **Joints interdits** : Antennes et joints passifs protégés
- ✅ **Validation duration** : Durée >= 0 requise

### Code

- ✅ **Bandit** : Scan sécurité automatique (CI)
- ✅ **Gitleaks** : Scan secrets automatique (CI)
- ✅ **pip-audit** : Audit dépendances (CI)
- ✅ **Validation entrées** : Path traversal, injection protégées

---

## 📋 Checklist Sécurité Avant Déploiement

### Configuration

- [ ] Variable d'environnement `BBIA_ENVIRONMENT=prod`
- [ ] Token API fort configuré (`BBIA_API_TOKEN`)
- [ ] CORS configuré strictement (origines spécifiques)
- [ ] Rate limiting activé
- [ ] Logs niveau INFO ou WARNING (pas DEBUG en prod)

### Réseau

- [ ] Firewall configuré (ports 8000, 8080)
- [ ] HTTPS activé (reverse proxy recommandé)
- [ ] Accès réseau limité au robot uniquement

### Robot

- [ ] Emergency stop testé
- [ ] Watchdog fonctionnel
- [ ] Limites mécaniques vérifiées
- [ ] Batterie et température surveillées

### Monitoring

- [ ] Métriques Prometheus configurées
- [ ] Alertes configurées (latence, erreurs)
- [ ] Logs centralisés (optionnel)

---

## 🚨 Procédures d'Urgence

### En cas de problème de sécurité

1. **Arrêt immédiat** : Utiliser emergency stop
2. **Isolation** : Déconnecter le robot du réseau si nécessaire
3. **Signalement** : Signaler via GitHub Issues
4. **Documentation** : Documenter l'incident

### Emergency Stop

#### Via Python

```python
from bbia_sim import RobotAPI

robot = RobotAPI()
robot.emergency_stop()  # Arrêt immédiat
```

#### Via API REST

```bash
curl -X POST http://localhost:8000/api/motors/emergency_stop \
  -H "Authorization: Bearer YOUR_TOKEN"
```

#### Via Dashboard

1. Ouvrir le dashboard : http://localhost:8000
2. Cliquer sur le bouton **Emergency Stop** (rouge)
3. Vérifier que le robot s'arrête immédiatement

### Procédures d'Urgence Robot

#### 1. Robot en mouvement incontrôlé

**Actions immédiates** :
1. Appuyer sur **Emergency Stop** (bouton physique si disponible)
2. Utiliser `robot.emergency_stop()` via API
3. Débrancher l'alimentation si nécessaire
4. Vérifier que tous les moteurs sont arrêtés

**Vérifications** :
- Tous les joints à position zéro
- Watchdog actif (timeout 2s)
- Aucun mouvement résiduel

#### 2. Surchauffe ou température élevée

**Actions immédiates** :
1. Arrêter tous les mouvements
2. Vérifier température via `/api/state/battery`
3. Si température > 60°C : arrêt complet
4. Attendre refroidissement avant redémarrage

**Prévention** :
- Surveiller température en continu
- Limiter durée mouvements intenses
- Vérifier ventilation

#### 3. Perte de connexion réseau

**Actions immédiates** :
1. Vérifier connexion réseau
2. Vérifier que le robot est toujours sous contrôle
3. Si perte de contrôle : emergency stop
4. Redémarrer connexion après vérification

**Prévention** :
- Utiliser watchdog (timeout 2s)
- Surveiller latence réseau
- Configurer reconnexion automatique

#### 4. Erreur logicielle critique

**Actions immédiates** :
1. Emergency stop
2. Vérifier logs (`log/bbia.log`)
3. Identifier l'erreur
4. Redémarrer si nécessaire

**Prévention** :
- Tests réguliers
- Monitoring continu
- Logs structurés

### Checklist Post-Urgence

Après un incident d'urgence :

- [ ] Robot arrêté et sécurisé
- [ ] Logs sauvegardés
- [ ] Cause identifiée
- [ ] Correctif appliqué (si nécessaire)
- [ ] Tests de validation effectués
- [ ] Documentation mise à jour
- [ ] Équipe informée

---

## 📚 Ressources

- **Documentation sécurité** : `docs/security/`
- **Tests sécurité** : `tests/test_security.py`, `tests/test_huggingface_security.py`
- **CI/CD sécurité** : `.github/workflows/ci.yml` (bandit, gitleaks, pip-audit)

---

## ✅ Historique des Vulnérabilités

Aucune vulnérabilité signalée à ce jour.

---

**Dernière mise à jour** : 24 novembre 2025

