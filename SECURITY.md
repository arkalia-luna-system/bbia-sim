# Politique de sécurité — BBIA-SIM

**Version** : 1.1 · **Mise à jour** : 2 novembre 2026

BBIA-SIM est un projet **sim-first** (MuJoCo / daemon logiciel). Le robot physique n’est joignable que si `BBIA_ALLOW_HARDWARE=1` est exporté volontairement.

Nous prenons au sérieux les rapports de vulnérabilités sur le **code BBIA**, l’API locale et la chaîne d’approvisionnement (dépendances). Nous ne prétendons **pas** corriger le firmware Pollen / Reachy Mini upstream.

---

## Signalement

**Préférer** (dans cet ordre) :

1. **GitHub Security Advisories** (privé) :  
   https://github.com/arkalia-luna-system/bbia-sim/security/advisories/new  
2. **E-mail** : `arkalia.luna.system@gmail.com` (objet `[SECURITY] bbia-sim …`)
3. Issue GitHub **publique** uniquement si le risque de divulgation est nul (typo doc, lien mort). Sinon : advisory privé.

Merci d’inclure : description, impact, étapes de reproduction, version / commit, correctif suggéré si possible.

### Délais (objectifs, de bonne foi)

| Étape | Objectif |
|:------|:---------|
| Accusé de réception | ≤ 5 jours ouvrés |
| Évaluation initiale | ≤ 14 jours |
| Correctif / mitigation documentée | selon sévérité (souvent 7–30 jours) |

Pas de programme de bug bounty payant à ce jour.

---

## Périmètre

**In scope (BBIA)** : auth / tokens API, CORS, rate-limit, upload média, garde `BBIA_ALLOW_HARDWARE`, fuites de secrets dans le dépôt, dépendances Python du cœur BBIA.

**Hors scope (rediriger vers Pollen)** : firmware robot, daemon officiel `reachy-mini` non forké, GHSA Bluetooth / image robot, GStreamer host côté Pollen — voir [`docs/hardware/POLLEN_ISSUES_ANTICIPATION.md`](docs/hardware/POLLEN_ISSUES_ANTICIPATION.md).

Guide sécurité robot BBIA : [`docs/hardware/SECURITE_ROBOT.md`](docs/hardware/SECURITE_ROBOT.md).

---

## Mesures côté BBIA (logiciel)

Déclaratif — vérifier le tip ; pas une attestation d’audit externe.

- Garde matériel : `BBIA_ALLOW_HARDWARE` (`src/bbia_sim/hardware_guard.py`)
- Clamp amplitudes / joints interdits documentés
- `emergency_stop` côté API / backends
- CI : Bandit, gitleaks, pip-audit (selon workflow)
- Auth Bearer / refus token faible en prod (selon settings daemon)

---

## Urgence robot physique

1. Arrêt d’urgence matériel si disponible  
2. `emergency_stop()` logiciel / API  
3. Couper alimentation si perte de contrôle  
4. Documenter et signaler (advisory ou e-mail)

---

## Historique

Aucune CVE BBIA publiée à ce jour (déclaratif mainteneurs). Les avis Pollen / GHSA upstream sont suivis dans la doc hardware, pas « résolus à 100 % » par BBIA.

**Contact** : `arkalia.luna.system@gmail.com`
