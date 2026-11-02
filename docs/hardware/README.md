# 🤖 Hardware - Documentation BBIA-SIM

> **Documentation hardware, robotique et Reachy Mini**

---

## 📚 Documentation Hardware

### 🤖 Reachy Mini - Installation et Maintenance

- **[Fiche technique Reachy Mini](FICHE_TECHNIQUE_REACHY_MINI.md)** - Spec officielle (dimensions, moteurs, électronique)
- **[Diagnostic moteurs (officiel)](DIAGNOSTIC_MOTEURS_OFFICIEL.md)** - Testbench, symptômes, reflash, inversion bras, lot QC 2544
- **[Description problème et composants](DESCRIPTION_PROBLEME_ET_COMPOSANTS_REACHY_MINI.md)** - Diagnostic moteurs 12/14, commande pièces, procédure reconfig
- **[Guide Installation Moteurs](GUIDE_INSTALLATION_MOTEURS_ETAPE_PAR_ETAPE.md)** - Guide complet d'installation
- **[Guide Prévention Problèmes](GUIDE_PREVENTION_PROBLEMES_MOTEURS.md)** - Prévention et surveillance
- **[Problèmes Moteurs QC Batch](../_archived/hardware/PROBLEME_MOTEURS_QC_BATCH_DEC2025.md)** - Historique archivé (batch QC déc. 2025)
- **[Ce Qui Reste À Faire](CE_QUI_RESTE_A_FAIRE.md)** - Checklist complète
- **[Checklist redémontage – vérif moteurs/câbles](CHECKLIST_REDEMONTAGE_VERIF_MOTEURS_CABLES.md)** - Étapes quand tu redémonte pour vérifier câbles et moteurs
- **[Résumé Final 26 jan 2026](../_archived/hardware/RESUME_FINAL_26JAN2026.md)** - Snapshot archivé (installation moteurs)

### 🎮 Mouvements et Contrôle

- **[Premier jour robot](PREMIER_JOUR_ROBOT.md)** - Procédure sûre (BBIA_ALLOW_HARDWARE)
- **[Mouvements Reachy Mini](MOUVEMENTS_REACHY_MINI.md)** - Guide des mouvements
- **[Sécurité Robot](SECURITE_ROBOT.md)** - DO/DON'T sécurité

### 📦 Réception et Assemblage

- **[Carte SD Reachy Mini](CARTE_SD_REACHY_MINI.md)** - Informations carte SD (incluse dans le kit)
- **[Suivi Communication Pollen](SUIVI_COMMUNICATION_POLLEN.md)** - Communication avec Pollen

### 🔍 Analyse et Documentation Technique

- **[Vérification alignement Pollen](../quality/VERIFICATION_ALIGNEMENT_POLLEN.md)** - Même niveau que Pollen (SDK, API, outils, Zenoh, doc) — tous domaines
- **[Anticipation issues/GHSA Pollen](POLLEN_ISSUES_ANTICIPATION.md)** - Tableau issue/GHSA → mitigation BBIA (gardes + doc)
- **[Issues Pollen impact BBIA](ISSUES_POLLEN_IMPACT_BBIA.md)** - Historique Zenoh / caméra (complément)
- **[Analyse Repo Officiel](../_archived/hardware/ANALYSE_REPO_OFFICIEL_JANVIER_2026.md)** - Snapshot archivé (janv. 2026)
- **[Impression 3D Stewart Arms](IMPRESSION_3D_STEWART_ARMS.md)** - Guide impression 3D
- **[Analyse 3MF Stewart Arm](ANALYSE_3MF_STEWART_ARM.md)** - Analyse fichier 3MF

## 📚 Documentation Associée

- **Installation** : [`../installation/`](../installation/)
- **Guides** : [`../guides/`](../guides/)
- **Développement** : [`../development/`](../development/)

---

**Dernière mise à jour** : 1 Nov 2026 (MIXTE anticipation + CI UP041 / ConnectionManager)
**SDK Officiel Latest** : v1.13.0 (PyPI) — [pollen-robotics/reachy_mini](https://github.com/pollen-robotics/reachy_mini) (≥ 1.12.0 requis pour correctif PIN GHSA-993g-hgjh-whmf) ; `certifi>=2026.10.19`
**Statut Robot** : ✅ Reçu le 18 déc 2025, monté le 20 déc 2025. **Installation moteurs 1, 2, 4 effectuée** : slots 1 et 2 = QC 2549, slot 4 = QC 2548 (ou 2549). En attente de premier rallumage et validation.
